#!/usr/bin/env python3
"""固定输入离线分析；缺事件只报告缺口，不用最近时间或 counter 回绕补业务身份。"""
import argparse
from collections import Counter, defaultdict
import csv
import hashlib
import io
import json
from pathlib import Path
import statistics
import sys

ROOT = Path(__file__).resolve().parents[2]
sys.path[:0] = [str(ROOT / "tools/adas_ingress_evidence"), str(ROOT / "tools/someip_evidence")]
from performance_stats import distribution, quantize, stamp_ns
import judge


def encoded(value):
    return (json.dumps(value, ensure_ascii=False, sort_keys=True, indent=2, allow_nan=False) + "\n").encode()


def freeze(directory):
    events = json.loads((directory / "events.json").read_bytes())
    if not events.get("flush_done_ns") or any(row.get("timeout") for row in events["exits"].values()):
        raise ValueError("capture not closed/flushed")
    for name, checksum in events["config_sha256"].items():
        content = Path(name).read_bytes()
        if hashlib.sha256(content).hexdigest() != checksum:
            raise ValueError("configuration drift since run: " + name)
        destination = directory / ("config-" + Path(name).parent.name + "-" + Path(name).name)
        with destination.open("xb") as handle:
            handle.write(content)
    facts = {}
    for path in sorted(directory.iterdir()):
        if path.is_file() and path.name != "frozen-inputs.json":
            content = path.read_bytes()
            facts[path.name] = {"sha256": hashlib.sha256(content).hexdigest(), "size": len(content)}
    with (directory / "frozen-inputs.json").open("xb") as handle:
        handle.write(encoded({"schema": 1, "files": facts}))


def load_inputs(directory):
    manifest = json.loads((directory / "frozen-inputs.json").read_bytes())
    inputs = {}
    for name, fact in manifest["files"].items():
        content = (directory / name).read_bytes()
        if len(content) != fact["size"] or hashlib.sha256(content).hexdigest() != fact["sha256"]:
            raise ValueError("frozen input changed: " + name)
        inputs[name] = content
    return inputs


def timing(content, status):
    lines = content.decode().splitlines()
    if not lines or not lines[0].startswith(("# control_link_timing_v1 clock=CLOCK_MONOTONIC", "# control_link_timing_v2 clock=CLOCK_MONOTONIC")):
        raise ValueError("unknown timing clock/schema")
    if not any(line.startswith("# drain ") for line in lines):
        raise ValueError("missing drain marker")
    rows = list(csv.DictReader(io.StringIO("\n".join(line for line in lines if not line.startswith("#")))))
    final = json.loads(status)
    if not final.get("data_closed") or final["io_errors"] or final["queued"] != final["written"] or len(rows) != final["written"]:
        raise ValueError("incomplete timing finalization/accounting")
    numeric = ("pid", "tid", "begin_ns", "end_ns", "scheduled_ns", "sequence", "source_stamp_ns", "frame_id", "result", "value")
    for row in rows:
        for name in numeric:
            row[name] = int(row[name])
        row["source"] = bytes.fromhex(row["source_hex"]).decode()
        row["schedule_clock"] = int(row.get("schedule_clock", 1))
        row["schedule_now_ns"] = int(row.get("schedule_now_ns", 0))
    final["lost_events"] = sum(final[key] for key in ("full", "contended", "invalid_identity", "closed"))
    final["population_complete"] = final["lost_events"] == 0
    return rows, final


def command_key(message):
    return (message["source_sequence"], stamp_ns(message))


def identity_key(row):
    return (row["source"], row["gid"], row["sequence"], row["source_stamp_ns"])


def groups(rows, key):
    result = defaultdict(list)
    for row in rows:
        result[key(row)].append(row)
    return result


def mailbox_handoffs(rows, start, end):
    """marker 来自 consume.begin；写回调区间、完整身份与本次 frame/mode 必须同时闭合。"""
    accepted = [row for row in rows if row["phase"] == "mailbox_write" and row["result"] == 1]
    updates = groups(accepted, lambda row: (row["pid"], identity_key(row)))
    frames = groups([row for row in rows if row["phase"] == "can_write" and row["result"] == 16],
                    lambda row: (row["pid"], row["frame_id"], identity_key(row)))
    modes = groups([row for row in rows if row["phase"] == "canonical_publish"], identity_key)
    seen, used_updates, evidence, counts, values = set(), set(), [], Counter(), defaultdict(list)
    for consume in sorted((row for row in rows if row["phase"] == "mailbox_consume" and row["end_ns"] < end), key=lambda row: row["end_ns"]):
        in_window = start <= consume["end_ns"]
        identity = identity_key(consume)
        candidates = [row for row in updates.get((consume["pid"], identity), []) if row["begin_ns"] <= consume["begin_ns"] <= row["end_ns"]]
        sends = frames.get((consume["pid"], consume["frame_id"], identity), [])
        command_modes = {row["result"] for row in modes.get(identity, [])}
        if len(candidates) != 1 or len(command_modes) != 1 or consume["result"] not in (0, 1):
            if in_window: counts["unmatched_update_or_ambiguous_mode"] += 1
            continue
        if len(sends) != 1 or sends[0]["begin_ns"] < consume["end_ns"] or sends[0]["value"] != consume["value"]:
            if in_window: counts["missing_or_ambiguous_successful_frame"] += 1
            continue
        canonical_mode = next(iter(command_modes))
        if consume["result"] == 1 and canonical_mode != 0:
            if in_window: counts["mode_mismatch"] += 1
            continue
        key = (consume["pid"], identity)
        category = "hold" if consume["result"] == 0 else "reused_active" if key in seen else "first_active_command"
        if consume["result"] == 1:
            seen.add(key)
        if not in_window:
            continue
        update = candidates[0]
        used_updates.add((update["pid"], update["begin_ns"], update["end_ns"], identity))
        span = (consume["end_ns"] - consume["begin_ns"]) / 1e6
        values["mailbox_update_mark_to_worker_frame_ready_" + category + "_ms"].append(span)
        counts[category] += 1
        evidence.append({"identity": identity, "pid": consume["pid"], "frame_id": consume["frame_id"], "counter": consume["value"],
                         "canonical_mode": canonical_mode, "can_mode": consume["result"], "category": category,
                         "mailbox_mark_ns": consume["begin_ns"], "worker_ready_ns": consume["end_ns"], "span_ms": span,
                         "write_callback_begin_ns": update["begin_ns"], "write_callback_end_ns": update["end_ns"],
                         "can_write_return_ns": sends[0]["end_ns"]})
    counts["observed_overwrite_flags"] = sum(row["value"] == 1 for row in accepted if start <= row["end_ns"] < end)
    counts["accepted_updates_without_observed_consume"] = sum((row["pid"], row["begin_ns"], row["end_ns"], identity_key(row)) not in used_updates
                                                             for row in accepted if start <= row["end_ns"] < end)
    return values, {"counts": dict(counts), "events": evidence,
                    "definition": "mailbox internal update mark to worker finished fetch/watchdog/frame generation, including locking/encoding; NOT pure queue wait; first per full command identity, reused ACTIVE and HOLD separate",
                    "limits": "observed records only; no matched consume may mean overwritten, cutoff or recorder loss; ambiguous canonical modes are not guessed; successful CAN write is not bus completion/ACK"}


def resources(rows, start, end, hz):
    result = {}
    for role, samples in groups(rows, lambda row: row["role"]).items():
        samples = sorted((row for row in samples if start <= row["monotonic_ns"] < end), key=lambda row: row["monotonic_ns"])
        cpu, seconds, ticks = [], 0.0, 0
        switches = Counter()
        for before, after in zip(samples, samples[1:]):
            if (before["pid"], before["starttime"]) != (after["pid"], after["starttime"]):
                raise ValueError("resource PID reused")
            elapsed = (after["monotonic_ns"] - before["monotonic_ns"]) / 1e9
            delta = after["cpu_ticks"] - before["cpu_ticks"]
            if elapsed <= 0 or delta < 0:
                raise ValueError("invalid resource interval")
            cpu.append(100 * delta / hz / elapsed)
            seconds += elapsed
            ticks += delta
            for tid in set(before["thread_switches"]) & set(after["thread_switches"]):
                old, new = before["thread_switches"][tid], after["thread_switches"][tid]
                if old["starttime"] != new["starttime"]:
                    switches["reused_tid_intervals"] += 1
                    continue
                for name in ("voluntary_ctxt_switches", "nonvoluntary_ctxt_switches"):
                    switches[name] += new[name] - old[name]
            switches["new_or_exited_tid_intervals"] += len(set(before["thread_switches"]) ^ set(after["thread_switches"]))
        result[role] = {"samples": len(samples), "interval_seconds": seconds,
                        "cpu_mean_one_core_pct": 100 * ticks / hz / seconds if seconds else None,
                        "cpu_max_one_core_pct": max(cpu) if cpu else None,
                        "rss_kib": distribution([row["rss_kib"] for row in samples]),
                        "threads": sorted({row["Threads"] for row in samples}),
                        "fd_count": sorted({row["fd_count"] for row in samples}),
                        "context_switch_delta_matched_tid_intervals": dict(switches)}
    return result


def analyze(directory):
    inputs = load_inputs(directory)
    events = json.loads(inputs["events.json"])
    mapping = judge.load_can_map(inputs["config-adas-can_signal_map.yaml"])
    if events["result"] != 0 or events["stop_reason"] != "fixed_window_complete":
        raise ValueError("not a completed fixed scene")
    start, end = events["window"]["start_ns"], events["window"]["end_ns"]
    streams = {kind: [json.loads(line) for line in inputs[kind + ".jsonl"].splitlines()]
               for kind in ("source", "canonical", "gateway", "status", "vehicle", "can", "resource")}
    selected = lambda row: start <= row["monotonic_ns"] < end
    source = [row for row in streams["source"] if selected(row)]
    canonical = groups([row for row in streams["canonical"] if row["message"]["mode"] == 0 and row["message"]["source_id"] == "planning"],
                       lambda row: command_key(row["message"]))
    by_quantized = groups(streams["source"], lambda row: tuple(row["quantized_key"]))
    if any(len(rows) != 1 for rows in by_quantized.values()):
        raise ValueError("ambiguous source quantization; no per-command CAN estimate allowed")
    can_first, invalid, can_counts = {}, 0, Counter()
    can_times = []
    for row in streams["can"]:
        frame = row["frame"]
        if frame["can_id"] != 0x180:
            continue
        classification, unused = judge.control_frame_fact(frame, mapping)
        payload = bytes.fromhex(frame["data_hex"])
        valid = classification != "invalid"
        if selected(row):
            invalid += not valid
            can_counts[classification] += 1
            can_times.append(row["monotonic_ns"])
        if not valid or payload[4] != 1:
            continue
        key = (int.from_bytes(payload[:2], "little", signed=True), int.from_bytes(payload[2:4], "little", signed=True))
        if key in by_quantized:
            source_row = by_quantized[key][0]
            if row["monotonic_ns"] < source_row["monotonic_ns"]:
                raise ValueError("CAN before unique source target")
            can_first.setdefault(command_key(source_row["message"]), row)
    latencies = defaultdict(list)
    matches = Counter()
    unmatched_commands = []
    for row in source:
        key = command_key(row["message"])
        canon = canonical.get(key, [])
        candidates = [item for item in canon if item["monotonic_ns"] >= row["monotonic_ns"] and
                      [quantize(item["message"]["linear_velocity_mps"], .01), quantize(item["message"]["angular_velocity_radps"], .001)] == row["quantized_key"]]
        if candidates:
            latencies["source_publish_to_first_canonical_observer_ms"].append((min(item["monotonic_ns"] for item in candidates) - row["monotonic_ns"]) / 1e6)
            matches["canonical"] += 1
        if key in can_first:
            latencies["source_publish_to_first_valid_active_vcan_observer_ms"].append((can_first[key]["monotonic_ns"] - row["monotonic_ns"]) / 1e6)
            matches["vcan"] += 1
        if not candidates or key not in can_first:
            unmatched_commands.append({"sequence": key[0], "source_stamp_ns": key[1], "published_ns": row["monotonic_ns"],
                "canonical_missing": not candidates, "vcan_missing": key not in can_first,
                "measurement_end_minus_publish_ms": (end - row["monotonic_ns"]) / 1e6,
                "capture_cutoff_minus_publish_ms": (events.get("cutoff_ns", end) - row["monotonic_ns"]) / 1e6})
    canon_times = [row["monotonic_ns"] for row in streams["canonical"] if selected(row)]
    gaps = [(after - before) / 1e6 for before, after in zip(canon_times, canon_times[1:])]
    gateway = [row for row in streams["gateway"] if selected(row)]
    planning_status = [row for row in streams["status"] if row["message"]["source_id"] == "planning"]
    baseline = [row for row in planning_status if row["monotonic_ns"] <= start]
    final_status = [row for row in planning_status if row["monotonic_ns"] < end]
    result = {"schema": 1, "window": events["window"], "inputs_sha256": hashlib.sha256(inputs["events.json"]).hexdigest(),
              "source_commands": len(source), "first_matches": dict(matches),
              "planning_accepted_delta": final_status[-1]["message"]["accepted_count"] - baseline[-1]["message"]["accepted_count"] if baseline and final_status else None,
              "planning_rejected_delta": final_status[-1]["message"]["rejected_count"] - baseline[-1]["message"]["rejected_count"] if baseline and final_status else None,
              "canonical_received_samples": len(canon_times),
              "canonical_unique_commands": len({command_key(row["message"]) for row in streams["canonical"] if selected(row)}),
              "unmatched": {name: len(source) - matches[name] for name in ("canonical", "vcan")},
              "unmatched_commands": unmatched_commands,
              "source_quantization_unique_whole_session": True, "can_counts": dict(can_counts),
              "canonical_receive_intervals_ms": distribution(gaps),
              "canonical_receive_absolute_deviation_from_20ms": distribution([abs(value - 20) for value in gaps]),
              "can_receive_intervals_ms": distribution([(after - before) / 1e6 for before, after in zip(can_times, can_times[1:])]),
              "gateway_state_observation_counts": dict(Counter(row["message"]["state"] for row in gateway)),
              "nonactive_gateway_observations": [row for row in gateway if row["message"]["state"] != 1],
              "source_status_last": streams["status"][-4:], "fixture_missed_slots_whole_session": events["missed_send_slots"],
              "resource": resources(streams["resource"], start, end, events["clk_tck"]),
              "timing_quality": {}, "scope": "VM_ONLY; observer endpoints include DDS/thread scheduling; CAN is not execution ACK"}
    phase_events = []
    for role in ("gateway", "can_adapter"):
        name = role + "-timing.csv"
        if name in inputs:
            rows, quality = timing(inputs[name], inputs[name + ".status.json"])
            result["timing_quality"][role] = quality
            phase_events.extend(rows)
            ticks = [row for row in rows if row["phase"] == "periodic_tick" and start <= row["begin_ns"] < end]
            legacy_gateway = role == "gateway" and content_is_v1(inputs[name])
            late = [(row["schedule_now_ns"] - row["scheduled_ns"] if row["schedule_clock"] == 2 else row["begin_ns"] - row["scheduled_ns"]) / 1e6
                    for row in ticks if row["scheduled_ns"] and not legacy_gateway]
            result[role + "_internal_deadline_lateness_ms"] = distribution(late)
            if legacy_gateway:
                result["legacy_gateway_deadline_invalidated"] = "v1 mixed MONOTONIC and RCL_STEADY (locked ROS uses RAW); never use prior numeric lateness"
            result[role + "_internal_ticks_observed"] = len(ticks)
            if role == "gateway":
                if not legacy_gateway:
                    result["gateway_entry_clock_query_span_ns"] = distribution([row["value"] for row in ticks])
                publishes = sorted(row["begin_ns"] for row in rows if row["phase"] == "canonical_publish" and start <= row["begin_ns"] < end)
                result["observed_internal_canonical_publish_intervals_ms"] = distribution([(after - before) / 1e6 for before, after in zip(publishes, publishes[1:])])
            else:
                result["can_worker_stop_timeline_ns"] = {row["phase"]: row["begin_ns"] for row in rows if row["phase"] in
                    ("stop_requested", "worker_exit", "worker_joined", "resources_closed")}
                result["can_worker_final_counts"] = [{"missed_periods": row["result"], "receive_budget_yields": row["value"]}
                    for row in rows if row["phase"] == "resources_closed"]
    if phase_events:
        by_phase = groups(phase_events, lambda row: row["phase"])
        incoming = groups(by_phase["source_receive"], lambda row: (row["sequence"], row["source_stamp_ns"]))
        decisions = groups(by_phase["source_result"], identity_key)
        selected_commands = groups(by_phase["source_select"], identity_key)
        publish_by_tick = groups(by_phase["canonical_publish"], lambda row: row["frame_id"])
        bridge_edges = set()
        for row in by_phase["source_select"]:
            outgoing = publish_by_tick.get(row["frame_id"], [])
            if row["result"] == 0 and len(outgoing) == 1:
                if (row["source"], row["sequence"], row["source_stamp_ns"]) != (outgoing[0]["source"], outgoing[0]["sequence"], outgoing[0]["source_stamp_ns"]):
                    raise ValueError("canonical bridge content mismatch")
                bridge_edges.add((row["gid"], outgoing[0]["gid"]))
        result["upstream_to_canonical_publisher_gid_edges"] = sorted(bridge_edges)
        mailbox = groups([row for row in by_phase["mailbox_write"] if row["result"] == 1], identity_key)
        consume = groups(by_phase["mailbox_consume"], lambda row: row["frame_id"])
        writes = groups([row for row in by_phase["can_write"] if row["result"] == 16], identity_key)
        ambiguous = Counter()
        for original in source:
            candidates = incoming.get(command_key(original["message"]), [])
            keys = {identity_key(row) for row in candidates}
            if len(keys) != 1:
                ambiguous["incoming_missing_or_ambiguous"] += 1
                continue
            identity = next(iter(keys))
            received = min(candidates, key=lambda row: row["begin_ns"])
            latencies["source_publish_to_gateway_receive_ms"].append((received["begin_ns"] - original["monotonic_ns"]) / 1e6)
            accepted = [row for row in decisions.get(identity, []) if row["result"] == 1]
            chosen = [row for row in selected_commands.get(identity, []) if row["result"] == 0]
            if accepted:
                latencies["gateway_receive_to_validation_end_ms"].append((accepted[0]["end_ns"] - received["begin_ns"]) / 1e6)
            if not accepted or not chosen:
                ambiguous["accepted_or_selected_missing"] += 1
                continue
            chosen = min(chosen, key=lambda row: row["begin_ns"])
            latencies["validation_end_to_first_selection_ms"].append((chosen["begin_ns"] - accepted[0]["end_ns"]) / 1e6)
            published = publish_by_tick.get(chosen["frame_id"], [])
            if len(published) != 1:
                ambiguous["publish_bridge_missing_or_ambiguous"] += 1
                continue
            published = published[0]
            own_key = identity_key(published)
            latencies["selection_end_to_publish_enter_ms"].append((published["begin_ns"] - chosen["end_ns"]) / 1e6)
            latencies["canonical_publish_call_ms"].append((published["end_ns"] - published["begin_ns"]) / 1e6)
            deliveries = mailbox.get(own_key, [])
            if deliveries:
                latencies["publish_enter_to_first_adapter_callback_ms"].append((min(row["begin_ns"] for row in deliveries) - published["begin_ns"]) / 1e6)
            successful = [row for row in writes.get(own_key, []) if any(
                item["result"] == 1 and identity_key(item) == own_key for item in consume.get(row["frame_id"], []))]
            if successful:
                sent = min(successful, key=lambda row: row["end_ns"])
                latencies["source_publish_to_first_CAN_write_return_ms"].append((sent["end_ns"] - original["monotonic_ns"]) / 1e6)
                latencies["CAN_write_syscall_ms"].append((sent["end_ns"] - sent["begin_ns"]) / 1e6)
        receive_frames = groups(by_phase["can_receive"], lambda row: row["frame_id"])
        for row in by_phase["feedback_validate"]:
            if start <= row["begin_ns"] < end and row["result"] == 1 and len(receive_frames.get(row["frame_id"], [])) == 1:
                latencies["CAN_read_return_to_feedback_validation_end_ms"].append((row["end_ns"] - receive_frames[row["frame_id"]][0]["begin_ns"]) / 1e6)
        feedback = groups(by_phase["feedback_publish"], identity_key)
        for row in by_phase["gateway_feedback"]:
            if start <= row["begin_ns"] < end and len(feedback.get(identity_key(row), [])) == 1:
                latencies["feedback_publish_enter_to_gateway_callback_ms"].append((row["begin_ns"] - feedback[identity_key(row)][0]["begin_ns"]) / 1e6)
        result["correlation_gaps"] = dict(ambiguous)
        result["phase_counts_in_window"] = dict(Counter(row["phase"] for row in phase_events if start <= row["begin_ns"] < end))
        result["source_reject_reasons_in_window"] = dict(Counter(row["value"] for row in by_phase["source_result"] if start <= row["begin_ns"] < end and row["result"] == 0))
        accepted_rows = [row for row in by_phase["source_result"] if start <= row["begin_ns"] < end and row["result"] == 1]
        normal_selected = {identity_key(row) for row in by_phase["source_select"] if row["result"] == 0}
        result["accepted_without_normal_selection"] = [
            {"sequence": row["sequence"], "source_stamp_ns": row["source_stamp_ns"], "accepted_at_ns": row["end_ns"],
             "superseded_by_later_accepted_snapshot_observed": any(
                 (later["source"], later["gid"]) == (row["source"], row["gid"]) and later["end_ns"] > row["end_ns"] for later in accepted_rows)}
            for row in accepted_rows if identity_key(row) not in normal_selected]
        result["mailbox_overwrites_in_window"] = sum(row["value"] == 1 for row in by_phase["mailbox_write"] if start <= row["begin_ns"] < end and row["result"] == 1)
        result["expired_snapshot_identities_in_window"] = len({identity_key(row) for row in by_phase["source_snapshot"] if
            start <= row["begin_ns"] < end and row["sequence"] and row["result"] != 3})
        result["feedback_rejections_in_window"] = sum(row["result"] != 1 for row in by_phase["feedback_validate"] if start <= row["begin_ns"] < end)
        handoff_values, result["mailbox_worker_handoff"] = mailbox_handoffs(phase_events, start, end)
        latencies.update(handoff_values)
    result["latencies_ms"] = {name: distribution(values) for name, values in sorted(latencies.items())}
    result["metrics_population"] = "observed events only; nonzero recorder losses prohibit complete-population claims"
    return result


def content_is_v1(content):
    return content.startswith(b"# control_link_timing_v1 ")


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("scene", type=Path)
    parser.add_argument("--freeze", action="store_true")
    parser.add_argument("--output", type=Path)
    args = parser.parse_args()
    if args.freeze:
        freeze(args.scene)
    if args.output:
        with args.output.open("xb") as handle:
            handle.write(encoded(analyze(args.scene)))
