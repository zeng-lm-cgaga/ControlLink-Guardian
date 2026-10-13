#!/usr/bin/env python3
"""冻结输入的 focused 性能统计；量化键是观测身份，不是新增业务规则。"""

import argparse
import csv
import hashlib
import json
import math
from pathlib import Path
import statistics
import struct
import sys

ROOT = next(parent for parent in Path(__file__).resolve().parents
            if (parent / "tools/someip_evidence/judge.py").is_file())
sys.path.insert(0, str(ROOT / "tools/someip_evidence"))
import judge


def encode_json(value):
    return (json.dumps(value, ensure_ascii=False, sort_keys=True, indent=2,
                       allow_nan=False) + "\n").encode()


def quantize(value, scale):
    """对应生产 CanCodec std::round：半整数远离零。"""
    return int(math.copysign(math.floor(abs(value / scale) + 0.5), value))


def target(sequence, mapping):
    """整场最多 6000 个不同低速组合；耗尽时失败，禁止回绕误配。"""
    if not 1 <= sequence <= 6000:
        raise ValueError("unique quantized target capacity exhausted")
    speed_raw = 20 + (sequence - 1) // 200
    yaw_raw = -100 + (sequence - 1) % 200
    signals = mapping["frames"]["control_command"]["signals"]
    values = []
    for raw, name in ((speed_raw, "target_speed_mps"), (yaw_raw, "target_yaw_rate_radps")):
        spec = signals[name]
        if spec["offset"] != 0:
            raise ValueError("target fixture requires zero signal offset")
        value = raw * spec["scale"]
        if not spec["minimum"] <= value <= spec["maximum"]:
            raise ValueError("fixture target outside CAN map range")
        if quantize(value, spec["scale"]) != raw:
            raise ValueError("production rounding does not preserve target key")
        values.append(value)
    return values, [speed_raw, yaw_raw]


def distribution(values):
    """使用确定性 linear interpolation；少于 100 样本不报告 p99。"""
    ordered = sorted(values)
    if not ordered:
        return {"samples": 0}

    def percentile(fraction):
        position = (len(ordered) - 1) * fraction
        lower = math.floor(position)
        upper = math.ceil(position)
        return ordered[lower] + (ordered[upper] - ordered[lower]) * (position - lower)

    return {"samples": len(ordered), "min": ordered[0], "mean": statistics.fmean(ordered),
            "p50": percentile(0.5), "p95": percentile(0.95),
            "p99": percentile(0.99) if len(ordered) >= 100 else None, "max": ordered[-1]}


def stamp_ns(message):
    return message["source_stamp"]["sec"] * 1_000_000_000 + message["source_stamp"]["nanosec"]


def trace_command(message):
    """生产 trace 用 IEEE754 bits 无损记录速度，不是 ROS 消息字段名。"""
    return {"source_stamp": message["source_stamp"], "source_id": message["source_id"],
            "source_sequence": message["source_sequence"], "mode": message["mode"],
            "linear_velocity_mps": struct.unpack("!d", bytes.fromhex(message["linear_velocity_bits"]))[0],
            "angular_velocity_radps": struct.unpack("!d", bytes.fromhex(message["angular_velocity_bits"]))[0]}


def frozen_inputs(report):
    freeze = json.loads((report / "frozen-inputs.json").read_bytes())
    inputs = {}
    for name, fact in freeze["files"].items():
        contents = (report / name).read_bytes()
        if hashlib.sha256(contents).hexdigest() != fact["sha256"]:
            raise ValueError("frozen input SHA mismatch: " + name)
        inputs[name] = contents
    return inputs


def calculate(report):
    """所有命令均保留；未匹配、异常与丢样计数不能被时延过滤掩盖。"""
    inputs = frozen_inputs(report)
    streams = {name: [json.loads(line) for line in inputs[name + ".jsonl"].splitlines()]
               for name in ("source", "canonical", "can", "gateway", "status", "vehicle", "resource")}
    plan = json.loads(inputs["windows.json"])
    events = json.loads(inputs["events.json"])
    mapping = judge.load_can_map(inputs["can_signal_map.yaml"])
    ticks, results = [], {}
    trace = [json.loads(line) for line in inputs["decision-trace.jsonl"].splitlines()]
    for item in trace:
        if item.get("event_type") == "output_tick":
            ticks.append(item)
        if item.get("record_type") == "result":
            results[item["event_sequence"]] = {**item, "canonical_command": trace_command(item["canonical_command"])}
    footer = trace[-1]
    event_sequences = [item["event_sequence"] for item in trace if item.get("record_type") == "event"]
    if (footer.get("record_type") != "footer" or not footer.get("trace_valid") or
            footer.get("trace_overflow") or footer.get("event_count") != len(event_sequences) or
            footer.get("result_count") != len(results) or len(results) != len(ticks) or
            event_sequences != list(range(1, len(event_sequences) + 1))):
        raise ValueError("production trace is incomplete or invalid")
    source_by_sequence = {}
    source_by_key = {}
    for item in streams["source"]:
        sequence = item["message"]["source_sequence"]
        key = tuple(item["quantized_key"])
        values, expected_key = target(sequence, mapping)
        if (sequence in source_by_sequence or key in source_by_key or list(key) != expected_key or
                [item["message"]["linear_velocity_mps"], item["message"]["angular_velocity_radps"]] != values):
            raise ValueError("source sequence/quantized key is not one-to-one")
        source_by_sequence[sequence] = item
        source_by_key[key] = sequence
    first_canonical, first_can = {}, {}
    canonical_mismatch = 0
    canonical_receive_counts = {}
    for item in streams["canonical"]:
        message = item["message"]
        if message["source_id"] != "planning" or message["mode"] != 0:
            continue
        sequence = message["source_sequence"]
        source = source_by_sequence.get(sequence)
        if source is None or message != {**source["message"], "source_id": "planning"}:
            canonical_mismatch += 1
            continue
        if item["monotonic_ns"] < source["publish_before_monotonic_ns"]:
            raise ValueError("canonical precedes its source publish clock")
        first_canonical.setdefault(sequence, item)
        canonical_receive_counts[sequence] = canonical_receive_counts.get(sequence, 0) + 1
    canonical_internal_counts = {}
    for item in results.values():
        message = item["canonical_command"]
        if message["source_id"] == "planning" and message["mode"] == 0:
            sequence = message["source_sequence"]
            canonical_internal_counts[sequence] = canonical_internal_counts.get(sequence, 0) + 1
    can_rows = []
    previous_counter = None
    for item in streams["can"]:
        fact, unused_speed = judge.control_frame_fact(item["frame"], mapping)
        if fact == "other":
            continue
        payload = bytes.fromhex(item["frame"]["data_hex"])
        key = (int.from_bytes(payload[:2], "little", signed=True),
               int.from_bytes(payload[2:4], "little", signed=True))
        sequence = source_by_key.get(key) if fact == "active_nonzero" else None
        counter = payload[5] & 15
        jump = previous_counter is not None and counter != (previous_counter + 1) % 16
        previous_counter = counter
        enriched = {"monotonic_ns": item["monotonic_ns"], "wall_ns": item["wall_ns"],
                    "kernel_realtime_ns": item["frame"]["kernel_wall_ns"],
                    "data_hex": item["frame"]["data_hex"], "fact": fact,
                    "counter": counter, "counter_discontinuity": jump,
                    "mode": payload[4], "valid": payload[6] & 1,
                    "crc_valid": judge.crc8(item["frame"]["can_id"], payload) == payload[7],
                    "speed_raw": key[0], "yaw_raw": key[1], "source_sequence": sequence}
        can_rows.append(enriched)
        if sequence is not None:
            if item["monotonic_ns"] < source_by_sequence[sequence]["publish_before_monotonic_ns"]:
                raise ValueError("CAN precedes its unique source publish clock")
            first_can.setdefault(sequence, enriched)
    latency_rows, summaries = [], []
    metric_values = {name: [] for name in (
        "source_to_canonical_receive_ms", "source_to_vcan_receive_ms",
        "internal_tick_interval_ms", "internal_tick_abs_deviation_ms",
        "canonical_receive_interval_ms", "canonical_receive_abs_deviation_ms")}
    resource_rows = []
    for window in plan["windows"]:
        start, end = window["start_monotonic_ns"], window["end_monotonic_ns"]

        def selected(name):
            return [item for item in streams[name] if start <= item["monotonic_ns"] < end]

        sources = [item for item in streams["source"]
                   if start <= item["publish_before_monotonic_ns"] < end]
        sequences = [item["message"]["source_sequence"] for item in sources]
        canonical = selected("canonical")
        control = [item for item in can_rows if start <= item["monotonic_ns"] < end]
        internal = [item for item in ticks
                    if window["start_wall_ns"] <= item["now_ros_ns"] < window["end_wall_ns"]]
        internal_results = [results[item["event_sequence"]] for item in internal]
        normal_ticks = [item for item in internal_results if item["state"] == 1]
        if len(internal_results) != len(internal):
            raise ValueError("missing trace result")
        values = {name: [] for name in metric_values}
        for item in sources:
            sequence = item["message"]["source_sequence"]
            origin = item["publish_before_monotonic_ns"]
            row = {"window": window["name"], "source_sequence": sequence,
                   "source_stamp_ns": stamp_ns(item["message"]), "source_publish_before_ns": origin,
                   "speed_raw": item["quantized_key"][0], "yaw_raw": item["quantized_key"][1]}
            for name, matches in (("source_to_canonical_receive_ms", first_canonical),
                                  ("source_to_vcan_receive_ms", first_can)):
                match = matches.get(sequence)
                row[name] = None if match is None else (match["monotonic_ns"] - origin) / 1_000_000
                if match is not None:
                    values[name].append(row[name])
                    row[name + "_end_ns"] = match["monotonic_ns"]
            latency_rows.append(row)
        for name, records, time_key in (("internal_tick", internal, "steady_offset_ns"),
                                        ("canonical_receive", canonical, "monotonic_ns")):
            intervals = [(right[time_key] - left[time_key]) / 1_000_000
                         for left, right in zip(records, records[1:])]
            values[name + "_interval_ms"] = intervals
            values[name + "_abs_deviation_ms"] = [abs(value - 20) for value in intervals]
        source_status = [item["message"] for item in selected("status")
                         if item["message"]["source_id"] == "planning"]
        gateways = selected("gateway")
        vehicles = selected("vehicle")
        internal_stamps = {item["now_ros_ns"] for item in internal}
        gateway_stamps = {item["message"]["observed_at"]["sec"] * 1_000_000_000 +
                          item["message"]["observed_at"]["nanosec"] for item in streams["gateway"]}
        receive_sequence_set = {item["message"]["source_sequence"] for item in canonical
                                if item["message"]["source_id"] == "planning" and item["message"]["mode"] == 0}
        trace_source_sequences = {item["canonical_command"]["source_sequence"] for item in normal_ticks
                                  if item["canonical_command"]["source_id"] == "planning"}
        duplicate_receive_deficits = {str(sequence): canonical_internal_counts.get(sequence, 0) -
            canonical_receive_counts.get(sequence, 0) for sequence in sequences
            if canonical_internal_counts.get(sequence, 0) > canonical_receive_counts.get(sequence, 0)}
        baseline_ticks = [item for item in ticks if item["now_ros_ns"] < window["start_wall_ns"]]
        before_sources = results[baseline_ticks[-1]["event_sequence"]]["sources"] if baseline_ticks else []
        accepted_before = next((item["accepted_count"] for item in before_sources
                                if item["source_id"] == "planning"), None)
        accepted_end = next((item["accepted_count"] for item in internal_results[-1]["sources"]
                             if item["source_id"] == "planning"), None) if internal_results else None
        resource_facts = {}
        for role in events["resource_roles"]:
            samples = [item for item in streams["resource"] if item["role"] == role]
            rss_samples = [item["rss_kib"] for item in samples if start <= item["monotonic_ns"] < end]
            intervals = []
            for previous, current in zip(samples, samples[1:]):
                if previous["starttime"] != current["starttime"]:
                    raise ValueError("PID reused during sampling")
                if not start <= previous["monotonic_ns"] < current["monotonic_ns"] <= end:
                    continue
                elapsed = (current["monotonic_ns"] - previous["monotonic_ns"]) / 1_000_000_000
                ticks_used = current["cpu_ticks"] - previous["cpu_ticks"]
                cpu = ticks_used / events["clk_tck"] / elapsed * 100
                interval = {"window": window["name"], "role": role, "pid": current["pid"],
                            "starttime": current["starttime"], "start_ns": previous["monotonic_ns"],
                            "end_ns": current["monotonic_ns"], "elapsed_s": elapsed,
                            "cpu_ticks_delta": ticks_used, "cpu_percent": cpu}
                intervals.append(interval)
                resource_rows.append(interval)
            elapsed = sum(item["elapsed_s"] for item in intervals)
            resource_facts[role] = {
                "pid": samples[0]["pid"], "starttime": samples[0]["starttime"],
                "cpu_intervals": len(intervals), "cpu_covered_seconds": elapsed,
                "cpu_mean_percent": sum(item["cpu_ticks_delta"] for item in intervals) /
                    events["clk_tck"] / elapsed * 100 if elapsed else None,
                "cpu_max_percent": max((item["cpu_percent"] for item in intervals), default=None),
                "rss_kib": distribution(rss_samples),
            }
        anomalies = {
            "internal_non_active_ticks": len(internal) - len(normal_ticks),
            "gateway_not_planning_active": sum(item["message"]["state"] != 1 or
                item["message"]["active_source_id"] != "planning" for item in gateways),
            "vehicle_not_running_or_fault": sum(item["message"]["state"] != 1 or
                item["message"]["fault_code"] != 0 for item in vehicles),
            "can_invalid": sum(item["fact"] == "invalid" for item in control),
            "can_not_active_nonzero": sum(item["fact"] != "active_nonzero" for item in control),
            "can_counter_discontinuities": sum(item["counter_discontinuity"] for item in control),
            "can_unmatched_active_frames": sum(item["fact"] == "active_nonzero" and
                item["source_sequence"] is None for item in control),
            "input_sequence_gaps": sum(max(0, right - left - 1)
                for left, right in zip(sequences, sequences[1:])),
            "source_to_canonical_unmatched": sum(sequence not in first_canonical for sequence in sequences),
            "source_to_can_unmatched": sum(sequence not in first_can for sequence in sequences),
            "canonical_observer_unique_sequence_missing_vs_internal": len(trace_source_sequences -
                {item["message"]["source_sequence"] for item in streams["canonical"]
                 if item["message"]["source_id"] == "planning" and item["message"]["mode"] == 0}),
            "gateway_observer_missing_ticks": len(internal_stamps - gateway_stamps),
            "canonical_observer_missing_duplicate_messages_by_full_session_sequence_counts":
                sum(duplicate_receive_deficits.values()),
            "missed_source_scheduled_slots": sum(max(0, (right["scheduled_monotonic_ns"] -
                left["scheduled_monotonic_ns"]) // 50_000_000 - 1) for left, right in zip(sources, sources[1:])),
            "internal_canonical_source_or_fields_mismatch": sum(
                item["canonical_command"] != {**source_by_sequence.get(
                    item["canonical_command"]["source_sequence"], {}).get("message", {}), "source_id": "planning"}
                for item in normal_ticks),
        }
        for name, samples in values.items():
            metric_values[name].extend(samples)
        summaries.append({"window": window, "source_sent": len(sources),
            "source_sequence_first": sequences[0] if sequences else None,
            "source_sequence_last": sequences[-1] if sequences else None,
            "canonical_received": len(canonical), "canonical_unique_sequences": len(receive_sequence_set),
            "internal_ticks": len(internal), "internal_unique_sequences": len(trace_source_sequences),
            "can_control_frames": len(control), "can_unique_sequences": len({item["source_sequence"]
                for item in control if item["source_sequence"] is not None}),
            "accepted_first": source_status[0]["accepted_count"] if source_status else None,
            "accepted_last": source_status[-1]["accepted_count"] if source_status else None,
            "accepted_increment": source_status[-1]["accepted_count"] - source_status[0]["accepted_count"]
                if source_status else None,
            "trace_accepted_before_window": accepted_before,
            "trace_accepted_end_of_window": accepted_end,
            "trace_accepted_increment": accepted_end - accepted_before
                if accepted_before is not None and accepted_end is not None else None,
            "canonical_count_received_minus_internal": len(canonical) - len(internal),
            "missing_duplicate_canonical_messages_by_sequence": duplicate_receive_deficits,
            "source_publish_interval_ms": distribution([(right["publish_before_monotonic_ns"] -
                left["publish_before_monotonic_ns"]) / 1_000_000 for left, right in zip(sources, sources[1:])]),
            "source_generation_count": len({tuple(item["endpoint_publisher_gid"]) for item in source_status}),
            "status_unhealthy": sum(not item["command_valid"] or not item["lease_valid"] or
                not item["endpoint_stable"] or item["endpoint_publisher_count"] != 1 or
                not item["endpoint_generation_valid"] for item in source_status),
            "anomalies": anomalies, "metrics": {name: distribution(samples) for name, samples in values.items()},
            "resources": resource_facts})
    clock_offsets = [item["wall_ns"] - item["monotonic_ns"] for kind in ("canonical", "can", "gateway", "status")
                     for item in streams[kind]]
    aggregate_resources = {}
    for role in events["resource_roles"]:
        intervals = [item for item in resource_rows if item["role"] == role]
        rss = [item["rss_kib"] for item in streams["resource"] if item["role"] == role and
               plan["windows"][0]["start_monotonic_ns"] <= item["monotonic_ns"] <
               plan["windows"][-1]["end_monotonic_ns"]]
        elapsed = sum(item["elapsed_s"] for item in intervals)
        aggregate_resources[role] = {"cpu_intervals": len(intervals), "cpu_covered_seconds": elapsed,
            "cpu_mean_percent": sum(item["cpu_ticks_delta"] for item in intervals) /
                events["clk_tck"] / elapsed * 100 if elapsed else None,
            "cpu_max_percent": max((item["cpu_percent"] for item in intervals), default=None),
            "rss_kib": distribution(rss)}
    summary = {"scope": "VM_ONLY; focused 10s warm-up + three serial 60s windows in one session; not E11",
        "definitions": {"latency_clock": "shared Linux CLOCK_MONOTONIC / time.monotonic_ns",
            "source_origin": "immediately before publisher.publish, after target/stamp generation",
            "canonical_endpoint": "observer subscription callback entry; includes DDS/observer scheduling",
            "can_endpoint": "observer SocketCAN recvmsg return monotonic; not send-complete or actuator ACK",
            "internal_interval": "existing DecisionTrace output_tick steady_offset_ns differences; not publish completion",
            "external_interval": "canonical callback receive differences; includes DDS/observer overhead",
            "absolute_deviation": "abs(interval_ms - 20.0); not phase error",
            "trace_windows": "now_ros_ns compared only with paired SYSTEM window boundaries; no cross-clock latency subtraction",
            "percentiles": "linear interpolation at (n-1)*p; p99 omitted for n<100",
            "cpu": "utime+stime delta / CLK_TCK / measured monotonic interval *100; 100% = one logical CPU",
            "resource_intervals": "only full adjacent 1s samples contained in each fixed window; actual coverage reported"},
        "trace_footer": footer, "canonical_field_mismatches": canonical_mismatch,
        "clock_wall_minus_monotonic_range_ns": max(clock_offsets) - min(clock_offsets),
        "ambiguous_quantized_keys": 0,
        "windows": summaries,
        "aggregate": {"source_sent": sum(item["source_sent"] for item in summaries),
            "canonical_received": sum(item["canonical_received"] for item in summaries),
            "internal_ticks": sum(item["internal_ticks"] for item in summaries),
            "can_control_frames": sum(item["can_control_frames"] for item in summaries),
            "metrics": {name: distribution(samples) for name, samples in metric_values.items()},
            "resources": aggregate_resources}}
    return summary, latency_rows, can_rows, resource_rows


def write_csv(path, records):
    if not records:
        return
    names = sorted({name for record in records for name in record})
    with path.open("x", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=names)
        writer.writeheader()
        writer.writerows(records)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--report", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--csv-dir", type=Path)
    args = parser.parse_args()
    summary, latency, frames, resources = calculate(args.report)
    with args.output.open("xb") as handle:
        handle.write(encode_json(summary))
    if args.csv_dir:
        args.csv_dir.mkdir(exist_ok=False)
        write_csv(args.csv_dir / "per-command-latency.csv", latency)
        write_csv(args.csv_dir / "decoded-can.csv", frames)
        write_csv(args.csv_dir / "cpu-intervals.csv", resources)


if __name__ == "__main__":
    main()
