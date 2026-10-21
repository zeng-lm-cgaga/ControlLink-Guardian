#!/usr/bin/env python3
"""由冻结原始输入生成本批表格；同输入无时间戳随机量，可逐字节重放。"""
import argparse
import csv
import hashlib
import json
from pathlib import Path

from analyze import analyze, distribution, encoded, load_inputs, timing


def write_csv(path, rows, columns):
    with path.open("x", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=columns)
        writer.writeheader(); writer.writerows(rows)


def generate(base, output):
    output.mkdir(parents=True, exist_ok=False)
    summaries = {}
    metrics, resources, unmatched = [], [], []
    scenes = sorted(path for path in (base / "scenes").iterdir() if path.name.startswith(("final-", "v2r1-", "v3-")))
    for scene in scenes:
        summary = analyze(scene)
        summaries[scene.name] = summary
        (output / (scene.name + ".json")).write_bytes(encoded(summary))
        for name, values in summary["latencies_ms"].items():
            metrics.append({"scene": scene.name, "metric": name, **{key: values.get(key) for key in ("samples", "min", "mean", "p50", "p95", "p99", "max")}})
        for name in ("gateway_internal_deadline_lateness_ms", "can_adapter_internal_deadline_lateness_ms",
                     "observed_internal_canonical_publish_intervals_ms", "canonical_receive_intervals_ms"):
            values = summary.get(name, {})
            metrics.append({"scene": scene.name, "metric": name, **{key: values.get(key) for key in ("samples", "min", "mean", "p50", "p95", "p99", "max")}})
        for role, values in summary["resource"].items():
            resources.append({"scene": scene.name, "role": role, "interval_seconds": values["interval_seconds"],
                              "samples": values["samples"], "cpu_mean_one_core_pct": values["cpu_mean_one_core_pct"],
                              "cpu_max_one_core_pct": values["cpu_max_one_core_pct"], "rss_mean_kib": values["rss_kib"].get("mean"),
                              "rss_max_kib": values["rss_kib"].get("max"), "thread_counts": json.dumps(values["threads"]),
                              "fd_counts": json.dumps(values["fd_count"])})
        unmatched.extend({"scene": scene.name, **row} for row in summary["unmatched_commands"])
    write_csv(output / "metrics.csv", metrics, ["scene", "metric", "samples", "min", "mean", "p50", "p95", "p99", "max"])
    write_csv(output / "resources.csv", resources, ["scene", "role", "interval_seconds", "samples", "cpu_mean_one_core_pct", "cpu_max_one_core_pct", "rss_mean_kib", "rss_max_kib", "thread_counts", "fd_counts"])
    write_csv(output / "unmatched.csv", unmatched, ["scene", "sequence", "source_stamp_ns", "published_ns", "canonical_missing", "vcan_missing", "measurement_end_minus_publish_ms", "capture_cutoff_minus_publish_ms"])
    overhead = {}
    for off, on in [("final-off-" + str(index), "final-on-" + str(index)) for index in range(1, 4)] + [("v2r1-off", "v2r1-on"), ("v3-off", "v3-on")]:
        pairs = {}
        for role in ("gateway", "can_adapter", "planning_ingress", "simulator", "idle_teleop", "fixture_observer"):
            before, after = summaries[off]["resource"][role], summaries[on]["resource"][role]
            pairs[role] = {"cpu_percentage_point_delta": after["cpu_mean_one_core_pct"] - before["cpu_mean_one_core_pct"],
                           "rss_mean_kib_delta": after["rss_kib"]["mean"] - before["rss_kib"]["mean"]}
        overhead[off + "_to_" + on] = pairs
    (output / "overhead.json").write_bytes(encoded({"paired_observations": overhead,
        "limits": "not randomized; VM background load/20ms CAN phase differ; no latency improvement claim; BPF overhead NOT MEASURED"}))
    pause = load_inputs(base / "scenes/v2r1-pause")
    events = json.loads(pause["events.json"])
    begin, end = events["planned_source_pause"]
    gateway = [json.loads(line) for line in pause["gateway.jsonl"].splitlines()]
    timeline, previous = [], None
    for row in gateway:
        message = row["message"]
        if not begin - 200000000 <= row["monotonic_ns"] <= end + 1500000000:
            continue
        key = (message["state"], message["reason_code"], message["recovery_valid_count"])
        if key == previous:
            continue
        previous = key
        timeline.append({"received_monotonic_ns": row["monotonic_ns"], "offset_from_pause_start_ms": (row["monotonic_ns"] - begin) / 1e6,
                         **{name: message[name] for name in ("state", "reason_code", "active_source_sequence", "recovery_valid_count", "transition_sequence")}})
    write_csv(output / "pause-gateway-timeline.csv", timeline, ["received_monotonic_ns", "offset_from_pause_start_ms", "state", "reason_code", "active_source_sequence", "recovery_valid_count", "transition_sequence"])
    gateway_events, ignored = timing(pause["gateway-timing.csv"], pause["gateway-timing.csv.status.json"])
    accepted = [row for row in gateway_events if row["phase"] == "source_result" and row["result"] == 1 and end <= row["end_ns"] <= end + 1000000000]
    first_active = next((row for row in gateway if end <= row["monotonic_ns"] < end + 1500000000 and row["message"]["state"] == 1), None)
    recovery = {"planned_pause": events["planned_source_pause"], "intentional_unsent_slots": events["intentional_pause_slots"],
                "accepted_after_resume": accepted, "first_active_after_resume": first_active,
                "accepted_records_before_first_active_receive": sum(row["end_ns"] <= first_active["monotonic_ns"] for row in accepted) if first_active else None,
                "limits": "gateway observer receive is later than production publish; not an atomic snapshot; original recovery rule unchanged"}
    (output / "pause-recovery.json").write_bytes(encoded(recovery))
    rows, quality = timing((base / "raw/workload.csv").read_bytes(), (base / "raw/workload.csv.status.json").read_bytes())
    workload = {phase: distribution([(row["end_ns"] - row["begin_ns"]) / 1e6 for row in rows if row["phase"] == phase])
                for phase in ("work_cpu", "work_sleep", "work_lock", "work_hold")}
    (output / "workload.json").write_bytes(encoded({"quality": quality, "durations_ms": workload, "BPF": "NOT LOADED; user-space known-cause durations only"}))
    lines = ["# 本批固定窗口结果（源 publish 前→CRC有效 ACTIVE 非零 vcan 观察接收）", "",
             "| 窗口 | 源命令 | canonical首匹配 | CAN首匹配 | mean ms | p50 ms | p95 ms | p99 ms | max ms |", "|---|---:|---:|---:|---:|---:|---:|---:|---:|"]
    for name, summary in sorted(summaries.items()):
        value = summary["latencies_ms"]["source_publish_to_first_valid_active_vcan_observer_ms"]
        lines.append("| " + " | ".join([name, str(summary["source_commands"]), str(summary["first_matches"].get("canonical", 0)),
            str(summary["first_matches"].get("vcan", 0))] + [f"{value[key]:.6f}" for key in ("mean", "p50", "p95", "p99", "max")]) + " |")
    lines += ["", "全部 10s warm-up + 20s 固定窗；pause 窗故意停源 1s，不能作为稳态延迟基准。未匹配逐条见 unmatched.csv，不能补成成功样本。",
              "v1 只作补充：其 Gateway deadline 跨钟错误已否定，summary 不给该指标；本表源/CAN端点均 MONOTONIC，不受该错误影响。",
              "详细阶段、资源、样本量、丢记录和边界分别见 metrics.csv、resources.csv、每窗 JSON。BPF未加载，表中 on 仅应用打点。", ""]
    (output / "results.md").write_text("\n".join(lines))
    manifest = {str(path.relative_to(output)): hashlib.sha256(path.read_bytes()).hexdigest()
                for path in sorted(output.iterdir()) if path.is_file()}
    (output / "outputs.json").write_bytes(encoded(manifest))


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("base", type=Path)
    parser.add_argument("output", type=Path)
    args = parser.parse_args()
    generate(args.base, args.output)
