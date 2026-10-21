#!/usr/bin/env python3
"""调度/系统调用区间与同 PID/TID 的应用区间相交；不会给 BPF 事件补造业务序号。"""
import argparse
from collections import Counter, defaultdict
import csv
import io
import json
from pathlib import Path
import re

from analyze import distribution, encoded, timing


def analyze_kernel(content, loader_log, app=None):
    lines = content.decode().splitlines()
    footers = [line for line in lines if line.startswith("# footer ")]
    exit_codes = re.findall(r"loader_exit=(\d+)", loader_log.decode())
    if len(footers) != 1 or not exit_codes or int(exit_codes[-1]) not in (0, 3):
        raise ValueError("BPF load/capture/finalization incomplete")
    footer = {name: int(value) for name, value in re.findall(r"(\w+)=(\d+)", footers[0])}
    rows = [{key: int(value) for key, value in row.items()} for row in csv.DictReader(
        io.StringIO("\n".join(line for line in lines if not line.startswith("#"))))]
    if footer["received"] != len(rows):
        raise ValueError("BPF CSV count mismatch")
    header = next((line for line in lines if line.startswith("# monotonic_ns ")), "")
    identity = {name: int(value) for name, value in re.findall(r"(\w+)=(\d+)", header)}
    attached = re.findall(r"# attached=(\d+) start_monotonic_ns=(\d+)", content.decode())
    stages = defaultdict(list)
    intervals = defaultdict(list)
    outgoing = {}
    for row in sorted(rows, key=lambda row: row["at_ns"]):
        key = (row["tgid"], row["tid"], row["task_start_boottime_ns"])
        at, kind = row["at_ns"], row["kind"]
        if kind == 2:
            outgoing[key] = row
            if row["duration_ns"]:
                stages["on_cpu_observed_ms"].append(row["duration_ns"] / 1e6)
                intervals[(row["tgid"], row["tid"])].append((at - row["duration_ns"], at, "on_cpu"))
        elif kind == 3:
            if row["value"] == 1:
                stages["successful_wakeup_to_on_cpu_ms"].append(row["duration_ns"] / 1e6)
                intervals[(row["tgid"], row["tid"])].append((at - row["duration_ns"], at, "wakeup_wait"))
            previous = outgoing.pop(key, None)
            if previous and row["auxiliary_ns"]:
                name = "runnable_off_cpu" if previous["value"] == 0 or previous["auxiliary_ns"] else "blocked_off_cpu"
                stages[name + "_ms"].append(row["auxiliary_ns"] / 1e6)
                intervals[(row["tgid"], row["tid"])].append((at - row["auxiliary_ns"], at, name))
        elif kind == 5:
            name = "syscall_" + str(row["auxiliary_ns"])
            stages[name + "_ms"].append(row["duration_ns"] / 1e6)
            intervals[(row["tgid"], row["tid"])].append((at - row["duration_ns"], at, name))
    correlation = []
    eligible = {"source_receive", "source_result", "source_select", "canonical_publish", "mailbox_write",
                "can_write", "feedback_validate", "gateway_feedback", "work_cpu", "work_sleep", "work_lock", "work_hold"}
    if app:
        for row in app:
            if row["phase"] not in eligible or row["end_ns"] < row["begin_ns"]:
                continue
            overlap = defaultdict(int)
            for begin, end, kind in intervals.get((row["pid"], row["tid"]), []):
                length = min(row["end_ns"], end) - max(row["begin_ns"], begin)
                if length > 0:
                    overlap[kind] += length
            if overlap:
                correlation.append({"phase": row["phase"], "pid": row["pid"], "tid": row["tid"],
                                    "source": row["source"], "gid": row["gid"], "sequence": row["sequence"],
                                    "source_stamp_ns": row["source_stamp_ns"], "begin_ns": row["begin_ns"],
                                    "end_ns": row["end_ns"], "overlap_ns_by_kind": dict(overlap)})
    accounting = int(exit_codes[-1]) == 0 and footer["ring_full"] == footer["map_full"] == 0 and footer["emitted"] == footer["received"]
    return {"schema": 2, "quality": footer, "accounting_complete": accounting,
            "population_complete": accounting and bool(rows), "coverage_status": "NOT_EVALUATED",
            "target_header": identity, "attached": int(attached[0][0]) if len(attached) == 1 else None,
            "event_kinds": dict(Counter(row["kind"] for row in rows)),
            "task_instances": sorted({(row["tgid"], row["tid"], row["task_start_boottime_ns"]) for row in rows}),
            "event_count": len(rows), "metrics": {name: distribution(values) for name, values in sorted(stages.items())},
            "application_overlaps": correlation,
            "limits": "overlap is not exclusive attribution; blocked state is not proof of lock/I/O ownership; no business sequence in BPF; initial PID namespace required"}


def validate_controlled(result, app, target, quality, iterations=100):
    """计数闭合不等于有效覆盖；已知夹具须匹配 owner 实例及语义区间。"""
    errors = []
    if not result["population_complete"]:
        errors.append("empty or lossy kernel population")
    if not quality.get("population_complete"):
        errors.append("application capture incomplete/lossy")
    header = result["target_header"]
    if (header.get("target_tgid"), header.get("start_ticks"), header.get("hz"), header.get("target_tid", 0)) != (target["pid"], target["starttime"], target["clk_tck"], 0):
        errors.append("target TGID/starttime/clock tick identity mismatch")
    if result["attached"] != 5:
        errors.append("expected five tracepoint programs not attached")
    for tgid, tid, started in result["task_instances"]:
        task = target["threads"].get(str(tid))
        if tgid != target["pid"] or not task or not (task["starttime"] * 1000000000 // target["clk_tck"] <= started < (task["starttime"] + 1) * 1000000000 // target["clk_tck"]):
            errors.append("kernel TGID/TID/task instance mismatch")
            break
    if any(not result["event_kinds"].get(kind) for kind in (1, 2, 3, 4, 5)):
        errors.append("missing wakeup/switch/syscall event kinds")
    phases = ("work_cpu", "work_sleep", "work_lock", "work_hold")
    expected = set(range(1, iterations + 1))
    for phase in phases:
        samples = [row for row in app if row["phase"] == phase]
        if len(samples) != iterations or {row["sequence"] for row in samples} != expected:
            errors.append("incomplete controlled application phase: " + phase)
        if any(row["pid"] != target["pid"] or str(row["tid"]) not in target["threads"] or row["source"] != "controlled_workload" for row in samples):
            errors.append("application TGID/TID/source mismatch: " + phase)
    overlaps = result["application_overlaps"]
    for phase, kind in (("work_cpu", "on_cpu"), ("work_sleep", "syscall_230"), ("work_lock", "syscall_202"), ("work_hold", "syscall_230")):
        if not any(row["phase"] == phase and row["overlap_ns_by_kind"].get(kind, 0) > 0 for row in overlaps):
            errors.append("missing known-cause interval overlap: " + phase + "/" + kind)
    if not result["metrics"].get("successful_wakeup_to_on_cpu_ms", {}).get("samples"):
        errors.append("no successful wakeup-to-on-CPU intervals")
    locks = [row for row in app if row["phase"] == "work_lock"]
    holds = [row for row in app if row["phase"] == "work_hold"]
    if not any(lock["sequence"] == hold["sequence"] and lock["tid"] != hold["tid"] and
               min(lock["end_ns"], hold["end_ns"]) > max(lock["begin_ns"], hold["begin_ns"]) for lock in locks for hold in holds):
        errors.append("no same-sequence cross-thread lock/holder overlap")
    result["controlled_acceptance"] = {"passed": not errors, "errors": errors, "architecture": "x86_64 syscall ABI: futex=202, clock_nanosleep=230",
                                       "expected_iterations": iterations}
    result["coverage_status"] = "CONTROLLED_WORKLOAD_ACCEPTED" if not errors else "CONTROLLED_WORKLOAD_INCOMPLETE"
    return not errors


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("kernel_csv", type=Path)
    parser.add_argument("loader_log", type=Path)
    parser.add_argument("--application", type=Path)
    parser.add_argument("--target-identity", type=Path)
    parser.add_argument("--require-controlled-workload", action="store_true")
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    app = None
    if args.application:
        app, quality = timing(args.application.read_bytes(), Path(str(args.application) + ".status.json").read_bytes())
    result = analyze_kernel(args.kernel_csv.read_bytes(), args.loader_log.read_bytes(), app)
    if args.application:
        result["application_quality"] = quality
    passed = True
    if args.require_controlled_workload:
        if not args.application or not args.target_identity:
            parser.error("controlled validation requires application and target identity")
        passed = validate_controlled(result, app, json.loads(args.target_identity.read_bytes()), quality)
    with args.output.open("xb") as handle:
        handle.write(encoded(result))
    if not passed:
        raise SystemExit(1)
