#!/usr/bin/env python3
"""调度/系统调用区间与同 PID/TID 的应用区间相交；不会给 BPF 事件补造业务序号。"""
import argparse
from collections import defaultdict
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
    return {"schema": 1, "quality": footer, "population_complete": int(exit_codes[-1]) == 0 and
            footer["ring_full"] == footer["map_full"] == 0 and footer["emitted"] == footer["received"],
            "event_count": len(rows), "metrics": {name: distribution(values) for name, values in sorted(stages.items())},
            "application_overlaps": correlation,
            "limits": "overlap is not exclusive attribution; blocked state is not proof of lock/I/O ownership; no business sequence in BPF; initial PID namespace required"}


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("kernel_csv", type=Path)
    parser.add_argument("loader_log", type=Path)
    parser.add_argument("--application", type=Path)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    app = None
    if args.application:
        app, quality = timing(args.application.read_bytes(), Path(str(args.application) + ".status.json").read_bytes())
    result = analyze_kernel(args.kernel_csv.read_bytes(), args.loader_log.read_bytes(), app)
    if args.application:
        result["application_quality"] = quality
    with args.output.open("xb") as handle:
        handle.write(encoded(result))
