#!/usr/bin/env python3
"""只读能力检查；不尝试安装或提升权限。"""
import argparse
import hashlib
import json
import os
from pathlib import Path
import platform
import subprocess
import time


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--output", required=True, type=Path)
    args = parser.parse_args()
    facts = {"kernel": platform.uname()._asdict(), "uid": os.getuid(),
             "monotonic_ns": time.monotonic_ns(), "wall_ns": time.time_ns(),
             "status": Path("/proc/self/status").read_text(),
             "unprivileged_bpf_disabled": Path("/proc/sys/kernel/unprivileged_bpf_disabled").read_text().strip(),
             "namespaces": {name: os.readlink("/proc/self/ns/" + name) for name in ("pid", "user", "net", "time")},
             "tracepoints": {}}
    btf = Path("/sys/kernel/btf/vmlinux")
    facts["btf_sha256"] = hashlib.sha256(btf.read_bytes()).hexdigest()
    for name in ("sched/sched_wakeup", "sched/sched_switch", "sched/sched_process_exit",
                 "raw_syscalls/sys_enter", "raw_syscalls/sys_exit"):
        path = Path("/sys/kernel/tracing/events") / name / "format"
        try:
            facts["tracepoints"][name] = {"path": str(path), "format": path.read_text()}
        except OSError as error:
            facts["tracepoints"][name] = {"path": str(path), "unavailable": str(error)}
    facts["bpftool"] = subprocess.run(["bpftool", "version"], capture_output=True, text=True).stdout
    with args.output.open("x") as handle:
        json.dump(facts, handle, sort_keys=True, indent=2)
        handle.write("\n")


if __name__ == "__main__":
    main()
