#!/usr/bin/env python3
"""真实协议夹具与生产链串行装配；判据只消费已停止观察者的冻结输入。"""

import argparse
import hashlib
import json
import os
from pathlib import Path
import subprocess
import sys
import time

import judge
from run_scene import marker, process_identity, runtime_identities, stop, frozen_safe_stop, LiveCapture


def phase_pairs(records):
    pending = {}
    result = []
    for record in records:
        if record["event"] == "phase_begin" and "commands" in record:
            if record["phase"] in pending:
                raise RuntimeError("duplicate phase begin hides incomplete phase boundaries")
            pending[record["phase"]] = record
        elif record["event"] == "phase_end" and record["phase"] in pending:
            result.append((pending.pop(record["phase"]), record))
    if pending or not result:
        raise RuntimeError("missing complete protocol phase boundaries")
    return result


def evaluate_phases(report, records, capture, root, observer_exit):
    results = []
    mapping = judge.load_can_map((root / "config/adas/can_signal_map.yaml").read_bytes())
    for begin, end in phase_pairs(records):
        phase = begin["phase"]
        delay = 500_000_000 if begin["commands"] and begin["duration_ms"] >= 1000 else 150_000_000
        window = {"schema_version": 1, "ros_time_domain": "SYSTEM",
                  "start_wall_ns": begin["wall_clock_ns"] + delay,
                  "end_wall_ns": end["wall_clock_ns"] - 10_000_000,
                  "start_monotonic_ns": begin["monotonic_ns"] + delay,
                  "end_monotonic_ns": end["monotonic_ns"] - 10_000_000}
        if window["end_monotonic_ns"] <= window["start_monotonic_ns"]:
            raise RuntimeError("empty protocol phase capture window")
        directory = report / phase
        directory.mkdir()
        judge.publish_json(directory / "window.json", window)
        snapshot = directory / "snapshot"
        judge.freeze_capture(capture, snapshot, directory / "window.json",
                             root / "config/adas/can_signal_map.yaml", begin["gid"], observer_exit)
        digest = hashlib.sha256((snapshot / "manifest.json").read_bytes()).hexdigest()
        verdict = judge.evaluate_snapshot(snapshot, digest)
        replay = judge.evaluate_snapshot(snapshot, digest)
        judge.publish_json(directory / "verdict.json", verdict)
        judge.publish_json(directory / "verdict-replay.json", replay)
        if (directory / "verdict.json").read_bytes() != (directory / "verdict-replay.json").read_bytes():
            raise RuntimeError("immutable phase replay differs")
        if begin["commands"]:
            if verdict["verdict"] != "PASS":
                raise RuntimeError("business phase failed: " + phase + ": " + str(verdict["failures"]))
            fact = {"phase": phase, "business_verdict": "PASS", "manifest_sha256": digest,
                    "accepted_first": verdict["accepted_first"], "accepted_last": verdict["accepted_last"],
                    "gid": begin["gid"], "valid_nonzero_can": verdict["can_counts"]["active_nonzero"]}
        else:
            fact = frozen_safe_stop(snapshot, window["start_wall_ns"], window["end_wall_ns"], mapping)
            _, inputs, _ = judge.load_snapshot(snapshot, digest)
            source_rows, _ = judge.records_in_window(inputs["source.jsonl"], "source",
                judge.parse_json(inputs["receipt.json"]), window)
            sources = [row["message"] for row in source_rows if row["message"]["source_id"] == "planning"]
            if not sources or len({row["accepted_count"] for row in sources}) != 1:
                raise RuntimeError("rejected/stopped phase refreshed Gateway acceptance: " + phase)
            fact.update({"phase": phase, "business_verdict": "FAIL_CLOSED", "manifest_sha256": digest,
                         "accepted_count": sources[0]["accepted_count"],
                         "rejected_count": max(row["rejected_count"] for row in sources),
                         "last_reject_reasons": sorted({row["last_reject_reason"] for row in sources})})
        judge.publish_json(directory / "business-facts.json", fact)
        results.append(fact)
    return results


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--root", required=True)
    parser.add_argument("--report", required=True)
    parser.add_argument("--probe", required=True)
    parser.add_argument("--server-json", required=True)
    parser.add_argument("--client-json", required=True)
    parser.add_argument("--domain", required=True, type=int)
    parser.add_argument("--mode", choices=("session", "malformed", "idle", "server-restart"), required=True)
    arguments = parser.parse_args()
    root = Path(arguments.root).resolve()
    report = Path(arguments.report).resolve()
    report.mkdir(parents=True, exist_ok=False)
    capture = report / "capture"
    env = os.environ.copy()
    env.update({"ROS_DOMAIN_ID": str(arguments.domain), "ROS_LOCALHOST_ONLY": "1",
                "RMW_IMPLEMENTATION": "rmw_fastrtps_cpp", "ROS_LOG_DIR": str(report / "ros-log"),
                "ASAN_OPTIONS": "detect_leaks=0:halt_on_error=1",
                "UBSAN_OPTIONS": "halt_on_error=1:print_stacktrace=1"})
    for name in ("VSOMEIP_CONFIGURATION", "VSOMEIP_APPLICATION_NAME",
                 "FASTRTPS_DEFAULT_PROFILES_FILE", "FASTDDS_DEFAULT_PROFILES_FILE"):
        env.pop(name, None)
    client_env = {key: value for key, value in env.items()
                  if not key.startswith(("ROS_", "AMENT_", "COLCON_")) and key != "RMW_IMPLEMENTATION"}
    processes = []
    handles = []
    commands = []
    events = {"started": marker(), "mode": arguments.mode}
    result = 2

    def launch(label, argv, environment=env):
        handle = (report / (label + ".log")).open("xb")
        handles.append(handle)
        process = subprocess.Popen(argv, env=environment, stdout=handle, stderr=subprocess.STDOUT,
                                   start_new_session=True)
        processes.append((label, process))
        commands.append({"label": label, "argv": argv, "pid": process.pid,
                         "environment": {name: environment.get(name) for name in (
                             "ROS_DOMAIN_ID", "AMENT_PREFIX_PATH", "LD_LIBRARY_PATH", "LD_PRELOAD",
                             "ASAN_OPTIONS", "UBSAN_OPTIONS")}, "started": marker()})
        return process

    try:
        observer = launch("observer", [sys.executable, str(root / "tools/someip_evidence/observer.py"),
            "--output-dir", str(capture), "--contract", str(root / "config/common/gateway_contract.yaml"),
            "--can-interface", "vcan0", "--max-seconds", "180"])
        deadline = time.monotonic() + 10
        while not (capture / "capture-meta.json").exists():
            if observer.poll() is not None or time.monotonic() >= deadline:
                raise RuntimeError("observer did not initialize")
            time.sleep(0.05)
        service = launch("launch", ["ros2", "launch", "control_link_bringup", "adas_vcan_demo.launch.py",
            "planning_transport:=someip", "start_source:=false", "can_interface:=vcan0",
            "vsomeip_config_path:=" + str(Path(arguments.server_json).resolve())])
        time.sleep(2)
        client = launch("probe", [str(Path(arguments.probe).resolve()), "--config",
            str(root / "config/adas/protocol_transports.yaml"), "--network-json",
            str(Path(arguments.client_json).resolve()), "--app-name", "control_link_someip_client",
            "--mode", arguments.mode, "--backend", "raw", "--baseline-ms", "1500",
            "--recover-ms", "1500", "--pause-ms", "400", "--observe-ms", "400", "--require-active"], client_env)
        time.sleep(0.2)
        events["probe_identity"] = process_identity(client.pid)
        events["runtime_identities"] = runtime_identities(arguments.domain)
        restart_injected = False
        live = LiveCapture(capture)
        mapping = judge.load_can_map((root / "config/adas/can_signal_map.yaml").read_bytes())
        deadline = time.monotonic() + 100
        while client.poll() is None:
            if observer.poll() is not None or service.poll() is not None or time.monotonic() >= deadline:
                raise RuntimeError("protocol scene runtime/observer stopped or exceeded bound")
            if arguments.mode == "server-restart" and not restart_injected:
                text = (report / "probe.log").read_text(errors="replace")
                if '"event":"service_restart_requested"' in text:
                    gid = live.active(mapping, events["started"]["wall_ns"])
                    if not gid:
                        raise RuntimeError("service fault lacks precondition ACTIVE/CRC-valid nonzero CAN")
                    bridges = [entry for entry in runtime_identities(arguments.domain)
                               if entry["exe"].endswith("someip_planning_bridge_node")]
                    if len(bridges) != 1:
                        raise RuntimeError("expected one production service/bridge process")
                    events["old_bridge_identity"] = bridges[0]
                    events["fault_injected"] = marker()
                    os.kill(bridges[0]["pid"], 9)
                    gone_deadline = time.monotonic() + 10
                    while Path("/proc", str(bridges[0]["pid"])).exists():
                        if time.monotonic() >= gone_deadline:
                            raise RuntimeError("old service process did not exit")
                        time.sleep(0.02)
                    launch("bridge-restart", bridges[0]["argv"])
                    restart_injected = True
            time.sleep(0.1)
        events["probe_exit"] = client.wait()
        if events["probe_exit"] != 0:
            raise RuntimeError("protocol probe failed")
        if arguments.mode == "server-restart" and not restart_injected:
            raise RuntimeError("service restart was not injected")
        events["observers_stop_requested"] = marker()
        observer_exit = stop(observer)
        events["observer_wait_exit"] = observer_exit
        if observer_exit != 0:
            raise RuntimeError("observer failed to flush")
        records = [json.loads(line) for line in (report / "probe.log").read_text().splitlines()
                   if line.startswith('{"event":')]
        judge.publish_json(report / "protocol-records.json", records)
        facts = evaluate_phases(report, records, capture, root, observer_exit)
        judge.publish_json(report / "business-summary.json", facts)
        result = 0
    except Exception as error:
        events["error"] = str(error)
        print(type(error).__name__ + ": " + str(error), file=sys.stderr)
    finally:
        events["cleanup_started"] = marker()
        for label, process in reversed(processes):
            try:
                events[label + "_wait_exit"] = stop(process)
                if label == "observer" and events[label + "_wait_exit"] != 0:
                    result = 2
            except Exception as error:
                events[label + "_cleanup_error"] = str(error)
                result = 2
        for handle in handles:
            handle.flush()
            os.fsync(handle.fileno())
            handle.close()
        events["exit_code"] = result
        judge.publish_json(report / "commands.json", commands)
        judge.publish_json(report / "events.json", events)
    return result


if __name__ == "__main__":
    raise SystemExit(main())
