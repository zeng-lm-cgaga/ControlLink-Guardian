#!/usr/bin/env python3
"""串行gRPC业务夹具；复用native观察/冻结判据，不实现控制规则。"""

import argparse
import hashlib
import json
import os
from pathlib import Path
import subprocess
import sys
import time

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "someip_evidence"))
import judge
from run_scene import LiveCapture, marker, process_identity, runtime_identities, stop
from run_protocol_scene import evaluate_phases


def records(path):
    return [json.loads(line) for line in path.read_text().splitlines()
            if line.startswith("{\"event\":") and line.endswith("}")]


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--root", required=True)
    parser.add_argument("--report", required=True)
    parser.add_argument("--probe", required=True)
    parser.add_argument("--domain", required=True, type=int)
    parser.add_argument("--arbitration-only", action="store_true")
    parser.add_argument("--teleop-executable", required=True)
    args = parser.parse_args()
    root, report = Path(args.root).resolve(), Path(args.report).resolve()
    report.mkdir(parents=True, exist_ok=False)
    gates = report / "preconditions"
    gates.mkdir()
    capture = report / "capture"
    environment = os.environ.copy()
    environment.update(ROS_DOMAIN_ID=str(args.domain), ROS_LOCALHOST_ONLY="1",
                       RMW_IMPLEMENTATION="rmw_fastrtps_cpp", ROS_LOG_DIR=str(report / "ros-log"),
                       LD_PRELOAD="", ASAN_OPTIONS="detect_leaks=0:halt_on_error=1",
                       UBSAN_OPTIONS="halt_on_error=1:print_stacktrace=1")
    for name in ("VSOMEIP_CONFIGURATION", "VSOMEIP_APPLICATION_NAME", "VSOMEIP_CONFIGURATION_MODULE",
                 "FASTRTPS_DEFAULT_PROFILES_FILE", "FASTDDS_DEFAULT_PROFILES_FILE"):
        environment.pop(name, None)
    client_environment = {key: value for key, value in environment.items()
                          if not key.startswith(("ROS_", "AMENT_", "COLCON_")) and key != "RMW_IMPLEMENTATION"}
    processes, handles, commands = [], [], []
    events = {"started": marker(), "domain": args.domain, "period_ms": 50, "keep_period_ms": 1000}
    result = 2

    def launch(label, command, env=environment):
        handle = (report / (label + ".log")).open("xb")
        handles.append(handle)
        process = subprocess.Popen(command, env=env, stdout=handle, stderr=subprocess.STDOUT,
                                   start_new_session=True)
        processes.append((label, process))
        commands.append({"label": label, "argv": command, "pid": process.pid, "started": marker(),
                         "environment": {key: env.get(key) for key in (
                             "ROS_DOMAIN_ID", "AMENT_PREFIX_PATH", "LD_LIBRARY_PATH", "LD_PRELOAD",
                             "ASAN_OPTIONS", "UBSAN_OPTIONS", "CMAKE_PREFIX_PATH")}})
        return process

    try:
        observer = launch("observer", [sys.executable, str(root / "tools/someip_evidence/observer.py"),
            "--output-dir", str(capture), "--contract", str(root / "config/common/gateway_contract.yaml"),
            "--can-interface", "vcan0", "--max-seconds", "150"])
        deadline = time.monotonic() + 10
        while not (capture / "capture-meta.json").exists():
            if observer.poll() is not None or time.monotonic() >= deadline:
                raise RuntimeError("native observer initialization failed")
            time.sleep(0.02)
        service = launch("launch", ["ros2", "launch", "control_link_bringup", "adas_vcan_demo.launch.py",
            "planning_transport:=grpc", "start_source:=false", "can_interface:=vcan0"])
        time.sleep(2)
        if service.poll() is not None:
            raise RuntimeError("gRPC launch failed before independent client start")
        probe = launch("probe", [str(Path(args.probe).resolve()), "--config",
            str(root / "config/adas/protocol_transports.yaml"), "--precondition-dir", str(gates),
            "--period-ms", "50", "--keep-period-ms", "1000"] +
            (["--arbitration-only"] if args.arbitration_only else []), client_environment)
        events["probe_identity"] = process_identity(probe.pid)
        live = LiveCapture(capture)
        mapping = judge.load_can_map((root / "config/adas/can_signal_map.yaml").read_bytes())
        handled = set()
        teleop = None
        teleop_started_at = None
        deadline = time.monotonic() + 120
        while probe.poll() is None:
            if service.poll() is not None or observer.poll() is not None or time.monotonic() >= deadline:
                raise RuntimeError("gRPC runtime or observer exited, or scene exceeded budget")
            log = report / "probe.log"
            observations = records(log)
            for record in observations:
                phase = record.get("phase")
                if record["event"] == "phase_begin" and phase == "normal" and "probe_live_identity" not in events:
                    events["probe_live_identity"] = process_identity(probe.pid)
                if record["event"] == "fault_ready_request" and phase not in handled:
                    gid = live.active(mapping, record["wall_clock_ns"])
                    if gid == record["gid"]:
                        proof = {"marker": marker(), "gid": gid, "request": record,
                                 "basis": "same-generation planning ACTIVE and CRC-valid nonzero CAN after request"}
                        judge.publish_json(gates / (phase + ".ok"), proof)
                        handled.add(phase)
                if record["event"] == "phase_begin" and phase == "teleop_arbitration" and teleop is None:
                    teleop = launch("teleop", [str(Path(args.teleop_executable).resolve(strict=True)),
                        "--ros-args", "-p", "profile_path:=" + str(root / "config/adas/adas_profile.yaml"),
                        "-p", "config_root:=" + str(root / "config"), "-p", "source_id:=teleop",
                        "-p", "publish_rate_hz:=20.0"])
                    events["teleop_started"] = marker()
                    events["teleop_identity"] = process_identity(teleop.pid)
                    teleop_started_at = time.monotonic()
            if teleop is not None and teleop_started_at is not None and time.monotonic() - teleop_started_at >= 3:
                if teleop.poll() is None:
                    events["teleop_live_identity"] = process_identity(teleop.pid)
                events["teleop_exit"] = stop(teleop)
                events["teleop_released"] = marker()
                teleop_started_at = None
            time.sleep(0.02)
        events["probe_exit"] = probe.wait()
        events["runtime_identities"] = runtime_identities(args.domain)
        if events["probe_exit"] != 0:
            raise RuntimeError("gRPC probe failed")
        end = marker()
        events["observer_stop_requested"] = end
        events["observer_wait_exit"] = stop(observer)
        if events["observer_wait_exit"] != 0:
            raise RuntimeError("observer failed to stop/flush")
        immutable_records = records(report / "probe.log")
        judge.publish_json(report / "protocol-records.json", immutable_records)
        facts = evaluate_phases(report, immutable_records, capture, root, events["observer_wait_exit"])
        if "teleop_released" not in events or events["teleop_exit"] != 0:
            raise RuntimeError("arbitration phase missing cooperative release")
        begin, release = events["teleop_started"], events["teleop_released"]
        window = {"schema_version": 1, "ros_time_domain": "SYSTEM",
                  "start_wall_ns": begin["wall_ns"] + 500000000, "end_wall_ns": release["wall_ns"] - 10000000,
                  "start_monotonic_ns": begin["monotonic_ns"] + 500000000,
                  "end_monotonic_ns": release["monotonic_ns"] - 10000000}
        judge.publish_json(report / "teleop-window.json", window)
        gid = next(record["gid"] for record in immutable_records
                   if record["event"] == "phase_begin" and record.get("phase") == "teleop_arbitration")
        snapshot = report / "teleop-snapshot"
        digest = judge.freeze_capture(capture, snapshot, report / "teleop-window.json",
            root / "config/adas/can_signal_map.yaml", gid, events["observer_wait_exit"])
        _, inputs, _ = judge.load_snapshot(snapshot, digest)
        receipt = judge.parse_json(inputs["receipt.json"])
        gateway, _ = judge.records_in_window(inputs["gateway.jsonl"], "gateway", receipt, window)
        can, _ = judge.records_in_window(inputs["can.jsonl"], "can", receipt, window)
        sources, _ = judge.records_in_window(inputs["source.jsonl"], "source", receipt, window)
        active = [row for row in gateway if row["message"]["state"] == 1 and
                  row["message"]["active_source_id"] == "teleop"]
        correlated = [row for row in active if any(
            source["message"]["source_id"] == "teleop" and
            source["message"]["endpoint_publisher_count"] == 1 and
            source["message"]["endpoint_generation_valid"] and
            source["message"]["last_source_sequence"] == row["message"]["active_source_sequence"] and
            judge.observed_ns(source["message"]) == judge.observed_ns(row["message"])
            for source in sources)]
        first_active = min((judge.observed_ns(row["message"]) for row in correlated), default=window["end_wall_ns"])
        nonzero = [row for row in can if row["frame"]["kernel_wall_ns"] >= first_active and
                   judge.control_frame_fact(row["frame"], mapping)[0] == "active_nonzero"]
        if not correlated or not nonzero:
            raise RuntimeError("teleop did not establish ACTIVE/nonzero CAN")
        recovered = live.active(mapping, release["wall_ns"])
        if recovered != gid:
            raise RuntimeError("planning did not recover with unchanged holder generation after teleop release")
        judge.publish_json(report / "arbitration.json", {"manifest_sha256": digest, "teleop_active": len(correlated),
            "crc_valid_nonzero_can": len(nonzero), "recovered_planning_gid": recovered,
            "scope": "correlated observations, not per-command execution ACK"})
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
                if label in ("observer", "launch") and events[label + "_wait_exit"] != 0:
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
