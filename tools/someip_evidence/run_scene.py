#!/usr/bin/env python3

import argparse
from collections import deque
import hashlib
import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import time

import yaml

import judge


class LiveCapture:
    def __init__(self, directory):
        self.directory = directory
        self.offsets = {kind: 0 for kind in judge.STREAMS}
        self.records = {kind: deque(maxlen=200) for kind in judge.STREAMS}

    def poll(self):
        for kind in judge.STREAMS:
            path = self.directory / (kind + ".jsonl")
            if not path.exists():
                continue
            with path.open("rb") as stream:
                stream.seek(self.offsets[kind])
                while True:
                    position = stream.tell()
                    line = stream.readline()
                    if not line.endswith(b"\n"):
                        self.offsets[kind] = position
                        break
                    self.records[kind].append(json.loads(line))

    def active(self, mapping, after_wall_ns, different_gid=None):
        self.poll()
        sources = [record for record in self.records["source"]
                   if record["message"]["source_id"] == "planning" and
                   record["received_wall_ns"] >= after_wall_ns]
        gateways = [record for record in self.records["gateway"]
                    if record["received_wall_ns"] >= after_wall_ns]
        for record in reversed(sources):
            source = record["message"]
            if (source["endpoint_stable"] and source["endpoint_state"] == 5 and
                    source["endpoint_generation_valid"] and source["endpoint_publisher_count"] == 1 and
                    source["accepted_count"] > 0 and source["command_valid"] and source["lease_valid"]):
                gid = judge.gid_hex(source["endpoint_publisher_gid"])
                if gid == different_gid:
                    continue
                for gateway in reversed(gateways):
                    state = gateway["message"]
                    if (state["state"] == 1 and state["active_source_id"] == "planning" and
                            state["active_source_sequence"] == source["last_source_sequence"] and
                            judge.observed_ns(state) == judge.observed_ns(source)):
                        for frame in self.records["can"]:
                            if (frame["received_wall_ns"] >= after_wall_ns and
                                    frame["frame"]["kernel_wall_ns"] >= judge.observed_ns(state) and
                                    judge.control_frame_fact(frame["frame"], mapping)[0] == "active_nonzero"):
                                return gid
        return None


def marker():
    return {"wall_ns": time.time_ns(), "monotonic_ns": time.monotonic_ns()}


def process_identity(pid):
    directory = Path("/proc") / str(pid)
    executable = directory.joinpath("exe").resolve(strict=True)
    mapped = sorted({line.split()[-1] for line in directory.joinpath("maps").read_text().splitlines()
                     if len(line.split()) >= 6 and line.split()[-1].startswith("/")})
    argv = directory.joinpath("cmdline").read_bytes().decode().strip("\0").split("\0")
    environment = dict(field.decode().split("=", 1) for field in directory.joinpath("environ").read_bytes().split(b"\0")
                       if b"=" in field)
    configuration = {}

    def save_input(path):
        resolved = Path(path).resolve(strict=True)
        data = resolved.read_bytes()
        if len(data) > 128 * 1024:
            raise RuntimeError("configuration exceeds identity capture bound")
        configuration[str(resolved)] = {"sha256": hashlib.sha256(data).hexdigest(),
                                        "bytes": len(data), "utf8": data.decode()}
        return data

    for index, argument in enumerate(argv[:-1]):
        if argument in ("--config", "--network-json", "--vsomeip-config"):
            save_input(argv[index + 1])
        if argument == "--params-file":
            document = yaml.safe_load(save_input(argv[index + 1]))
            for node in document.values():
                parameters = node.get("ros__parameters", {})
                if parameters.get("config_root"):
                    for path in Path(parameters["config_root"]).rglob("*"):
                        if path.is_file() and path.suffix in (".yaml", ".xml"):
                            save_input(path)
                for key in ("profile_path", "network_config_path", "vsomeip_config_path"):
                    if parameters.get(key):
                        save_input(parameters[key])
    for name in ("FASTRTPS_DEFAULT_PROFILES_FILE", "FASTDDS_DEFAULT_PROFILES_FILE", "VSOMEIP_CONFIGURATION"):
        if environment.get(name):
            save_input(environment[name])
    return {"pid": pid, "exe": str(executable),
            "exe_sha256": hashlib.sha256(executable.read_bytes()).hexdigest(),
            "argv": argv, "configuration_inputs": configuration,
            "environment": {name: environment.get(name) for name in (
                "ROS_DOMAIN_ID", "RMW_IMPLEMENTATION", "LD_LIBRARY_PATH", "LD_PRELOAD",
                "ASAN_OPTIONS", "UBSAN_OPTIONS", "FASTRTPS_DEFAULT_PROFILES_FILE", "FASTDDS_DEFAULT_PROFILES_FILE",
                "VSOMEIP_CONFIGURATION", "VSOMEIP_APPLICATION_NAME")},
            "mapped_inputs": {path: hashlib.sha256(Path(path).read_bytes()).hexdigest()
                              for path in mapped if Path(path).is_file()}}


def runtime_identities(domain):
    result = []
    for directory in Path("/proc").iterdir():
        if not directory.name.isdecimal():
            continue
        try:
            environment = directory.joinpath("environ").read_bytes().split(b"\0")
            if ("ROS_DOMAIN_ID=" + str(domain)).encode() not in environment:
                continue
            identity = process_identity(int(directory.name))
            if any(name in identity["exe"] for name in (
                    "bridge_node", "gateway_node", "socketcan_adapter", "vehicle_simulator")):
                result.append(identity)
        except (FileNotFoundError, ProcessLookupError, PermissionError):
            continue
    return result


def frozen_safe_stop(snapshot, start_wall_ns, end_wall_ns, mapping,
                     start_monotonic_ns=None, end_monotonic_ns=None, strict=True):
    manifest, inputs, _ = judge.load_snapshot(snapshot)
    window = judge.parse_json(inputs["window.json"])
    if start_monotonic_ns is None:
        start_monotonic_ns = window["start_monotonic_ns"]
    if end_monotonic_ns is None:
        end_monotonic_ns = window["end_monotonic_ns"]

    window.update({"start_wall_ns": start_wall_ns, "end_wall_ns": end_wall_ns,
                   "start_monotonic_ns": start_monotonic_ns, "end_monotonic_ns": end_monotonic_ns})
    receipt = judge.parse_json(inputs["receipt.json"])
    judge.validate_receipt(receipt, window, manifest["waited_exit"])

    def records(kind):
        selected, _ = judge.records_in_window(inputs[kind + ".jsonl"], kind, receipt, window)
        return selected
    gateway = records("gateway")
    frames = records("can")
    safe = sum(record["message"]["state"] == 3 for record in gateway)
    frame_facts = [judge.control_frame_fact(record["frame"], mapping)[0] for record in frames]
    hold = frame_facts.count("hold")
    if not safe or not hold:
        raise RuntimeError("frozen fault window lacks SAFE_STOP and CRC-valid HOLD")
    if "invalid" in frame_facts:
        raise RuntimeError("invalid control CAN in fail-closed window")
    if strict and (any(record["message"]["state"] == 1 for record in gateway) or
                   any(fact.startswith("active_") for fact in frame_facts)):
        raise RuntimeError("stopped-only window contains ACTIVE business output")
    return {"safe_stop_records": safe, "hold_frames": hold}


def stop(process):
    if process is None:
        return None
    if process.poll() is None:
        os.killpg(process.pid, signal.SIGINT)
    try:
        return process.wait(timeout=15)
    except subprocess.TimeoutExpired:
        os.killpg(process.pid, signal.SIGTERM)
        try:
            process.wait(timeout=5)
        except subprocess.TimeoutExpired:
            os.killpg(process.pid, signal.SIGKILL)
            process.wait(timeout=5)
        raise RuntimeError("process did not stop cooperatively: " + str(process.pid))


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--report", required=True)
    parser.add_argument("--root", required=True)
    parser.add_argument("--client", required=True)
    parser.add_argument("--server-json", required=True)
    parser.add_argument("--client-json", required=True)
    parser.add_argument("--domain", required=True, type=int)
    parser.add_argument("--transport", choices=("ros2", "edge", "someip"), default="someip")
    parser.add_argument("--edge-command-channel")
    parser.add_argument("--edge-feedback-channel")
    parser.add_argument("--probe")
    parser.add_argument("--second-client-json")
    parser.add_argument("--scene", choices=("normal", "pause", "graceful", "sigkill",
                        "client-restart", "arbitration", "busy-slow"), default="normal")
    arguments = parser.parse_args()
    if arguments.transport != "someip" and arguments.scene != "normal":
        parser.error("original-chain smoke only supports normal scene")
    if arguments.transport == "edge" and (not arguments.edge_command_channel or
                                           not arguments.edge_feedback_channel):
        parser.error("Edge smoke requires explicit isolated command and feedback channels")
    root = Path(arguments.root).resolve()
    report = Path(arguments.report).resolve()
    report.mkdir(parents=True, exist_ok=False)
    capture = report / "capture"
    env = os.environ.copy()
    env["ROS_DOMAIN_ID"] = str(arguments.domain)
    env["ROS_LOCALHOST_ONLY"] = "1"
    env["RMW_IMPLEMENTATION"] = "rmw_fastrtps_cpp"
    env["ROS_LOG_DIR"] = str(report / "ros-log")
    for name in ("VSOMEIP_CONFIGURATION", "VSOMEIP_APPLICATION_NAME"):
        env.pop(name, None)
    for name in ("FASTRTPS_DEFAULT_PROFILES_FILE", "FASTDDS_DEFAULT_PROFILES_FILE"):
        env.pop(name, None)
    env["ASAN_OPTIONS"] = "detect_leaks=0:halt_on_error=1"
    env["UBSAN_OPTIONS"] = "halt_on_error=1:print_stacktrace=1"
    client_env = env.copy()
    for name in tuple(client_env):
        if name.startswith(("ROS_", "AMENT_", "COLCON_")) or name == "RMW_IMPLEMENTATION":
            client_env.pop(name, None)
    started = marker()
    events = {"started": started, "scene": arguments.scene, "transport": arguments.transport}
    commands = []
    processes = []
    handles = []

    def launch(command, label, environment=env):
        handle = (report / (label + ".log")).open("xb")
        handles.append(handle)
        process = subprocess.Popen(command, stdout=handle, stderr=subprocess.STDOUT,
                                   env=environment, start_new_session=True)
        processes.append((label, process))
        commands.append({"label": label, "argv": command, "pid": process.pid,
                         "environment": {key: environment.get(key) for key in (
                             "ROS_DOMAIN_ID", "ROS_LOCALHOST_ONLY", "RMW_IMPLEMENTATION",
                             "AMENT_PREFIX_PATH", "LD_LIBRARY_PATH", "VSOMEIP_CONFIGURATION",
                             "FASTRTPS_DEFAULT_PROFILES_FILE", "FASTDDS_DEFAULT_PROFILES_FILE",
                             "ASAN_OPTIONS", "UBSAN_OPTIONS", "LD_PRELOAD")},
                         "started": marker()})
        return process

    observer = service = client = None
    exit_code = 2
    try:
        observer = launch([sys.executable, str(root / "tools/someip_evidence/observer.py"),
            "--output-dir", str(capture), "--contract", str(root / "config/common/gateway_contract.yaml"),
            "--can-interface", "vcan0", "--max-seconds", "180"], "observer")
        deadline = time.monotonic() + 10
        while not (capture / "capture-meta.json").exists():
            if observer.poll() is not None or time.monotonic() >= deadline:
                raise RuntimeError("observer readiness failed")
            time.sleep(0.05)
        service = launch(["ros2", "launch", "control_link_bringup", "adas_vcan_demo.launch.py",
            "planning_transport:=" + arguments.transport,
            "start_source:=" + ("false" if arguments.transport == "someip" else "true"), "can_interface:=vcan0",
            "edge_command_channel:=" + (arguments.edge_command_channel or "control_link_p1_smoke_command"),
            "edge_feedback_channel:=" + (arguments.edge_feedback_channel or "control_link_p1_smoke_feedback"),
            "vsomeip_config_path:=" + str(Path(arguments.server_json).resolve())], "launch")
        time.sleep(2)
        command = [str(Path(arguments.client).resolve()), "--config",
            str(root / "config/adas/protocol_transports.yaml"), "--vsomeip-config",
            str(Path(arguments.client_json).resolve()), "--application", "control_link_someip_client",
            "--run-seconds", "120", "--period-ms", "50", "--keep-period-ms", "1000"]
        if arguments.scene == "pause":
            command += ["--pause-command-after-ms", "4000", "--pause-command-duration-ms", "1200"]
        if arguments.transport == "someip":
            client = launch(command, "client", client_env)
        mapping = judge.load_can_map((root / "config/adas/can_signal_map.yaml").read_bytes())
        live = LiveCapture(capture)
        deadline = time.monotonic() + 25
        gid = None
        while not gid:
            if service.poll() is not None or (client is not None and client.poll() is not None) or observer.poll() is not None:
                raise RuntimeError("runtime exited before planning ACTIVE and nonzero CAN")
            gid = live.active(mapping, started["wall_ns"])
            if time.monotonic() >= deadline:
                raise RuntimeError("planning ACTIVE/nonzero CAN precondition timeout")
            time.sleep(0.05)
        events["precondition_active"] = marker()
        events["gid"] = gid
        events["runtime_before"] = runtime_identities(arguments.domain)
        if client is not None:
            events["client_identity_before"] = process_identity(client.pid)
        baseline_start = marker()
        time.sleep(2)
        baseline_end = marker()
        previous_gid = None
        slow_reader = None
        if arguments.scene == "busy-slow":
            if not arguments.probe or not arguments.second_client_json:
                raise RuntimeError("busy-slow requires explicit installed probe and second peer JSON")
            competitor = launch([str(Path(arguments.probe).resolve()), "--config",
                str(root / "config/adas/protocol_transports.yaml"), "--network-json",
                str(Path(arguments.second_client_json).resolve()), "--app-name", "control_link_someip_second",
                "--mode", "second-peer", "--backend", "raw"], "second-peer", client_env)
            events["second_peer_exit"] = competitor.wait(timeout=10)
            if events["second_peer_exit"] != 0:
                raise RuntimeError("second peer BUSY/no-token probe failed")
            readonly_command = [str(Path(arguments.client).resolve()), "--config",
                str(root / "config/adas/protocol_transports.yaml"), "--vsomeip-config",
                str(Path(arguments.second_client_json).resolve()), "--application", "control_link_someip_second",
                "--run-seconds", "120", "--readonly-only"]
            slow_reader = launch(readonly_command, "slow-reader", client_env)
            deadline = time.monotonic() + 10
            while "gateway_validity=1" not in (report / "slow-reader.log").read_text(errors="replace"):
                if slow_reader.poll() is not None or time.monotonic() >= deadline:
                    raise RuntimeError("readonly subscriber did not receive valid network status")
                time.sleep(0.05)
            events["slow_reader_identity"] = process_identity(slow_reader.pid)
            os.kill(slow_reader.pid, signal.SIGSTOP)
            events["reader_suspended"] = marker()
            time.sleep(12)
        if arguments.scene in ("graceful", "sigkill"):
            bridge = [item for item in events["runtime_before"]
                      if item["exe"].endswith("someip_planning_bridge_node")]
            if len(bridge) != 1:
                raise RuntimeError("expected exactly one bridge resource owner")
            previous_gid = gid
            events["fault_injected"] = marker()
            os.kill(bridge[0]["pid"], signal.SIGINT if arguments.scene == "graceful" else signal.SIGKILL)
            deadline = time.monotonic() + 15
            while Path("/proc", str(bridge[0]["pid"])).exists():
                if time.monotonic() >= deadline:
                    raise RuntimeError("old bridge did not exit")
                time.sleep(0.02)
            replacement = launch(bridge[0]["argv"], "bridge-restart")
            events["restart_spawned"] = marker()
            deadline = time.monotonic() + 45
            gid = None
            while gid is None:
                if replacement.poll() is not None or client.poll() is not None or observer.poll() is not None:
                    raise RuntimeError("runtime exited during bridge recovery")
                gid = live.active(mapping, events["restart_spawned"]["wall_ns"], previous_gid)
                if time.monotonic() >= deadline:
                    raise RuntimeError("new GID ACTIVE/nonzero CAN recovery timed out")
                time.sleep(0.05)
            events["recovered"] = marker()
            events["recovery_observed_seconds"] = (
                events["recovered"]["monotonic_ns"] - events["fault_injected"]["monotonic_ns"]) / 1e9
        elif arguments.scene == "client-restart":
            previous_gid = gid
            events["fault_injected"] = marker()
            events["first_client_exit"] = stop(client)
            time.sleep(0.5)
            client = launch(command, "client-restart", client_env)
            events["restart_spawned"] = marker()
            deadline = time.monotonic() + 45
            gid = None
            while gid is None:
                if client.poll() is not None or observer.poll() is not None:
                    raise RuntimeError("runtime exited during client recovery")
                gid = live.active(mapping, events["restart_spawned"]["wall_ns"], previous_gid)
                if time.monotonic() >= deadline:
                    raise RuntimeError("client restart new generation did not recover")
                time.sleep(0.05)
            events["recovered"] = marker()
        elif arguments.scene == "arbitration":
            common = ["--ros-args", "-p", "profile_path:=" + str(root / "config/adas/adas_profile.yaml"),
                      "-p", "config_root:=" + str(root / "config"), "-p", "publish_rate_hz:=20.0"]
            teleop = launch(["ros2", "run", "control_link_adapters", "mock_control_source_node"] + common +
                            ["-p", "source_id:=teleop"], "teleop")
            events["teleop_started"] = marker()
            deadline = time.monotonic() + 10
            while True:
                live.poll()
                if any(record["received_wall_ns"] >= events["teleop_started"]["wall_ns"] and
                       record["message"]["state"] == 1 and
                       record["message"]["active_source_id"] == "teleop" for record in live.records["gateway"]):
                    break
                if time.monotonic() >= deadline or teleop.poll() is not None:
                    raise RuntimeError("teleop did not take control")
                time.sleep(0.05)
            events["teleop_active"] = marker()
            time.sleep(1)
            events["teleop_exit"] = stop(teleop)
            events["teleop_released"] = marker()
            deadline = time.monotonic() + 10
            while live.active(mapping, events["teleop_released"]["wall_ns"]) != gid:
                if time.monotonic() >= deadline:
                    raise RuntimeError("planning did not regain control after teleop")
                time.sleep(0.05)
            extra = launch(["ros2", "run", "control_link_adapters", "mock_control_source_node"] + common +
                           ["-p", "source_id:=planning"], "extra-planning")
            events["fault_injected"] = marker()
            deadline = time.monotonic() + 10
            while True:
                live.poll()
                if any(record["received_wall_ns"] >= events["fault_injected"]["wall_ns"] and
                       record["message"]["source_id"] == "planning" and
                       record["message"]["endpoint_publisher_count"] > 1 and
                       not record["message"]["command_valid"] for record in live.records["source"]):
                    break
                if time.monotonic() >= deadline or extra.poll() is not None:
                    raise RuntimeError("extra publisher did not produce ambiguity rejection")
                time.sleep(0.05)
            events["ambiguity_observed"] = marker()
            time.sleep(1)
            events["extra_planning_exit"] = stop(extra)
            events["ambiguity_removed"] = marker()
            deadline = time.monotonic() + 15
            while live.active(mapping, events["ambiguity_removed"]["wall_ns"]) != gid:
                if time.monotonic() >= deadline:
                    raise RuntimeError("same generation did not recover from ambiguity")
                time.sleep(0.05)
            events["recovered"] = marker()
        if arguments.scene == "pause":
            pause_seen = False
            deadline = time.monotonic() + 15
            while not pause_seen:
                live.poll()
                text = (report / "client.log").read_text(errors="replace")
                pause_seen = "command_pause=true" in text
                if time.monotonic() >= deadline:
                    raise RuntimeError("client command pause not observed")
                time.sleep(0.05)
            events["pause_observed"] = marker()
            deadline = time.monotonic() + 3
            safe = False
            while not safe:
                live.poll()
                after = events["pause_observed"]["wall_ns"]
                safe = (any(record["message"]["state"] == 3 and record["received_wall_ns"] > after
                            for record in live.records["gateway"]) and
                        any(record["received_wall_ns"] > after and
                            judge.control_frame_fact(record["frame"], mapping)[0] == "hold"
                            for record in live.records["can"]))
                if time.monotonic() >= deadline:
                    raise RuntimeError("command pause did not produce SAFE_STOP/HOLD")
                time.sleep(0.05)
            events["safe_stop_hold"] = marker()
            deadline = time.monotonic() + 10
            recovered = None
            while recovered != gid:
                recovered = live.active(mapping, events["safe_stop_hold"]["wall_ns"])
                if time.monotonic() >= deadline:
                    raise RuntimeError("same-session pause recovery failed")
                time.sleep(0.05)
            events["recovered"] = marker()
        start = marker()
        time.sleep(3)
        end = marker()
        window = {"schema_version": 1, "ros_time_domain": "SYSTEM",
                  "start_wall_ns": start["wall_ns"], "end_wall_ns": end["wall_ns"],
                  "start_monotonic_ns": start["monotonic_ns"], "end_monotonic_ns": end["monotonic_ns"]}
        judge.publish_json(report / "window.json", window)
        observer_exit = stop(observer)
        events["observer_wait_exit"] = observer_exit
        observer = None
        events["runtime_after"] = runtime_identities(arguments.domain)
        if client is not None:
            events["client_identity_after"] = process_identity(client.pid)
        snapshot = report / "snapshot"
        judge.freeze_capture(capture, snapshot, report / "window.json",
            root / "config/adas/can_signal_map.yaml", gid, observer_exit, previous_gid)
        manifest_digest = hashlib.sha256((snapshot / "manifest.json").read_bytes()).hexdigest()
        verdict = judge.evaluate_snapshot(snapshot, manifest_digest)
        judge.publish_json(report / "verdict.json", verdict)
        repeat = judge.evaluate_snapshot(snapshot, manifest_digest)
        judge.publish_json(report / "verdict-replay.json", repeat)
        if arguments.scene == "busy-slow":
            if verdict["can_counts"]["hold"] or verdict["can_counts"]["invalid"]:
                raise RuntimeError("slow readonly subscriber affected steady control window")
            os.kill(slow_reader.pid, signal.SIGCONT)
            events["reader_resumed"] = marker()
        if (report / "verdict.json").read_bytes() != (report / "verdict-replay.json").read_bytes():
            raise RuntimeError("frozen replay differs")
        baseline = {"schema_version": 1, "ros_time_domain": "SYSTEM",
                    "start_wall_ns": baseline_start["wall_ns"], "end_wall_ns": baseline_end["wall_ns"],
                    "start_monotonic_ns": baseline_start["monotonic_ns"],
                    "end_monotonic_ns": baseline_end["monotonic_ns"]}
        judge.publish_json(report / "baseline-window.json", baseline)
        baseline_snapshot = report / "baseline-snapshot"
        judge.freeze_capture(capture, baseline_snapshot, report / "baseline-window.json",
                             root / "config/adas/can_signal_map.yaml", events["gid"], observer_exit)
        baseline_digest = hashlib.sha256((baseline_snapshot / "manifest.json").read_bytes()).hexdigest()
        baseline_verdict = judge.evaluate_snapshot(baseline_snapshot, baseline_digest)
        judge.publish_json(report / "baseline-verdict.json", baseline_verdict)
        if baseline_verdict["verdict"] != "PASS":
            raise RuntimeError("frozen pre-fault business precondition failed")
        if arguments.scene in ("graceful", "sigkill", "client-restart", "arbitration"):
            events["frozen_fault_facts"] = frozen_safe_stop(snapshot,
                events["fault_injected"]["wall_ns"], events["recovered"]["wall_ns"], mapping,
                events["fault_injected"]["monotonic_ns"], events["recovered"]["monotonic_ns"], strict=False)
        exit_code = 0 if verdict["verdict"] == "PASS" else 1
        events["snapshot_manifest_sha256"] = manifest_digest
    except Exception as error:
        events["error"] = str(error)
        print(type(error).__name__ + ": " + str(error), file=sys.stderr)
    finally:
        events["cleanup_started"] = marker()
        if 'slow_reader' in locals() and slow_reader is not None and slow_reader.poll() is None:
            os.kill(slow_reader.pid, signal.SIGCONT)
        for label, process in reversed(processes):
            try:
                status = stop(process)
                events[label + "_exit"] = status
                if label == "observer" and status != 0:
                    exit_code = 2
            except Exception as error:
                events[label + "_cleanup_error"] = str(error)
                exit_code = 2
        events["exit_code"] = exit_code
        for handle in handles:
            handle.flush()
            os.fsync(handle.fileno())
            handle.close()
        judge.publish_json(report / "events.json", events)
        judge.publish_json(report / "commands.json", commands)
    return exit_code


if __name__ == "__main__":
    raise SystemExit(main())
