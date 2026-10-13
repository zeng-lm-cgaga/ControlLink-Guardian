#!/usr/bin/env python3
"""单轮真实 ADAS ingress 验收；复用 native observer/CRC/GID 判据，不实现业务规则。"""

import argparse
import json
import os
from pathlib import Path
import subprocess
import sys
import time

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "tools/someip_evidence"))
import judge
from observer import runtime_qos
from run_scene import LiveCapture, frozen_safe_stop, marker, process_identity, runtime_identities, stop


def evaluate(report):
    """只消费 stop/wait/flush 后的 SHA-bound snapshot；双时钟分别取半开窗。"""
    phases = json.loads((report / "phases.json").read_bytes())
    facts = {}
    for phase in phases:
        snapshot = report / (phase["name"] + "-frozen")
        manifest, inputs, digest = judge.load_snapshot(snapshot)
        window = judge.parse_json(inputs["window.json"])
        if phase["active_source"]:
            result = judge.evaluate_snapshot(snapshot, digest)
            if result["verdict"] != "PASS":
                raise RuntimeError(phase["name"] + ": " + str(result["failures"]))
            facts[phase["name"]] = result
            if phase["name"] in ("teleop", "fallback"):
                from control_link_interfaces.msg import GatewayState
                reason = GatewayState.REASON_SOURCE_SWITCH if phase["name"] == "teleop" else GatewayState.REASON_SOURCE_FALLBACK
                receipt = judge.parse_json(inputs["receipt.json"])
                gateways, _ = judge.records_in_window(inputs["gateway.jsonl"], "gateway", receipt, window)
                if not any(record["message"]["reason_code"] == reason for record in gateways):
                    raise RuntimeError("missing source switch/fallback reason: " + phase["name"])
                result["observed_transition_reason"] = reason
        else:
            result = frozen_safe_stop(snapshot, window["start_wall_ns"], window["end_wall_ns"],
                judge.load_can_map(inputs["can_signal_map.yaml"]),
                window["start_monotonic_ns"], window["end_monotonic_ns"])
            receipt = judge.parse_json(inputs["receipt.json"])
            selected, _ = judge.records_in_window(inputs["source.jsonl"], "source", receipt, window)
            sources = [record["message"] for record in selected if record["message"]["source_id"] == "planning"]
            if len(sources) < 2 or any(message["lease_valid"] for message in sources):
                raise RuntimeError("stopped/rejected input refreshed planning lease")
            counts = {message["accepted_count"] for message in sources}
            sequences = {message["last_source_sequence"] for message in sources}
            if len(counts) != 1 or len(sequences) != 1:
                raise RuntimeError("stopped/rejected input refreshed accepted count/sequence")
            result.update({"accepted_count": sources[0]["accepted_count"],
                "last_sequence": sources[0]["last_source_sequence"], "lease_always_false": True,
                "manifest_sha256": digest})
            if phase["name"] in ("duplicate", "stale"):
                from control_link_interfaces.msg import SourceStatus
                reason = SourceStatus.REJECT_SEQUENCE_NOT_INCREASING if phase["name"] == "duplicate" else SourceStatus.REJECT_STALE
                if (sources[-1]["rejected_count"] <= sources[0]["rejected_count"] or
                        any(message["last_reject_reason"] != reason for message in sources)):
                    raise RuntimeError("reject reason/counter did not prove " + phase["name"])
                result["reject_reason"] = reason
                result["rejected_first"] = sources[0]["rejected_count"]
                result["rejected_last"] = sources[-1]["rejected_count"]
            facts[phase["name"]] = result
    if not (facts["stopped"]["accepted_count"] == facts["duplicate"]["accepted_count"] ==
            facts["stale"]["accepted_count"]):
        raise RuntimeError("invalid commands refreshed accepted count between windows")
    wire = [json.loads(line) for line in (report / "commands.jsonl").read_bytes().splitlines()]
    transmitted = [item for item in wire if item["kind"] == "sent" and item["mode"] in ("duplicate", "stale")]
    forwarded = [item["message"] for item in wire if item["kind"] == "planning_relay"]
    matched = {}
    for mode in ("duplicate", "stale"):
        matched[mode] = sum({**item["message"], "source_id": "planning"} in forwarded
                            for item in transmitted if item["mode"] == mode)
        if matched[mode] < 5:
            raise RuntimeError("missing original stamp/sequence relay proof: " + mode)
    valid = {item["message"]["source_sequence"]: item["message"] for item in wire
             if item["kind"] == "sent" and item["mode"] == "valid"}
    canonical_proof = 0
    for item in wire:
        message = item["message"]
        if item["kind"] != "canonical" or message["source_id"] != "planning" or message["mode"] != 0:
            continue
        expected = valid.get(message["source_sequence"])
        if expected is None or {**expected, "source_id": "planning"} != message:
            raise RuntimeError("canonical planning did not preserve the original valid command")
        canonical_proof += 1
    if canonical_proof < 5:
        raise RuntimeError("no canonical original stamp/sequence proof")
    return {"scope": "VM_ONLY; isolated user/net namespace; 50ms real planning + Twist teleop",
            "facts": facts, "unchanged_rejected_relay_samples": matched,
            "unchanged_valid_canonical_samples": canonical_proof,
            "commands_sha256": judge.sha256((report / "commands.jsonl").read_bytes())}


def run(report):
    import rclpy
    from rclpy.qos import QoSProfile
    from rosidl_runtime_py.convert import message_to_ordereddict
    from control_link_interfaces.msg import ControlCommand
    from geometry_msgs.msg import Twist
    import yaml

    report.mkdir(parents=True, exist_ok=False)
    capture = report / "capture"
    events, children, handles, phases = {}, [], [], []
    node = None
    result = 1
    journal = (report / "commands.jsonl").open("xb")

    def record(kind, message, mode=""):
        journal.write(judge.json_bytes({"kind": kind, "mode": mode, **marker(),
            "message": message_to_ordereddict(message)}))

    def launch(label, command):
        handle = (report / (label + ".log")).open("xb")
        handles.append(handle)
        process = subprocess.Popen(command, stdout=handle, stderr=subprocess.STDOUT,
                                   start_new_session=True)
        children.append((label, process))
        events[label + "_argv"] = command
        return process

    try:
        observer = launch("observer", [sys.executable, str(ROOT / "tools/someip_evidence/observer.py"),
            "--output-dir", str(capture), "--contract", str(ROOT / "config/common/gateway_contract.yaml"),
            "--can-interface", "vcan0", "--max-seconds", "90"])
        deadline = time.monotonic() + 10
        while not (capture / "capture-meta.json").exists():
            if observer.poll() is not None or time.monotonic() > deadline:
                raise RuntimeError("observer initialization failed")
            time.sleep(0.02)
        rclpy.init(args=[])
        node = rclpy.create_node("real_adas_ingress_fixture")
        contract = yaml.safe_load((ROOT / "config/common/gateway_contract.yaml").read_bytes())
        def command_qos(name):
            return runtime_qos({"qos_profiles": contract["qos_profiles"], "state_topics": {
                topic: {"qos_profile": name} for topic in ("gateway_state", "source_status", "vehicle_state")}})

        qos = command_qos(contract["input"]["qos_profile"])
        planning = node.create_publisher(ControlCommand, "/planning/control_cmd", qos)
        teleop = node.create_publisher(Twist, "/teleop/cmd_vel", QoSProfile(depth=10))
        subscriptions = [node.create_subscription(ControlCommand, "/control_link/input/planning",
            lambda message: record("planning_relay", message), qos)]
        canonical_qos = command_qos(contract["output"]["qos_profile"])
        subscriptions.append(node.create_subscription(ControlCommand, contract["output"]["topic"],
            lambda message: record("canonical", message), canonical_qos))
        service = launch("launch", ["ros2", "launch", "control_link_bringup", "adas_vcan_demo.launch.py",
            "planning_transport:=ros2", "ingress_mode:=real", "can_interface:=vcan0"])
        sequence, stale_sequence, next_send, last_command = 0, 1000000, 0, None
        live = LiveCapture(capture)
        mapping = judge.load_can_map((ROOT / "config/adas/can_signal_map.yaml").read_bytes())

        def pump(duration, mode="valid", with_teleop=False):
            nonlocal sequence, stale_sequence, next_send, last_command
            end = time.monotonic() + duration
            while time.monotonic() < end:
                if observer.poll() is not None or service.poll() is not None:
                    raise RuntimeError("runtime or observer exited during phase")
                now = time.monotonic()
                if now >= next_send:
                    next_send = now + 0.05
                    if mode != "stopped":
                        command = ControlCommand()
                        command.source_id = "external_planner"
                        command.source_stamp = node.get_clock().now().to_msg()
                        command.mode = ControlCommand.MODE_NORMAL
                        command.linear_velocity_mps = 0.25
                        command.angular_velocity_radps = 0.10
                        if mode == "valid":
                            sequence += 1
                            command.source_sequence = sequence
                            last_command = command
                        elif mode == "duplicate":
                            command.source_sequence = sequence
                        else:
                            stale_sequence += 1
                            command.source_sequence = stale_sequence
                            command.source_stamp = last_command.source_stamp
                        record("sent", command, mode)
                        planning.publish(command)
                    if with_teleop:
                        twist = Twist()
                        twist.linear.x, twist.angular.z = 0.40, -0.15
                        teleop.publish(twist)
                rclpy.spin_once(node, timeout_sec=0.002)
                live.poll()

        def active_gid(source, after):
            candidates = [record["message"] for record in live.records["source"]
                if record["received_wall_ns"] >= after and record["message"]["source_id"] == source]
            for message in reversed(candidates):
                if (message["endpoint_state"] != 5 or not message["endpoint_stable"] or
                        message["endpoint_publisher_count"] != 1 or not message["endpoint_generation_valid"] or
                        not message["command_valid"] or not message["lease_valid"]):
                    continue
                for record in reversed(live.records["gateway"]):
                    state = record["message"]
                    if (state["state"] == 1 and state["active_source_id"] == source and
                            state["active_source_sequence"] == message["last_source_sequence"] and
                            judge.observed_ns(state) == judge.observed_ns(message)):
                        if any(frame["frame"]["kernel_wall_ns"] >= judge.observed_ns(state) and
                               judge.control_frame_fact(frame["frame"], mapping)[0] == "active_nonzero"
                               for frame in live.records["can"]):
                            return judge.gid_hex(message["endpoint_publisher_gid"])
            return None

        planning_gid = None
        for name, mode, remote, active in (
                ("planning", "valid", False, "planning"),
                ("teleop", "valid", True, "teleop"),
                ("fallback", "valid", False, "planning"),
                ("stopped", "stopped", False, ""),
                ("duplicate", "duplicate", False, ""),
                ("stale", "stale", False, ""),
                ("recovery", "valid", False, "planning")):
            beginning = marker()
            gid = planning_gid
            if name == "duplicate":
                sequence = next(record["message"]["last_source_sequence"]
                    for record in reversed(live.records["source"])
                    if record["message"]["source_id"] == "planning")
            if active:
                deadline = time.monotonic() + 20
                while not (gid := active_gid(active, beginning["wall_ns"])):
                    if time.monotonic() > deadline:
                        raise RuntimeError("bounded ACTIVE/CAN precondition failed: " + name)
                    pump(0.05, mode, remote)
                if active == "planning":
                    if planning_gid and gid != planning_gid:
                        raise RuntimeError("unexpected planning publisher generation change")
                    planning_gid = gid
            else:
                pump(0.5, mode, remote)
            start = marker()
            pump(1.5, mode, remote)
            end = marker()
            phase = {"name": name, "active_source": active, "gid": gid, "window": {
                "schema_version": 1, "ros_time_domain": "SYSTEM",
                "expected_source_id": active or "planning",
                "start_wall_ns": start["wall_ns"], "end_wall_ns": end["wall_ns"],
                "start_monotonic_ns": start["monotonic_ns"], "end_monotonic_ns": end["monotonic_ns"]}}
            phases.append(phase)
        events["driver_identity"] = process_identity(os.getpid())
        events["observer_identity"] = process_identity(observer.pid)
        events["runtime_identities"] = runtime_identities(os.environ["ROS_DOMAIN_ID"])
        events["net_namespace"] = os.readlink("/proc/self/ns/net")
        events["ingress_identities"] = []
        for directory in Path("/proc").iterdir():
            if not directory.name.isdecimal():
                continue
            try:
                if (os.readlink(directory / "ns/net") == events["net_namespace"] and
                        Path(os.readlink(directory / "exe")).name in (
                            "control_command_ingress_adapter_node", "twist_ingress_adapter_node")):
                    events["ingress_identities"].append(process_identity(int(directory.name)))
            except (FileNotFoundError, ProcessLookupError, PermissionError):
                continue
        if len(events["ingress_identities"]) != 2:
            raise RuntimeError("missing the two actual real ingress process identities")
        events["observer_wait_exit"] = stop(observer)
        if events["observer_wait_exit"] != 0:
            raise RuntimeError("observer did not stop/wait/flush successfully")
        journal.flush()
        os.fsync(journal.fileno())
        journal.close()
        judge.publish_json(report / "phases.json", phases)
        for phase in phases:
            window_path = report / (phase["name"] + "-window.json")
            judge.publish_json(window_path, phase["window"])
            judge.freeze_capture(capture, report / (phase["name"] + "-frozen"), window_path,
                ROOT / "config/adas/can_signal_map.yaml", phase["gid"], events["observer_wait_exit"])
        judge.publish_json(report / "verdict.json", evaluate(report))
        result = 0
    except Exception as failure:
        events["failure"] = str(failure)
        print(str(failure), file=sys.stderr)
    finally:
        if node is not None:
            node.destroy_node()
            rclpy.shutdown()
        if not journal.closed:
            journal.flush()
            os.fsync(journal.fileno())
            journal.close()
        for label, process in reversed(children):
            events[label + "_exit"] = stop(process)
            if events[label + "_exit"] != 0:
                result = 1
        for handle in handles:
            handle.close()
        events["result"] = result
        judge.publish_json(report / "events.json", events)
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("command", choices=("run", "replay"))
    parser.add_argument("--report", required=True)
    parser.add_argument("--output")
    args = parser.parse_args()
    report = Path(args.report).resolve()
    if args.command == "run":
        return run(report)
    result = evaluate(report)
    if args.output:
        judge.publish_json(args.output, result)
    else:
        sys.stdout.buffer.write(judge.json_bytes(result))
    return 0


if __name__ == "__main__":
    sys.exit(main())
