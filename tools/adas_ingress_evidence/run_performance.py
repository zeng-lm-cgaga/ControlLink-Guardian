#!/usr/bin/env python3
"""仅测已验收 real ROS planning 链；不实现校验、仲裁或恢复规则。"""

import argparse
import hashlib
import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import time
import traceback

ROOT = next(parent for parent in Path(__file__).resolve().parents
            if (parent / "tools/someip_evidence/judge.py").is_file())
BASE = ROOT / "reports/bf2-real-adas-20261013T115306Z"
sys.path.insert(0, str(ROOT / "tools/someip_evidence"))
from observer import open_can, read_can, runtime_qos
from run_scene import process_identity, stop
from performance_stats import encode_json, target

MANIFEST_SHA = "57e17a90fd85a05a1ee64870a122c1fd68137a453777a1fb7024bbc722d795ae"
ROLES = {
    "control_link_gateway_node": "gateway",
    "control_command_ingress_adapter_node": "planning_ingress",
    "socketcan_adapter_node": "can_adapter",
    "vcan_vehicle_simulator": "simulator",
    "twist_ingress_adapter_node": "idle_teleop_ingress",
}


def digest(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def save(path, value):
    with path.open("xb") as handle:
        handle.write(encode_json(value))
        handle.flush()
        os.fsync(handle.fileno())


def proc_sample(pid):
    """绑定 /proc starttime；RSS 是 resident KiB，不是 VmHWM。"""
    started = time.monotonic_ns()
    directory = Path("/proc") / str(pid)
    stat = directory.joinpath("stat").read_text().rsplit(")", 1)[1].split()
    status = directory.joinpath("status").read_text().splitlines()
    rss = next(int(line.split()[1]) for line in status if line.startswith("VmRSS:"))
    ended = time.monotonic_ns()
    return {"pid": pid, "starttime": int(stat[19]), "cpu_ticks": int(stat[11]) + int(stat[12]),
            "rss_kib": rss, "monotonic_ns": (started + ended) // 2,
            "sample_read_span_ns": ended - started, "wall_ns": time.time_ns()}


def namespace_pids():
    found = {}
    namespace = os.readlink("/proc/self/ns/net")
    for directory in Path("/proc").iterdir():
        if not directory.name.isdecimal():
            continue
        try:
            if os.readlink(directory / "ns/net") != namespace:
                continue
            name = Path(os.readlink(directory / "exe")).name
            if name in ROLES:
                role = ROLES[name]
                if role in found:
                    raise RuntimeError("duplicate role in exclusive namespace: " + role)
                found[role] = int(directory.name)
        except (FileNotFoundError, PermissionError, ProcessLookupError):
            continue
    return found


def preflight(report):
    """一次检查实际安装闭包、ELF/ldd/配置；不按目录名猜测同源。"""
    from ament_index_python.packages import get_package_prefix, get_package_share_directory
    if digest(BASE / "delivery/manifest.json") != MANIFEST_SHA:
        raise RuntimeError("accepted manifest identity changed")
    if subprocess.check_output(["git", "rev-parse", "HEAD"], cwd=ROOT, text=True).strip() != \
            "979e4c3c5cbfdb2a1280d84ee34018691973be22":
        raise RuntimeError("unexpected business HEAD")
    previous = json.loads((BASE / "real-combination-03/events.json").read_bytes())
    expected = {Path(item["exe"]).name: item for item in
                previous["runtime_identities"] + previous["ingress_identities"]}
    adapters = Path(get_package_prefix("control_link_adapters")) / "lib/control_link_adapters"
    bringup = Path(get_package_prefix("control_link_bringup")) / "lib/control_link_bringup"
    gateway = Path(get_package_prefix("control_link_gateway")) / "lib/control_link_gateway"
    executables = {}
    for name in ROLES:
        prefix = gateway if name == "control_link_gateway_node" else \
            bringup if name == "vcan_vehicle_simulator" else adapters
        path = (prefix / name).resolve(strict=True)
        if not os.access(path, os.X_OK) or digest(path) != expected[name]["exe_sha256"]:
            raise RuntimeError("actual executable differs from accepted assembly: " + str(path))
        ldd = subprocess.run(["ldd", "-r", str(path)], capture_output=True, text=True)
        if ldd.returncode or "not found" in ldd.stdout or "undefined symbol" in ldd.stdout:
            raise RuntimeError("native runtime closure invalid: " + str(path))
        executables[name] = {"path": str(path), "sha256": digest(path), "ldd_r": ldd.stdout}
    share = Path(get_package_share_directory("control_link_bringup"))
    activator = bringup / "gateway_lifecycle_activator"
    if not os.access(activator, os.X_OK):
        raise RuntimeError("missing lifecycle activator")
    assembly = json.loads((BASE / "runtime-assembly.json").read_bytes())
    for item in assembly["copies"]:
        if digest(item["destination"]) != item["sha256"]:
            raise RuntimeError("runtime configuration/helper identity changed")
    configurations = {}
    for relative in ("common/gateway_contract.yaml", "common/source_policy.yaml",
                     "common/fastdds_default_profiles.xml", "adas/adas_profile.yaml", "adas/can_signal_map.yaml"):
        path = share / "config" / relative
        if digest(path) != digest(ROOT / "config" / relative):
            raise RuntimeError("source/install configuration mismatch")
        configurations[str(path.resolve())] = digest(path)
    facts = {"baseline_manifest": str(BASE / "delivery/manifest.json"),
             "baseline_manifest_sha256": MANIFEST_SHA, "executables": executables,
             "activator_sha256": digest(activator), "configurations": configurations,
             "launch_sha256": digest(share / "launch/adas_vcan_demo.launch.py"),
             "net_namespace": os.readlink("/proc/self/ns/net"),
             "user_namespace": os.readlink("/proc/self/ns/user"),
             "time_namespace": os.readlink("/proc/self/ns/time"),
             "clock": time.get_clock_info("monotonic").__dict__,
             "vcan": subprocess.check_output(["ip", "-details", "link", "show", "vcan0"], text=True),
             "environment": {key: os.environ.get(key) for key in (
                 "ROS_DOMAIN_ID", "ROS_DISTRO", "RMW_IMPLEMENTATION", "AMENT_PREFIX_PATH", "LD_LIBRARY_PATH",
                 "LD_PRELOAD", "ASAN_OPTIONS", "UBSAN_OPTIONS", "FASTRTPS_DEFAULT_PROFILES_FILE",
                 "RMW_FASTRTPS_USE_QOS_FROM_XML")},
             "vcpus": os.cpu_count(), "memory": Path("/proc/meminfo").read_text(),
             "kernel": dict(zip(("sysname", "nodename", "release", "version", "machine"), os.uname())),
             "loadavg": Path("/proc/loadavg").read_text(),
             "compiler": subprocess.check_output(["c++", "--version"], text=True),
             "adapter_cmake_cache": {line.split("=", 1)[0]: line.split("=", 1)[1] for line in
                 (BASE / "build/control_link_adapters/CMakeCache.txt").read_text().splitlines()
                 if line.startswith(("CMAKE_BUILD_TYPE:", "CMAKE_CXX_FLAGS:", "CMAKE_CXX_FLAGS_RELWITHDEBINFO:"))}}
    save(report / "preflight.json", facts)
    return expected


def run(report):
    import rclpy
    from rclpy.signals import SignalHandlerOptions
    from rosidl_runtime_py.convert import message_to_ordereddict
    from control_link_interfaces.msg import ControlCommand, GatewayState, SourceStatus, VehicleState
    import yaml

    report.mkdir(exist_ok=False)
    streams = {name: [] for name in ("source", "canonical", "can", "gateway", "status", "vehicle", "resource")}
    events = {"argv": sys.argv, "clk_tck": os.sysconf("SC_CLK_TCK"), "errors": []}
    latest, state = {}, {"stop": False}
    runtime = None
    node = None
    can_socket = None
    launch_log = None
    result = 1
    previous_handlers = {number: signal.signal(number, lambda unused_number, unused_frame:
                         state.update(stop=True)) for number in (signal.SIGINT, signal.SIGTERM)}

    def append(kind, item):
        if sum(len(records) for records in streams.values()) >= 120000:
            raise RuntimeError("bounded in-memory capture exhausted")
        streams[kind].append(item)

    def callback(kind, message):
        mono = time.monotonic_ns()
        wall = time.time_ns()
        converted = dict(message_to_ordereddict(message))
        append(kind, {"monotonic_ns": mono, "wall_ns": wall, "message": converted})
        if kind != "status" or converted["source_id"] == "planning":
            latest[kind] = converted

    try:
        expected = preflight(report)
        contract_bytes = (ROOT / "config/common/gateway_contract.yaml").read_bytes()
        map_bytes = (ROOT / "config/adas/can_signal_map.yaml").read_bytes()
        contract = yaml.safe_load(contract_bytes)
        mapping = yaml.safe_load(map_bytes)
        for name, data in (("gateway_contract.yaml", contract_bytes), ("can_signal_map.yaml", map_bytes)):
            with (report / name).open("xb") as handle:
                handle.write(data)
        can_socket = open_can("vcan0")
        rclpy.init(args=[], signal_handler_options=SignalHandlerOptions.NO)
        node = rclpy.create_node("real_planning_performance_fixture")

        def qos(name):
            return runtime_qos({"qos_profiles": contract["qos_profiles"], "state_topics": {
                key: {"qos_profile": name} for key in ("gateway_state", "source_status", "vehicle_state")}})

        publisher = node.create_publisher(ControlCommand, "/planning/control_cmd", qos(contract["input"]["qos_profile"]))
        subscriptions = [node.create_subscription(ControlCommand, contract["output"]["topic"],
            lambda message: callback("canonical", message), qos(contract["output"]["qos_profile"]))]
        for kind, key, message_type in (("gateway", "gateway_state", GatewayState),
                                        ("status", "source_status", SourceStatus),
                                        ("vehicle", "vehicle_state", VehicleState)):
            subscriptions.append(node.create_subscription(message_type, contract["state_topics"][key]["topic"],
                lambda message, captured_kind=kind: callback(captured_kind, message), runtime_qos(contract)))
        command = ["ros2", "launch", "control_link_bringup", "adas_vcan_demo.launch.py",
                   "planning_transport:=ros2", "ingress_mode:=real", "can_interface:=vcan0",
                   "decision_trace_path:=" + str(report / "decision-trace.jsonl")]
        events["launch_argv"] = command
        launch_log = (report / "launch.log").open("xb")
        runtime = subprocess.Popen(command, stdout=launch_log, stderr=subprocess.STDOUT, start_new_session=True)
        sequence = 0
        next_send = time.monotonic_ns()
        next_sample = next_send
        runtime_start = next_send
        plan = None
        identities = None
        capture_end = None
        missed_send_slots = 0
        pids = {}
        while not state["stop"]:
            now = time.monotonic_ns()
            if runtime.poll() is not None:
                raise RuntimeError("product launch exited during capture")
            if now >= next_send:
                lagged_slots = (now - next_send) // 50_000_000
                missed_send_slots += lagged_slots
                scheduled = next_send
                next_send += (lagged_slots + 1) * 50_000_000
                sequence += 1
                values, key = target(sequence, mapping)
                message = ControlCommand()
                message.source_id = "external_planner"
                message.source_sequence = sequence
                message.source_stamp = node.get_clock().now().to_msg()
                message.mode = ControlCommand.MODE_NORMAL
                message.linear_velocity_mps, message.angular_velocity_radps = values
                wall = time.time_ns()
                before = time.monotonic_ns()
                publisher.publish(message)
                after = time.monotonic_ns()
                append("source", {"publish_before_monotonic_ns": before,
                    "publish_return_monotonic_ns": after, "wall_ns": wall,
                    "scheduled_monotonic_ns": scheduled, "quantized_key": key,
                    "message": dict(message_to_ordereddict(message))})
            rclpy.spin_once(node, timeout_sec=0.001)
            for unused_index in range(64):
                try:
                    frame, wall, mono = read_can(can_socket)
                except BlockingIOError:
                    break
                append("can", {"monotonic_ns": mono, "wall_ns": wall, "frame": frame})
            if identities is None and now - runtime_start > 1_000_000_000:
                found = namespace_pids()
                if len(found) == len(ROLES):
                    identities = {}
                    baseline = json.loads((BASE / "delivery/manifest.json").read_bytes())
                    for role, pid in found.items():
                        identity = process_identity(pid)
                        name = Path(identity["exe"]).name
                        if identity["exe_sha256"] != expected[name]["exe_sha256"]:
                            raise RuntimeError("actual loaded product executable mismatch")
                        for path, checksum in identity["mapped_inputs"].items():
                            if path in baseline["runtime_code"] and checksum != baseline["runtime_code"][path]["sha256"]:
                                raise RuntimeError("actual loaded dependency mismatch: " + path)
                        identity["starttime"] = proc_sample(pid)["starttime"]
                        identities[role] = identity
                    pids = {**found, "fixture_observer_sampler": os.getpid(), "launch": runtime.pid}
                    events["runtime_identities"] = identities
                    events["resource_roles"] = pids
                    next_sample = time.monotonic_ns()
            if pids and now >= next_sample:
                for role, pid in pids.items():
                    sample = proc_sample(pid)
                    if role in identities and sample["starttime"] != identities[role]["starttime"]:
                        raise RuntimeError("runtime PID starttime changed")
                    append("resource", {"role": role, **sample})
                next_sample += 1_000_000_000
            gateway = latest.get("gateway", {})
            status = latest.get("status", {})
            vehicle = latest.get("vehicle", {})
            healthy = (gateway.get("state") == 1 and gateway.get("active_source_id") == "planning" and
                status.get("source_id") == "planning" and status.get("command_valid") and status.get("lease_valid") and
                status.get("endpoint_stable") and status.get("endpoint_generation_valid") and
                status.get("endpoint_publisher_count") == 1 and vehicle.get("state") == 1 and vehicle.get("fault_code") == 0)
            if plan is None and identities is not None and healthy:
                anchor_mono = time.monotonic_ns()
                anchor_wall = time.time_ns()
                plan = {"warmup_start_monotonic_ns": anchor_mono, "warmup_start_wall_ns": anchor_wall,
                    "warmup_seconds": 10, "health_at_warmup": latest.copy(), "windows": []}
                for index in range(3):
                    offset = (10 + index * 60) * 1_000_000_000
                    plan["windows"].append({"name": "window-" + str(index + 1),
                        "start_monotonic_ns": anchor_mono + offset,
                        "end_monotonic_ns": anchor_mono + offset + 60_000_000_000,
                        "start_wall_ns": anchor_wall + offset, "end_wall_ns": anchor_wall + offset + 60_000_000_000,
                        "wall_boundary_definition": "paired SYSTEM anchor plus fixed offsets; verified against clock drift"})
                save(report / "windows.json", plan)
                capture_end = plan["windows"][-1]["end_monotonic_ns"] + 1_500_000_000
                print("healthy gate confirmed; fixed 10s warmup and three 60s windows", flush=True)
            if plan is None and now - runtime_start > 60_000_000_000:
                raise RuntimeError("planning health gate not reached within bounded startup")
            if capture_end is not None and now >= capture_end:
                result = 0
                break
        if state["stop"]:
            raise RuntimeError("capture interrupted before fixed windows completed")
        events["missed_send_slots_whole_session"] = missed_send_slots
        events["fixture_identity"] = process_identity(os.getpid())
        events["launch_identity"] = process_identity(runtime.pid)
        events["loadavg_at_end"] = Path("/proc/loadavg").read_text()
    except Exception as failure:
        result = 1
        events["errors"].append(str(failure))
        traceback.print_exc()
    finally:
        events["capture_stop_monotonic_ns"] = time.monotonic_ns()
        events["capture_stop_wall_ns"] = time.time_ns()
        if can_socket is not None:
            can_socket.close()
        if node is not None:
            node.destroy_node()
            rclpy.shutdown()
        if runtime is not None:
            try:
                events["launch_wait_exit"] = stop(runtime)
                if events["launch_wait_exit"] != 0:
                    result = 1
            except Exception as failure:
                events["errors"].append(str(failure))
                result = 1
        if launch_log is not None:
            launch_log.close()
        for kind, records in streams.items():
            with (report / (kind + ".jsonl")).open("xb") as handle:
                for record in records:
                    handle.write(json.dumps(record, ensure_ascii=False, allow_nan=False,
                                            separators=(",", ":")).encode() + b"\n")
                handle.flush()
                os.fsync(handle.fileno())
        events["stream_counts"] = {kind: len(records) for kind, records in streams.items()}
        events["flush_complete"] = True
        events["result"] = result
        save(report / "events.json", events)
        for number, handler in previous_handlers.items():
            signal.signal(number, handler)
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--report", type=Path, required=True)
    arguments = parser.parse_args()
    return run(arguments.report.resolve())


if __name__ == "__main__":
    sys.exit(main())
