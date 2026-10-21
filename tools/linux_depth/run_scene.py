#!/usr/bin/env python3
"""短时独占真实链；窗口与采集上限预先固定，不实现第二套业务判断。"""
import argparse
import functools
import hashlib
import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import time
import traceback

ROOT = Path(__file__).resolve().parents[2]
sys.path[:0] = [str(ROOT / "tools/adas_ingress_evidence"), str(ROOT / "tools/someip_evidence")]
from run_performance import proc_sample
from performance_stats import target
from observer import open_can, read_can, runtime_qos
from business_trace import TraceGate, publish
from capture_target import snapshot


@functools.lru_cache(maxsize=1024)
def digest(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def sample(pid):
    directory = Path("/proc") / str(pid)
    fact = proc_sample(pid)
    fact["fd_count"] = len(list((directory / "fd").iterdir()))
    for line in (directory / "status").read_text().splitlines():
        if line.startswith(("Threads:", "voluntary_ctxt_switches:", "nonvoluntary_ctxt_switches:")):
            key, value = line.split(":")
            fact[key] = int(value)
    fact["thread_switches"] = {}
    for task in (directory / "task").iterdir():
        try:
            fields = task.joinpath("stat").read_text().rsplit(")", 1)[1].split()
            counters = {"starttime": int(fields[19])}
            for line in task.joinpath("status").read_text().splitlines():
                if line.startswith(("voluntary_ctxt_switches:", "nonvoluntary_ctxt_switches:")):
                    key, value = line.split(":")
                    counters[key] = int(value)
            fact["thread_switches"][task.name] = counters
        except (FileNotFoundError, ProcessLookupError):
            continue
    return fact


def identity(pid):
    directory = Path("/proc") / str(pid)
    maps = (directory / "maps").read_text()
    paths = sorted({line.split()[-1] for line in maps.splitlines()
                    if len(line.split()) >= 6 and line.split()[-1].startswith("/")})
    return {"pid": pid, "sample": sample(pid), "exe": str((directory / "exe").resolve()),
            "cmdline": (directory / "cmdline").read_bytes().decode().split("\0"), "maps": maps,
            "mapped_sha256": {path: digest(path) for path in paths
                              if not path.startswith("/dev/shm/") and Path(path).is_file()}}


def run(args):
    control = getattr(args, 'bpf_control', None)
    if control and (os.geteuid() == 0 or not args.timing):
        raise ValueError('BPF business must run as an ordinary user with identical application timing enabled')
    import yaml
    import rclpy
    from rclpy.signals import SignalHandlerOptions
    from rosidl_runtime_py.convert import message_to_ordereddict
    from control_link_interfaces.msg import ControlCommand, GatewayState, SourceStatus, VehicleState
    from rcl_interfaces.srv import SetParametersAtomically
    from rclpy.parameter import Parameter

    args.output.mkdir(parents=True, exist_ok=False)
    old = ROOT / "reports/valgrind-focused-20261013T142907Z-3199f9"
    config = ROOT / "config"
    contract = yaml.safe_load((config / "common/gateway_contract.yaml").read_bytes())
    mapping = yaml.safe_load((config / "adas/can_signal_map.yaml").read_bytes())
    executables = {"gateway": args.runtime / "build/control_link_gateway/control_link_gateway_node",
                   "can_adapter": args.runtime / "build/control_link_adapters/socketcan_adapter_node",
                   "planning_ingress": args.runtime / "build/control_link_adapters/control_command_ingress_adapter_node",
                   "idle_teleop": args.runtime / "build/control_link_adapters/twist_ingress_adapter_node",
                   "simulator": args.runtime / "build/peers/vcan_vehicle_simulator",
                   "activator": old / "build/control_link_bringup/gateway_lifecycle_activator"}
    events = {"schema": 1, "uid": os.getuid(), "euid": os.geteuid(), "driver_sha256": digest(str(Path(__file__).resolve())), "arguments": vars(args).copy(), "commands": {}, "identities": {}, "errors": [],
              "environment": dict(os.environ), "kernel": list(os.uname()), "vcpus": os.cpu_count(),
              "meminfo": Path("/proc/meminfo").read_text(), "clk_tck": os.sysconf("SC_CLK_TCK"),
              "loadavg_start": Path("/proc/loadavg").read_text(), "vm_resource_samples": [],
              "clock": time.get_clock_info("monotonic").__dict__, "preflight": {},
              "namespaces": {key: os.readlink("/proc/self/ns/" + key) for key in ("net", "user", "time", "pid")},
              "config_sha256": {str(path): digest(str(path)) for path in (
                  config / "common/gateway_contract.yaml", config / "common/source_policy.yaml",
                  config / "common/fastdds_default_profiles.xml", config / "adas/adas_profile.yaml",
                  config / "adas/can_signal_map.yaml")}}
    events["arguments"] = {key: str(value) if isinstance(value, Path) else value for key, value in vars(args).items()}
    if control:
        keys = ('PATH', 'LD_LIBRARY_PATH', 'LD_PRELOAD', 'AMENT_PREFIX_PATH', 'PYTHONPATH', 'RMW_IMPLEMENTATION',
                'RMW_FASTRTPS_USE_QOS_FROM_XML', 'FASTRTPS_DEFAULT_PROFILES_FILE', 'ROS_DOMAIN_ID', 'ROS_LOG_DIR',
                'LANG', 'LC_ALL', 'HOME', 'USER', 'LOGNAME', 'ASAN_OPTIONS', 'UBSAN_OPTIONS')
        events['environment'] = {key: os.environ[key] for key in keys if key in os.environ}
    streams = {kind: [] for kind in ("source", "canonical", "gateway", "status", "vehicle", "can", "resource")}
    children, logs, latest = {}, {}, {}
    node = None
    can_socket = None
    interrupted = False
    trace_gate = TraceGate(control) if control else None
    trace_targets = None

    def stop(number, frame):
        nonlocal interrupted
        interrupted = True

    handlers = {number: signal.signal(number, stop) for number in (signal.SIGINT, signal.SIGTERM)}

    def append(kind, row):
        if sum(map(len, streams.values())) >= 100000:
            raise RuntimeError("bounded capture exhausted")
        streams[kind].append(row)

    def callback(kind, message):
        row = {"monotonic_ns": time.monotonic_ns(), "message": dict(message_to_ordereddict(message))}
        append(kind, row)
        if kind != "status" or message.source_id == "planning":
            latest[kind] = row

    try:
        for role, path in executables.items():
            if not os.access(path, os.X_OK):
                raise RuntimeError("missing executable: " + str(path))
            linked = subprocess.run(["ldd", "-r", str(path)], text=True, capture_output=True)
            if linked.returncode or any(value in linked.stdout + linked.stderr for value in
                                       ("not found", "undefined symbol", "libasan", "libubsan")):
                raise RuntimeError("invalid native closure: " + str(path) + linked.stdout + linked.stderr)
            events["preflight"][role] = {"sha256": digest(str(path)), "ldd_r": linked.stdout}
        events["vcan"] = subprocess.check_output(["ip", "-details", "link", "show", "vcan0"], text=True)
        if "vcan" not in events["vcan"]:
            raise RuntimeError("interface is not vcan")
        loopback = json.loads(subprocess.check_output(["ip", "-json", "link", "show", "lo"], text=True))
        events["loopback"] = loopback
        if "UP" not in loopback[0]["flags"]:
            raise RuntimeError("namespace loopback must be UP before ROS launch")
        can_socket = open_can("vcan0")
        rclpy.init(args=[], signal_handler_options=SignalHandlerOptions.NO)
        node = rclpy.create_node("linux_depth_source_observer")

        def qos(name):
            return runtime_qos({"qos_profiles": contract["qos_profiles"], "state_topics": {
                key: {"qos_profile": name} for key in ("gateway_state", "source_status", "vehicle_state")}})

        publisher = node.create_publisher(ControlCommand, "/planning/control_cmd", qos(contract["input"]["qos_profile"]))
        subscriptions = [node.create_subscription(ControlCommand, contract["output"]["topic"],
            lambda message: callback("canonical", message), qos(contract["output"]["qos_profile"]))]
        for kind, key, message_type in (("gateway", "gateway_state", GatewayState),
                                       ("status", "source_status", SourceStatus), ("vehicle", "vehicle_state", VehicleState)):
            subscriptions.append(node.create_subscription(message_type, contract["state_topics"][key]["topic"],
                lambda message, kind=kind: callback(kind, message), qos(contract["state_topics"][key]["qos_profile"])))
        for role in ("simulator", "can_adapter", "idle_teleop", "planning_ingress", "gateway", "activator"):
            name = {"simulator": "vehicle_simulator", "can_adapter": "vehicle_adapter",
                    "idle_teleop": "teleop_ingress", "planning_ingress": "planning_ingress",
                    "gateway": "gateway", "activator": "gateway_lifecycle_activator"}[role]
            command = [str(executables[role]), "--ros-args", "-r", "__node:=" + name]
            if role not in ("simulator", "activator"):
                command += ["-r", "__ns:=/control_link"]
            if role == "activator":
                command += ["-p", "target_fqn:=/control_link/gateway", "-p", "timeout_ms:=60000"]
            else:
                command += ["-p", "profile_path:=" + str(config / "adas/adas_profile.yaml"), "-p", "config_root:=" + str(config)]
            if role in ("planning_ingress", "idle_teleop"):
                command += ["-p", "source_id:=" + ("planning" if role == "planning_ingress" else "teleop")]
            if args.timing and role in ("gateway", "can_adapter"):
                command += ["-p", "runtime_timing_path:=" + str(args.output / (role + "-timing.csv"))]
            events["commands"][role] = command
            logs[role] = (args.output / (role + ".log")).open("xb")
            children[role] = subprocess.Popen(command, stdout=logs[role], stderr=subprocess.STDOUT, start_new_session=True)
        if trace_gate:
            publish(control / 'owners.json', {role: {'pid': child.pid, 'starttime': sample(child.pid)['starttime']}
                                             for role, child in children.items() if child.poll() is None})
        started = time.monotonic_ns()
        events["start_monotonic_ns"], events["start_wall_ns"] = started, time.time_ns()
        sequence, next_send, next_sample, missed = 0, started, started, 0
        parameter_checks = {}
        window = None
        identity_done = False
        while not interrupted:
            now = time.monotonic_ns()
            for role, child in children.items():
                if role != "activator" and child.poll() is not None:
                    raise RuntimeError(role + " exited: " + str(child.returncode))
            if now >= next_send:
                skip = (now - next_send) // 50000000
                missed += skip
                scheduled, next_send = next_send, next_send + (skip + 1) * 50000000
                paused = args.pause_source and window and window["start_ns"] + 5000000000 <= now < window["start_ns"] + 6000000000
                if paused:
                    events["intentional_pause_slots"] = events.get("intentional_pause_slots", 0) + 1
                else:
                    sequence += 1
                    values, key = target(sequence, mapping)
                    message = ControlCommand()
                    message.source_id, message.source_sequence = "external_planner", sequence
                    message.source_stamp = node.get_clock().now().to_msg()
                    message.mode = ControlCommand.MODE_NORMAL
                    message.linear_velocity_mps, message.angular_velocity_radps = values
                    before = time.monotonic_ns()
                    publisher.publish(message)
                    append("source", {"monotonic_ns": before, "end_ns": time.monotonic_ns(),
                                      "scheduled_ns": scheduled, "quantized_key": key,
                                      "message": dict(message_to_ordereddict(message))})
            rclpy.spin_once(node, timeout_sec=0.001)
            for role, check in parameter_checks.items():
                if check["future"].done() and "verdict" not in check:
                    response = check["future"].result().result
                    check["verdict"] = {"successful": response.successful, "reason": response.reason}
                    if response.successful:
                        raise RuntimeError("read-only timing parameter unexpectedly changed: " + role)
                    events.setdefault("readonly_parameter_checks", {})[role] = check["verdict"]
                if window and now >= window["start_ns"] and not check["future"].done():
                    raise RuntimeError("bounded parameter check timeout: " + role)
            for index in range(64):
                try:
                    frame, wall, mono = read_can(can_socket)
                except BlockingIOError:
                    break
                append("can", {"monotonic_ns": mono, "wall_ns": wall, "frame": frame})
            if now >= next_sample:
                events["vm_resource_samples"].append({"monotonic_ns": time.monotonic_ns(),
                    "wall_ns": time.time_ns(), "loadavg": Path("/proc/loadavg").read_text(),
                    "meminfo": Path("/proc/meminfo").read_text(), "cpu_stat": Path("/proc/stat").read_text()})
                for role, pid in {**{role: child.pid for role, child in children.items()}, "fixture_observer": os.getpid()}.items():
                    try:
                        append("resource", {"role": role, **sample(pid)})
                    except (FileNotFoundError, ProcessLookupError, StopIteration):
                        continue
                next_sample = now + 1000000000
            if not identity_done and now - started >= 2000000000:
                for role, child in children.items():
                    if child.poll() is None:
                        events["identities"][role] = identity(child.pid)
                events["identities"]["fixture_observer"] = identity(os.getpid())
                if trace_gate:
                    trace_targets = {role: snapshot(children[role].pid) for role in ('gateway', 'can_adapter')}
                identity_done = True
                continue
            gateway, status, vehicle = (latest.get(key, {}) for key in ("gateway", "status", "vehicle"))
            healthy = all(now - row.get("monotonic_ns", 0) < 150000000 for row in (gateway, status, vehicle))
            healthy = healthy and gateway["message"]["state"] == 1 and gateway["message"]["active_source_id"] == "planning"
            healthy = healthy and status["message"]["accepted_count"] >= 5 and status["message"]["command_valid"] and status["message"]["lease_valid"]
            healthy = healthy and vehicle["message"]["state"] == 1 and vehicle["message"]["fault_code"] == 0
            trace_ready = not trace_gate or (identity_done and trace_gate.ready(trace_targets, healthy, now))
            if window is None and healthy and identity_done and trace_ready:
                if trace_gate:
                    events['trace_gate'] = {'request': trace_gate.request, 'reply': trace_gate.reply}
                anchor = time.monotonic_ns()
                window = {"ready_ns": anchor, "warmup_s": args.warmup, "duration_s": args.seconds,
                          "start_ns": anchor + args.warmup * 1000000000,
                          "end_ns": anchor + (args.warmup + args.seconds) * 1000000000}
                events["window"] = window
                if args.pause_source:
                    events["planned_source_pause"] = [window["start_ns"] + 5000000000, window["start_ns"] + 6000000000]
                if args.check_readonly:
                    for role, fqn in (("gateway", "/control_link/gateway"), ("can_adapter", "/control_link/vehicle_adapter")):
                        client = node.create_client(SetParametersAtomically, fqn + "/set_parameters_atomically")
                        request = SetParametersAtomically.Request()
                        request.parameters = [Parameter("runtime_timing_path", value=str(args.output / "forbidden.csv")).to_parameter_msg()]
                        future = client.call_async(request)
                        parameter_checks[role] = {"client": client, "future": future}
                if args.cpu_load:
                    cpus = sorted(os.sched_getaffinity(0))
                    events["load_cpu"] = cpus[0]
                    for role in ("gateway", "can_adapter"):
                        for task in (Path("/proc") / str(children[role].pid) / "task").iterdir():
                            os.sched_setaffinity(int(task.name), {cpus[0]})
                    for index in range(2):
                        role = "cpu_competitor_" + str(index)
                        command = ["taskset", "-c", str(cpus[0]), "python3", "-c", "while True: pass"]
                        events["commands"][role] = command
                        children[role] = subprocess.Popen(command, start_new_session=True)
                    for role in ("cpu_competitor_0", "cpu_competitor_1"):
                        events["identities"][role] = identity(children[role].pid)
            if window and now >= window["end_ns"]:
                events["stop_reason"] = "fixed_window_complete"
                break
            if not window and now - started >= 60000000000:
                raise RuntimeError("bounded readiness timeout")
        events["missed_send_slots"] = missed
        events["result"] = 0 if window and events.get("stop_reason") == "fixed_window_complete" else 2
    except Exception as error:
        events["errors"].append(type(error).__name__ + ": " + str(error))
        events["traceback"], events["result"] = traceback.format_exc(), 1
    finally:
        events["cutoff_ns"] = time.monotonic_ns()
        if trace_gate:
            publish(control / 'cutoff.json', {'cutoff_ns': events['cutoff_ns'], 'result': events.get('result')})
        events["exits"] = {}
        for role, child in children.items():
            requested = time.monotonic_ns()
            if child.poll() is None:
                os.killpg(child.pid, signal.SIGINT)
            try:
                code = child.wait(timeout=15)
                events["exits"][role] = {"exit": code, "stop_requested_ns": requested, "wait_done_ns": time.monotonic_ns()}
            except subprocess.TimeoutExpired:
                os.killpg(child.pid, signal.SIGTERM)
                events["exits"][role] = {"exit": child.wait(timeout=10), "timeout": True}
                events["errors"].append(role + " stop timeout")
        if can_socket:
            can_socket.close()
        if node:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        for handle in logs.values():
            handle.flush(); os.fsync(handle.fileno()); handle.close()
        for kind, rows in streams.items():
            with (args.output / (kind + ".jsonl")).open("x") as handle:
                for row in rows:
                    handle.write(json.dumps(row, sort_keys=True) + "\n")
                handle.flush(); os.fsync(handle.fileno())
        events["counts"] = {kind: len(rows) for kind, rows in streams.items()}
        events["flush_done_ns"] = time.monotonic_ns()
        events["loadavg_end"] = Path("/proc/loadavg").read_text()
        with (args.output / "events.json").open("x") as handle:
            json.dump(events, handle, indent=2, sort_keys=True); handle.write("\n")
        for number, handler in handlers.items():
            signal.signal(number, handler)
    print(json.dumps({key: events.get(key) for key in ("result", "stop_reason", "errors", "counts", "exits")}))
    return events["result"]


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--runtime", required=True, type=lambda path: Path(path).resolve())
    parser.add_argument("--output", required=True, type=lambda path: Path(path).resolve())
    parser.add_argument("--seconds", type=int, default=20, choices=range(5, 61))
    parser.add_argument("--warmup", type=int, default=10, choices=range(1, 31))
    parser.add_argument("--timing", action="store_true")
    parser.add_argument("--cpu-load", action="store_true")
    parser.add_argument("--pause-source", action="store_true")
    parser.add_argument("--check-readonly", action="store_true")
    parser.add_argument("--bpf-control", type=lambda path: Path(path).resolve())
    raise SystemExit(run(parser.parse_args()))
