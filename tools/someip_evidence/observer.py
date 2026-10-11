#!/usr/bin/env python3
"""有界旁路观察者：三个 ROS Topic + native SocketCAN，不启动产品进程。

SIGINT → 停采并锁存真实结束边界 → 释放 I/O → flush/fsync → receipt。
timeout、容量耗尽或写盘错误不得被清理阶段迟到的 SIGINT 改写为成功。
"""

import argparse
import math
import os
from pathlib import Path
import signal
import socket
import struct
import sys
import time

from judge import STREAMS, json_bytes, publish_json, read_frozen, sha256


class CaptureLifecycle:
    """首个停止原因锁存；cleanup 不扩展捕获窗口、不修复先前异常。"""

    def __init__(self, wall_clock=time.time_ns, monotonic_clock=time.monotonic_ns):
        self.wall_clock = wall_clock
        self.monotonic_clock = monotonic_clock
        self.start_wall = wall_clock()
        self.start_monotonic = monotonic_clock()
        self.requested_signal = 0
        self.requested_wall = None
        self.requested_monotonic = None
        self.exit_code = None
        self.stop_reason = None
        self.end_wall = None
        self.end_monotonic = None

    def request_stop(self, received_signal, unused_frame=None):
        if self.end_wall is not None or self.requested_signal:
            return
        self.requested_signal = received_signal
        self.requested_wall = self.wall_clock()
        self.requested_monotonic = self.monotonic_clock()

    def begin_capture(self):
        if self.end_wall is not None:
            raise RuntimeError("cannot begin a completed capture")
        self.start_wall = self.wall_clock()
        self.start_monotonic = self.monotonic_clock()

    def finish(self, reason, status):
        if self.end_wall is None:
            self.end_wall = self.wall_clock()
            self.end_monotonic = self.monotonic_clock()
            self.stop_reason = reason
            self.exit_code = status

    def check_stop(self, deadline):
        if self.exit_code is not None:
            return True
        # 截止时间优先，避免到期后刚收到信号将 timeout 冒充协作结束。
        if self.monotonic_clock() >= deadline:
            self.finish("timeout", 124)
        elif self.requested_signal:
            self.finish("cooperative_sigint" if self.requested_signal == signal.SIGINT
                        else "unexpected_signal", 0 if self.requested_signal == signal.SIGINT
                        else 128 + self.requested_signal)
        return self.exit_code is not None

    def receipt(self, streams, errors):
        if self.end_wall is None:
            raise RuntimeError("capture end must be locked before cleanup")
        return {"schema_version": 1, "kind": "native_ros_can", "pid": os.getpid(),
                "exit_code": self.exit_code if not errors else 1,
                "requested_signal": self.requested_signal, "raw_child_exit": None,
                "stop_reason": self.stop_reason, "requested_wall_ns": self.requested_wall,
                "requested_monotonic_ns": self.requested_monotonic,
                "capture_start_wall_ns": self.start_wall, "capture_end_wall_ns": self.end_wall,
                "capture_start_monotonic_ns": self.start_monotonic,
                "capture_end_monotonic_ns": self.end_monotonic,
                "flush_complete": not errors, "errors": errors, "streams": streams,
                "ros_time_domain": "SYSTEM"}


def complete_json_value(value):
    """非 finite 原值用显式标签保存，避免 JSON NaN 或丢字段掩盖坏观测。"""
    if isinstance(value, dict):
        return {name: complete_json_value(item) for name, item in value.items()}
    if isinstance(value, (list, tuple)):
        return [complete_json_value(item) for item in value]
    if isinstance(value, float) and not math.isfinite(value):
        return {"nonfinite": repr(value)}
    return value


class CaptureWriter:
    """四个有界 JSONL，容量用尽明确结束；不静默丢样或循环覆盖证据。"""

    def __init__(self, directory, max_records, max_bytes):
        self.directory = Path(directory)
        self.directory.mkdir(parents=True, exist_ok=False)
        self.max_records = max_records
        self.max_bytes = max_bytes
        self.counts = {name: 0 for name in STREAMS}
        self.sizes = {name: 0 for name in STREAMS}
        self.handles = {}
        try:
            for name in STREAMS:
                self.handles[name] = (self.directory / (name + ".jsonl")).open("xb")
        except OSError:
            for handle in self.handles.values():
                handle.close()
            raise

    def write(self, kind, value, received_wall_ns=None, received_monotonic_ns=None):
        record = {"schema_version": 1, "kind": kind, "record_index": self.counts[kind] + 1,
                  "received_wall_ns": time.time_ns() if received_wall_ns is None else received_wall_ns,
                  "received_monotonic_ns": time.monotonic_ns() if received_monotonic_ns is None
                  else received_monotonic_ns,
                  "frame" if kind == "can" else "message": complete_json_value(value)}
        encoded = json_bytes(record)
        if (sum(self.counts.values()) >= self.max_records or
                sum(self.sizes.values()) + len(encoded) > self.max_bytes or
                self.sizes[kind] + len(encoded) > 64 * 1024 * 1024):
            raise RuntimeError("bounded capture capacity exhausted")
        self.handles[kind].write(encoded)
        self.handles[kind].flush()
        self.counts[kind] += 1
        self.sizes[kind] += len(encoded)

    def close(self):
        errors = []
        for name, handle in self.handles.items():
            try:
                handle.flush()
                os.fsync(handle.fileno())
            except OSError as error:
                errors.append(name + " flush: " + str(error))
            finally:
                try:
                    handle.close()
                except OSError as error:
                    errors.append(name + " close: " + str(error))
        facts = {}
        for name in STREAMS:
            try:
                data = read_frozen(self.directory / (name + ".jsonl"))
                facts[name] = {"records": self.counts[name], "bytes": len(data), "sha256": sha256(data)}
                if len(data) != self.sizes[name]:
                    errors.append(name + " size does not match completed writes")
            except (OSError, ValueError) as error:
                errors.append(name + " readback: " + str(error))
        return facts, errors


def open_can(interface):
    handle = socket.socket(socket.AF_CAN, socket.SOCK_RAW, socket.CAN_RAW)
    try:
        # Linux SO_TIMESTAMPNS 保留实际帧到达内核的 SYSTEM 时间，排除旧缓冲帧。
        handle.setsockopt(socket.SOL_SOCKET, getattr(socket, "SO_TIMESTAMPNS", 35), 1)
        handle.setsockopt(socket.SOL_SOCKET, getattr(socket, "SO_RXQ_OVFL", 40), 1)
        handle.setblocking(False)
        handle.bind((interface,))
    except OSError:
        handle.close()
        raise
    return handle


def read_can(handle):
    data, ancillary, flags, address = handle.recvmsg(16, 128)
    received_wall = time.time_ns()
    received_mono = time.monotonic_ns()
    if len(data) != 16 or flags & (socket.MSG_TRUNC | socket.MSG_CTRUNC):
        raise RuntimeError("truncated native CAN observation")
    kernel_wall = None
    for level, kind, value in ancillary:
        if level == socket.SOL_SOCKET and kind == getattr(socket, "SO_TIMESTAMPNS", 35):
            seconds, nanoseconds = struct.unpack("@ll", value[:struct.calcsize("@ll")])
            kernel_wall = seconds * 1_000_000_000 + nanoseconds
        if level == socket.SOL_SOCKET and kind == getattr(socket, "SO_RXQ_OVFL", 40):
            if struct.unpack("=I", value)[0]:
                raise RuntimeError("SocketCAN observation queue dropped frames")
    if kernel_wall is None:
        raise RuntimeError("CAN kernel timestamp missing")
    raw_id, dlc, payload = struct.unpack("=IB3x8s", data)
    frame = {"interface": address[0], "raw_can_id": raw_id,
             "can_id": raw_id & 0x1FFFFFFF, "dlc": dlc,
             "is_extended": bool(raw_id & 0x80000000),
             "is_remote": bool(raw_id & 0x40000000), "is_error": bool(raw_id & 0x20000000),
             "data_hex": payload[:min(dlc, 8)].hex(), "kernel_wall_ns": kernel_wall}
    return frame, received_wall, received_mono


def runtime_qos(contract):
    from rclpy.duration import Duration
    from rclpy.qos import (DurabilityPolicy, HistoryPolicy, LivelinessPolicy,
                          QoSProfile, ReliabilityPolicy)

    name = contract["state_topics"]["gateway_state"]["qos_profile"]
    profile = contract["qos_profiles"][name]
    for topic in ("source_status", "vehicle_state"):
        if contract["state_topics"][topic]["qos_profile"] != name:
            raise ValueError("observer requires shared runtime_state QoS profile")
    qos = QoSProfile(depth=int(profile["depth"]),
                     history={"keep_last": HistoryPolicy.KEEP_LAST}[profile["history"]],
                     reliability={"reliable": ReliabilityPolicy.RELIABLE,
                                  "best_effort": ReliabilityPolicy.BEST_EFFORT}[profile["reliability"]],
                     durability={"volatile": DurabilityPolicy.VOLATILE,
                                 "transient_local": DurabilityPolicy.TRANSIENT_LOCAL}[profile["durability"]],
                     liveliness={"automatic": LivelinessPolicy.AUTOMATIC,
                                 "manual_by_topic": LivelinessPolicy.MANUAL_BY_TOPIC}[profile["liveliness"]])
    for key, attribute in (("deadline_ms", "deadline"), ("lifespan_ms", "lifespan"),
                           ("liveliness_lease_duration_ms", "liveliness_lease_duration")):
        if key in profile:
            setattr(qos, attribute, Duration(nanoseconds=int(profile[key]) * 1_000_000))
    return qos


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output-dir", required=True, help="must not already exist")
    parser.add_argument("--contract", required=True, help="actual gateway_contract.yaml")
    parser.add_argument("--can-interface", required=True)
    parser.add_argument("--max-seconds", type=float, default=180)
    parser.add_argument("--max-records", type=int, default=200000)
    parser.add_argument("--max-bytes", type=int, default=64 * 1024 * 1024)
    arguments = parser.parse_args(argv)
    if (not math.isfinite(arguments.max_seconds) or not 0 < arguments.max_seconds <= 3600 or
            not 0 < arguments.max_records <= 1000000 or
            not 0 < arguments.max_bytes <= 128 * 1024 * 1024):
        parser.error("capture limits must be finite, positive and bounded")
    lifecycle = CaptureLifecycle()
    previous_handlers = {received: signal.signal(received, lifecycle.request_stop)
                         for received in (signal.SIGINT, signal.SIGTERM)}
    deadline = lifecycle.start_monotonic + int(arguments.max_seconds * 1_000_000_000)
    writer = None
    node = None
    can_socket = None
    ros = None
    errors = []
    try:
        writer = CaptureWriter(arguments.output_dir, arguments.max_records, arguments.max_bytes)
        import yaml
        import rclpy
        from rclpy.signals import SignalHandlerOptions
        from rosidl_runtime_py.convert import message_to_ordereddict
        from control_link_interfaces.msg import GatewayState, SourceStatus, VehicleState

        ros = rclpy
        raw_contract = read_frozen(arguments.contract)
        contract = yaml.safe_load(raw_contract)
        with (writer.directory / "gateway_contract.yaml").open("xb") as contract_copy:
            contract_copy.write(raw_contract)
            contract_copy.flush()
            os.fsync(contract_copy.fileno())
        qos = runtime_qos(contract)
        ros.init(args=[], signal_handler_options=SignalHandlerOptions.NO)
        node = ros.create_node("someip_evidence_" + str(os.getpid()))
        if node.get_parameter("use_sim_time").value:
            raise ValueError("observer requires SYSTEM, not simulation time")
        can_socket = open_can(arguments.can_interface)
        subscriptions = []
        for kind, key, message_type in (("gateway", "gateway_state", GatewayState),
                                        ("source", "source_status", SourceStatus),
                                        ("vehicle", "vehicle_state", VehicleState)):
            def callback(message, captured_kind=kind):
                if not lifecycle.check_stop(deadline):
                    writer.write(captured_kind, message_to_ordereddict(message))

            subscriptions.append(node.create_subscription(message_type,
                                 contract["state_topics"][key]["topic"], callback, qos))
        lifecycle.begin_capture()
        publish_json(writer.directory / "capture-meta.json", {
            "schema_version": 1, "pid": os.getpid(), "argv": sys.argv if argv is None else argv,
            "python_executable": sys.executable, "ros_domain_id": os.environ.get("ROS_DOMAIN_ID"),
            "rmw_implementation": os.environ.get("RMW_IMPLEMENTATION"),
            "contract_path": str(Path(arguments.contract).resolve()),
            "contract_sha256": sha256(raw_contract), "can_interface": arguments.can_interface,
            "capture_start_wall_ns": lifecycle.start_wall,
            "capture_start_monotonic_ns": lifecycle.start_monotonic,
            "ros_time_domain": "SYSTEM", "can_time_domain": "kernel SYSTEM",
            "limits": {"records": arguments.max_records, "bytes": arguments.max_bytes,
                       "seconds": arguments.max_seconds}})
        while not lifecycle.check_stop(deadline):
            ros.spin_once(node, timeout_sec=0.01)
            for frame_index in range(64):
                if lifecycle.check_stop(deadline):
                    break
                try:
                    frame, wall, mono = read_can(can_socket)
                except BlockingIOError:
                    break
                writer.write("can", frame, wall, mono)
    except (Exception, KeyboardInterrupt) as error:
        lifecycle.finish("capture_error", 1)
        errors.append(type(error).__name__ + ": " + str(error))
    finally:
        if lifecycle.end_wall is None:
            lifecycle.finish("unexpected_end", 1)
        # 边界已冻结，清理耗时不冒充仍在采集。
        for label, close in (("CAN close", can_socket.close if can_socket else None),
                             ("ROS node destroy", node.destroy_node if node else None),
                             ("ROS shutdown", ros.try_shutdown if ros else None)):
            if close:
                try:
                    close()
                except Exception as error:
                    errors.append(label + ": " + str(error))
        facts = {}
        if writer:
            facts, write_errors = writer.close()
            errors.extend(write_errors)
            try:
                publish_json(writer.directory / "receipt.json", lifecycle.receipt(facts, errors))
            except OSError as error:
                errors.append("receipt publish: " + str(error))
        for received, handler in previous_handlers.items():
            signal.signal(received, handler)
    if errors:
        print("INVALID observer: " + "; ".join(errors), file=sys.stderr, flush=True)
    return lifecycle.exit_code if not errors else 1


if __name__ == "__main__":
    raise SystemExit(main())
