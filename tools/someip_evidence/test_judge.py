#!/usr/bin/env python3
"""仅 synthetic 判据/停止/写盘测试，不建立 ROS/vcan 产品场景。"""

import copy
import errno
import os
from pathlib import Path
import signal
import struct
import subprocess
import sys
import tempfile
import types
import unittest
from unittest import mock

import judge
import observer


ROOT = Path(__file__).resolve().parents[2]
NEW_GID = "01" + "00" * 23
OLD_GID = "02" + "00" * 23
WALL = 100_000_000_000
MONO = 1_000_000_000
GOLDEN_ACTIVE = "2a0083ff010f01de"


def stamp(offset):
    value = WALL + offset
    return {"sec": value // 1_000_000_000, "nanosec": value % 1_000_000_000}


def source(offset, accepted, sequence, gid=NEW_GID):
    return {"observed_at": stamp(offset), "source_id": "planning", "priority": 20,
            "enabled": True, "lease_valid": True, "command_valid": True,
            "last_reject_reason": 0, "last_source_sequence": sequence,
            "command_age_ms": 10.0, "accepted_count": accepted, "rejected_count": 0,
            "endpoint_stable": True, "endpoint_state": 5, "endpoint_publisher_count": 1,
            "endpoint_generation_valid": True, "endpoint_publisher_gid": list(bytes.fromhex(gid))}


def gateway(offset, sequence, state=1):
    return {"observed_at": stamp(offset), "state": state, "reason_code": 0,
            "active_source_id": "planning", "active_source_sequence": sequence,
            "active_command_age_ms": 10.0, "recovery_valid_count": 0,
            "transition_sequence": 3}


def vehicle(offset):
    return {"observed_at": stamp(offset), "state": 1, "fault_code": 0,
            "linear_velocity_mps": 0.42, "angular_velocity_radps": -0.125,
            "rolling_counter": 15}


def frame(offset, payload=GOLDEN_ACTIVE):
    return {"interface": "synthetic_only", "raw_can_id": 0x180, "can_id": 0x180,
            "dlc": 8, "is_extended": False, "is_remote": False, "is_error": False,
            "data_hex": payload, "kernel_wall_ns": WALL + offset}


def record(kind, offset, value):
    return {"schema_version": 1, "kind": kind, "received_wall_ns": WALL + offset + 1_000_000,
            "received_monotonic_ns": MONO + offset + 1_000_000,
            "frame" if kind == "can" else "message": value}


def synthetic_records():
    return {"source": [record("source", offset, source(offset, accepted, sequence))
                       for offset, accepted, sequence in ((100_000_000, 70, 10),
                                                          (150_000_000, 71, 11),
                                                          (200_000_000, 72, 12))],
            "gateway": [record("gateway", offset, gateway(offset, sequence))
                        for offset, sequence in ((100_000_000, 10), (150_000_000, 11),
                                                 (200_000_000, 12))],
            "vehicle": [record("vehicle", 180_000_000, vehicle(180_000_000))],
            "can": [record("can", offset, frame(offset))
                    for offset in (170_000_000, 220_000_000)]}


class EvidenceTests(unittest.TestCase):
    def setUp(self):
        parent = os.environ.get("SOMEIP_EVIDENCE_TEST_ROOT")
        if parent:
            Path(parent).mkdir(parents=True, exist_ok=True)
            self.directory = Path(tempfile.mkdtemp(prefix=self._testMethodName + "-", dir=parent))
        else:
            temporary = tempfile.TemporaryDirectory(prefix="control-link-evidence-test-")
            self.directory = Path(temporary.name)
            self.addCleanup(temporary.cleanup)
        self.serial = 0

    def fixture(self, records=None, receipt_changes=None):
        self.serial += 1
        directory = self.directory / str(self.serial)
        capture = directory / "capture"
        capture.mkdir(parents=True)
        records = copy.deepcopy(records if records is not None else synthetic_records())
        facts = {}
        for kind in judge.STREAMS:
            values = records[kind]
            for index, item in enumerate(values, 1):
                item["record_index"] = index
            encoded = b"".join(judge.json_bytes(item) for item in values)
            (capture / (kind + ".jsonl")).write_bytes(encoded)
            facts[kind] = {"bytes": len(encoded), "records": len(values), "sha256": judge.sha256(encoded)}
        receipt = {"schema_version": 1, "kind": "native_ros_can", "pid": 1,
                   "exit_code": 0, "requested_signal": 2, "raw_child_exit": None,
                   "stop_reason": "cooperative_sigint", "flush_complete": True,
                   "errors": [], "ros_time_domain": "SYSTEM", "streams": facts,
                   "capture_start_wall_ns": WALL - 100_000_000,
                   "capture_start_monotonic_ns": MONO - 100_000_000,
                   "capture_end_wall_ns": WALL + 1_100_000_000,
                   "capture_end_monotonic_ns": MONO + 1_100_000_000,
                   "requested_wall_ns": WALL + 1_050_000_000,
                   "requested_monotonic_ns": MONO + 1_050_000_000}
        receipt.update(receipt_changes or {})
        (capture / "receipt.json").write_bytes(judge.json_bytes(receipt))
        contract = (ROOT / "config/common/gateway_contract.yaml").read_bytes()
        (capture / "gateway_contract.yaml").write_bytes(contract)
        (capture / "capture-meta.json").write_bytes(judge.json_bytes({
            "schema_version": 1, "pid": 1, "ros_time_domain": "SYSTEM",
            "contract_sha256": judge.sha256(contract), "scenario": "synthetic_only"}))
        window = {"schema_version": 1, "ros_time_domain": "SYSTEM",
                  "start_wall_ns": WALL, "end_wall_ns": WALL + 1_000_000_000,
                  "start_monotonic_ns": MONO, "end_monotonic_ns": MONO + 1_000_000_000}
        window_path = directory / "window.json"
        window_path.write_bytes(judge.json_bytes(window))
        return capture, directory / "frozen", window_path, receipt

    def freeze(self, fixture):
        capture, frozen, window_path, receipt = fixture
        digest = judge.freeze_capture(capture, frozen, window_path,
                                      ROOT / "config/adas/can_signal_map.yaml", NEW_GID, 0, OLD_GID)
        return frozen, digest

    def replay_cli(self, frozen, label, extra=()):
        command = [sys.executable, str(ROOT / "tools/someip_evidence/judge.py"), "replay",
                   "--snapshot-dir", str(frozen), *extra]
        result = subprocess.run(command, capture_output=True, check=False)
        (self.directory / (label + ".stdout")).write_bytes(result.stdout)
        (self.directory / (label + ".stderr")).write_bytes(result.stderr)
        (self.directory / (label + ".command.json")).write_bytes(judge.json_bytes(command))
        (self.directory / (label + ".exit")).write_text(str(result.returncode) + "\n")
        return result

    def result(self, records):
        frozen, digest = self.freeze(self.fixture(records))
        return judge.evaluate_snapshot(frozen, digest)

    def rewrite_input(self, frozen, name, value):
        frozen.chmod(0o755)
        path = frozen / name
        path.chmod(0o644)
        data = judge.json_bytes(value)
        path.write_bytes(data)
        manifest_path = frozen / "manifest.json"
        manifest_path.chmod(0o644)
        manifest = judge.parse_json(manifest_path.read_bytes())
        manifest["inputs"][name] = {"sha256": judge.sha256(data), "bytes": len(data)}
        manifest_path.write_bytes(judge.json_bytes(manifest))

    def test_complete_cooperative_freeze_and_identical_replay(self):
        frozen, digest = self.freeze(self.fixture())
        first = judge.evaluate_snapshot(frozen, digest)
        second = judge.evaluate_snapshot(frozen, digest)
        self.assertEqual(judge.json_bytes(first), judge.json_bytes(second))
        self.assertEqual(first["verdict"], "PASS")
        self.assertEqual((first["accepted_first"], first["accepted_last"]), (70, 72))
        self.assertEqual(first["correlated_nonzero_can"], 2)
        command = [sys.executable, str(ROOT / "tools/someip_evidence/judge.py"), "replay",
                   "--snapshot-dir", str(frozen), "--manifest-sha256", digest]
        results = [subprocess.run(command, capture_output=True, check=False) for index in range(2)]
        for index, result in enumerate(results):
            (self.directory / ("replay-" + str(index) + ".stdout")).write_bytes(result.stdout)
            (self.directory / ("replay-" + str(index) + ".stderr")).write_bytes(result.stderr)
            self.assertEqual(result.returncode, 0, result.stderr)
        self.assertEqual(results[0].stdout, results[1].stdout)

    def test_business_failure_replay_is_identical_and_exit_one(self):
        values = synthetic_records()
        for item in values["source"]:
            item["message"].update(accepted_count=70, last_source_sequence=10)
        frozen, digest = self.freeze(self.fixture(values))
        first = self.replay_cli(frozen, "failure-1")
        second = self.replay_cli(frozen, "failure-2")
        self.assertEqual((first.returncode, second.returncode), (1, 1))
        self.assertEqual(first.stdout, second.stdout)
        self.assertEqual(first.stderr, second.stderr)

    def test_invalid_receipt_replay_is_identical_and_exit_two(self):
        frozen, digest = self.freeze(self.fixture())
        receipt = judge.parse_json((frozen / "receipt.json").read_bytes())
        receipt.update(exit_code=124, requested_signal=0)
        self.rewrite_input(frozen, "receipt.json", receipt)
        first = self.replay_cli(frozen, "invalid-1")
        second = self.replay_cli(frozen, "invalid-2")
        self.assertEqual((first.returncode, second.returncode), (2, 2))
        self.assertEqual(first.stdout, second.stdout)
        self.assertEqual(first.stderr, second.stderr)

    def test_crc_matches_production_golden_vector(self):
        payload = bytes.fromhex(GOLDEN_ACTIVE)
        self.assertEqual(judge.crc8(0x180, payload), 0xDE)
        mapping = judge.load_can_map((ROOT / "config/adas/can_signal_map.yaml").read_bytes())
        self.assertEqual(judge.control_frame_fact(frame(0), mapping), ("active_nonzero", 42))

    def test_hold_with_nonzero_counter_and_crc_never_passes(self):
        payload = bytes((0, 0, 0, 0, 0, 15, 1))
        payload += bytes((judge.crc8(0x180, payload),))
        self.assertNotEqual(payload[-1], 0)
        values = synthetic_records()
        for item in values["can"]:
            item["frame"]["data_hex"] = payload.hex()
        result = self.result(values)
        self.assertEqual(result["verdict"], "FAIL")
        self.assertEqual(result["can_counts"]["hold"], 2)
        self.assertEqual(result["correlated_nonzero_can"], 0)

    def test_ready_without_business_is_failure(self):
        values = synthetic_records()
        for item in values["source"]:
            item["message"].update(accepted_count=70, last_source_sequence=10)
        for item in values["gateway"]:
            item["message"]["state"] = 3
        self.assertEqual(self.result(values)["verdict"], "FAIL")

    def test_old_gid_acceptance_and_edge_active_cannot_prove_new_holder(self):
        values = synthetic_records()
        for item in values["source"]:
            item["message"]["endpoint_publisher_gid"] = list(bytes.fromhex(OLD_GID))
        values["source"] += [record("source", 250_000_000, source(250_000_000, 72, 12))]
        result = self.result(values)
        self.assertEqual(result["verdict"], "FAIL")
        self.assertEqual(result["correlated_active_records"], 0)

    def test_gateway_sequence_and_observation_tick_must_correlate(self):
        for wrong in ("sequence", "tick"):
            with self.subTest(wrong=wrong):
                values = synthetic_records()
                for item in values["gateway"]:
                    if wrong == "sequence":
                        item["message"]["active_source_sequence"] += 50
                    else:
                        item["message"]["observed_at"]["nanosec"] += 5
                self.assertEqual(self.result(values)["verdict"], "FAIL")

    def test_old_system_samples_received_inside_window_do_not_count(self):
        values = synthetic_records()
        for kind in ("source", "gateway", "vehicle"):
            for item in values[kind]:
                item["message"]["observed_at"] = stamp(-1)
        result = self.result(values)
        self.assertEqual(result["verdict"], "FAIL")
        self.assertEqual(result["sole_gid_records"], 0)

    def test_old_kernel_can_received_inside_window_does_not_count(self):
        values = synthetic_records()
        for item in values["can"]:
            item["frame"]["kernel_wall_ns"] = WALL - 1
        self.assertEqual(self.result(values)["correlated_nonzero_can"], 0)

    def test_pre_active_nonzero_can_cannot_prove_recovery(self):
        values = synthetic_records()
        values["can"] = [record("can", 120_000_000, frame(120_000_000))]
        self.assertEqual(self.result(values)["verdict"], "FAIL")

    def test_wall_and_monotonic_filter_independently(self):
        for clock in ("wall", "monotonic"):
            with self.subTest(clock=clock):
                values = synthetic_records()
                for item in values["can"]:
                    item["received_" + clock + "_ns"] = (WALL if clock == "wall" else MONO) - 1
                self.assertEqual(self.result(values)["correlated_nonzero_can"], 0)

    def test_crc_reserved_flags_mode_and_dlc_are_not_valid_control(self):
        variants = (("data_hex", GOLDEN_ACTIVE[:-2] + "df"), ("dlc", 7),
                    ("is_extended", True), ("is_remote", True), ("is_error", True),
                    ("raw_can_id", 0x280),
                    ("data_hex", "2a0083ff020f01de"))
        for key, value in variants:
            with self.subTest(key=key, value=value):
                records = synthetic_records()
                for item in records["can"]:
                    item["frame"][key] = value
                result = self.result(records)
                self.assertEqual(result["verdict"], "FAIL")
                self.assertEqual(result["can_counts"]["invalid"], 2)

    def test_boolean_does_not_impersonate_ros_numeric_fields(self):
        for kind, field in (("source", "endpoint_publisher_count"), ("gateway", "state")):
            with self.subTest(kind=kind, field=field):
                values = synthetic_records()
                values[kind][1]["message"][field] = True
                with self.assertRaises(judge.InvalidEvidence):
                    self.result(values)

    def test_ambiguous_or_unstable_endpoint_is_failure(self):
        for change in ({"endpoint_stable": False}, {"endpoint_publisher_count": 2},
                       {"endpoint_generation_valid": False}, {"endpoint_state": 1}):
            with self.subTest(change=change):
                values = synthetic_records()
                values["source"][1]["message"].update(change)
                self.assertEqual(self.result(values)["verdict"], "FAIL")

    def test_no_vehicle_feedback_is_not_complete_roundtrip(self):
        values = synthetic_records()
        values["vehicle"] = []
        self.assertEqual(self.result(values)["verdict"], "FAIL")

    def test_unrequested_timeout_or_signal_rejected_online_and_replay(self):
        for status in (124, 130, 143):
            with self.subTest(status=status):
                fixture = self.fixture(receipt_changes={"exit_code": status, "requested_signal": 0})
                with self.assertRaises(judge.InvalidEvidence):
                    self.freeze(fixture)
                frozen, digest = self.freeze(self.fixture())
                receipt = judge.parse_json((frozen / "receipt.json").read_bytes())
                receipt.update(exit_code=status, requested_signal=0)
                self.rewrite_input(frozen, "receipt.json", receipt)
                with self.assertRaises(judge.InvalidEvidence):
                    judge.evaluate_snapshot(frozen)
                result = self.replay_cli(frozen, "unrequested-" + str(status))
                self.assertEqual(result.returncode, 2)
                self.assertNotIn(b'"verdict":"PASS"', result.stdout)

    def test_early_end_in_either_clock_is_invalid(self):
        for clock, end in (("wall", WALL), ("monotonic", MONO)):
            with self.subTest(clock=clock):
                fixture = self.fixture(receipt_changes={"capture_end_" + clock + "_ns": end})
                with self.assertRaises(judge.InvalidEvidence):
                    self.freeze(fixture)
                frozen, digest = self.freeze(self.fixture())
                receipt = judge.parse_json((frozen / "receipt.json").read_bytes())
                receipt["capture_end_" + clock + "_ns"] = end
                self.rewrite_input(frozen, "receipt.json", receipt)
                self.assertEqual(self.replay_cli(frozen, "early-" + clock).returncode, 2)

    def test_request_before_window_end_is_invalid(self):
        fixture = self.fixture(receipt_changes={"requested_wall_ns": WALL + 900_000_000})
        with self.assertRaises(judge.InvalidEvidence):
            self.freeze(fixture)

    def test_missing_corrupt_or_duplicate_receipt_is_invalid(self):
        for replacement in (None, b"{broken", b'{"exit_code":0,"exit_code":0}\n'):
            with self.subTest(replacement=replacement):
                fixture = self.fixture()
                receipt = fixture[0] / "receipt.json"
                if replacement is None:
                    receipt.unlink()
                else:
                    receipt.write_bytes(replacement)
                with self.assertRaises((OSError, ValueError)):
                    self.freeze(fixture)

    def test_failed_flush_and_raw_child_mismatch_are_invalid(self):
        for changes in ({"flush_complete": False}, {"errors": ["ENOSPC"]},
                        {"raw_child_exit": -2}, {"stop_reason": "timeout"}, {"exit_code": True}):
            with self.subTest(changes=changes):
                with self.assertRaises(judge.InvalidEvidence):
                    self.freeze(self.fixture(receipt_changes=changes))

    def test_waited_exit_must_match_receipt(self):
        capture, frozen, window, receipt = self.fixture()
        with self.assertRaises(judge.InvalidEvidence):
            judge.freeze_capture(capture, frozen, window,
                                 ROOT / "config/adas/can_signal_map.yaml", NEW_GID, 124)

    def test_changed_stream_or_manifest_sha_is_invalid(self):
        frozen, digest = self.freeze(self.fixture())
        with self.assertRaises(judge.InvalidEvidence):
            judge.evaluate_snapshot(frozen, "00" * 32)
        path = frozen / "can.jsonl"
        path.chmod(0o644)
        path.write_bytes(path.read_bytes() + b"{}\n")
        with self.assertRaises(judge.InvalidEvidence):
            judge.evaluate_snapshot(frozen, digest)

    def test_stream_index_truncation_and_receipt_binding_are_invalid(self):
        values = synthetic_records()
        fixture = self.fixture(values)
        (fixture[0] / "can.jsonl").write_bytes(b"{}")
        with self.assertRaises(judge.InvalidEvidence):
            self.freeze(fixture)
        values["source"][1]["received_monotonic_ns"] = MONO + 1
        with self.assertRaises(judge.InvalidEvidence):
            self.result(values)

    def test_native_can_receipt_missing_stream_cannot_pass(self):
        fixture = self.fixture()
        receipt = fixture[3]
        del receipt["streams"]["vehicle"]
        (fixture[0] / "receipt.json").write_bytes(judge.json_bytes(receipt))
        with self.assertRaises(KeyError):
            self.freeze(fixture)

    def test_frozen_receipt_missing_or_corrupt_is_invalid_cli(self):
        for case in ("missing", "corrupt"):
            with self.subTest(case=case):
                frozen, digest = self.freeze(self.fixture())
                frozen.chmod(0o755)
                receipt = frozen / "receipt.json"
                receipt.chmod(0o644)
                if case == "missing":
                    receipt.unlink()
                else:
                    receipt.write_bytes(b"{broken")
                self.assertEqual(self.replay_cli(frozen, case).returncode, 2)

    def test_metadata_or_captured_contract_mismatch_is_invalid(self):
        frozen, digest = self.freeze(self.fixture())
        metadata = judge.parse_json((frozen / "capture-meta.json").read_bytes())
        metadata["contract_sha256"] = "00" * 32
        self.rewrite_input(frozen, "capture-meta.json", metadata)
        self.assertEqual(self.replay_cli(frozen, "contract-mismatch").returncode, 2)

    def test_write_or_fsync_failure_never_emits_pass(self):
        frozen, digest = self.freeze(self.fixture())
        command = [sys.executable, str(ROOT / "tools/someip_evidence/judge.py"), "replay",
                   "--snapshot-dir", str(frozen), "--output", "/dev/full"]
        result = subprocess.run(command, capture_output=True, check=False)
        self.assertNotEqual(result.returncode, 0)
        self.assertNotIn(b'"verdict":"PASS"', result.stdout)
        (self.directory / "dev-full.stdout").write_bytes(result.stdout)
        (self.directory / "dev-full.stderr").write_bytes(result.stderr)
        with mock.patch("judge.os.fsync", side_effect=OSError(errno.ENOSPC, "injected full disk")):
            with self.assertRaises(OSError):
                judge.publish_json(self.directory / "verdict.json", {"verdict": "PASS"})
        self.assertFalse((self.directory / "verdict.json").exists())
        writer = observer.CaptureWriter(self.directory / "full-writer", 10, 10000)
        writer.handles["source"].close()
        writer.handles["source"] = open("/dev/full", "wb", buffering=0)
        with self.assertRaises(OSError):
            writer.write("source", source(100_000_000, 70, 10))
        facts, errors = writer.close()
        self.assertTrue(errors)

    def test_freeze_fsync_failure_does_not_publish_manifest(self):
        fixture = self.fixture()
        with mock.patch("judge.os.fsync", side_effect=OSError(errno.ENOSPC, "injected full disk")):
            with self.assertRaises(OSError):
                self.freeze(fixture)
        self.assertFalse((fixture[1] / "manifest.json").exists())

    def test_existing_output_not_overwritten(self):
        destination = self.directory / "historical.json"
        destination.write_bytes(b"original")
        with self.assertRaises(FileExistsError):
            judge.publish_json(destination, {"verdict": "PASS"})
        self.assertEqual(destination.read_bytes(), b"original")

    def test_lifecycle_late_signal_cannot_repair_timeout_or_error(self):
        clock = {"wall": 100, "mono": 1000}
        lifecycle = observer.CaptureLifecycle(lambda: clock["wall"], lambda: clock["mono"])
        clock.update(wall=200, mono=2000)
        self.assertTrue(lifecycle.check_stop(1500))
        lifecycle.request_stop(2)
        clock.update(wall=900, mono=9000)
        lifecycle.finish("cooperative_sigint", 0)
        self.assertEqual(lifecycle.exit_code, 124)
        self.assertEqual(lifecycle.end_monotonic, 2000)
        self.assertEqual(lifecycle.requested_signal, 0)
        failed = observer.CaptureLifecycle(lambda: 100, lambda: 1000)
        failed.finish("capture_error", 1)
        failed.request_stop(2)
        self.assertEqual(failed.exit_code, 1)
        self.assertEqual(failed.requested_signal, 0)

    def test_lifecycle_cooperative_end_does_not_expand_during_cleanup(self):
        clock = {"wall": 100, "mono": 1000}
        lifecycle = observer.CaptureLifecycle(lambda: clock["wall"], lambda: clock["mono"])
        clock.update(wall=200, mono=2000)
        lifecycle.request_stop(2)
        self.assertTrue(lifecycle.check_stop(10000))
        clock.update(wall=900, mono=9000)
        lifecycle.finish("unexpected_end", 1)
        self.assertEqual(lifecycle.end_wall, 200)
        self.assertEqual(lifecycle.end_monotonic, 2000)
        self.assertEqual(lifecycle.receipt({}, ["fsync failed"])["exit_code"], 1)

    def test_native_socket_timestamp_and_drop_detection_without_io(self):
        handle = mock.Mock()
        payload = struct.pack("=IB3x8s", 0x180, 8, bytes.fromhex(GOLDEN_ACTIVE))
        timestamp = struct.pack("@ll", 100, 123)
        handle.recvmsg.return_value = (payload, [(1, 35, timestamp)], 0, ("synthetic_only",))
        value, wall, mono = observer.read_can(handle)
        self.assertEqual(value["kernel_wall_ns"], 100_000_000_123)
        self.assertEqual(value["data_hex"], GOLDEN_ACTIVE)
        handle.recvmsg.return_value = (payload, [(1, 35, timestamp), (1, 40, struct.pack("=I", 1))],
                                      0, ("synthetic_only",))
        with self.assertRaises(RuntimeError):
            observer.read_can(handle)
        handle.recvmsg.return_value = (payload, [], 0, ("synthetic_only",))
        with self.assertRaises(RuntimeError):
            observer.read_can(handle)

    def test_capture_capacity_is_explicit_not_silent_drop(self):
        writer = observer.CaptureWriter(self.directory / "bounded", 1, 10000)
        writer.write("source", source(100_000_000, 70, 10))
        with self.assertRaises(RuntimeError):
            writer.write("source", source(150_000_000, 71, 11))
        facts, errors = writer.close()
        self.assertEqual(errors, [])
        self.assertEqual(facts["source"]["records"], 1)

    def test_observer_main_cooperative_mock_path_has_durable_receipt(self):
        ros = types.ModuleType("rclpy")
        ros_signals = types.ModuleType("rclpy.signals")
        ros_signals.SignalHandlerOptions = types.SimpleNamespace(NO=0)
        ros.init = mock.Mock()
        ros.try_shutdown = mock.Mock()
        node = mock.Mock()
        node.get_parameter.return_value.value = False
        ros.create_node = mock.Mock(return_value=node)
        ros.spin_once = mock.Mock(side_effect=lambda *args, **kwargs: signal.raise_signal(signal.SIGINT))
        convert = types.ModuleType("rosidl_runtime_py.convert")
        convert.message_to_ordereddict = dict
        messages = types.ModuleType("control_link_interfaces.msg")
        messages.GatewayState = messages.SourceStatus = messages.VehicleState = object
        modules = {"rclpy": ros, "rclpy.signals": ros_signals,
                   "rosidl_runtime_py": types.ModuleType("rosidl_runtime_py"),
                   "rosidl_runtime_py.convert": convert,
                   "control_link_interfaces": types.ModuleType("control_link_interfaces"),
                   "control_link_interfaces.msg": messages}
        output = self.directory / "mock-main"
        with mock.patch.dict(sys.modules, modules), mock.patch("observer.runtime_qos"), \
                mock.patch("observer.open_can", return_value=mock.Mock()):
            status = observer.main(["--output-dir", str(output), "--can-interface", "synthetic_only",
                                    "--contract", str(ROOT / "config/common/gateway_contract.yaml")])
        self.assertEqual(status, 0)
        receipt = judge.parse_json((output / "receipt.json").read_bytes())
        self.assertEqual(receipt["requested_signal"], 2)
        self.assertEqual(receipt["stop_reason"], "cooperative_sigint")
        self.assertTrue(receipt["flush_complete"])
        self.assertEqual(node.destroy_node.call_count, 1)


if __name__ == "__main__":
    unittest.main(verbosity=2)
