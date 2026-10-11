#!/usr/bin/env python3
"""仅测分段证据判据；synthetic JSONL 不证明真实 SOME/IP/ROS/vcan 业务通过。"""

import copy
import errno
import json
from pathlib import Path
import unittest
from unittest import mock

import judge
import run_protocol_scene as driver
import test_judge as fixtures


MILLISECOND_NS = 1_000_000
BASELINE_NS = 0
STOP_NS = 1_500 * MILLISECOND_NS
RECOVERY_NS = 2_100 * MILLISECOND_NS
END_NS = 3_600 * MILLISECOND_NS


def phase_markers(name, start_ns, duration_ms, commands, gid=fixtures.NEW_GID):
    end_ns = start_ns + duration_ms * MILLISECOND_NS
    return [
        {"event": "phase_begin", "phase": name, "commands": commands,
         "keep_enabled": True, "duration_ms": duration_ms, "gid": gid,
         "wall_clock_ns": fixtures.WALL + start_ns,
         "monotonic_ns": fixtures.MONO + start_ns},
        {"event": "phase_end", "phase": name,
         "wall_clock_ns": fixtures.WALL + end_ns,
         "monotonic_ns": fixtures.MONO + end_ns},
    ]


def active_samples(start_ns, accepted=70, sequence=10):
    values = fixtures.synthetic_records()
    for kind, records in values.items():
        for item in records:
            item["received_wall_ns"] += start_ns + 500 * MILLISECOND_NS
            item["received_monotonic_ns"] += start_ns + 500 * MILLISECOND_NS
            if kind == "can":
                item["frame"]["kernel_wall_ns"] += start_ns + 500 * MILLISECOND_NS
            else:
                old_offset = judge.observed_ns(item["message"]) - fixtures.WALL
                item["message"]["observed_at"] = fixtures.stamp(
                    old_offset + start_ns + 500 * MILLISECOND_NS)
            if kind == "source":
                item["message"]["accepted_count"] += accepted - 70
                item["message"]["last_source_sequence"] += sequence - 10
            elif kind == "gateway":
                item["message"]["active_source_sequence"] += sequence - 10
    return values


def hold_payload():
    payload = bytes((0, 0, 0, 0, 0, 15, 1))
    return (payload + bytes((judge.crc8(0x180, payload),))).hex()


def stopped_samples(start_ns, accepted=72, sequence=12):
    values = {name: [] for name in judge.STREAMS}
    for delta_ms in (200, 250, 300):
        offset = start_ns + delta_ms * MILLISECOND_NS
        source = fixtures.source(offset, accepted, sequence)
        source.update(command_valid=False, lease_valid=False, command_age_ms=200.0,
                      rejected_count=3, last_reject_reason=4)
        gateway = fixtures.gateway(offset, sequence, state=3)
        gateway["active_source_id"] = ""
        vehicle = fixtures.vehicle(offset)
        vehicle.update(state=2, linear_velocity_mps=0.0, angular_velocity_radps=0.0)
        values["source"].append(fixtures.record("source", offset, source))
        values["gateway"].append(fixtures.record("gateway", offset, gateway))
        values["vehicle"].append(fixtures.record("vehicle", offset, vehicle))
        values["can"].append(fixtures.record(
            "can", offset, fixtures.frame(offset, hold_payload())))
    return values


class ProtocolSceneTests(unittest.TestCase):
    """复用 native receipt/stream 结构，直接执行只读 driver 与冻结判据。"""

    def setUp(self):
        self.fixture_owner = fixtures.EvidenceTests()
        self.fixture_owner._testMethodName = self._testMethodName
        self.fixture_owner.setUp()
        self.addCleanup(self.fixture_owner.doCleanups)
        self.directory = self.fixture_owner.directory
        self.evaluation_serial = 0

    def capture(self, records, receipt_changes=None):
        receipt = {
            "capture_end_wall_ns": fixtures.WALL + END_NS + 100 * MILLISECOND_NS,
            "capture_end_monotonic_ns": fixtures.MONO + END_NS + 100 * MILLISECOND_NS,
            "requested_wall_ns": fixtures.WALL + END_NS + 50 * MILLISECOND_NS,
            "requested_monotonic_ns": fixtures.MONO + END_NS + 50 * MILLISECOND_NS,
        }
        receipt.update(receipt_changes or {})
        result = self.fixture_owner.fixture(records, receipt)
        return result[0]

    def evaluate(self, markers, capture, observer_exit=0):
        self.evaluation_serial += 1
        report = self.directory / ("evaluation-" + str(self.evaluation_serial))
        report.mkdir()
        (report / "protocol-records.json").write_bytes(judge.json_bytes(markers))
        result = driver.evaluate_phases(report, markers, capture, fixtures.ROOT, observer_exit)
        (report / "facts.json").write_bytes(judge.json_bytes(result))
        return result, report

    def assert_rejected(self, markers, records):
        capture = self.capture(records)
        with self.assertRaises((RuntimeError, judge.InvalidEvidence)):
            self.evaluate(markers, capture)

    def test_phase_pairs_keep_complete_command_phases_in_order(self):
        baseline = phase_markers("baseline", BASELINE_NS, 1500, True)
        stopped = phase_markers("stopped", STOP_NS, 600, False)
        recovery = phase_markers("recovery", RECOVERY_NS, 1500, True)
        records = [{"event": "receipt", "transport_result": "FORWARDED"}, *baseline,
                   {"event": "phase_begin", "phase": "readonly", "wall_clock_ns": fixtures.WALL},
                   {"event": "phase_end", "phase": "readonly"}, *stopped, *recovery]
        pairs = driver.phase_pairs(records)
        self.assertEqual([begin["phase"] for begin, end in pairs],
                         ["baseline", "stopped", "recovery"])
        self.assertEqual([end["phase"] for begin, end in pairs],
                         ["baseline", "stopped", "recovery"])

    def test_missing_phase_end_is_not_partial_success(self):
        markers = phase_markers("baseline", BASELINE_NS, 1500, True)
        markers += phase_markers("stopped", STOP_NS, 600, False)[:1]
        with self.assertRaisesRegex(RuntimeError, "phase boundaries"):
            driver.phase_pairs(markers)

    def test_duplicate_phase_begin_cannot_hide_a_missing_end(self):
        markers = phase_markers("baseline", BASELINE_NS, 1500, True)
        markers.insert(1, copy.deepcopy(markers[0]))
        with self.assertRaises(RuntimeError):
            driver.phase_pairs(markers)

    def test_only_receipts_cannot_prove_any_phase(self):
        records = [{"event": "receipt", "transport_result": "FORWARDED"}]
        with self.assertRaises(RuntimeError):
            driver.phase_pairs(records)

    def test_normal_stop_recover_segments_use_real_facts(self):
        records = {name: active_samples(BASELINE_NS)[name] + stopped_samples(STOP_NS)[name] +
                   active_samples(RECOVERY_NS, accepted=73, sequence=13)[name]
                   for name in judge.STREAMS}
        markers = (phase_markers("baseline", BASELINE_NS, 1500, True) +
                   phase_markers("stopped", STOP_NS, 600, False) +
                   phase_markers("recovery", RECOVERY_NS, 1500, True))
        facts, report = self.evaluate(markers, self.capture(records))
        self.assertEqual([fact["business_verdict"] for fact in facts],
                         ["PASS", "FAIL_CLOSED", "PASS"])
        self.assertEqual([(facts[0]["accepted_first"], facts[0]["accepted_last"]),
                          (facts[2]["accepted_first"], facts[2]["accepted_last"])],
                         [(70, 72), (73, 75)])
        self.assertEqual(facts[1]["accepted_count"], 72)
        self.assertEqual((facts[1]["safe_stop_records"], facts[1]["hold_frames"]), (3, 3))
        self.assertEqual([fact["valid_nonzero_can"] for fact in (facts[0], facts[2])], [2, 2])
        for phase in ("baseline", "stopped", "recovery"):
            snapshot = report / phase / "snapshot"
            self.assertFalse(snapshot.stat().st_mode & 0o222)
            self.assertTrue(all(not item.stat().st_mode & 0o222 for item in snapshot.iterdir()))

    def test_success_receipts_without_acceptance_are_rejected(self):
        records = active_samples(BASELINE_NS)
        for item in records["source"]:
            item["message"].update(accepted_count=70, last_source_sequence=10)
        markers = [{"event": "receipt", "transport_result": "FORWARDED", "received": True,
                    "sequence": sequence} for sequence in (10, 11, 12)]
        markers += phase_markers("baseline", BASELINE_NS, 1500, True)
        self.assert_rejected(markers, records)

    def test_accepted_growth_without_active_is_rejected(self):
        records = active_samples(BASELINE_NS)
        for item in records["gateway"]:
            item["message"]["state"] = 3
        self.assert_rejected(phase_markers("baseline", BASELINE_NS, 1500, True), records)

    def test_active_from_wrong_gid_is_rejected(self):
        records = active_samples(BASELINE_NS)
        for item in records["source"]:
            item["message"]["endpoint_publisher_gid"] = list(bytes.fromhex(fixtures.OLD_GID))
        self.assert_rejected(phase_markers("baseline", BASELINE_NS, 1500, True), records)

    def test_ambiguous_planning_gid_is_rejected(self):
        records = active_samples(BASELINE_NS)
        for item in records["source"]:
            item["message"]["endpoint_publisher_count"] = 2
        self.assert_rejected(phase_markers("baseline", BASELINE_NS, 1500, True), records)

    def test_active_without_crc_valid_can_is_rejected(self):
        records = active_samples(BASELINE_NS)
        for item in records["can"]:
            item["frame"]["data_hex"] = fixtures.GOLDEN_ACTIVE[:-2] + "00"
        self.assert_rejected(phase_markers("baseline", BASELINE_NS, 1500, True), records)

    def test_hold_nonzero_counter_crc_is_not_nonzero_control(self):
        records = active_samples(BASELINE_NS)
        payload = bytes.fromhex(hold_payload())
        self.assertNotEqual(payload[5], 0)
        self.assertNotEqual(payload[7], 0)
        for item in records["can"]:
            item["frame"]["data_hex"] = payload.hex()
        self.assert_rejected(phase_markers("baseline", BASELINE_NS, 1500, True), records)

    def test_rejected_phase_accepted_growth_is_rejected(self):
        records = stopped_samples(STOP_NS)
        records["source"][-1]["message"]["accepted_count"] += 1
        self.assert_rejected(phase_markers("invalid", STOP_NS, 600, False), records)

    def test_stopped_phase_without_safe_stop_is_rejected(self):
        records = stopped_samples(STOP_NS)
        for item in records["gateway"]:
            item["message"].update(state=1, active_source_id="planning")
        self.assert_rejected(phase_markers("stopped", STOP_NS, 600, False), records)

    def test_stopped_phase_without_hold_is_rejected(self):
        records = stopped_samples(STOP_NS)
        for item in records["can"]:
            item["frame"]["data_hex"] = fixtures.GOLDEN_ACTIVE
        self.assert_rejected(phase_markers("stopped", STOP_NS, 600, False), records)

    def test_stopped_phase_mixed_active_can_is_not_fail_closed(self):
        records = stopped_samples(STOP_NS)
        records["gateway"][-1]["message"].update(state=1, active_source_id="planning")
        records["can"][-1]["frame"]["data_hex"] = fixtures.GOLDEN_ACTIVE
        self.assert_rejected(phase_markers("stopped", STOP_NS, 600, False), records)

    def test_stopped_phase_invalid_crc_is_not_valid_hold_window(self):
        records = stopped_samples(STOP_NS)
        records["can"][-1]["frame"]["data_hex"] = hold_payload()[:-2] + "00"
        self.assert_rejected(phase_markers("stopped", STOP_NS, 600, False), records)

    def test_stale_safe_stop_and_kernel_hold_cannot_prove_stop(self):
        records = stopped_samples(STOP_NS)
        for item in records["gateway"]:
            item["message"]["observed_at"] = fixtures.stamp(100 * MILLISECOND_NS)
        for item in records["can"]:
            item["frame"]["kernel_wall_ns"] = fixtures.WALL + 100 * MILLISECOND_NS
        self.assert_rejected(phase_markers("stopped", STOP_NS, 600, False), records)

    def test_stop_records_outside_monotonic_window_cannot_prove_stop(self):
        records = stopped_samples(STOP_NS)
        for kind in ("gateway", "can"):
            for item in records[kind]:
                item["received_monotonic_ns"] = fixtures.MONO + 100 * MILLISECOND_NS
        self.assert_rejected(phase_markers("stopped", STOP_NS, 600, False), records)

    def test_completed_capture_missing_planning_records_is_rejected(self):
        records = stopped_samples(STOP_NS)
        records["source"] = []
        self.assert_rejected(phase_markers("stopped", STOP_NS, 600, False), records)

    def test_empty_phase_window_is_rejected(self):
        markers = phase_markers("short", BASELINE_NS, 100, True)
        self.assert_rejected(markers, active_samples(BASELINE_NS))

    def test_unrequested_observer_exit_never_passes(self):
        markers = phase_markers("baseline", BASELINE_NS, 1500, True)
        for exit_code in (124, 130, 143):
            with self.subTest(exit_code=exit_code):
                capture = self.capture(active_samples(BASELINE_NS),
                                       {"exit_code": exit_code, "requested_signal": 0})
                with self.assertRaises(judge.InvalidEvidence):
                    self.evaluate(markers, capture, observer_exit=exit_code)

    def test_cooperative_receipt_ending_before_phase_is_rejected(self):
        markers = phase_markers("baseline", BASELINE_NS, 1500, True)
        for field, value in (("capture_end_wall_ns", fixtures.WALL + 100 * MILLISECOND_NS),
                             ("capture_end_monotonic_ns", fixtures.MONO + 100 * MILLISECOND_NS)):
            with self.subTest(field=field):
                capture = self.capture(active_samples(BASELINE_NS), {field: value})
                with self.assertRaises(judge.InvalidEvidence):
                    self.evaluate(markers, capture)

    def test_output_write_failure_never_returns_success(self):
        markers = phase_markers("baseline", BASELINE_NS, 1500, True)
        capture = self.capture(active_samples(BASELINE_NS))
        original = judge.publish_json

        def fail_business_write(path, value):
            if Path(path).name == "business-facts.json":
                raise OSError(errno.ENOSPC, "synthetic no-space output failure")
            return original(path, value)

        with mock.patch.object(driver.judge, "publish_json", side_effect=fail_business_write):
            with self.assertRaises(OSError) as caught:
                self.evaluate(markers, capture)
        self.assertEqual(caught.exception.errno, errno.ENOSPC)

    def test_freeze_flush_failure_never_returns_success(self):
        markers = phase_markers("baseline", BASELINE_NS, 1500, True)
        capture = self.capture(active_samples(BASELINE_NS))
        with mock.patch.object(judge.os, "fsync", side_effect=OSError(errno.EIO, "synthetic flush failure")):
            with self.assertRaises(OSError) as caught:
                self.evaluate(markers, capture)
        self.assertEqual(caught.exception.errno, errno.EIO)

    def test_same_immutable_inputs_replay_byte_identically(self):
        markers = phase_markers("baseline", BASELINE_NS, 1500, True)
        capture = self.capture(active_samples(BASELINE_NS))
        initial = {path.name: judge.sha256(path.read_bytes()) for path in capture.iterdir()}
        first, first_report = self.evaluate(markers, capture)
        second, second_report = self.evaluate(markers, capture)
        self.assertEqual(judge.json_bytes(first), judge.json_bytes(second))
        for name in ("window.json", "verdict.json", "verdict-replay.json", "business-facts.json"):
            self.assertEqual((first_report / "baseline" / name).read_bytes(),
                             (second_report / "baseline" / name).read_bytes())
        for report in (first_report, second_report):
            snapshot = report / "baseline" / "snapshot"
            before = {path.name: judge.sha256(path.read_bytes()) for path in snapshot.iterdir()}
            first_replay = judge.json_bytes(judge.evaluate_snapshot(snapshot, first[0]["manifest_sha256"]))
            second_replay = judge.json_bytes(judge.evaluate_snapshot(snapshot, first[0]["manifest_sha256"]))
            self.assertEqual(first_replay, second_replay)
            self.assertEqual(before, {path.name: judge.sha256(path.read_bytes())
                                      for path in snapshot.iterdir()})
        self.assertEqual(initial, {path.name: judge.sha256(path.read_bytes()) for path in capture.iterdir()})

    def test_failed_business_snapshot_replays_identically(self):
        records = active_samples(BASELINE_NS)
        for item in records["can"]:
            item["frame"]["data_hex"] = hold_payload()
        capture = self.capture(records)
        markers = phase_markers("baseline", BASELINE_NS, 1500, True)
        with self.assertRaisesRegex(RuntimeError, "business phase failed"):
            self.evaluate(markers, capture)
        directory = self.directory / "evaluation-1" / "baseline"
        first = (directory / "verdict.json").read_bytes()
        second = (directory / "verdict-replay.json").read_bytes()
        self.assertEqual(first, second)
        self.assertEqual(json.loads(first)["verdict"], "FAIL")


if __name__ == "__main__":
    unittest.main(verbosity=2)
