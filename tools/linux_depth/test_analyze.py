import csv
import hashlib
import io
import json
from pathlib import Path
import tempfile
import unittest

import analyze


class TimingAnalysisTest(unittest.TestCase):
    def setUp(self):
        self.temporary = tempfile.TemporaryDirectory()
        self.addCleanup(self.temporary.cleanup)
        self.scene = Path(self.temporary.name)
        command = {"source_stamp": {"sec": 1, "nanosec": 0}, "source_sequence": 1,
                   "source_id": "planning", "mode": 0, "linear_velocity_mps": .2, "angular_velocity_radps": -.1}
        status = lambda at, count: {"monotonic_ns": at, "message": {"source_id": "planning", "accepted_count": count, "rejected_count": 0}}
        payload = bytes.fromhex("14009cff010001")
        payload += bytes([analyze.judge.crc8(0x180, payload)])
        streams = {"source": [{"monotonic_ns": 2100, "quantized_key": [20, -100], "message": command}],
                   "canonical": [{"monotonic_ns": 2200, "message": command}],
                   "can": [{"monotonic_ns": 2250, "frame": {"can_id": 0x180, "raw_can_id": 0x180,
                       "dlc": 8, "is_extended": False, "is_remote": False, "is_error": False, "data_hex": payload.hex()}}],
                   "gateway": [{"monotonic_ns": 2300, "message": {"state": 1}}],
                   "status": [status(1900, 0), status(2400, 1)], "vehicle": [], "resource": []}
        self.inputs = {name + ".jsonl": b"".join((json.dumps(row, sort_keys=True) + "\n").encode() for row in rows)
                       for name, rows in streams.items()}
        self.inputs["events.json"] = analyze.encoded({"result": 0, "stop_reason": "fixed_window_complete",
            "window": {"start_ns": 2000, "end_ns": 3000}, "missed_send_slots": 0, "clk_tck": 100})
        self.inputs["config-adas-can_signal_map.yaml"] = (analyze.ROOT / "config/adas/can_signal_map.yaml").read_bytes()
        self.rows = []
        for phase, gid, begin, end, result, frame in (
            ("source_receive", "a1", 2110, 2115, 0, 0), ("source_result", "a1", 2110, 2120, 1, 0),
            ("source_select", "a1", 2130, 2131, 0, 100), ("canonical_publish", "b2", 2140, 2145, 0, 100),
            ("mailbox_write", "b2", 2142, 2146, 1, 0), ("mailbox_consume", "b2", 2143, 2150, 1, 1),
            ("can_write", "b2", 2152, 2154, 16, 1)):
            self.rows.append({"phase": phase, "gid": gid, "begin_ns": begin, "end_ns": end, "result": result,
                "frame_id": frame, "source_hex": "planning".encode().hex(), "pid": 10, "tid": 11,
                "source_stamp_ns": 1000000000, "sequence": 1, "scheduled_ns": 0, "value": 0})

    def prepare(self, lost=0, version=1):
        buffer = io.StringIO()
        buffer.write(f"# control_link_timing_v{version} clock=CLOCK_MONOTONIC\n")
        if version == 2:
            for row in self.rows:
                row.setdefault("schedule_clock", 1)
                row.setdefault("schedule_now_ns", 0)
        writer = csv.DictWriter(buffer, fieldnames=list(self.rows[0]))
        writer.writeheader(); writer.writerows(self.rows)
        buffer.write("# drain queued=7 written=7\n")
        self.inputs["gateway-timing.csv"] = buffer.getvalue().encode()
        self.inputs["gateway-timing.csv.status.json"] = analyze.encoded({"data_closed": True, "queued": len(self.rows),
            "written": len(self.rows), "io_errors": 0, "full": 0, "contended": lost, "invalid_identity": 0, "closed": 0})
        facts = {}
        for name, content in self.inputs.items():
            (self.scene / name).write_bytes(content)
            facts[name] = {"size": len(content), "sha256": hashlib.sha256(content).hexdigest()}
        (self.scene / "frozen-inputs.json").write_bytes(analyze.encoded({"files": facts}))

    def test_bridge_different_gid_and_overlapping_publish(self):
        self.prepare()
        summary = analyze.analyze(self.scene)
        self.assertEqual(summary["first_matches"], {"canonical": 1, "vcan": 1})
        self.assertEqual(summary["upstream_to_canonical_publisher_gid_edges"], [("a1", "b2")])
        self.assertAlmostEqual(summary["latencies_ms"]["source_publish_to_first_CAN_write_return_ms"]["mean"], .000054)
        self.assertAlmostEqual(summary["latencies_ms"]["publish_enter_to_first_adapter_callback_ms"]["mean"], .000002)
        self.assertEqual(summary["mailbox_worker_handoff"]["counts"]["first_active_command"], 1)
        self.assertAlmostEqual(summary["latencies_ms"]["mailbox_update_mark_to_worker_frame_ready_first_active_command_ms"]["mean"], .000007)

    def test_mailbox_distinguishes_reuse_hold_and_ambiguous_generation(self):
        self.prepare()
        inputs = analyze.load_inputs(self.scene)
        rows, unused = analyze.timing(inputs['gateway-timing.csv'], inputs['gateway-timing.csv.status.json'])
        consume = next(row for row in rows if row['phase'] == 'mailbox_consume')
        send = next(row for row in rows if row['phase'] == 'can_write')
        repeated = [{**consume, 'frame_id': 2, 'end_ns': 2160}, {**send, 'frame_id': 2, 'begin_ns': 2161, 'end_ns': 2162}]
        hold = [{**consume, 'frame_id': 3, 'result': 0, 'end_ns': 2170}, {**send, 'frame_id': 3, 'begin_ns': 2171, 'end_ns': 2172}]
        values, result = analyze.mailbox_handoffs(rows + repeated + hold, 2000, 3000)
        self.assertEqual(result['counts']['first_active_command'], 1)
        self.assertEqual(result['counts']['reused_active'], 1)
        self.assertEqual(result['counts']['hold'], 1)
        conflicting = [{**row, 'result': 1} for row in rows if row['phase'] == 'canonical_publish']
        self.assertFalse(analyze.mailbox_handoffs(rows + conflicting, 2000, 3000)[0])
        other_generation = [{**row, 'gid': 'c3'} if row['phase'] == 'mailbox_consume' else row for row in rows]
        self.assertFalse(analyze.mailbox_handoffs(other_generation, 2000, 3000)[0])
        values, boundary = analyze.mailbox_handoffs(rows + repeated, 2155, 3000)
        self.assertNotIn('first_active_command', boundary['counts'])
        self.assertEqual(boundary['counts']['reused_active'], 1)

    def test_drops_remain_visible_not_complete_population(self):
        self.prepare(lost=3)
        result = analyze.analyze(self.scene)
        self.assertFalse(result["timing_quality"]["gateway"]["population_complete"])
        self.assertEqual(result["timing_quality"]["gateway"]["lost_events"], 3)

    def test_ambiguous_upstream_generation_not_guessed(self):
        self.rows.append({**self.rows[0], "gid": "a2"})
        self.prepare()
        summary = analyze.analyze(self.scene)
        self.assertEqual(summary["correlation_gaps"]["incoming_missing_or_ambiguous"], 1)
        self.assertNotIn("source_publish_to_first_CAN_write_return_ms", summary["latencies_ms"])

    def test_repeated_quantization_forbidden(self):
        self.inputs["source.jsonl"] *= 2
        self.prepare()
        with self.assertRaisesRegex(ValueError, "ambiguous source quantization"):
            analyze.analyze(self.scene)

    def test_identity_includes_gid_and_original_stamp(self):
        base = {"source": "planning", "gid": "a1", "sequence": 1, "source_stamp_ns": 100}
        self.assertNotEqual(analyze.identity_key(base), analyze.identity_key({**base, "gid": "a2"}))
        self.assertNotEqual(analyze.identity_key(base), analyze.identity_key({**base, "source_stamp_ns": 101}))

    def test_changed_frozen_input_rejected(self):
        self.prepare()
        (self.scene / "source.jsonl").write_bytes(b"modified")
        with self.assertRaisesRegex(ValueError, "frozen input changed"):
            analyze.analyze(self.scene)

    def test_missing_drain_rejected(self):
        self.prepare()
        with self.assertRaisesRegex(ValueError, "missing drain"):
            analyze.timing(b"# control_link_timing_v1 clock=CLOCK_MONOTONIC\n", b"{}")

    def test_failed_close_status_rejected(self):
        self.prepare()
        final = json.loads(self.inputs["gateway-timing.csv.status.json"])
        final["io_errors"] = 1
        with self.assertRaisesRegex(ValueError, "incomplete timing"):
            analyze.timing(self.inputs["gateway-timing.csv"], analyze.encoded(final))

    def test_raw_timer_lateness_uses_same_clock_not_monotonic(self):
        self.rows.append({**self.rows[0], "phase": "periodic_tick", "begin_ns": 2500, "end_ns": 0,
                          "scheduled_ns": 1200, "schedule_clock": 2, "schedule_now_ns": 1207})
        self.prepare(version=2)
        result = analyze.analyze(self.scene)
        self.assertAlmostEqual(result["gateway_internal_deadline_lateness_ms"]["mean"], .000007)

    def test_legacy_gateway_deadline_metric_invalidated(self):
        self.rows.append({**self.rows[0], "phase": "periodic_tick", "begin_ns": 2500, "end_ns": 0, "scheduled_ns": 1200})
        self.prepare()
        result = analyze.analyze(self.scene)
        self.assertEqual(result["gateway_internal_deadline_lateness_ms"], {"samples": 0})
        self.assertIn("legacy_gateway_deadline_invalidated", result)


if __name__ == "__main__":
    unittest.main()
