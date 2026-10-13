"""只验证新测量器的关联/统计，不重开产品验收矩阵。"""

import copy
import hashlib
import json
import os
import struct
from pathlib import Path
import tempfile
import unittest

from performance_stats import calculate, distribution, encode_json, judge, quantize, target

ROOT = next(parent for parent in Path(__file__).resolve().parents
            if (parent / "config/adas/can_signal_map.yaml").is_file())
MAPPING_BYTES = (ROOT / "config/adas/can_signal_map.yaml").read_bytes()
MAPPING = judge.load_can_map(MAPPING_BYTES)


def specimen():
    """单命令、重复 canonical/CAN、一个正常固定窗的最小离线夹具。"""
    origin = 20_000_000_000
    values, key = target(1, MAPPING)
    message = {"source_id": "external_planner", "source_sequence": 1,
               "source_stamp": {"sec": 120, "nanosec": 0}, "mode": 0,
               "linear_velocity_mps": values[0], "angular_velocity_radps": values[1]}
    canonical = {**message, "source_id": "planning"}
    trace_canonical = {name: value for name, value in canonical.items()
                       if name not in ("linear_velocity_mps", "angular_velocity_radps")}
    trace_canonical.update(linear_velocity_bits=struct.pack("!d", values[0]).hex(),
                           angular_velocity_bits=struct.pack("!d", values[1]).hex())
    source = {"message": message, "quantized_key": key, "publish_before_monotonic_ns": origin,
              "scheduled_monotonic_ns": origin}
    payload = bytearray(key[0].to_bytes(2, "little", signed=True) +
                        key[1].to_bytes(2, "little", signed=True) + bytes([1, 0, 1, 0]))
    payload[7] = judge.crc8(0x180, payload)
    frame = {"can_id": 0x180, "raw_can_id": 0x180, "dlc": 8, "is_extended": False,
             "is_remote": False, "is_error": False, "data_hex": payload.hex(), "kernel_wall_ns": 120_005_000_000}
    status = {"source_id": "planning", "accepted_count": 1, "command_valid": True, "lease_valid": True,
              "endpoint_stable": True, "endpoint_publisher_count": 1, "endpoint_generation_valid": True,
              "endpoint_publisher_gid": [1] * 24}
    records = {"source": [source], "canonical": [], "can": [], "gateway": [], "status": [],
               "vehicle": [], "resource": []}
    trace = []
    for index in range(2):
        mono = origin + 5_000_000 + index * 20_000_000
        wall = mono + 100_000_000_000
        records["canonical"].append({"monotonic_ns": mono, "wall_ns": wall, "message": canonical})
        observed = {"sec": wall // 1_000_000_000, "nanosec": wall % 1_000_000_000}
        records["gateway"].append({"monotonic_ns": mono, "wall_ns": wall,
                                   "message": {"state": 1, "active_source_id": "planning", "observed_at": observed}})
        records["status"].append({"monotonic_ns": mono, "wall_ns": wall, "message": status})
        records["vehicle"].append({"monotonic_ns": mono, "wall_ns": wall, "message": {"state": 1, "fault_code": 0}})
        current = copy.deepcopy(frame)
        data = bytearray.fromhex(current["data_hex"])
        data[5] = index
        data[7] = judge.crc8(0x180, data)
        current["data_hex"] = data.hex()
        records["can"].append({"monotonic_ns": mono + 2_000_000, "wall_ns": wall + 2_000_000, "frame": current})
        trace.extend([{"record_type": "event", "event_type": "output_tick", "event_sequence": index + 1,
                       "now_ros_ns": wall, "steady_offset_ns": mono},
                      {"record_type": "result", "event_sequence": index + 1, "state": 1,
                       "canonical_command": trace_canonical, "sources": [status]}])
    trace.append({"record_type": "footer", "trace_valid": True, "trace_overflow": False,
                  "event_count": 2, "result_count": 2})
    for index in range(3):
        records["resource"].append({"role": "gateway", "pid": 12, "starttime": 45,
            "monotonic_ns": origin + index * 1_000_000_000, "cpu_ticks": 100 + index * 10,
            "rss_kib": 1000 + index * 10})
    plan = {"windows": [{"name": "window-1", "start_monotonic_ns": origin,
        "end_monotonic_ns": origin + 60_000_000_000, "start_wall_ns": origin + 100_000_000_000,
        "end_wall_ns": origin + 160_000_000_000}]}
    contents = {name + ".jsonl": b"".join(json.dumps(record).encode() + b"\n" for record in values)
                for name, values in records.items()}
    contents.update({"windows.json": encode_json(plan), "can_signal_map.yaml": MAPPING_BYTES,
        "events.json": encode_json({"resource_roles": {"gateway": 12}, "clk_tck": 100}),
        "decision-trace.jsonl": b"".join(json.dumps(item).encode() + b"\n" for item in trace)})
    return contents


def evaluate(contents):
    with tempfile.TemporaryDirectory() as directory:
        report = Path(directory)
        for name, data in contents.items():
            (report / name).write_bytes(data)
        (report / "frozen-inputs.json").write_bytes(encode_json({"files": {
            name: {"sha256": hashlib.sha256(data).hexdigest()} for name, data in contents.items()}}))
        return calculate(report)


class PerformanceStatsTest(unittest.TestCase):
    def test_all_targets_unique_and_match_production_rounding(self):
        keys = [tuple(target(sequence, MAPPING)[1]) for sequence in range(1, 6001)]
        self.assertEqual(len(set(keys)), 6000)
        with self.assertRaises(ValueError):
            target(6001, MAPPING)

    def test_rounding_half_away_from_zero(self):
        self.assertEqual(quantize(1.5, 1), 2)
        self.assertEqual(quantize(-1.5, 1), -2)

    def test_actual_proc_pid_starttime_and_resident_sample(self):
        from run_performance import proc_sample
        sample = proc_sample(os.getpid())
        self.assertEqual(sample["pid"], os.getpid())
        self.assertGreater(sample["starttime"], 0)
        self.assertGreater(sample["rss_kib"], 0)
        self.assertGreater(sample["sample_read_span_ns"], 0)
        self.assertGreaterEqual(sample["cpu_ticks"], 0)

    def test_percentiles_and_small_sample_p99(self):
        self.assertIsNone(distribution([1, 2])["p99"])
        self.assertEqual(distribution(list(range(101)))["p99"], 99)

    def test_first_match_not_repeated_frame_or_counter_identity(self):
        summary, latency, frames, unused_resources = evaluate(specimen())
        self.assertEqual(latency[0]["source_to_canonical_receive_ms"], 5)
        self.assertEqual(latency[0]["source_to_vcan_receive_ms"], 7)
        self.assertEqual(len(frames), 2)
        self.assertEqual(summary["windows"][0]["anomalies"]["internal_canonical_source_or_fields_mismatch"], 0)
        self.assertEqual(summary["aggregate"]["metrics"]["internal_tick_interval_ms"]["mean"], 20)
        self.assertEqual(summary["aggregate"]["resources"]["gateway"]["cpu_mean_percent"], 10)

    def test_missing_can_preserved_as_unmatched(self):
        contents = specimen()
        contents["can.jsonl"] = b""
        summary, latency, unused_frames, unused_resources = evaluate(contents)
        self.assertIsNone(latency[0]["source_to_vcan_receive_ms"])
        self.assertEqual(summary["windows"][0]["anomalies"]["source_to_can_unmatched"], 1)

    def test_duplicate_canonical_loss_is_not_a_lost_command(self):
        contents = specimen()
        contents["canonical.jsonl"] = contents["canonical.jsonl"].splitlines()[0] + b"\n"
        summary, latency, unused_frames, unused_resources = evaluate(contents)
        window = summary["windows"][0]
        self.assertEqual(window["missing_duplicate_canonical_messages_by_sequence"], {"1": 1})
        self.assertEqual(window["anomalies"]["source_to_canonical_unmatched"], 0)
        self.assertEqual(latency[0]["source_to_canonical_receive_ms"], 5)

    def test_crc_invalid_cannot_match(self):
        contents = specimen()
        records = [json.loads(line) for line in contents["can.jsonl"].splitlines()]
        for record in records:
            record["frame"]["data_hex"] = record["frame"]["data_hex"][:-2] + "00"
        contents["can.jsonl"] = b"".join(json.dumps(record).encode() + b"\n" for record in records)
        summary, latency, unused_frames, unused_resources = evaluate(contents)
        self.assertIsNone(latency[0]["source_to_vcan_receive_ms"])
        self.assertEqual(summary["windows"][0]["anomalies"]["can_invalid"], 2)

    def test_duplicate_source_key_rejected(self):
        contents = specimen()
        contents["source.jsonl"] *= 2
        with self.assertRaisesRegex(ValueError, "one-to-one"):
            evaluate(contents)

    def test_canonical_clock_inversion_rejected(self):
        contents = specimen()
        record = json.loads(contents["canonical.jsonl"].splitlines()[0])
        record["monotonic_ns"] = 1
        contents["canonical.jsonl"] = json.dumps(record).encode() + b"\n"
        with self.assertRaisesRegex(ValueError, "precedes"):
            evaluate(contents)

    def test_pid_reuse_rejected(self):
        contents = specimen()
        records = [json.loads(line) for line in contents["resource.jsonl"].splitlines()]
        records[-1]["starttime"] += 1
        contents["resource.jsonl"] = b"".join(json.dumps(record).encode() + b"\n" for record in records)
        with self.assertRaisesRegex(ValueError, "PID reused"):
            evaluate(contents)

    def test_replay_deterministic(self):
        self.assertEqual(encode_json(evaluate(specimen())[0]), encode_json(evaluate(specimen())[0]))


if __name__ == "__main__":
    unittest.main()
