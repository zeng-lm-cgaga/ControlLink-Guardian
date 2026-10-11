#!/usr/bin/env python3
"""ROS-free 固定窗事实判据；不实现 Validator、仲裁或业务 freshness。

freeze 在观察者已 wait 后复制不可变输入；replay 不读运行中的日志。
wall 用于 SYSTEM/内核帧关联，monotonic 只核对本机捕获覆盖，各自比较。
"""

import argparse
import hashlib
import json
import math
import os
from pathlib import Path
import re
import stat
import sys


STREAMS = ("gateway", "source", "vehicle", "can")
INPUT_FILES = tuple(name + ".jsonl" for name in STREAMS) + (
    "receipt.json", "capture-meta.json", "gateway_contract.yaml", "window.json", "can_signal_map.yaml")
MAX_FILE_BYTES = 64 * 1024 * 1024
MAX_TOTAL_BYTES = 128 * 1024 * 1024


class InvalidEvidence(ValueError):
    """缺失、损坏、未正常冻结的证据不能给出业务 PASS。"""


def json_bytes(value):
    return (json.dumps(value, sort_keys=True, separators=(",", ":"),
                       ensure_ascii=False, allow_nan=False) + "\n").encode("utf-8")


def sha256(data):
    return hashlib.sha256(data).hexdigest()


def publish_json(path, value):
    """仅发布新文件；文件和目录 fsync 失败向调用者传播，禁止覆盖历史。"""
    destination = Path(path)
    pending = destination.with_name(destination.name + ".pending-" + str(os.getpid()))
    published = False
    try:
        with pending.open("xb") as stream:
            stream.write(json_bytes(value))
            stream.flush()
            os.fsync(stream.fileno())
        os.link(pending, destination)
        published = True
        pending.unlink()
        directory = os.open(destination.parent, os.O_RDONLY | os.O_DIRECTORY)
        try:
            os.fsync(directory)
        finally:
            os.close(directory)
    except OSError:
        if published:
            destination.unlink(missing_ok=True)
        pending.unlink(missing_ok=True)
        raise


def _unique_object(pairs):
    result = {}
    for name, value in pairs:
        if name in result:
            raise InvalidEvidence("duplicate JSON field: " + name)
        result[name] = value
    return result


def parse_json(data):
    def reject_constant(value):
        raise InvalidEvidence("non-standard JSON number: " + value)

    try:
        return json.loads(data, object_pairs_hook=_unique_object,
                          parse_constant=reject_constant)
    except (UnicodeError, json.JSONDecodeError) as error:
        raise InvalidEvidence("malformed JSON") from error


def read_frozen(path):
    descriptor = os.open(path, os.O_RDONLY | os.O_NOFOLLOW)
    with os.fdopen(descriptor, "rb") as stream:
        before = os.fstat(stream.fileno())
        if not stat.S_ISREG(before.st_mode) or before.st_size > MAX_FILE_BYTES:
            raise InvalidEvidence("input is not a bounded regular file")
        data = stream.read(MAX_FILE_BYTES + 1)
        after = os.fstat(stream.fileno())
        identity = lambda value: (value.st_dev, value.st_ino, value.st_size,
                                  value.st_mtime_ns, value.st_ctime_ns)
        if identity(before) != identity(after) or len(data) != before.st_size:
            raise InvalidEvidence("input changed while being read")
        return data


def integer(value, label, minimum=0, maximum=2 ** 63 - 1):
    if type(value) is not int or not minimum <= value <= maximum:
        raise InvalidEvidence("invalid integer: " + label)
    return value


def gid_hex(value):
    if isinstance(value, list):
        if len(value) != 24 or any(type(item) is not int or not 0 <= item <= 255
                                   for item in value):
            raise InvalidEvidence("GID must contain 24 bytes")
        value = bytes(value).hex()
    if not isinstance(value, str) or not re.fullmatch(r"[0-9a-fA-F]{48}", value):
        raise InvalidEvidence("GID must be 48 hexadecimal digits")
    if int(value, 16) == 0:
        raise InvalidEvidence("zero GID is not an admitted publisher")
    return value.lower()


def window_bounds(window):
    if window.get("schema_version") != 1 or window.get("ros_time_domain") != "SYSTEM":
        raise InvalidEvidence("window requires schema 1 and explicit SYSTEM domain")
    result = tuple(integer(window[name], name, 1) for name in (
        "start_wall_ns", "end_wall_ns", "start_monotonic_ns", "end_monotonic_ns"))
    if result[0] >= result[1] or result[2] >= result[3]:
        raise InvalidEvidence("empty or reversed fixed window")
    return result


def validate_receipt(receipt, window, waited_exit):
    start_wall, end_wall, start_mono, end_mono = window_bounds(window)
    if receipt.get("schema_version") != 1 or receipt.get("kind") != "native_ros_can":
        raise InvalidEvidence("unsupported observer receipt")
    if (type(waited_exit) is not int or waited_exit != 0 or
            type(receipt.get("exit_code")) is not int or receipt["exit_code"] != waited_exit or
            type(receipt.get("requested_signal")) is not int or receipt["requested_signal"] != 2 or
            receipt.get("stop_reason") != "cooperative_sigint" or
            receipt.get("raw_child_exit", "missing") is not None or
            receipt.get("flush_complete") is not True or receipt.get("errors") != []):
        raise InvalidEvidence("observer did not complete requested SIGINT and durable flush")
    names = ("capture_start_wall_ns", "capture_end_wall_ns",
             "capture_start_monotonic_ns", "capture_end_monotonic_ns",
             "requested_wall_ns", "requested_monotonic_ns")
    values = {name: integer(receipt[name], name, 1) for name in names}
    for clock, window_start, window_end in (("wall", start_wall, end_wall),
                                          ("monotonic", start_mono, end_mono)):
        captured_start = values["capture_start_" + clock + "_ns"]
        captured_end = values["capture_end_" + clock + "_ns"]
        requested = values["requested_" + clock + "_ns"]
        if not captured_start <= window_start < window_end <= requested <= captured_end:
            raise InvalidEvidence("capture/request does not cover fixed " + clock + " window")
    if receipt.get("ros_time_domain") != "SYSTEM":
        raise InvalidEvidence("observer did not declare SYSTEM observations")


def validate_capture_metadata(inputs, receipt):
    metadata = parse_json(inputs["capture-meta.json"])
    if (metadata.get("schema_version") != 1 or metadata.get("ros_time_domain") != "SYSTEM" or
            metadata.get("pid") != receipt.get("pid") or
            metadata.get("contract_sha256") != sha256(inputs["gateway_contract.yaml"])):
        raise InvalidEvidence("capture metadata/contract identity mismatch")


def freeze_capture(capture_dir, snapshot_dir, window_path, can_map_path,
                   expected_gid, waited_exit, previous_gid=None):
    """waited_exit 必须来自 shell wait；不从仍活着的进程推断退出成功。"""
    capture = Path(capture_dir)
    destination = Path(snapshot_dir)
    target_gid = gid_hex(expected_gid)
    old_gid = gid_hex(previous_gid) if previous_gid is not None else None
    if target_gid == old_gid:
        raise InvalidEvidence("recovery expected a different publisher GID")
    inputs = {name + ".jsonl": read_frozen(capture / (name + ".jsonl"))
              for name in STREAMS}
    inputs["receipt.json"] = read_frozen(capture / "receipt.json")
    inputs["capture-meta.json"] = read_frozen(capture / "capture-meta.json")
    inputs["gateway_contract.yaml"] = read_frozen(capture / "gateway_contract.yaml")
    inputs["window.json"] = read_frozen(window_path)
    inputs["can_signal_map.yaml"] = read_frozen(can_map_path)
    receipt = parse_json(inputs["receipt.json"])
    window = parse_json(inputs["window.json"])
    validate_receipt(receipt, window, waited_exit)
    validate_capture_metadata(inputs, receipt)
    if sum(map(len, inputs.values())) > MAX_TOTAL_BYTES:
        raise InvalidEvidence("snapshot exceeds bounded input budget")
    for name in STREAMS:
        data = inputs[name + ".jsonl"]
        facts = receipt["streams"][name]
        if (facts["sha256"] != sha256(data) or facts["bytes"] != len(data) or
                facts["records"] != data.count(b"\n")):
            raise InvalidEvidence("receipt does not bind stream: " + name)
    destination.mkdir(parents=True, exist_ok=False)
    for name, data in inputs.items():
        with (destination / name).open("xb") as stream:
            stream.write(data)
            stream.flush()
            os.fsync(stream.fileno())
    manifest = {"schema_version": 1, "expected_gid": target_gid,
                "previous_gid": old_gid, "waited_exit": waited_exit,
                "inputs": {name: {"sha256": sha256(data), "bytes": len(data)}
                           for name, data in inputs.items()}}
    publish_json(destination / "manifest.json", manifest)
    # 后续只读 SHA 核验；权限仅防误写，不宣称抵御有权限篡改者。
    for path in destination.iterdir():
        path.chmod(0o444)
    destination.chmod(0o555)
    return sha256(json_bytes(manifest))


def load_snapshot(snapshot_dir, expected_manifest_sha=None):
    snapshot = Path(snapshot_dir)
    raw_manifest = read_frozen(snapshot / "manifest.json")
    digest = sha256(raw_manifest)
    if expected_manifest_sha is not None and digest != expected_manifest_sha:
        raise InvalidEvidence("manifest SHA256 mismatch")
    manifest = parse_json(raw_manifest)
    if manifest.get("schema_version") != 1 or set(manifest["inputs"]) != set(INPUT_FILES):
        raise InvalidEvidence("unsupported/incomplete frozen manifest")
    inputs = {}
    for name in INPUT_FILES:
        data = read_frozen(snapshot / name)
        facts = manifest["inputs"][name]
        if facts["sha256"] != sha256(data) or facts["bytes"] != len(data):
            raise InvalidEvidence("frozen SHA256/size mismatch: " + name)
        inputs[name] = data
    if sum(map(len, inputs.values())) > MAX_TOTAL_BYTES:
        raise InvalidEvidence("snapshot exceeds bounded input budget")
    return manifest, inputs, digest


def observed_ns(message):
    stamp = message["observed_at"]
    seconds = integer(stamp["sec"], "observed_at.sec")
    nanos = integer(stamp["nanosec"], "observed_at.nanosec")
    if nanos >= 1_000_000_000:
        raise InvalidEvidence("invalid SYSTEM nanosecond component")
    return integer(seconds * 1_000_000_000 + nanos, "observed_at", 1)


def records_in_window(data, kind, receipt, window):
    start_wall, end_wall, start_mono, end_mono = window_bounds(window)
    if data and not data.endswith(b"\n"):
        raise InvalidEvidence("truncated JSONL: " + kind)
    selected = []
    last_mono = 0
    total = 0
    for total, line in enumerate(data.splitlines(), 1):
        record = parse_json(line)
        if record.get("schema_version") != 1 or record.get("kind") != kind:
            raise InvalidEvidence("unexpected JSONL schema/kind")
        if integer(record["record_index"], "record_index", 1) != total:
            raise InvalidEvidence("missing/duplicate stream record index")
        wall = integer(record["received_wall_ns"], "received_wall_ns", 1)
        mono = integer(record["received_monotonic_ns"], "received_monotonic_ns", 1)
        if (mono < last_mono or
                not receipt["capture_start_monotonic_ns"] <= mono <= receipt["capture_end_monotonic_ns"] or
                not receipt["capture_start_wall_ns"] <= wall <= receipt["capture_end_wall_ns"]):
            raise InvalidEvidence("record outside capture or reversed monotonic clock")
        last_mono = mono
        if not (start_wall <= wall < end_wall and start_mono <= mono < end_mono):
            continue
        sample_wall = (integer(record["frame"]["kernel_wall_ns"], "kernel_wall_ns", 1)
                       if kind == "can" else observed_ns(record["message"]))
        if start_wall <= sample_wall < end_wall:
            selected.append(record)
    facts = receipt["streams"][kind]
    if (integer(facts["records"], "records") != total or facts["bytes"] != len(data) or
            facts["sha256"] != sha256(data)):
        raise InvalidEvidence("receipt stream facts mismatch: " + kind)
    return selected, total


def load_can_map(data):
    import yaml

    try:
        mapping = yaml.safe_load(data)
    except yaml.YAMLError as error:
        raise InvalidEvidence("malformed CAN map YAML") from error
    if not isinstance(mapping, dict):
        raise InvalidEvidence("CAN map must be a mapping")
    # 固定 demo ABI 与 map 必须一致，拒绝静默解释新布局；不复制业务 limits。
    expected = {"schema_version": 1, "protocol_id": "control_link_demo_can_v1",
                "byte_order": "little_endian", "dlc": 8}
    if any(mapping.get(name) != value for name, value in expected.items()):
        raise InvalidEvidence("unsupported CAN signal map ABI")
    crc = mapping["crc"]
    if crc != {"algorithm": "crc8_sae_j1850", "polynomial": 0x1D,
               "initial_value": 0xFF, "final_xor": 0xFF,
               "reflect_input": False, "reflect_output": False,
               "include_can_id_lsb_first": True,
               "protected_payload_bytes": list(range(7))}:
        raise InvalidEvidence("unsupported CAN CRC contract")
    signals = mapping["frames"]["control_command"]["signals"]
    for name, offset, bits, signed in (("target_speed_mps", 0, 16, True),
                                       ("target_yaw_rate_radps", 2, 16, True),
                                       ("command_mode", 4, 8, False),
                                       ("rolling_counter", 5, 4, False),
                                       ("command_valid", 6, 1, False),
                                       ("crc", 7, 8, False)):
        signal_map = signals[name]
        if (signal_map["byte_offset"] != offset or signal_map["bit_offset"] != 0 or
                signal_map["bit_length"] != bits or
                ("signed" in signal_map and signal_map["signed"] != signed)):
            raise InvalidEvidence("unsupported CAN field layout: " + name)
    if signals["command_mode"]["enum"] != {"hold": 0, "active": 1}:
        raise InvalidEvidence("unsupported CAN mode encoding")
    for name, offset, bit_offset, length in (("reserved_counter_high_nibble", 5, 4, 4),
                                            ("reserved_flags", 6, 1, 7)):
        if signals[name] != {"byte_offset": offset, "bit_offset": bit_offset,
                             "bit_length": length, "constant": 0}:
            raise InvalidEvidence("unsupported reserved CAN field: " + name)
    return mapping


def crc8(can_id, payload):
    crc = 0xFF
    for value in bytes((can_id & 0xFF, (can_id >> 8) & 0xFF)) + payload[:7]:
        crc ^= value
        for bit_index in range(8):
            crc = ((crc << 1) ^ (0x1D if crc & 0x80 else 0)) & 0xFF
    return crc ^ 0xFF


def control_frame_fact(frame, mapping):
    """只解码实际线上的 frame/CRC/信号事实，不运行 command 治理规则。"""
    can_id = integer(frame["can_id"], "can_id")
    if can_id != mapping["frames"]["control_command"]["can_id"]:
        return "other", None
    raw_id = integer(frame["raw_can_id"], "raw_can_id", maximum=2 ** 32 - 1)
    if (raw_id != can_id or frame["is_extended"] is not False or frame["is_remote"] is not False or
            frame["is_error"] is not False or type(frame["dlc"]) is not int or
            frame["dlc"] != mapping["dlc"]):
        return "invalid", None
    if not isinstance(frame["data_hex"], str) or not re.fullmatch(r"[0-9a-fA-F]{16}", frame["data_hex"]):
        return "invalid", None
    payload = bytes.fromhex(frame["data_hex"])
    if (payload[5] & 0xF0 or payload[6] != 1 or payload[4] not in (0, 1) or
            payload[5] >= mapping["rolling_counter"]["modulo"] or
            crc8(can_id, payload) != payload[7]):
        return "invalid", None
    speed = int.from_bytes(payload[0:2], "little", signed=True)
    yaw = int.from_bytes(payload[2:4], "little", signed=True)
    signals = mapping["frames"]["control_command"]["signals"]
    for raw, name in ((speed, "target_speed_mps"), (yaw, "target_yaw_rate_radps")):
        spec = signals[name]
        physical = raw * spec["scale"] + spec["offset"]
        if not spec["minimum"] <= physical <= spec["maximum"]:
            return "invalid", None
    if payload[4] == 0:
        return ("hold", speed) if speed == yaw == 0 else ("invalid", None)
    return ("active_nonzero" if speed != 0 else "active_zero"), speed


def evaluate_snapshot(snapshot_dir, expected_manifest_sha=None):
    manifest, inputs, digest = load_snapshot(snapshot_dir, expected_manifest_sha)
    target_gid = gid_hex(manifest["expected_gid"])
    previous = manifest.get("previous_gid")
    if previous is not None and gid_hex(previous) == target_gid:
        raise InvalidEvidence("new GID equals retired GID")
    receipt = parse_json(inputs["receipt.json"])
    window = parse_json(inputs["window.json"])
    validate_receipt(receipt, window, manifest["waited_exit"])
    validate_capture_metadata(inputs, receipt)
    mapping = load_can_map(inputs["can_signal_map.yaml"])
    selected, totals = {}, {}
    for name in STREAMS:
        selected[name], totals[name] = records_in_window(
            inputs[name + ".jsonl"], name, receipt, window)
    sources = [record for record in selected["source"]
               if record["message"].get("source_id") == "planning"]
    admitted, mismatch = [], 0
    for record in sources:
        message = record["message"]
        integer(message["endpoint_state"], "endpoint_state", maximum=255)
        integer(message["endpoint_publisher_count"], "endpoint_publisher_count", maximum=2 ** 32 - 1)
        usable = (message.get("endpoint_stable") is True and message.get("endpoint_state") == 5 and
                  message.get("endpoint_publisher_count") == 1 and
                  message.get("endpoint_generation_valid") is True)
        if not usable or gid_hex(message["endpoint_publisher_gid"]) != target_gid:
            mismatch += 1
            continue
        integer(message["accepted_count"], "accepted_count", maximum=2 ** 64 - 1)
        integer(message["last_source_sequence"], "last_source_sequence", maximum=2 ** 64 - 1)
        admitted.append(record)
    for record in selected["gateway"]:
        integer(record["message"]["state"], "gateway state", maximum=255)
        integer(record["message"]["active_source_sequence"], "active_source_sequence", maximum=2 ** 64 - 1)
    failures = []
    if len(admitted) < 2 or mismatch:
        failures.append("window lacks exclusively stable sole expected planning GID")
    baseline_count = admitted[0]["message"]["accepted_count"] if admitted else 0
    baseline_sequence = admitted[0]["message"]["last_source_sequence"] if admitted else 0
    counts = [record["message"]["accepted_count"] for record in admitted]
    sequences = [record["message"]["last_source_sequence"] for record in admitted]
    if (not counts or max(counts) <= baseline_count or max(sequences) <= baseline_sequence or
            counts != sorted(counts) or sequences != sorted(sequences)):
        failures.append("new GID accepted count/sequence did not grow within fixed window")
    # Gateway 与 SourceStatus 来自同一 output tick 的 observed_at；精确关联新 GID。
    accepted_ticks = {(observed_ns(record["message"]), record["message"]["last_source_sequence"])
                      for record in admitted
                      if record["message"]["accepted_count"] > baseline_count and
                      record["message"].get("command_valid") is True and
                      record["message"].get("lease_valid") is True}
    active = [record for record in selected["gateway"]
              if record["message"].get("state") == 1 and
              record["message"].get("active_source_id") == "planning" and
              (observed_ns(record["message"]), record["message"].get("active_source_sequence"))
              in accepted_ticks]
    if not active:
        failures.append("no planning ACTIVE tick correlated to new GID acceptance")
    first_active_wall = min((observed_ns(record["message"]) for record in active), default=None)
    can_counts = {name: 0 for name in ("other", "invalid", "hold", "active_zero", "active_nonzero")}
    correlated_nonzero = 0
    for record in selected["can"]:
        fact, speed = control_frame_fact(record["frame"], mapping)
        can_counts[fact] += 1
        if (fact == "active_nonzero" and first_active_wall is not None and
                record["frame"]["kernel_wall_ns"] >= first_active_wall):
            correlated_nonzero += 1
    if not correlated_nonzero:
        failures.append("no CRC-valid ACTIVE nonzero speed CAN after correlated ACTIVE")
    if can_counts["invalid"]:
        failures.append("invalid control CAN structure/CRC in fixed window")
    vehicle_running = 0
    for record in selected["vehicle"]:
        message = record["message"]
        values = (message.get("linear_velocity_mps"), message.get("angular_velocity_radps"))
        if (message.get("state") == 1 and message.get("fault_code") == 0 and
                all(type(value) in (int, float) and math.isfinite(value) for value in values) and
                first_active_wall is not None and observed_ns(message) >= first_active_wall):
            vehicle_running += 1
    if not vehicle_running:
        failures.append("no independent RUNNING/fault-free vehicle observation in window")
    return {"schema_version": 1, "verdict": "FAIL" if failures else "PASS",
            "failures": failures, "manifest_sha256": digest,
            "input_sha256": {name: sha256(data) for name, data in sorted(inputs.items())},
            "window": window, "expected_gid": target_gid, "previous_gid": previous,
            "total_records": totals, "window_records": {name: len(records) for name, records in selected.items()},
            "planning_source_records": len(sources), "sole_gid_records": len(admitted),
            "mismatched_gid_records": mismatch, "accepted_first": baseline_count,
            "accepted_last": counts[-1] if counts else None,
            "sequence_first": baseline_sequence, "sequence_last": sequences[-1] if sequences else None,
            "correlated_active_records": len(active), "first_active_system_ns": first_active_wall,
            "can_counts": can_counts, "correlated_nonzero_can": correlated_nonzero,
            "vehicle_running_records": vehicle_running,
            "interpretation": "fixed-window facts, not atomic state or per-task ACK"}


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    commands = parser.add_subparsers(dest="command", required=True)
    freeze = commands.add_parser("freeze", help="only after SIGINT and wait")
    freeze.add_argument("--capture-dir", required=True)
    freeze.add_argument("--snapshot-dir", required=True)
    freeze.add_argument("--window", required=True)
    freeze.add_argument("--can-map", required=True)
    freeze.add_argument("--expected-gid", required=True)
    freeze.add_argument("--previous-gid")
    freeze.add_argument("--waited-exit", required=True, type=int)
    replay = commands.add_parser("replay", help="reads frozen SHA-bound files only")
    replay.add_argument("--snapshot-dir", required=True)
    replay.add_argument("--manifest-sha256")
    replay.add_argument("--output", help="new durable JSON; stdout is also emitted")
    arguments = parser.parse_args(argv)
    try:
        if arguments.command == "freeze":
            digest = freeze_capture(arguments.capture_dir, arguments.snapshot_dir,
                                    arguments.window, arguments.can_map, arguments.expected_gid,
                                    arguments.waited_exit, arguments.previous_gid)
            result = {"frozen_manifest_sha256": digest}
            status = 0
        else:
            result = evaluate_snapshot(arguments.snapshot_dir, arguments.manifest_sha256)
            status = 0 if result["verdict"] == "PASS" else 1
            if arguments.output:
                publish_json(arguments.output, result)
        sys.stdout.write(json_bytes(result).decode("utf-8"))
        sys.stdout.flush()
        return status
    except (OSError, ValueError, KeyError, TypeError, ImportError) as error:
        print("INVALID evidence: " + str(error), file=sys.stderr, flush=True)
        return 2


if __name__ == "__main__":
    raise SystemExit(main())
