#!/usr/bin/env python3

import argparse
import csv
import hashlib
import json
from pathlib import Path

import yaml


COMPARABLE_MANIFEST_FIELDS = (
	"profile",
	"git_commit",
	"git_dirty",
	"build_type",
	"rmw",
	"can_interface",
	"warmup_seconds",
	"measurement_seconds",
	"minimum_output_ticks",
	"source_switch_scenario",
)

CONTRACT_SNAPSHOT_FILES = (
	"adas_profile.yaml",
	"can_signal_map.yaml",
	"fastdds_default_profiles.xml",
	"gateway_contract.yaml",
	"source_policy.yaml",
)

METRICS = (
	("gateway_cpu_mean_percent", ("resources", "gateway", "cpu_percent", "mean")),
	("gateway_cpu_p95_percent", ("resources", "gateway", "cpu_percent", "p95")),
	("gateway_rss_mean_kib", ("resources", "gateway", "rss_kib", "mean")),
	("canonical_abs_jitter_mean_ms", ("metrics", "canonical_delivery_abs_jitter_ms", "mean")),
	("canonical_abs_jitter_p95_ms", ("metrics", "canonical_delivery_abs_jitter_ms", "p95")),
	("canonical_abs_jitter_p99_ms", ("metrics", "canonical_delivery_abs_jitter_ms", "p99")),
	("canonical_interval_mean_ms", ("metrics", "canonical_delivery_interval_ms", "mean")),
	("callback_to_output_p95_ms", ("metrics", "observer_callback_to_output_ms", "p95")),
	("can_round_trip_p95_ms", ("metrics", "can_observer_round_trip_ms", "p95")),
	("observed_output_ticks", ("protocol", "observed_output_ticks")),
)


def parse_arguments():
	parser = argparse.ArgumentParser(
		description="Compare one controlled decision-trace-off/on performance pair")
	parser.add_argument("--off-run", required=True, type=Path)
	parser.add_argument("--on-run", required=True, type=Path)
	parser.add_argument("--output-dir", required=True, type=Path)
	return parser.parse_args()


def load_yaml(path):
	with path.open("r", encoding="utf-8") as stream:
		value = yaml.safe_load(stream)
	if not isinstance(value, dict):
		raise ValueError(f"{path}: expected a YAML mapping")
	return value


def load_json(path):
	with path.open("r", encoding="utf-8") as stream:
		value = json.load(stream)
	if not isinstance(value, dict):
		raise ValueError(f"{path}: expected a JSON object")
	return value


def nested_number(value, path, source):
	current = value
	for field in path:
		if not isinstance(current, dict) or field not in current:
			raise ValueError(f"{source}: missing numeric field {'.'.join(path)}")
		current = current[field]
	if isinstance(current, bool) or not isinstance(current, (int, float)):
		raise ValueError(f"{source}: field {'.'.join(path)} is not numeric")
	return current


def sha256(path):
	digest = hashlib.sha256()
	with path.open("rb") as stream:
		for chunk in iter(lambda: stream.read(1024 * 1024), b""):
			digest.update(chunk)
	return digest.hexdigest()


def contract_snapshot_hashes(run_dir):
	snapshot_dir = run_dir / "contract_snapshot"
	if not snapshot_dir.is_dir():
		raise ValueError(f"{snapshot_dir}: contract snapshot directory is missing")
	actual_files = sorted(
		path.name for path in snapshot_dir.iterdir() if path.is_file())
	if actual_files != sorted(CONTRACT_SNAPSHOT_FILES):
		raise ValueError(
			f"{snapshot_dir}: unexpected snapshot files; actual={actual_files!r}")
	return {
		name: sha256(snapshot_dir / name)
		for name in CONTRACT_SNAPSHOT_FILES
	}


def last_json_record(path):
	last_line = None
	with path.open("r", encoding="utf-8") as stream:
		for line in stream:
			if line.strip():
				last_line = line
	if last_line is None:
		raise ValueError(f"{path}: trace is empty")
	value = json.loads(last_line)
	if not isinstance(value, dict):
		raise ValueError(f"{path}: final trace record is not an object")
	return value


def validate_run(run_dir, expected_trace_enabled):
	manifest_path = run_dir / "run_manifest.yaml"
	summary_path = run_dir / "performance" / "summary.json"
	manifest = load_yaml(manifest_path)
	summary = load_json(summary_path)
	if manifest.get("status") != "PASS":
		raise ValueError(f"{manifest_path}: run status is not PASS")
	if manifest.get("decision_trace_enabled") is not expected_trace_enabled:
		raise ValueError(
			f"{manifest_path}: decision_trace_enabled does not match comparison side")
	if manifest.get("measurement_exit_code") != 0:
		raise ValueError(f"{manifest_path}: measurement_exit_code is not zero")
	if summary.get("completed") is not True:
		raise ValueError(f"{summary_path}: measurement is incomplete")
	protocol = summary.get("protocol")
	if not isinstance(protocol, dict) or protocol.get("protocol_conformant_parameters") is not True:
		raise ValueError(f"{summary_path}: measurement protocol is not conformant")
	return manifest, summary, manifest_path, summary_path


def main():
	args = parse_arguments()
	off_run = args.off_run.resolve(strict=True)
	on_run = args.on_run.resolve(strict=True)
	if off_run == on_run:
		raise ValueError("off-run and on-run must be different directories")

	off_manifest, off_summary, off_manifest_path, off_summary_path = validate_run(
		off_run, False)
	on_manifest, on_summary, on_manifest_path, on_summary_path = validate_run(
		on_run, True)
	for field in COMPARABLE_MANIFEST_FIELDS:
		if off_manifest.get(field) != on_manifest.get(field):
			raise ValueError(
				f"run manifests differ at {field}: "
				f"off={off_manifest.get(field)!r}, on={on_manifest.get(field)!r}")
	off_snapshot_hashes = contract_snapshot_hashes(off_run)
	on_snapshot_hashes = contract_snapshot_hashes(on_run)
	if off_snapshot_hashes != on_snapshot_hashes:
		raise ValueError(
			"off/on runs used different Contract, Profile, CAN map or FastDDS snapshots")

	trace_path = on_run / "decision_trace" / "trace.jsonl"
	replay_path = on_run / "decision_trace" / "replay_result.json"
	footer = last_json_record(trace_path)
	replay = load_json(replay_path)
	if footer.get("record_type") != "footer" or footer.get("trace_valid") is not True:
		raise ValueError(f"{trace_path}: final footer does not mark a valid trace")
	if footer.get("trace_overflow") is not False:
		raise ValueError(f"{trace_path}: trace footer reports overflow")
	if replay.get("valid") is not True or replay.get("matched") is not True:
		raise ValueError(f"{replay_path}: live trace replay did not match")

	rows = []
	for name, field_path in METRICS:
		off_value = nested_number(off_summary, field_path, off_summary_path)
		on_value = nested_number(on_summary, field_path, on_summary_path)
		delta = on_value - off_value
		relative_percent = None if off_value == 0 else delta / off_value * 100.0
		rows.append({
			"metric": name,
			"trace_off": off_value,
			"trace_on": on_value,
			"absolute_delta": delta,
			"relative_delta_percent": relative_percent,
		})

	output_dir = args.output_dir.resolve()
	output_dir.mkdir(parents=True, exist_ok=True)
	csv_path = output_dir / "trace_overhead.csv"
	json_path = output_dir / "comparison.json"
	for output in (csv_path, json_path):
		if output.exists():
			raise FileExistsError(f"refusing to overwrite existing output: {output}")

	with csv_path.open("w", encoding="utf-8", newline="") as stream:
		writer = csv.DictWriter(stream, fieldnames=rows[0].keys())
		writer.writeheader()
		writer.writerows(rows)

	result = {
		"schema_version": 1,
		"valid": True,
		"scope": "VM_ONLY_ADAS_EXTERNAL_OBSERVER_SINGLE_CONTROLLED_PAIR",
		"off_run": str(off_run),
		"on_run": str(on_run),
		"comparable_manifest_fields": {
			field: off_manifest[field] for field in COMPARABLE_MANIFEST_FIELDS
		},
		"metrics": rows,
		"trace": {
			"size_bytes": trace_path.stat().st_size,
			"event_count": footer.get("event_count"),
			"result_count": footer.get("result_count"),
			"trace_valid": footer.get("trace_valid"),
			"trace_overflow": footer.get("trace_overflow"),
			"replay_matched": replay.get("matched"),
		},
		"source_sha256": {
			"off_manifest": sha256(off_manifest_path),
			"off_summary": sha256(off_summary_path),
			"on_manifest": sha256(on_manifest_path),
			"on_summary": sha256(on_summary_path),
			"trace": sha256(trace_path),
			"replay_result": sha256(replay_path),
			"contract_snapshot": off_snapshot_hashes,
		},
		"limitations": [
			"single controlled run pair does not establish a population confidence interval",
			"VMware and vcan results do not represent physical CAN, real hardware, or hard real-time behavior",
			"callback and CAN timing are external observer arrival deltas",
		],
	}
	with json_path.open("w", encoding="utf-8") as stream:
		json.dump(result, stream, ensure_ascii=True, indent=2, sort_keys=True)
		stream.write("\n")

	print(f"Trace overhead comparison written to {output_dir}")


if __name__ == "__main__":
	main()
