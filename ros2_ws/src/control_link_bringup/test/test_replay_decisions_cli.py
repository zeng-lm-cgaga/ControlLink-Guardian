import json
import os
import pathlib
import subprocess


EXECUTABLE = pathlib.Path(os.environ["CONTROL_LINK_REPLAY_DECISIONS"])
PROFILE = pathlib.Path(os.environ["CONTROL_LINK_TEST_ROBOT_PROFILE"])
CONFIG_ROOT = pathlib.Path(os.environ["CONTROL_LINK_TEST_CONFIG_ROOT"])
FIXTURE = pathlib.Path(os.environ["CONTROL_LINK_TEST_DECISION_TRACE"])


def run_replay(trace: pathlib.Path, result: pathlib.Path, repeat: str = "1"):
	return subprocess.run(
		[
			str(EXECUTABLE),
			"--profile-path",
			str(PROFILE),
			"--config-root",
			str(CONFIG_ROOT),
			"--trace",
			str(trace),
			"--repeat",
			repeat,
			"--result",
			str(result),
		],
		check=False,
		capture_output=True,
		text=True,
	)


def read_records(path: pathlib.Path):
	return [json.loads(line) for line in path.read_text().splitlines()]


def write_records(path: pathlib.Path, records):
	path.write_text("\n".join(json.dumps(record, separators=(",", ":")) for record in records) + "\n")


def test_fixed_fixture_replays_one_hundred_times(tmp_path):
	result_path = tmp_path / "result.json"
	completed = run_replay(FIXTURE, result_path, "100")

	assert completed.returncode == 0, completed.stderr
	result = json.loads(result_path.read_text())
	assert result["valid"] is True
	assert result["matched"] is True
	assert result["completed_repetitions"] == 100
	assert result["event_count"] == 17
	assert result["result_count"] == 6


def test_divergence_has_distinct_exit_code_and_first_event(tmp_path):
	records = read_records(FIXTURE)
	changed = next(record for record in records if record["record_type"] == "result")
	changed["reason"] = 8
	trace_path = tmp_path / "diverged.jsonl"
	result_path = tmp_path / "result.json"
	write_records(trace_path, records)

	completed = run_replay(trace_path, result_path)

	assert completed.returncode == 4
	result = json.loads(result_path.read_text())
	assert result["valid"] is True
	assert result["matched"] is False
	assert result["first_difference_event_sequence"] == changed["event_sequence"]
	assert result["first_difference_field"] == "reason"


def test_invalid_identity_and_truncation_return_invalid_exit_code(tmp_path):
	records = read_records(FIXTURE)
	records[0]["decision_config_hash"] = "0" * 64
	identity_path = tmp_path / "identity.jsonl"
	identity_result = tmp_path / "identity_result.json"
	write_records(identity_path, records)
	identity = run_replay(identity_path, identity_result)

	assert identity.returncode == 3
	assert json.loads(identity_result.read_text())["error_category"] == "invalid_replay"

	truncated_path = tmp_path / "truncated.jsonl"
	truncated_result = tmp_path / "truncated_result.json"
	write_records(truncated_path, read_records(FIXTURE)[:-1])
	truncated = run_replay(truncated_path, truncated_result)

	assert truncated.returncode == 3
	assert json.loads(truncated_result.read_text())["error_category"] == "invalid_trace"


def test_usage_error_does_not_create_a_result(tmp_path):
	result_path = tmp_path / "unused.json"
	completed = subprocess.run(
		[str(EXECUTABLE), "--result", str(result_path)],
		check=False,
		capture_output=True,
		text=True,
	)

	assert completed.returncode == 2
	assert not result_path.exists()
