import importlib.util
import os
import tempfile

import pytest

from control_link_bringup.profile_bootstrap import (
	load_adas_replay_config,
	load_record_topics,
	require_up_vcan_interface,
)
from control_link_bringup import profile_bootstrap


def _load_replay_module():
	path = os.environ["CONTROL_LINK_REPLAY_LAUNCH_PATH"]
	spec = importlib.util.spec_from_file_location("control_link_replay_launch", path)
	module = importlib.util.module_from_spec(spec)
	spec.loader.exec_module(module)
	return module


def _profile():
	return {
		"profile_id": "adas",
		"record_topics": [
			"/control_link/input/planning",
			"/diagnostics",
		],
		"replay": {
			"input_namespace": "/replay/control_link",
		},
	}


def test_profile_replay_and_record_topics_are_strict():
	profile = _profile()
	assert load_record_topics(profile, "profile.yaml") == profile["record_topics"]
	assert load_adas_replay_config(profile, "profile.yaml") == profile["replay"]

	profile["record_topics"].append("/diagnostics")
	with pytest.raises(RuntimeError, match="duplicate record Topic"):
		load_record_topics(profile, "profile.yaml")

	profile = _profile()
	profile["replay"]["input_namespace"] = False
	with pytest.raises(RuntimeError, match="expected=non-empty absolute ROS namespace"):
		load_adas_replay_config(profile, "profile.yaml")

	profile = _profile()
	profile["replay"]["unexpected"] = False
	with pytest.raises(RuntimeError, match="unknown field"):
		load_adas_replay_config(profile, "profile.yaml")


def test_player_command_remaps_every_topic_into_replay_namespace():
	module = _load_replay_module()
	topics = _profile()["record_topics"]
	command = module._build_player_command(
		"/tmp/example_bag",
		2.0,
		topics,
		"/control_link",
		"/replay/control_link",
	)
	remap_index = command.index("--remap")
	assert command[remap_index + 1:] == [
		"/control_link/input/planning:=/replay/control_link/input/planning",
		"/diagnostics:=/replay/control_link/diagnostics",
	]
	assert all(
		mapping.split(":=", 1)[1].startswith("/replay/control_link/")
		for mapping in command[remap_index + 1:]
	)
	with pytest.raises(RuntimeError, match="must differ from the live Gateway namespace"):
		module._build_player_command(
			"/tmp/example_bag",
			1.0,
			topics,
			"/control_link",
			"/control_link",
		)
	with pytest.raises(RuntimeError, match="collide after replay namespace remapping"):
		module._build_player_command(
			"/tmp/example_bag",
			1.0,
			["/control_link/diagnostics", "/diagnostics"],
			"/control_link",
			"/replay/control_link",
		)


def test_bag_and_vcan_guards_fail_closed(monkeypatch):
	module = _load_replay_module()
	with tempfile.TemporaryDirectory() as temporary_directory:
		with pytest.raises(RuntimeError, match="missing metadata.yaml"):
			module._canonical_bag_directory(temporary_directory)

	with pytest.raises(RuntimeError, match="restricted to a vcanN"):
		require_up_vcan_interface("can0")

	class FakeIpResult:
		stdout = '[{"flags":["UP"],"linkinfo":{"info_kind":"can"}}]'

	monkeypatch.setattr(
		profile_bootstrap.subprocess,
		"run",
		lambda *args, **kwargs: FakeIpResult(),
	)
	with pytest.raises(RuntimeError, match="not an actual vcan link"):
		require_up_vcan_interface("vcan7")
