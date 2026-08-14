import datetime
import os
import re

from ament_index_python.packages import get_package_share_directory
from control_link_bringup.profile_bootstrap import (
	load_fastdds_profile_path,
	load_record_topics,
	load_yaml_mapping,
)
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, LogInfo, OpaqueFunction
from launch.substitutions import LaunchConfiguration


def _non_negative_integer(context, argument_name):
	value = LaunchConfiguration(argument_name).perform(context)
	if not value.isdecimal():
		raise RuntimeError(
			argument_name + " must be a non-negative integer; actual=" + value)
	return int(value)


def _launch_recorder(context, *args, **kwargs):
	profile_id = LaunchConfiguration("profile").perform(context)
	if profile_id not in ("robot", "adas"):
		raise RuntimeError(
			"profile must be one of: robot, adas; actual=" + profile_id)

	package_share = get_package_share_directory("control_link_bringup")
	installed_config_root = os.path.join(package_share, "config")
	installed_profile_path = os.path.join(
		installed_config_root,
		profile_id,
		profile_id + "_profile.yaml")
	profile_path = os.path.realpath(installed_profile_path)
	config_root = os.path.dirname(os.path.dirname(profile_path))
	profile = load_yaml_mapping(profile_path, profile_id + " Profile")
	if profile.get("profile_id") != profile_id:
		raise RuntimeError(
			"profile_id mismatch; expected=" + profile_id + "; actual=" +
			str(profile.get("profile_id")))

	topics = load_record_topics(profile, profile_path)
	use_sim_time = profile.get("use_sim_time")
	if not isinstance(use_sim_time, bool):
		raise RuntimeError(
			profile_path +
			":use_sim_time: wrong YAML type; expected=bool")
	fastdds_profile_path = load_fastdds_profile_path(profile_path, config_root)

	output_dir = LaunchConfiguration("output_dir").perform(context)
	if not output_dir:
		timestamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
		output_dir = os.path.join(
			os.getcwd(), "control_link_" + profile_id + "_" + timestamp)
	output_dir = os.path.abspath(os.path.expanduser(output_dir))
	if os.path.exists(output_dir):
		raise RuntimeError(
			"output_dir must not already exist; actual=" + output_dir)
	output_parent = os.path.dirname(output_dir)
	if not os.path.isdir(output_parent):
		raise RuntimeError(
			"output_dir parent must be an existing directory; actual=" +
			output_parent)

	storage_id = LaunchConfiguration("storage_id").perform(context)
	if re.fullmatch(r"[A-Za-z0-9_]+", storage_id) is None:
		raise RuntimeError(
			"storage_id must contain only letters, digits, or underscore; actual=" +
			storage_id)
	max_bag_duration_s = _non_negative_integer(context, "max_bag_duration_s")

	command = [
		"ros2", "bag", "record",
		"--output", output_dir,
		"--storage", storage_id,
	]
	if use_sim_time:
		command.append("--use-sim-time")
	if max_bag_duration_s > 0:
		# rosbag2 的 duration 参数负责分包，不负责让 recorder 自动退出
		command.extend(["--max-bag-duration", str(max_bag_duration_s)])
	if any("/_" in topic for topic in topics):
		command.append("--include-hidden-topics")
	command.extend(topics)

	participant_env = {
		"RMW_IMPLEMENTATION": "rmw_fastrtps_cpp",
		"RMW_FASTRTPS_USE_QOS_FROM_XML": "0",
		"FASTRTPS_DEFAULT_PROFILES_FILE": fastdds_profile_path,
	}
	return [
		LogInfo(msg="Recording ControlLink " + profile_id + " topics to " + output_dir),
		ExecuteProcess(
			cmd=command,
			name="control_link_rosbag2_recorder",
			output="screen",
			emulate_tty=True,
			additional_env=participant_env,
		),
	]


def generate_launch_description():
	return LaunchDescription([
		DeclareLaunchArgument(
			"profile",
			default_value="robot",
			description="Profile whose record_topics list is recorded: robot or adas",
		),
		DeclareLaunchArgument(
			"output_dir",
			default_value="",
			description="New rosbag2 output directory; empty selects a timestamped cwd path",
		),
		DeclareLaunchArgument(
			"storage_id",
			default_value="sqlite3",
			description="Installed rosbag2 storage plugin identifier",
		),
		DeclareLaunchArgument(
			"max_bag_duration_s",
			default_value="0",
			description="Split bag files after this many seconds; zero disables splitting",
		),
		OpaqueFunction(function=_launch_recorder),
	])
