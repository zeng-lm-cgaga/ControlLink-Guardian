import math
import os

from ament_index_python.packages import get_package_share_directory
from control_link_bringup.profile_bootstrap import (
	load_adas_replay_config,
	load_fastdds_profile_path,
	load_record_topics,
	load_yaml_mapping,
	require_up_vcan_interface,
)
import launch
from launch.actions import (
	DeclareLaunchArgument,
	EmitEvent,
	ExecuteProcess,
	IncludeLaunchDescription,
	LogInfo,
	OpaqueFunction,
	RegisterEventHandler,
)
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _bool_argument(context, name):
	value = LaunchConfiguration(name).perform(context).lower()
	if value not in ("true", "false"):
		raise RuntimeError(name + " must be true or false; actual=" + value)
	return value == "true"


def _positive_float_argument(context, name):
	value = LaunchConfiguration(name).perform(context)
	try:
		parsed = float(value)
	except ValueError as exception:
		raise RuntimeError(name + " must be a finite positive number") from exception
	if not math.isfinite(parsed) or parsed <= 0.0:
		raise RuntimeError(name + " must be a finite positive number")
	return parsed


def _safe_replay_topic(topic, live_namespace, replay_namespace):
	if topic == live_namespace:
		raise RuntimeError("record Topic cannot equal the live namespace: " + topic)
	if topic.startswith(live_namespace + "/"):
		return replay_namespace + topic[len(live_namespace):]
	# diagnostics 等旁路 Topic 也必须落入隔离命名空间，不能污染 live Graph
	return replay_namespace + topic


def _build_player_command(
	bag_path,
	rate,
	topics,
	live_namespace,
	replay_namespace,
):
	if replay_namespace == live_namespace:
		raise RuntimeError(
			"replay namespace must differ from the live Gateway namespace; actual=" +
			replay_namespace)
	replay_topics = [
		_safe_replay_topic(topic, live_namespace, replay_namespace)
		for topic in topics
	]
	if len(set(replay_topics)) != len(replay_topics):
		raise RuntimeError(
			"record Topics collide after replay namespace remapping")
	remaps = [
		topic + ":=" + replay_topic
		for topic, replay_topic in zip(topics, replay_topics)
	]
	return [
		"ros2", "bag", "play", bag_path,
		"--rate", format(rate, ".17g"),
		"--disable-keyboard-controls",
		"--topics", *topics,
		"--remap", *remaps,
	]


def _canonical_bag_directory(candidate):
	bag_path = os.path.realpath(os.path.abspath(os.path.expanduser(candidate)))
	if not os.path.isdir(bag_path):
		raise RuntimeError(
			"bag_path must be an existing rosbag2 directory; actual=" + bag_path)
	metadata_path = os.path.join(bag_path, "metadata.yaml")
	if not os.path.isfile(metadata_path):
		raise RuntimeError(
			"bag_path is missing metadata.yaml; actual=" + bag_path)
	return bag_path


def _after_readiness_exit(event, _context, player):
	if event.returncode != 0:
		return [
			LogInfo(msg="ERROR: Gateway readiness failed; rosbag playback will not start"),
			EmitEvent(event=Shutdown(reason="Gateway replay readiness failed")),
		]
	return [player]


def _launch_replay(context, *args, **kwargs):
	package_share = get_package_share_directory("control_link_bringup")
	profile_path = os.path.realpath(
		os.path.join(package_share, "config", "adas", "adas_profile.yaml"))
	config_root = os.path.dirname(os.path.dirname(profile_path))
	profile = load_yaml_mapping(profile_path, "adas Profile")
	replay = load_adas_replay_config(profile, profile_path)
	topics = load_record_topics(profile, profile_path)
	fastdds_profile_path = load_fastdds_profile_path(profile_path, config_root)

	bag_path_argument = LaunchConfiguration("bag_path").perform(context)
	if not bag_path_argument:
		raise RuntimeError("bag_path is required")
	bag_path = _canonical_bag_directory(bag_path_argument)
	rate = _positive_float_argument(context, "rate")
	allow_can_tx = _bool_argument(context, "allow_can_tx")
	live_namespace = "/control_link"
	replay_namespace = replay["input_namespace"]
	player_command = _build_player_command(
		bag_path,
		rate,
		topics,
		live_namespace,
		replay_namespace,
	)
	participant_env = {
		"RMW_IMPLEMENTATION": "rmw_fastrtps_cpp",
		"RMW_FASTRTPS_USE_QOS_FROM_XML": "0",
		"FASTRTPS_DEFAULT_PROFILES_FILE": fastdds_profile_path,
	}
	player = ExecuteProcess(
		cmd=player_command,
		name="control_link_rosbag2_player",
		output="screen",
		emulate_tty=True,
		additional_env=participant_env,
	)
	player_exit = RegisterEventHandler(
		OnProcessExit(
			target_action=player,
			on_exit=[EmitEvent(event=Shutdown(reason="rosbag2 replay completed"))],
		))

	if not allow_can_tx:
		return [
			LogInfo(
				msg="Safe replay only: all recorded Topics are isolated under " +
					replay_namespace + "; no Gateway, adapter, or CAN process is started"),
			player_exit,
			player,
		]

	adapter = profile.get("adapter")
	if not isinstance(adapter, dict):
		raise RuntimeError(profile_path + ":adapter: expected=map")
	configured_interface = adapter.get("interface")
	if not isinstance(configured_interface, str) or not configured_interface:
		raise RuntimeError(profile_path + ":adapter.interface: expected=string")
	requested_interface = LaunchConfiguration("can_interface").perform(context)
	if not requested_interface:
		requested_interface = configured_interface
	if requested_interface != configured_interface:
		raise RuntimeError(
			"can_interface must match ADAS Profile; expected=" +
			configured_interface + "; actual=" + requested_interface)
	require_up_vcan_interface(requested_interface)

	demo = IncludeLaunchDescription(
		PythonLaunchDescriptionSource(
			os.path.join(package_share, "launch", "adas_vcan_demo.launch.py")),
		launch_arguments={
			"can_interface": requested_interface,
			"start_source": "false",
			"start_lifecycle_activator": "false",
		}.items(),
	)
	common_parameters = {
		"profile_path": profile_path,
		"config_root": config_root,
		"use_sim_time": False,
	}
	bridge = Node(
		package="control_link_bringup",
		executable="replay_input_bridge",
		name="replay_input_bridge",
		namespace="control_link",
		output="screen",
		emulate_tty=True,
		additional_env=participant_env,
		parameters=[{
			**common_parameters,
			"replay_namespace": replay_namespace,
		}],
	)
	readiness = Node(
		package="control_link_bringup",
		executable="gateway_lifecycle_activator",
		name="replay_gateway_readiness",
		output="screen",
		emulate_tty=True,
		additional_env=participant_env,
		parameters=[{
			"target_fqn": "/control_link/gateway",
			"timeout_ms": 45000,
		}],
	)
	readiness_exit = RegisterEventHandler(
		OnProcessExit(
			target_action=readiness,
			on_exit=lambda event, launch_context: _after_readiness_exit(
				event, launch_context, player),
		))
	return [
		LogInfo(
			msg="Explicit vcan replay enabled: bag inputs will be retimed through " +
				replay_namespace + " before entering the live Gateway"),
		demo,
		bridge,
		readiness,
		readiness_exit,
		player_exit,
	]


def generate_launch_description():
	return launch.LaunchDescription([
		DeclareLaunchArgument(
			"bag_path",
			default_value="",
			description="Existing rosbag2 directory to replay",
		),
		DeclareLaunchArgument(
			"rate",
			default_value="1.0",
			description="Positive rosbag2 playback rate",
		),
		DeclareLaunchArgument(
			"allow_can_tx",
			default_value="false",
			description="Explicitly enable the ADAS closed loop on vcan only",
		),
		DeclareLaunchArgument(
			"can_interface",
			default_value="",
			description="Existing UP vcanN interface; ignored unless CAN TX is enabled",
		),
		OpaqueFunction(function=_launch_replay),
	])
