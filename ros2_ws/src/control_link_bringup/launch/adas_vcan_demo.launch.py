import math
import os

from ament_index_python.packages import get_package_share_directory
from control_link_bringup.profile_bootstrap import (
	load_fastdds_profile_path,
	load_yaml_mapping,
)
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, OpaqueFunction, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _bool_argument(context, name):
	value = LaunchConfiguration(name).perform(context).lower()
	if value not in ("true", "false"):
		raise RuntimeError(name + " must be true or false; actual=" + value)
	return value == "true"


def _byte_argument(context, name):
	value = LaunchConfiguration(name).perform(context)
	if not value.isdecimal() or int(value) > 255:
		raise RuntimeError(name + " must be an integer in [0,255]; actual=" + value)
	return int(value)


def _positive_float_argument(context, name):
	value = LaunchConfiguration(name).perform(context)
	try:
		parsed = float(value)
	except ValueError as exception:
		raise RuntimeError(name + " must be a finite positive number") from exception
	if not math.isfinite(parsed) or parsed <= 0.0:
		raise RuntimeError(name + " must be a finite positive number")
	return parsed


def _launch_adas_vcan(context, *args, **kwargs):
	package_share = get_package_share_directory("control_link_bringup")
	profile_path = os.path.realpath(
		os.path.join(package_share, "config", "adas", "adas_profile.yaml"))
	# symlink-install 下 Profile 会落到源码 config 树，间接引用必须使用同一棵真实目录
	config_root = os.path.dirname(os.path.dirname(profile_path))
	profile = load_yaml_mapping(profile_path, "adas Profile")
	if profile.get("profile_id") != "adas":
		raise RuntimeError(
			"profile_id mismatch; expected=adas; actual=" +
			str(profile.get("profile_id")))

	adapter = profile.get("adapter")
	if not isinstance(adapter, dict):
		raise RuntimeError(profile_path + ":adapter: expected=map")
	configured_interface = adapter.get("interface")
	if not isinstance(configured_interface, str) or not configured_interface:
		raise RuntimeError(
			profile_path + ":adapter.interface: expected=non-empty string")
	requested_interface = LaunchConfiguration("can_interface").perform(context)
	if not requested_interface:
		requested_interface = configured_interface
	if requested_interface != configured_interface:
		raise RuntimeError(
			"can_interface must match ADAS Profile adapter.interface; expected=" +
			configured_interface + "; actual=" + requested_interface)

	fastdds_profile_path = load_fastdds_profile_path(profile_path, config_root)
	participant_env = {
		"RMW_IMPLEMENTATION": "rmw_fastrtps_cpp",
		"RMW_FASTRTPS_USE_QOS_FROM_XML": "0",
		"FASTRTPS_DEFAULT_PROFILES_FILE": fastdds_profile_path,
	}
	common_parameters = {
		"profile_path": profile_path,
		"config_root": config_root,
		"use_sim_time": False,
	}

	start_source = _bool_argument(context, "start_source")
	start_secondary_source = _bool_argument(context, "start_secondary_source")
	source_id = LaunchConfiguration("source_id").perform(context)
	secondary_source_id = LaunchConfiguration("secondary_source_id").perform(context)
	enabled_sources = profile.get("enabled_sources")
	if not isinstance(enabled_sources, list) or not enabled_sources:
		raise RuntimeError(profile_path + ":enabled_sources: expected=non-empty list")
	if not source_id:
		source_id = enabled_sources[0]
	if source_id not in enabled_sources:
		raise RuntimeError(
			"source_id must be enabled by ADAS Profile; actual=" + source_id)
	if start_secondary_source:
		if not start_source:
			raise RuntimeError(
				"start_secondary_source requires start_source=true")
		if secondary_source_id not in enabled_sources:
			raise RuntimeError(
				"secondary_source_id must be enabled by ADAS Profile; actual=" +
				secondary_source_id)
		if secondary_source_id == source_id:
			raise RuntimeError(
				"secondary_source_id must differ from source_id")
		secondary_source_delay_seconds = _positive_float_argument(
			context,
			"secondary_source_delay_seconds")

	simulator_parameters = {
		**common_parameters,
		"drop_state": _bool_argument(context, "drop_state"),
		"corrupt_crc": _bool_argument(context, "corrupt_crc"),
		"freeze_counter": _bool_argument(context, "freeze_counter"),
		"fault_code": _byte_argument(context, "fault_code"),
	}

	# simulator 负责 CAN 侧执行端，adapter 负责 ROS2 canonical 与 VehicleState 边界
	simulator = Node(
		package="control_link_bringup",
		executable="vcan_vehicle_simulator",
		name="vehicle_simulator",
		output="screen",
		emulate_tty=True,
		additional_env=participant_env,
		parameters=[simulator_parameters],
	)
	adapter_node = Node(
		package="control_link_adapters",
		executable="socketcan_adapter_node",
		name="vehicle_adapter",
		namespace="control_link",
		output="screen",
		emulate_tty=True,
		additional_env=participant_env,
		parameters=[common_parameters],
	)
	source_node = Node(
		package="control_link_adapters",
		executable="mock_control_source_node",
		name="mock_source_" + source_id,
		namespace="control_link",
		output="screen",
		emulate_tty=True,
		additional_env=participant_env,
		parameters=[{**common_parameters, "source_id": source_id}],
	)
	secondary_source_node = Node(
		package="control_link_adapters",
		executable="mock_control_source_node",
		name="mock_source_" + secondary_source_id,
		namespace="control_link",
		output="screen",
		emulate_tty=True,
		additional_env=participant_env,
		parameters=[{**common_parameters, "source_id": secondary_source_id}],
	)

	actions = [
		LogInfo(msg="ADAS vcan demo requires an existing and up " + requested_interface),
		simulator,
		adapter_node,
	]
	if start_source:
		actions.append(source_node)
		if start_secondary_source:
			actions.append(
				TimerAction(
					period=secondary_source_delay_seconds,
					actions=[secondary_source_node]))
		actions.extend([
		Node(
			package="control_link_gateway",
			executable="control_link_gateway_node",
			name="gateway",
			namespace="control_link",
			output="screen",
			emulate_tty=True,
			additional_env=participant_env,
			parameters=[common_parameters],
		),
		Node(
			package="control_link_bringup",
			executable="gateway_lifecycle_activator",
			name="gateway_lifecycle_activator",
			output="screen",
			additional_env=participant_env,
			parameters=[{
				"target_fqn": "/control_link/gateway",
				"timeout_ms": 45000,
			}],
		),
	])
	return actions


def generate_launch_description():
	return LaunchDescription([
		DeclareLaunchArgument(
			"can_interface",
			default_value="",
			description="Existing Linux SocketCAN/vcan interface; must match ADAS Profile",
		),
		DeclareLaunchArgument(
			"start_source",
			default_value="true",
			description="Start one Contract-bound mock planning source",
		),
		DeclareLaunchArgument(
			"source_id",
			default_value="planning",
			description="Enabled SourcePolicy source used by the mock source",
		),
		DeclareLaunchArgument(
			"start_secondary_source",
			default_value="false",
			description="Start a second Contract-bound source after a delay for switch observation",
		),
		DeclareLaunchArgument(
			"secondary_source_id",
			default_value="teleop",
			description="Enabled source started by the optional delayed source action",
		),
		DeclareLaunchArgument(
			"secondary_source_delay_seconds",
			default_value="5.0",
			description="Delay before starting the optional secondary source",
		),
		DeclareLaunchArgument(
			"drop_state",
			default_value="false",
			description="Stop simulator state frames after startup",
		),
		DeclareLaunchArgument(
			"corrupt_crc",
			default_value="false",
			description="Corrupt simulator state-frame CRC",
		),
		DeclareLaunchArgument(
			"freeze_counter",
			default_value="false",
			description="Hold simulator state counter to reproduce duplicates",
		),
		DeclareLaunchArgument(
			"fault_code",
			default_value="0",
			description="Inject a demo vehicle fault code in the range 0..255",
		),
		OpaqueFunction(function=_launch_adas_vcan),
	])
