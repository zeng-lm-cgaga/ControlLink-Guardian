import math
import os
import re

from ament_index_python.packages import get_package_prefix, get_package_share_directory
from control_link_bringup.profile_bootstrap import (
	load_fastdds_profile_path,
	load_yaml_mapping,
	require_up_vcan_interface,
)
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, LogInfo, OpaqueFunction, TimerAction
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


def _finite_float_argument(context, name):
	value = LaunchConfiguration(name).perform(context)
	try:
		parsed = float(value)
	except ValueError as exception:
		raise RuntimeError(name + " must be finite") from exception
	if not math.isfinite(parsed):
		raise RuntimeError(name + " must be finite")
	return parsed


def _positive_integer_argument(context, name):
	value = LaunchConfiguration(name).perform(context)
	if not value.isdecimal() or int(value) <= 0:
		raise RuntimeError(name + " must be a positive integer; actual=" + value)
	return int(value)


def _channel_argument(context, name):
	value = LaunchConfiguration(name).perform(context)
	if (not re.fullmatch(r"[A-Za-z0-9_.-]{1,64}", value) or
		".." in value):
		raise RuntimeError(
			name + " must match EdgeRuntime channel-name rules; actual=" + value)
	return value


def _require_adapters_executable(executable, build_option):
	path = os.path.join(
		get_package_prefix("control_link_adapters"),
		"lib", "control_link_adapters", executable)
	if not os.path.isfile(path) or not os.access(path, os.X_OK):
		raise RuntimeError(
			executable + " requires control_link_adapters built and installed with " +
			"-D" + build_option + "=ON; missing=" + path)
	return path


def _configuration_file_argument(context, name, default_path=""):
	value = LaunchConfiguration(name).perform(context) or default_path
	if not value or not os.path.isabs(value):
		raise RuntimeError(name + " must explicitly name an absolute readable file")
	path = os.path.realpath(value)
	if not os.path.isfile(path) or not os.access(path, os.R_OK):
		raise RuntimeError(name + " must name a readable file; actual=" + path)
	return path


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
	require_up_vcan_interface(requested_interface)

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
	gateway_parameters = {
		**common_parameters,
		"decision_trace_capture_config": LaunchConfiguration("decision_trace_capture_config").perform(context),
		"decision_trace_path": LaunchConfiguration(
			"decision_trace_path").perform(context),
		"decision_trace_queue_capacity": _positive_integer_argument(
			context, "decision_trace_queue_capacity"),
	}

	start_source = _bool_argument(context, "start_source")
	start_lifecycle_activator = _bool_argument(
		context, "start_lifecycle_activator")
	start_secondary_source = _bool_argument(context, "start_secondary_source")
	ingress_mode = LaunchConfiguration("ingress_mode").perform(context).lower()
	if ingress_mode not in ("real", "mock"):
		raise RuntimeError("ingress_mode must be real or mock; actual=" + ingress_mode)
	planning_transport = LaunchConfiguration("planning_transport").perform(context).lower()
	if planning_transport not in ("ros2", "edge", "someip", "grpc"):
		raise RuntimeError(
			"planning_transport must be ros2, edge, someip or grpc; actual=" + planning_transport)
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
	if ingress_mode == "real" and source_id != "planning":
		raise RuntimeError("real ingress fixes source binding to planning; actual=" + source_id)
	if ingress_mode == "real" and start_secondary_source:
		raise RuntimeError("start_secondary_source is mock-only; real teleop ingress is already enabled")
	if planning_transport in ("edge", "someip", "grpc") and source_id != "planning":
		raise RuntimeError(
			"planning_transport=" + planning_transport + " fixes source binding to planning; actual=" +
			source_id)
	if start_secondary_source:
		if not start_source and planning_transport == "ros2":
			raise RuntimeError(
				"start_secondary_source requires start_source=true")
		if secondary_source_id not in enabled_sources:
			raise RuntimeError(
				"secondary_source_id must be enabled by ADAS Profile; actual=" +
				secondary_source_id)
		if planning_transport in ("edge", "someip", "grpc") and secondary_source_id == "planning":
			raise RuntimeError(
				"planning_transport=" + planning_transport + " cannot start a second ROS planning publisher")
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
	if _bool_argument(context, "supervise_adapter"):
		adapter_node = ExecuteProcess(
			cmd=[os.path.join(get_package_prefix("control_link_bringup"), "lib", "control_link_bringup", "socketcan_supervisor"),
				"--policy", os.path.join(config_root, "adas", "adapter_supervisor.yaml"), "--",
				os.path.join(get_package_prefix("control_link_adapters"), "lib", "control_link_adapters", "socketcan_adapter_node"),
				"--ros-args", "-r", "__node:=vehicle_adapter", "-r", "__ns:=/control_link",
				"-p", "profile_path:=" + profile_path, "-p", "config_root:=" + config_root,
				"-p", "exit_on_fatal_can_io:=true"],
			output="screen", additional_env=participant_env)
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
	if planning_transport == "edge":
		_require_adapters_executable(
			"edge_planning_bridge_node", "CONTROL_LINK_ENABLE_EDGE_RUNTIME")
		command_channel = _channel_argument(context, "edge_command_channel")
		feedback_channel = _channel_argument(context, "edge_feedback_channel")
		edge_linear_velocity_mps = _finite_float_argument(
			context, "edge_linear_velocity_mps")
		edge_angular_velocity_radps = _finite_float_argument(
			context, "edge_angular_velocity_radps")
		bridge_node = Node(
			package="control_link_adapters",
			executable="edge_planning_bridge_node",
			name="edge_planning_bridge",
			namespace="control_link",
			output="screen",
			emulate_tty=True,
			additional_env=participant_env,
			parameters=[{
				**common_parameters,
				"command_channel": command_channel,
				"feedback_channel": feedback_channel,
				"wait_mode": LaunchConfiguration("edge_wait_mode"),
				"source_id": "planning",
			}],
		)
		edge_client_path = _require_adapters_executable(
			"edge_planning_client", "CONTROL_LINK_ENABLE_EDGE_RUNTIME")
		edge_supervisor_path = _require_adapters_executable(
			"edge_planning_supervisor", "CONTROL_LINK_ENABLE_EDGE_RUNTIME")
		edge_supervisor = ExecuteProcess(
			cmd=[
				edge_supervisor_path,
				"--command-channel", command_channel,
				"--feedback-channel", feedback_channel,
				"--wait-mode", LaunchConfiguration("edge_wait_mode"),
				"--client", edge_client_path,
				"--linear-velocity-mps",
				str(edge_linear_velocity_mps),
				"--angular-velocity-radps",
				str(edge_angular_velocity_radps),
			],
			output="screen",
			emulate_tty=True,
		)
	elif planning_transport == "grpc":
		_require_adapters_executable("grpc_planning_bridge_node", "CONTROL_LINK_ENABLE_GRPC")
		network_config_path = _configuration_file_argument(context, "network_config_path",
			os.path.join(config_root, "adas", "protocol_transports.yaml"))
		bridge_node = Node(package="control_link_adapters", executable="grpc_planning_bridge_node",
			name="grpc_planning_bridge", namespace="control_link", output="screen", emulate_tty=True,
			additional_env=participant_env,
			parameters=[{**common_parameters, "network_config_path": network_config_path, "source_id": "planning"}])
	elif planning_transport == "someip":
		_require_adapters_executable(
			"someip_planning_bridge_node", "CONTROL_LINK_ENABLE_SOMEIP")
		network_config_path = _configuration_file_argument(
			context, "network_config_path",
			os.path.join(config_root, "adas", "protocol_transports.yaml"))
		vsomeip_config_path = _configuration_file_argument(context, "vsomeip_config_path")
		bridge_node = Node(
			package="control_link_adapters",
			executable="someip_planning_bridge_node",
			name="someip_planning_bridge",
			namespace="control_link",
			output="screen",
			emulate_tty=True,
			additional_env={
				**participant_env,
				"VSOMEIP_CONFIGURATION": vsomeip_config_path,
			},
			parameters=[{
				**common_parameters,
				"network_config_path": network_config_path,
				"vsomeip_config_path": vsomeip_config_path,
				"source_id": "planning",
			}],
		)

	actions = [
		LogInfo(msg="ADAS vcan demo requires an existing and up " + requested_interface),
		simulator,
		adapter_node,
	]
	# 同一来源只装配一个 producer；真实 teleop 独立于 planning transport。
	if ingress_mode == "real":
		actions.append(Node(
			package="control_link_adapters", executable="twist_ingress_adapter_node",
			name="teleop_ingress", namespace="control_link", output="screen",
			emulate_tty=True, additional_env=participant_env,
			parameters=[{**common_parameters, "source_id": "teleop"}]))
	if planning_transport == "edge":
		actions.append(bridge_node)
		if start_source:
			actions.append(edge_supervisor)
	elif planning_transport in ("someip", "grpc"):
		actions.append(bridge_node)
		actions.append(LogInfo(msg=(
			planning_transport + " planning uses an external installed client; start_source does not start a ROS planning publisher")))
	elif start_source:
		if ingress_mode == "mock":
			actions.append(source_node)
		else:
			actions.append(Node(
				package="control_link_adapters", executable="control_command_ingress_adapter_node",
				name="planning_ingress", namespace="control_link", output="screen",
				emulate_tty=True, additional_env=participant_env,
				parameters=[{**common_parameters, "source_id": "planning"}]))
	if start_secondary_source:
		actions.append(
			TimerAction(
				period=secondary_source_delay_seconds,
				actions=[secondary_source_node]))
	actions.append(
		Node(
			package="control_link_gateway",
			executable="control_link_gateway_node",
			name="gateway",
			namespace="control_link",
			output="screen",
			emulate_tty=True,
			additional_env=participant_env,
			parameters=[gateway_parameters],
		))
	if start_lifecycle_activator:
		actions.append(
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
			))
	return actions


def generate_launch_description():
	return LaunchDescription([
		DeclareLaunchArgument("decision_trace_capture_config", default_value="",
			description="Optional absolute fault_capture.yaml; decision_trace_path becomes a new owned directory"),
		DeclareLaunchArgument("supervise_adapter", default_value="false",
			description="Opt-in bounded SocketCAN child rebuild; Graph and recovery gates remain unchanged"),
		DeclareLaunchArgument(
			"can_interface",
			default_value="",
			description="Existing Linux SocketCAN/vcan interface; must match ADAS Profile",
		),
		DeclareLaunchArgument(
			"start_source",
			default_value="true",
			description="Enable ROS2 planning ingress (mock in mock mode) or bundled Edge client; network clients are external",
		),
		DeclareLaunchArgument(
			"ingress_mode",
			default_value="real",
			description="real: configured planning + Twist teleop; mock: explicit demo producers instead of real ROS ingress",
		),
		DeclareLaunchArgument(
			"start_lifecycle_activator",
			default_value="true",
			description="Start the normal Gateway Lifecycle activation gate",
		),
		DeclareLaunchArgument(
			"planning_transport",
			default_value="ros2",
			description="Mutually exclusive planning ingress: ros2, optional edge, someip or grpc",
		),
		DeclareLaunchArgument(
			"network_config_path",
			default_value="",
			description="SOME/IP network contract YAML; empty uses config_root/adas/protocol_transports.yaml",
		),
		DeclareLaunchArgument(
			"vsomeip_config_path",
			default_value="",
			description="Explicit absolute vsomeip runtime JSON for SOME/IP; no implicit environment fallback",
		),
		DeclareLaunchArgument(
			"source_id",
			default_value="planning",
			description="Source for explicit ROS2 mock mode; real/edge/someip/grpc bind planning",
		),
		DeclareLaunchArgument(
			"edge_command_channel",
			default_value="control_link_planning_command",
			description="EdgeRuntime command channel name for planning transport",
		),
		DeclareLaunchArgument(
			"edge_wait_mode", default_value="futex", choices=["futex", "eventfd"],
			description="Edge SPSC waiting mode for both command and feedback; eventfd requires SDK capability",
		),
		DeclareLaunchArgument(
			"edge_feedback_channel",
			default_value="control_link_planning_feedback",
			description="EdgeRuntime feedback channel name for planning transport",
		),
		DeclareLaunchArgument(
			"edge_linear_velocity_mps",
			default_value="0.25",
			description="Standalone Edge planning client linear velocity",
		),
		DeclareLaunchArgument(
			"edge_angular_velocity_radps",
			default_value="0.10",
			description="Standalone Edge planning client angular velocity",
		),
		DeclareLaunchArgument(
			"start_secondary_source",
			default_value="false",
			description="Mock-only: start a second Contract-bound source after a delay",
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
		DeclareLaunchArgument(
			"decision_trace_path",
			default_value="",
			description="Absolute JSONL path for optional Gateway decision recording",
		),
		DeclareLaunchArgument(
			"decision_trace_queue_capacity",
			default_value="4096",
			description="Bounded Gateway decision trace queue capacity",
		),
		OpaqueFunction(function=_launch_adas_vcan),
	])
