import math
import os

from ament_index_python.packages import get_package_share_directory
from control_link_bringup.profile_bootstrap import (
	load_fastdds_profile_path,
	load_yaml_mapping,
	package_resource,
)
from launch import LaunchDescription
from launch.actions import (
	DeclareLaunchArgument,
	IncludeLaunchDescription,
	LogInfo,
	OpaqueFunction,
	RegisterEventHandler,
	SetEnvironmentVariable,
)
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _require_mapping(parent, key, field_path):
	value = parent.get(key)
	if not isinstance(value, dict):
		raise RuntimeError(field_path + ": wrong YAML type; expected=map")
	return value


def _require_string(parent, key, field_path):
	value = parent.get(key)
	if not isinstance(value, str) or not value.strip():
		raise RuntimeError(
			field_path + ": wrong YAML value; expected=non-empty string")
	return value


def _require_positive_number(parent, key, field_path):
	value = parent.get(key)
	if isinstance(value, bool) or not isinstance(value, (int, float)) or \
		not math.isfinite(float(value)) or value <= 0:
		raise RuntimeError(
			field_path + ": wrong YAML value; expected=positive number")
	return float(value)


def _launch_flag(context, name):
	value = LaunchConfiguration(name).perform(context).lower()
	if value not in ("true", "false"):
		raise RuntimeError(name + " must be true or false; actual=" + value)
	return value == "true"


def _launch_nonnegative_integer(context, name):
	value = LaunchConfiguration(name).perform(context)
	try:
		result = int(value, 10)
	except ValueError as error:
		raise RuntimeError(
			name + " must be a non-negative integer; actual=" + value) from error
	if result < 0:
		raise RuntimeError(
			name + " must be a non-negative integer; actual=" + value)
	return result


def _launch_positive_integer(context, name):
	result = _launch_nonnegative_integer(context, name)
	if result == 0:
		raise RuntimeError(name + " must be a positive integer; actual=0")
	return result


def _launch_robot_demo(context, *args, **kwargs):
	package_share = get_package_share_directory("control_link_bringup")
	installed_config_root = os.path.join(package_share, "config")
	installed_profile_path = os.path.join(
		installed_config_root, "robot", "robot_profile.yaml")
	profile_path = os.path.realpath(installed_profile_path)
	config_root = os.path.dirname(os.path.dirname(profile_path))
	profile = load_yaml_mapping(profile_path, "robot Profile")
	if profile.get("profile_id") != "robot":
		raise RuntimeError(
			"profile_id mismatch; expected=robot; actual=" +
			str(profile.get("profile_id")))
	adapter_config = _require_mapping(profile, "adapter", "adapter")
	controller_manager_fqn = _require_string(
		adapter_config,
		"controller_manager_fqn",
		"adapter.controller_manager_fqn",
	)

	resources = _require_mapping(profile, "resources", "resources")
	resource_package = _require_string(
		resources, "package", "resources.package")
	resource_package_share = get_package_share_directory(resource_package)
	map_yaml_path = package_resource(
		resource_package,
		resource_package_share,
		_require_string(resources, "map_yaml", "resources.map_yaml"),
		"resources.map_yaml")
	nav2_params_path = package_resource(
		resource_package,
		resource_package_share,
		_require_string(resources, "nav2_params", "resources.nav2_params"),
		"resources.nav2_params",
		additional_symlink_roots=(config_root,))
	frames = _require_mapping(profile, "frames", "frames")
	map_frame = _require_string(frames, "map", "frames.map")
	base_frame = _require_string(
		frames, "base_footprint", "frames.base_footprint")

	nav2_params = load_yaml_mapping(nav2_params_path, "resources.nav2_params")
	controller_server = _require_mapping(
		nav2_params, "controller_server", "controller_server")
	controller_parameters = _require_mapping(
		controller_server, "ros__parameters", "controller_server.ros__parameters")
	goal_checker_name = controller_parameters.get("goal_checker_plugins")
	if not isinstance(goal_checker_name, list) or len(goal_checker_name) != 1 or \
		not isinstance(goal_checker_name[0], str):
		raise RuntimeError(
			"controller_server.ros__parameters.goal_checker_plugins must contain one name")
	goal_checker = _require_mapping(
		controller_parameters,
		goal_checker_name[0],
		"controller_server.ros__parameters." + goal_checker_name[0])
	position_tolerance_m = _require_positive_number(
		goal_checker,
		"xy_goal_tolerance",
		"controller_server.ros__parameters." + goal_checker_name[0] +
		".xy_goal_tolerance")
	yaw_tolerance_rad = _require_positive_number(
		goal_checker,
		"yaw_goal_tolerance",
		"controller_server.ros__parameters." + goal_checker_name[0] +
		".yaw_goal_tolerance")

	fastdds_profile_path = load_fastdds_profile_path(profile_path, config_root)
	participant_env = {
		"RMW_IMPLEMENTATION": "rmw_fastrtps_cpp",
		"RMW_FASTRTPS_USE_QOS_FROM_XML": "0",
		"FASTRTPS_DEFAULT_PROFILES_FILE": fastdds_profile_path,
	}
	common_parameters = {
		"profile_path": profile_path,
		"config_root": config_root,
		"use_sim_time": True,
	}
	gateway_parameters = {
		**common_parameters,
		"decision_trace_path": LaunchConfiguration(
			"decision_trace_path").perform(context),
		"decision_trace_queue_capacity": _launch_positive_integer(
			context, "decision_trace_queue_capacity"),
	}

	robot_platform = IncludeLaunchDescription(
		PythonLaunchDescriptionSource(
			os.path.join(package_share, "launch", "robot_platform.launch.py")),
		launch_arguments={
			"gui": LaunchConfiguration("gui"),
			"hardware_backend": LaunchConfiguration("hardware_backend"),
			"mock_fail_read_after_cycles": LaunchConfiguration(
				"mock_fail_read_after_cycles"),
			"mock_fail_write_after_cycles": LaunchConfiguration(
				"mock_fail_write_after_cycles"),
			"start_adapter": "false",
			"standalone_tf_fixture": LaunchConfiguration(
				"start_mock_hardware_failure_scenario"),
		}.items(),
	)
	controllers_gate = Node(
		package="control_link_bringup",
		executable="robot_readiness_gate",
		name="controllers_readiness_gate",
		output="screen",
		parameters=[{
			"phase": "controllers",
			"timeout_ms": 60000,
			"controller_manager_fqn": controller_manager_fqn,
			"use_sim_time": True,
		}],
	)
	nav2_bringup = IncludeLaunchDescription(
		PythonLaunchDescriptionSource(
			os.path.join(
				get_package_share_directory("nav2_bringup"),
				"launch",
				"bringup_launch.py")),
		launch_arguments={
			"map": map_yaml_path,
			"params_file": nav2_params_path,
			"use_sim_time": "true",
			"autostart": "true",
			# Humble Nav2 用 PythonExpression 拼接这些值，必须是 Python 布尔字面量
			"slam": "False",
			"use_namespace": "False",
			"use_composition": "False",
			"use_respawn": "False",
		}.items(),
	)
	nav2_gate = Node(
		package="control_link_bringup",
		executable="robot_readiness_gate",
		name="nav2_readiness_gate",
		output="screen",
		parameters=[{
			"phase": "nav2",
			"timeout_ms": 90000,
			"map_frame": map_frame,
			"base_frame": base_frame,
			"use_sim_time": True,
		}],
	)

	adapter = Node(
		package="control_link_adapters",
		executable="ros2_control_adapter_node",
		name="vehicle_adapter",
		namespace="control_link",
		output="screen",
		emulate_tty=True,
		additional_env=participant_env,
		parameters=[common_parameters],
	)
	nav2_ingress = Node(
		package="control_link_adapters",
		executable="twist_ingress_adapter_node",
		name="nav2_ingress",
		namespace="control_link",
		output="screen",
		emulate_tty=True,
		additional_env=participant_env,
		parameters=[{
			**common_parameters,
			"source_id": "nav2",
		}],
	)
	teleop_ingress = Node(
		package="control_link_adapters",
		executable="twist_ingress_adapter_node",
		name="teleop_ingress",
		namespace="control_link",
		output="screen",
		emulate_tty=True,
		additional_env=participant_env,
		parameters=[{
			**common_parameters,
			"source_id": "teleop",
		}],
	)
	gateway = Node(
		package="control_link_gateway",
		executable="control_link_gateway_node",
		name="gateway",
		namespace="control_link",
		output="screen",
		emulate_tty=True,
		additional_env=participant_env,
		parameters=[gateway_parameters],
	)
	lifecycle_activator = Node(
		package="control_link_bringup",
		executable="gateway_lifecycle_activator",
		name="gateway_lifecycle_activator",
		output="screen",
		additional_env=participant_env,
		parameters=[{
			"target_fqn": "/control_link/gateway",
			"timeout_ms": 45000,
			"use_sim_time": True,
		}],
	)
	goal_sender = Node(
		package="control_link_bringup",
		executable="send_fixed_nav_goal",
		name="send_fixed_nav_goal",
		output="screen",
		additional_env=participant_env,
		parameters=[{
			**common_parameters,
			"position_tolerance_m": position_tolerance_m,
			"yaw_tolerance_rad": yaw_tolerance_rad,
			"clock_start_timeout_ms": 15000,
			"timeout_ms": 120000,
		}],
	)
	takeover_scenario = Node(
		package="control_link_bringup",
		executable="verify_teleop_takeover",
		name="verify_teleop_takeover",
		output="screen",
		additional_env=participant_env,
		parameters=[{
			**common_parameters,
			"timeout_ms": 45000,
		}],
	)
	tf_stale_scenario = Node(
		package="control_link_bringup",
		executable="verify_robot_tf_stale",
		name="verify_robot_tf_stale",
		output="screen",
		additional_env=participant_env,
		parameters=[{
			**common_parameters,
			"timeout_ms": 45000,
		}],
	)
	clock_stall_scenario = Node(
		package="control_link_bringup",
		executable="verify_robot_clock_stall",
		name="verify_robot_clock_stall",
		output="screen",
		additional_env=participant_env,
		parameters=[{
			**common_parameters,
			"timeout_ms": 45000,
		}],
	)
	controller_loss_scenario = Node(
		package="control_link_bringup",
		executable="verify_robot_controller_loss",
		name="verify_robot_controller_loss",
		output="screen",
		additional_env=participant_env,
		parameters=[{
			**common_parameters,
			"timeout_ms": 45000,
		}],
	)
	command_timeout_scenario = Node(
		package="control_link_bringup",
		executable="verify_robot_command_timeout",
		name="verify_robot_command_timeout",
		output="screen",
		additional_env=participant_env,
		parameters=[{
			**common_parameters,
			"timeout_ms": 45000,
		}],
	)
	hardware_backend = LaunchConfiguration("hardware_backend").perform(context)
	mock_fail_read_after_cycles = _launch_nonnegative_integer(
		context, "mock_fail_read_after_cycles")
	mock_fail_write_after_cycles = _launch_nonnegative_integer(
		context, "mock_fail_write_after_cycles")
	mock_failure_mode = (
		"read" if mock_fail_read_after_cycles > 0 else "write")
	mock_hardware_failure_scenario = Node(
		package="control_link_bringup",
		executable="verify_mock_hardware_failure",
		name="verify_mock_hardware_failure",
		output="screen",
		additional_env=participant_env,
		parameters=[{
			**common_parameters,
			"failure_mode": mock_failure_mode,
			"timeout_ms": 45000,
		}],
	)
	start_goal = _launch_flag(context, "start_goal")
	start_takeover_scenario = _launch_flag(context, "start_takeover_scenario")
	start_tf_stale_scenario = _launch_flag(context, "start_tf_stale_scenario")
	start_clock_stall_scenario = _launch_flag(
		context, "start_clock_stall_scenario")
	start_controller_loss_scenario = _launch_flag(
		context, "start_controller_loss_scenario")
	start_command_timeout_scenario = _launch_flag(
		context, "start_command_timeout_scenario")
	start_mock_hardware_failure_scenario = _launch_flag(
		context, "start_mock_hardware_failure_scenario")
	if sum((
		start_takeover_scenario,
		start_tf_stale_scenario,
		start_clock_stall_scenario,
		start_controller_loss_scenario,
		start_command_timeout_scenario,
		start_mock_hardware_failure_scenario,
	)) > 1:
		raise RuntimeError("Robot fault/takeover scenarios must run one at a time")
	if start_takeover_scenario and not start_goal:
		raise RuntimeError(
			"start_takeover_scenario requires start_goal=true so nav2 remains a live source")
	if (start_tf_stale_scenario or start_clock_stall_scenario or
		start_controller_loss_scenario or start_command_timeout_scenario or
		start_mock_hardware_failure_scenario) and start_goal:
		raise RuntimeError(
			"Robot fault scenarios require start_goal=false because they publish their own raw nav2 command")
	if start_mock_hardware_failure_scenario:
		if hardware_backend != "mock":
			raise RuntimeError(
				"mock hardware failure scenario requires hardware_backend=mock")
		if (mock_fail_read_after_cycles > 0) == (mock_fail_write_after_cycles > 0):
			raise RuntimeError(
				"mock hardware failure scenario requires exactly one positive failure threshold")

	def gateway_stack_actions():
		return [
			adapter,
			nav2_ingress,
			teleop_ingress,
			gateway,
			lifecycle_activator,
		]

	def on_controllers_ready(event, launch_context):
		if event.returncode != 0:
			raise RuntimeError(
				"Controller readiness failed with exit code " +
				str(event.returncode))
		# mock 故障验证由 verifier 提供 raw Nav2 命令，不依赖 AMCL/Nav2 生命周期
		if start_mock_hardware_failure_scenario:
			return gateway_stack_actions()
		return [nav2_bringup, nav2_gate]

	def on_nav2_ready(event, launch_context):
		if event.returncode != 0:
			raise RuntimeError(
				"Nav2 readiness failed with exit code " + str(event.returncode))
		return gateway_stack_actions()

	def on_gateway_active(event, launch_context):
		if event.returncode != 0:
			raise RuntimeError(
				"Gateway activation failed with exit code " + str(event.returncode))
		if start_goal:
			actions = [goal_sender]
		else:
			actions = []
		if start_takeover_scenario:
			actions.append(takeover_scenario)
		if start_tf_stale_scenario:
			actions.append(tf_stale_scenario)
		if start_clock_stall_scenario:
			actions.append(clock_stall_scenario)
		if start_controller_loss_scenario:
			actions.append(controller_loss_scenario)
		if start_command_timeout_scenario:
			actions.append(command_timeout_scenario)
		if start_mock_hardware_failure_scenario:
			actions.append(mock_hardware_failure_scenario)
		if actions:
			return actions
		return [LogInfo(msg="Robot demo is ready for a manual NavigateToPose goal")]

	def on_goal_exit(event, launch_context):
		if event.returncode != 0:
			raise RuntimeError(
				"Fixed NavigateToPose goal failed with exit code " +
				str(event.returncode))
		return [LogInfo(msg="Fixed NavigateToPose goal completed successfully")]

	def on_takeover_exit(event, launch_context):
		if event.returncode != 0:
			raise RuntimeError(
				"Teleop takeover scenario failed with exit code " +
				str(event.returncode))
		return [LogInfo(msg="Teleop takeover scenario completed successfully")]

	def on_tf_stale_exit(event, launch_context):
		if event.returncode != 0:
			raise RuntimeError(
				"Robot TF stale scenario failed with exit code " +
				str(event.returncode))
		return [LogInfo(msg="Robot TF stale scenario completed successfully")]

	def on_clock_stall_exit(event, launch_context):
		if event.returncode != 0:
			raise RuntimeError(
				"Robot Clock stall scenario failed with exit code " +
				str(event.returncode))
		return [LogInfo(msg="Robot Clock stall scenario completed successfully")]

	def on_controller_loss_exit(event, launch_context):
		if event.returncode != 0:
			raise RuntimeError(
				"Robot controller loss scenario failed with exit code " +
				str(event.returncode))
		return [LogInfo(msg="Robot controller loss scenario completed successfully")]

	def on_command_timeout_exit(event, launch_context):
		if event.returncode != 0:
			raise RuntimeError(
				"Robot command timeout scenario failed with exit code " +
				str(event.returncode))
		return [LogInfo(msg="Robot command timeout scenario completed successfully")]

	def on_mock_hardware_failure_exit(event, launch_context):
		if event.returncode != 0:
			raise RuntimeError(
				"Mock hardware failure scenario failed with exit code " +
				str(event.returncode))
		return [LogInfo(msg="Mock hardware failure scenario completed successfully")]

	return [
		SetEnvironmentVariable("RMW_IMPLEMENTATION", "rmw_fastrtps_cpp"),
		SetEnvironmentVariable("RMW_FASTRTPS_USE_QOS_FROM_XML", "0"),
		SetEnvironmentVariable(
			"FASTRTPS_DEFAULT_PROFILES_FILE", fastdds_profile_path),
		RegisterEventHandler(
			event_handler=OnProcessExit(
				target_action=controllers_gate,
				on_exit=on_controllers_ready)),
		RegisterEventHandler(
			event_handler=OnProcessExit(
				target_action=nav2_gate,
				on_exit=on_nav2_ready)),
		RegisterEventHandler(
			event_handler=OnProcessExit(
				target_action=lifecycle_activator,
				on_exit=on_gateway_active)),
		RegisterEventHandler(
			event_handler=OnProcessExit(
				target_action=goal_sender,
				on_exit=on_goal_exit)),
		RegisterEventHandler(
			event_handler=OnProcessExit(
				target_action=takeover_scenario,
				on_exit=on_takeover_exit)),
		RegisterEventHandler(
			event_handler=OnProcessExit(
				target_action=tf_stale_scenario,
				on_exit=on_tf_stale_exit)),
		RegisterEventHandler(
			event_handler=OnProcessExit(
				target_action=clock_stall_scenario,
				on_exit=on_clock_stall_exit)),
		RegisterEventHandler(
			event_handler=OnProcessExit(
				target_action=controller_loss_scenario,
				on_exit=on_controller_loss_exit)),
		RegisterEventHandler(
			event_handler=OnProcessExit(
				target_action=command_timeout_scenario,
				on_exit=on_command_timeout_exit)),
		RegisterEventHandler(
			event_handler=OnProcessExit(
				target_action=mock_hardware_failure_scenario,
				on_exit=on_mock_hardware_failure_exit)),
		robot_platform,
		controllers_gate,
	]


def generate_launch_description():
	return LaunchDescription([
		DeclareLaunchArgument(
			"hardware_backend",
			default_value="gazebo",
			description="ros2_control hardware backend forwarded to robot_platform",
		),
		DeclareLaunchArgument(
			"mock_fail_read_after_cycles",
			default_value="0",
			description="Successful mock read cycles before a persistent injected error",
		),
		DeclareLaunchArgument(
			"mock_fail_write_after_cycles",
			default_value="0",
			description="Successful mock write cycles before a persistent injected error",
		),
		DeclareLaunchArgument(
			"gui",
			default_value="false",
			description="Start the Gazebo graphical client",
		),
		DeclareLaunchArgument(
			"start_goal",
			default_value="true",
			description="Send the fixed Robot Profile goal after Gateway activation",
		),
		DeclareLaunchArgument(
			"start_takeover_scenario",
			default_value="false",
			description="Verify nav2 to teleop takeover and lease-driven fallback",
		),
		DeclareLaunchArgument(
			"start_tf_stale_scenario",
			default_value="false",
			description="Verify AMCL TF stale safe-stop and recovery",
		),
		DeclareLaunchArgument(
			"start_clock_stall_scenario",
			default_value="false",
			description="Verify Gazebo Clock stall safe-stop and recovery",
		),
		DeclareLaunchArgument(
			"start_controller_loss_scenario",
			default_value="false",
			description="Verify ros2_control endpoint loss safe-stop and recovery",
		),
		DeclareLaunchArgument(
			"start_command_timeout_scenario",
			default_value="false",
			description="Verify nav2 command lease timeout safe-stop and recovery",
		),
		DeclareLaunchArgument(
			"start_mock_hardware_failure_scenario",
			default_value="false",
			description="Verify mock SystemInterface failure propagation into Guardian",
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
		OpaqueFunction(function=_launch_robot_demo),
	])
