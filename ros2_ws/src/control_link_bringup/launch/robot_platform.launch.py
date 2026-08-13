import math
import os
import tempfile

import xacro
import yaml
from ament_index_python.packages import get_package_share_directory
from control_link_bringup.profile_bootstrap import (
	canonical_regular_file_within_root,
	load_fastdds_profile_path,
	load_yaml_mapping,
	package_resource,
)
from launch import LaunchDescription
from launch.actions import (
	DeclareLaunchArgument,
	IncludeLaunchDescription,
	OpaqueFunction,
	RegisterEventHandler,
	SetEnvironmentVariable,
)
from launch.event_handlers import OnProcessExit, OnShutdown
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
	if isinstance(value, bool) or not isinstance(value, (int, float)):
		raise RuntimeError(
			field_path + ": wrong YAML type; expected=positive finite number")
	result = float(value)
	if not math.isfinite(result) or result <= 0.0:
		raise RuntimeError(
			field_path + ": invalid geometry; expected=positive finite meters; actual=" +
			str(value))
	return result


def _require_single_joint_name(parameters, key, field_path):
	value = parameters.get(key)
	if not isinstance(value, list) or len(value) != 1 or \
		not isinstance(value[0], str) or not value[0]:
		raise RuntimeError(
			field_path + ": unsupported wheel layout; expected=one non-empty joint name")
	return value[0]


def _write_controller_config(template_path, wheel_separation, wheel_radius, frames):
	document = load_yaml_mapping(template_path, "adapter.config")
	controller_manager = _require_mapping(
		document, "controller_manager", "adapter.config.controller_manager")
	_require_mapping(
		controller_manager, "ros__parameters",
		"adapter.config.controller_manager.ros__parameters")
	diff_drive = _require_mapping(
		document, "diff_drive_controller", "adapter.config.diff_drive_controller")
	parameters = _require_mapping(
		diff_drive, "ros__parameters",
		"adapter.config.diff_drive_controller.ros__parameters")

	left_wheel_joint = _require_single_joint_name(
		parameters, "left_wheel_names",
		"adapter.config.diff_drive_controller.ros__parameters.left_wheel_names")
	right_wheel_joint = _require_single_joint_name(
		parameters, "right_wheel_names",
		"adapter.config.diff_drive_controller.ros__parameters.right_wheel_names")

	# Profile 是几何和 frame 的唯一 owner，临时文件只是 gz_ros2_control 的运行输入
	parameters["wheel_separation"] = wheel_separation
	parameters["wheel_radius"] = wheel_radius
	parameters["odom_frame_id"] = frames["odom"]
	parameters["base_frame_id"] = frames["base_footprint"]

	with tempfile.NamedTemporaryFile(
		mode="w",
		encoding="utf-8",
		prefix="control_link_ros2_control_",
		suffix=".yaml",
		delete=False,
	) as generated_file:
		yaml.safe_dump(document, generated_file, sort_keys=False)
		return generated_file.name, left_wheel_joint, right_wheel_joint


def _remove_generated_file(file_path):
	try:
		os.remove(file_path)
	except FileNotFoundError:
		pass


def _launch_flag(context, name):
	value = LaunchConfiguration(name).perform(context).lower()
	if value not in ("true", "false"):
		raise RuntimeError(name + " must be true or false; actual=" + value)
	return value == "true"


def _launch_robot_platform(context, *args, **kwargs):
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
	if profile.get("clock_mode") != "sim" or profile.get("use_sim_time") is not True:
		raise RuntimeError(
			"Robot platform requires clock_mode=sim and use_sim_time=true")

	geometry = _require_mapping(profile, "geometry", "geometry")
	wheel_separation = _require_positive_number(
		geometry, "wheel_separation_m", "geometry.wheel_separation_m")
	wheel_radius = _require_positive_number(
		geometry, "wheel_radius_m", "geometry.wheel_radius_m")
	adapter = _require_mapping(profile, "adapter", "adapter")
	adapter_config_reference = _require_string(
		adapter, "config", "adapter.config")
	controller_node_fqn = _require_string(
		adapter, "controller_node_fqn", "adapter.controller_node_fqn")
	odometry_topic = _require_string(
		adapter, "odometry_topic", "adapter.odometry_topic")
	if os.path.isabs(adapter_config_reference):
		raise RuntimeError(
			"adapter.config: absolute config reference; expected=relative path")
	controller_template_path = canonical_regular_file_within_root(
		os.path.join(os.path.dirname(profile_path), adapter_config_reference),
		config_root,
		"adapter.config")

	resources = _require_mapping(profile, "resources", "resources")
	resource_package = _require_string(
		resources, "package", "resources.package")
	resource_package_share = get_package_share_directory(resource_package)
	xacro_path = package_resource(
		resource_package,
		resource_package_share,
		_require_string(
			resources, "robot_description", "resources.robot_description"),
		"resources.robot_description")
	world_path = package_resource(
		resource_package,
		resource_package_share,
		_require_string(resources, "world", "resources.world"),
		"resources.world")

	frames = _require_mapping(profile, "frames", "frames")
	frame_names = {
		name: _require_string(frames, name, "frames." + name)
		for name in ("map", "odom", "base_footprint", "base_link", "laser")
	}
	fastdds_profile_path = load_fastdds_profile_path(profile_path, config_root)
	generated_config, left_wheel_joint, right_wheel_joint = _write_controller_config(
		controller_template_path,
		wheel_separation,
		wheel_radius,
		frame_names)

	try:
		robot_description = xacro.process_file(
			xacro_path,
			mappings={
				"wheel_separation": format(wheel_separation, ".17g"),
				"wheel_radius": format(wheel_radius, ".17g"),
				"left_wheel_joint": left_wheel_joint,
				"right_wheel_joint": right_wheel_joint,
				"controller_node_fqn": controller_node_fqn,
				"odometry_topic": odometry_topic,
				"base_footprint_frame": frame_names["base_footprint"],
				"base_link_frame": frame_names["base_link"],
				"laser_frame": frame_names["laser"],
				"controller_config": generated_config,
			},
		).toxml()
	except Exception:
		_remove_generated_file(generated_config)
		raise

	start_adapter = _launch_flag(context, "start_adapter")
	standalone_tf_fixture = _launch_flag(context, "standalone_tf_fixture")
	gui = _launch_flag(context, "gui")
	# VMware Xwayland 下 Ogre2 GUI 只显示空 Scene，Ogre1 GUI 不改变 server 侧 lidar renderer
	gz_args = "-r --render-engine-gui ogre -v 2 " + world_path
	if not gui:
		gz_args = "-r -s --headless-rendering -v 2 " + world_path

	robot_state_publisher = Node(
		package="robot_state_publisher",
		executable="robot_state_publisher",
		name="robot_state_publisher",
		output="screen",
		parameters=[{
			"robot_description": robot_description,
			"use_sim_time": True,
		}],
	)
	spawn_robot = Node(
		package="ros_gz_sim",
		executable="create",
		name="spawn_control_link_diffbot",
		output="screen",
		prefix="timeout --signal=INT --kill-after=2s 20s",
		arguments=[
			"-world", "control_link_flat",
			"-topic", "/robot_description",
			"-name", "control_link_diffbot",
			"-allow_renaming", "false",
		],
	)
	controller_spawner = Node(
		package="controller_manager",
		executable="spawner",
		name="control_link_controller_spawner",
		output="screen",
		arguments=[
			"joint_state_broadcaster",
			"diff_drive_controller",
			"--activate-as-group",
			"--controller-manager-timeout", "20",
			"--service-call-timeout", "10",
			"--switch-timeout", "20",
		],
	)
	adapter_node = Node(
		package="control_link_adapters",
		executable="ros2_control_adapter_node",
		name="vehicle_adapter",
		namespace="control_link",
		output="screen",
		emulate_tty=True,
		additional_env={
			"RMW_IMPLEMENTATION": "rmw_fastrtps_cpp",
			"RMW_FASTRTPS_USE_QOS_FROM_XML": "0",
			"FASTRTPS_DEFAULT_PROFILES_FILE": fastdds_profile_path,
		},
		parameters=[{
			"profile_path": profile_path,
			"config_root": config_root,
			"use_sim_time": True,
		}],
	)

	def on_spawn_exit(event, launch_context):
		if event.returncode != 0:
			raise RuntimeError(
				"Robot spawn failed with exit code " + str(event.returncode))
		return [controller_spawner]

	def on_spawner_exit(event, launch_context):
		if event.returncode != 0:
			raise RuntimeError(
				"Controller activation failed with exit code " +
				str(event.returncode))
		if start_adapter:
			return [adapter_node]
		return None

	def on_shutdown(event, launch_context):
		_remove_generated_file(generated_config)
		return None

	actions = [
		# 当前 Robot demo 的 Gazebo Transport 只在本机进程间通信，避免无路由网卡刷屏
		SetEnvironmentVariable("IGN_IP", "127.0.0.1"),
		SetEnvironmentVariable("RMW_IMPLEMENTATION", "rmw_fastrtps_cpp"),
		SetEnvironmentVariable("RMW_FASTRTPS_USE_QOS_FROM_XML", "0"),
		SetEnvironmentVariable(
			"FASTRTPS_DEFAULT_PROFILES_FILE", fastdds_profile_path),
		IncludeLaunchDescription(
			PythonLaunchDescriptionSource(
				os.path.join(
					get_package_share_directory("ros_gz_sim"),
					"launch",
					"gz_sim.launch.py")),
			launch_arguments={"gz_args": gz_args}.items(),
		),
		robot_state_publisher,
		Node(
			package="ros_gz_bridge",
			executable="parameter_bridge",
			name="control_link_gz_bridge",
			output="screen",
			arguments=[
				"/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock",
				"/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan",
				"/world/control_link_flat/control@ros_gz_interfaces/srv/ControlWorld",
			],
		),
		spawn_robot,
		RegisterEventHandler(
			event_handler=OnProcessExit(
				target_action=spawn_robot,
				on_exit=on_spawn_exit)),
		RegisterEventHandler(
			event_handler=OnProcessExit(
				target_action=controller_spawner,
				on_exit=on_spawner_exit)),
		RegisterEventHandler(
			event_handler=OnShutdown(on_shutdown=on_shutdown)),
	]

	if standalone_tf_fixture:
		# Task 5 单机闭环暂代 AMCL 的 map -> odom，Nav2 单元必须关闭该 fixture
		actions.append(Node(
			package="tf2_ros",
			executable="static_transform_publisher",
			name="standalone_map_to_odom_fixture",
			output="screen",
			arguments=[
				"--frame-id", frame_names["map"],
				"--child-frame-id", frame_names["odom"],
			],
		))

	return actions


def generate_launch_description():
	return LaunchDescription([
		DeclareLaunchArgument(
			"gui",
			default_value="false",
			description="Start the Gazebo graphical client",
		),
		DeclareLaunchArgument(
			"start_adapter",
			default_value="true",
			description="Start Ros2ControlAdapter after both controllers are active",
		),
		DeclareLaunchArgument(
			"standalone_tf_fixture",
			default_value="true",
			description="Provide map to odom only for the pre-Nav2 standalone platform smoke",
		),
		OpaqueFunction(function=_launch_robot_platform),
	])
