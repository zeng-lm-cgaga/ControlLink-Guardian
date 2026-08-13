import os

import yaml
from ament_index_python.packages import get_package_share_directory
from control_link_bringup.profile_bootstrap import load_fastdds_profile_path
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def _launch_gateway(context, *args, **kwargs):
	profile = LaunchConfiguration("profile").perform(context)
	if profile not in ("robot", "adas"):
		raise RuntimeError(
			"profile must be one of: robot, adas; actual=" + profile)

	package_share = get_package_share_directory("control_link_bringup")
	installed_config_root = os.path.join(package_share, "config")
	installed_profile_path = os.path.join(
		installed_config_root,
		profile,
		profile + "_profile.yaml")
	# symlink-install 会让单个 YAML 指向源码树，parser 需要实际文件与实际根目录属于同一棵树
	profile_path = os.path.realpath(installed_profile_path)
	config_root = os.path.dirname(os.path.dirname(profile_path))
	fastdds_profile_path = load_fastdds_profile_path(
		profile_path,
		config_root)

	actions = [
		Node(
			package="control_link_gateway",
			executable="control_link_gateway_node",
			name="gateway",
			namespace="control_link",
			output="screen",
			emulate_tty=True,
			# FastDDS participant 在 rclcpp::init() 期间建立，必须在子进程启动前注入
			additional_env={
				"RMW_IMPLEMENTATION": "rmw_fastrtps_cpp",
				"RMW_FASTRTPS_USE_QOS_FROM_XML": "0",
				"FASTRTPS_DEFAULT_PROFILES_FILE": fastdds_profile_path,
			},
			parameters=[
				{
					"profile_path": profile_path,
					"config_root": config_root,
				}
			],
		)
	]

	software_loop = LaunchConfiguration("software_loop").perform(context).lower()
	if software_loop not in ("true", "false"):
		raise RuntimeError(
			"software_loop must be true or false; actual=" + software_loop)
	if software_loop == "true":
		if profile != "adas":
			raise RuntimeError(
				"software_loop currently requires profile:=adas; Robot needs a Gazebo /clock")

		try:
			with open(profile_path, "r", encoding="utf-8") as profile_file:
				profile_document = yaml.safe_load(profile_file)
		except (OSError, yaml.YAMLError) as error:
			raise RuntimeError(
				profile_path + ": cannot read software loop Profile fields; actual=" +
				str(error)) from error

		enabled_sources = profile_document.get("enabled_sources", [])
		if not isinstance(enabled_sources, list) or not enabled_sources:
			raise RuntimeError(
				profile_path + ":enabled_sources: software loop needs at least one source")
		mock_source_id = LaunchConfiguration("mock_source_id").perform(context)
		if not mock_source_id:
			mock_source_id = enabled_sources[0]
		if mock_source_id not in enabled_sources:
			raise RuntimeError(
				"mock_source_id must be enabled by the selected Profile; actual=" +
				mock_source_id)

		common_mock_parameters = [
			{
				"profile_path": profile_path,
				"config_root": config_root,
				"use_sim_time": False,
			}
		]
		actions.extend([
			Node(
				package="control_link_adapters",
				executable="mock_vehicle_adapter_node",
				name="vehicle_adapter",
				namespace="control_link",
				output="screen",
				emulate_tty=True,
				additional_env={
					"RMW_IMPLEMENTATION": "rmw_fastrtps_cpp",
					"RMW_FASTRTPS_USE_QOS_FROM_XML": "0",
					"FASTRTPS_DEFAULT_PROFILES_FILE": fastdds_profile_path,
				},
				parameters=common_mock_parameters,
			),
			Node(
				package="control_link_adapters",
				executable="mock_control_source_node",
				name="mock_source_" + mock_source_id,
				namespace="control_link",
				output="screen",
				emulate_tty=True,
				additional_env={
					"RMW_IMPLEMENTATION": "rmw_fastrtps_cpp",
					"RMW_FASTRTPS_USE_QOS_FROM_XML": "0",
					"FASTRTPS_DEFAULT_PROFILES_FILE": fastdds_profile_path,
				},
				parameters=[
					{
						"profile_path": profile_path,
						"config_root": config_root,
						"source_id": mock_source_id,
						"use_sim_time": False,
					}
				],
			),
		])

	return actions


def generate_launch_description():
	return LaunchDescription([
		DeclareLaunchArgument(
			"profile",
			default_value="robot",
			description="Guardian profile to load: robot or adas",
		),
		DeclareLaunchArgument(
			"software_loop",
			default_value="false",
			description="Start the optional ADAS system-clock mock source and adapter",
		),
		DeclareLaunchArgument(
			"mock_source_id",
			default_value="",
			description="Enabled source used by the optional mock control source",
		),
		OpaqueFunction(function=_launch_gateway),
	])
