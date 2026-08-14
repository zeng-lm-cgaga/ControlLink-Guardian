import os
import shutil
import unittest

from ament_index_python.packages import get_package_share_directory
from control_link_bringup.profile_bootstrap import (
	canonical_regular_file_within_root,
	load_fastdds_profile_path,
	load_yaml_mapping,
)
import launch
from launch_testing.actions import ReadyToTest
from launch_ros.actions import Node
import launch_testing
import pytest
import yaml


def prepare_test_profile(package_share):
	installed_config_root = os.path.join(package_share, "config")
	installed_profile_path = os.path.join(
		installed_config_root, "adas", "adas_profile.yaml")
	# symlink-install 下单个 Profile 会指回源码树，配置根必须跟随同一棵物理树
	profile_path = os.path.realpath(installed_profile_path)
	config_root = os.path.dirname(os.path.dirname(profile_path))
	fastdds_profile_name = os.environ.get(
		"CONTROL_LINK_TEST_FASTDDS_PROFILE", "")
	if not fastdds_profile_name:
		return profile_path, config_root

	if os.path.basename(fastdds_profile_name) != fastdds_profile_name:
		raise RuntimeError(
			"CONTROL_LINK_TEST_FASTDDS_PROFILE must be a config filename")
	canonical_regular_file_within_root(
		os.path.join(config_root, "common", fastdds_profile_name),
		config_root,
		"CONTROL_LINK_TEST_FASTDDS_PROFILE",
	)

	# 测试副本保持根配置为唯一输入，只替换 transport 引用，不修改产品配置
	generated_config_root = os.path.join(
		os.getcwd(), "generated_gateway_lifecycle_config", fastdds_profile_name)
	if os.path.exists(generated_config_root):
		shutil.rmtree(generated_config_root)
	shutil.copytree(config_root, generated_config_root)
	profile_path = os.path.join(
		generated_config_root, "adas", "adas_profile.yaml")
	document = load_yaml_mapping(profile_path, "generated ADAS test Profile")
	document["fastdds_profile"] = "../common/" + fastdds_profile_name
	with open(profile_path, "w", encoding="utf-8") as profile_file:
		yaml.safe_dump(document, profile_file, sort_keys=False)
	return os.path.realpath(profile_path), os.path.realpath(generated_config_root)


@pytest.mark.launch_test
def generate_test_description():
	package_share = get_package_share_directory("control_link_bringup")
	profile_path, config_root = prepare_test_profile(package_share)
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

	gateway = Node(
		package="control_link_gateway",
		executable="control_link_gateway_node",
		name="gateway",
		namespace="control_link",
		output="screen",
		emulate_tty=True,
		additional_env=participant_env,
		parameters=[common_parameters],
	)
	adapter = Node(
		package="control_link_adapters",
		executable="mock_vehicle_adapter_node",
		name="vehicle_adapter",
		namespace="control_link",
		output="screen",
		emulate_tty=True,
		additional_env=participant_env,
		parameters=[common_parameters],
	)
	verifier = Node(
		package="control_link_bringup",
		executable="verify_gateway_lifecycle",
		name="gateway_lifecycle_verifier",
		output="screen",
		emulate_tty=True,
		additional_env=participant_env,
		parameters=[{
			**common_parameters,
			"target_fqn": "/control_link/gateway",
		}],
	)

	return launch.LaunchDescription([
		gateway,
		adapter,
		verifier,
		ReadyToTest(),
	]), {"verifier": verifier}


class TestGatewayLifecycleLaunch(unittest.TestCase):
	def test_verifier_completes(self, proc_info, proc_output, verifier):
		proc_output.assertWaitFor(
			expected_output="E4_GATEWAY_LIFECYCLE_VERIFIED",
			process=verifier,
			timeout=90.0,
		)
		proc_info.assertWaitForShutdown(process=verifier, timeout=10.0)
		launch_testing.asserts.assertExitCodes(proc_info, process=verifier)
