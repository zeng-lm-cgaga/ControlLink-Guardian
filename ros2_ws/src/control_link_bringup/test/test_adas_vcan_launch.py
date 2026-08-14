import os
import unittest

from ament_index_python.packages import get_package_share_directory
from control_link_bringup.profile_bootstrap import (
	load_fastdds_profile_path,
	require_up_vcan_interface,
)
import launch
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_testing.actions import ReadyToTest
from launch_ros.actions import Node
import launch_testing
import pytest


@pytest.mark.launch_test
def generate_test_description():
	require_up_vcan_interface("vcan0")

	package_share = get_package_share_directory("control_link_bringup")
	profile_path = os.path.realpath(
		os.path.join(package_share, "config", "adas", "adas_profile.yaml"))
	# symlink-install 下 profile 文件指向源码 config 树，根目录必须跟随同一棵真实树
	config_root = os.path.dirname(os.path.dirname(profile_path))
	fastdds_profile_path = load_fastdds_profile_path(profile_path, config_root)
	participant_env = {
		"RMW_IMPLEMENTATION": "rmw_fastrtps_cpp",
		"RMW_FASTRTPS_USE_QOS_FROM_XML": "0",
		"FASTRTPS_DEFAULT_PROFILES_FILE": fastdds_profile_path,
	}

	demo = IncludeLaunchDescription(
		PythonLaunchDescriptionSource(
			os.path.join(package_share, "launch", "adas_vcan_demo.launch.py")),
		launch_arguments={
			"can_interface": "vcan0",
			"start_source": "true",
			"source_id": "planning",
		}.items(),
	)
	verifier = Node(
		package="control_link_bringup",
		executable="verify_adas_vcan",
		name="adas_vcan_verifier",
		output="screen",
		emulate_tty=True,
		additional_env=participant_env,
		parameters=[{
			"profile_path": profile_path,
			"config_root": config_root,
			"use_sim_time": False,
			"timeout_ms": 120000,
		}],
	)

	return launch.LaunchDescription([
		demo,
		verifier,
		ReadyToTest(),
	]), {"verifier": verifier}


class TestAdasVcanLaunch(unittest.TestCase):
	def test_verifier_completes(self, proc_info, proc_output, verifier):
		proc_output.assertWaitFor(
			expected_output="E10_ADAS_VCAN_VERIFIED",
			process=verifier,
			timeout=130.0,
		)
		proc_info.assertWaitForShutdown(process=verifier, timeout=10.0)
		launch_testing.asserts.assertExitCodes(proc_info, process=verifier)


@launch_testing.post_shutdown_test()
class TestAdasVcanProcessExitCodes(unittest.TestCase):
	def test_all_processes_exit_cleanly(self, proc_info):
		launch_testing.asserts.assertExitCodes(proc_info)
