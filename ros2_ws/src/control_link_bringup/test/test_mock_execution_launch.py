import os
import unittest

from ament_index_python.packages import get_package_share_directory
import launch
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_testing.actions import ReadyToTest
import launch_testing
import pytest


@pytest.mark.launch_test
def generate_test_description():
	scenario = os.environ.get("CONTROL_LINK_MOCK_SCENARIO", "")
	launch_arguments = {
		"gui": "false",
		"hardware_backend": "mock",
		"start_goal": "false",
	}
	if scenario == "command_timeout":
		launch_arguments["start_command_timeout_scenario"] = "true"
		process_name = "verify_robot_command_timeout"
		expected_output = "ROBOT_COMMAND_TIMEOUT_SUCCEEDED"
	elif scenario == "read_failure":
		launch_arguments.update({
			"start_mock_hardware_failure_scenario": "true",
			"mock_fail_read_after_cycles": "1000",
		})
		process_name = "verify_mock_hardware_failure"
		expected_output = "MOCK_HARDWARE_FAILURE_SUCCEEDED mode=read"
	elif scenario == "write_failure":
		launch_arguments.update({
			"start_mock_hardware_failure_scenario": "true",
			"mock_fail_write_after_cycles": "1000",
		})
		process_name = "verify_mock_hardware_failure"
		expected_output = "MOCK_HARDWARE_FAILURE_SUCCEEDED mode=write"
	else:
		raise RuntimeError(
			"CONTROL_LINK_MOCK_SCENARIO must be command_timeout, read_failure or write_failure")

	package_share = get_package_share_directory("control_link_bringup")
	robot_demo = IncludeLaunchDescription(
		PythonLaunchDescriptionSource(
			os.path.join(package_share, "launch", "robot_demo.launch.py")),
		launch_arguments=launch_arguments.items(),
	)
	return launch.LaunchDescription([
		robot_demo,
		ReadyToTest(),
	]), {
		"process_name": process_name,
		"expected_output": expected_output,
	}


class TestMockExecutionLaunch(unittest.TestCase):
	def test_scenario_verdict(
		self,
		proc_info,
		proc_output,
		process_name,
		expected_output,
	):
		proc_output.assertWaitFor(
			expected_output=expected_output,
			process=process_name,
			timeout=90.0,
		)
		proc_info.assertWaitForShutdown(process=process_name, timeout=10.0)
		launch_testing.asserts.assertExitCodes(
			proc_info,
			process=process_name,
		)
