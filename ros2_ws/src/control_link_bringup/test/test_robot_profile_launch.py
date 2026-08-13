import os
import unittest

from ament_index_python.packages import get_package_share_directory
from control_link_interfaces.msg import ControlCommand, GatewayState, VehicleState
import launch
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
import launch_testing
from launch_testing.actions import ReadyToTest
from launch_testing_ros import WaitForTopics
import pytest


@pytest.mark.launch_test
def generate_test_description():
	package_share = get_package_share_directory("control_link_bringup")
	robot_demo = IncludeLaunchDescription(
		PythonLaunchDescriptionSource(
			os.path.join(package_share, "launch", "robot_demo.launch.py")),
		launch_arguments={
			"gui": "false",
			"start_goal": "true",
			"start_takeover_scenario": "false",
			"start_command_timeout_scenario": "false",
		}.items(),
	)

	return launch.LaunchDescription([
		robot_demo,
		ReadyToTest(),
	])


class TestRobotProfileLaunch(unittest.TestCase):
	def test_control_chain_and_scenario_verdict(
		self,
		proc_info,
		proc_output,
	):
		# C++ readiness/verdict 工具拥有 TF、Lifecycle、状态机和最终 pose 规则
		# 这里仅验证 bringup 能把公共数据链装配起来，并传播场景进程退出码
		waiter = WaitForTopics(
			[
				("/control_link/state", GatewayState),
				("/control_link/output/control_cmd", ControlCommand),
				("/control_link/vehicle_state", VehicleState),
			],
			timeout=120.0,
		)
		try:
			self.assertTrue(
				waiter.wait(),
				"Robot launch did not publish the required control-chain Topics: " +
				str(waiter.topics_not_received()),
			)
			proc_output.assertWaitFor(
				expected_output="NAV_GOAL_SUCCEEDED",
				timeout=150.0,
			)
			proc_info.assertWaitForShutdown(
				process="send_fixed_nav_goal",
				timeout=10.0,
			)
			launch_testing.asserts.assertExitCodes(
				proc_info,
				process="send_fixed_nav_goal",
			)
		finally:
			waiter.shutdown()
