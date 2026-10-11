import importlib.util
from pathlib import Path
import sys
import tempfile
import unittest
from unittest.mock import patch

from launch import LaunchContext


BRINGUP = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(BRINGUP))


def load_launch():
	spec = importlib.util.spec_from_file_location(
		"adas_planning_selector_under_test", BRINGUP / "launch" / "adas_vcan_demo.launch.py")
	module = importlib.util.module_from_spec(spec)
	spec.loader.exec_module(module)
	return module


class PlanningTransportSelector(unittest.TestCase):
	def setUp(self):
		self.temporary = tempfile.TemporaryDirectory(prefix="control-link-selector-")
		self.addCleanup(self.temporary.cleanup)
		self.directory = Path(self.temporary.name)
		self.module = load_launch()
		self.profile = {
			"profile_id": "adas",
			"adapter": {"interface": "vcan0"},
			"enabled_sources": ["planning", "teleop"],
		}
		self.runtime_config = self.directory / "vsomeip.json"
		self.runtime_config.write_text("{}\n")
		self.network_config = self.directory / "config" / "adas" / "protocol_transports.yaml"
		self.network_config.parent.mkdir(parents=True)
		self.network_config.write_text("schema_version: 1\n")
		self.executables = self.directory / "lib" / "control_link_adapters"
		self.executables.mkdir(parents=True)
		self.context = LaunchContext()
		self.context.launch_configurations.update({
			"can_interface": "vcan0",
			"start_source": "true",
			"start_lifecycle_activator": "false",
			"start_secondary_source": "false",
			"planning_transport": "ros2",
			"source_id": "planning",
			"secondary_source_id": "teleop",
			"secondary_source_delay_seconds": "5.0",
			"drop_state": "false",
			"corrupt_crc": "false",
			"freeze_counter": "false",
			"fault_code": "0",
			"decision_trace_path": "",
			"decision_trace_queue_capacity": "4096",
			"edge_command_channel": "selector_command",
			"edge_feedback_channel": "selector_feedback",
			"edge_linear_velocity_mps": "0.25",
			"edge_angular_velocity_radps": "0.10",
			"network_config_path": "",
			"vsomeip_config_path": str(self.runtime_config),
		})
		for name, replacement in {
			"get_package_share_directory": lambda package: str(self.directory),
			"get_package_prefix": lambda package: str(self.directory),
			"load_yaml_mapping": lambda *args: self.profile,
			"load_fastdds_profile_path": lambda *args: "/explicit/fastdds.xml",
			"require_up_vcan_interface": lambda *args: None,
			"Node": lambda **kwargs: {"action": "node", **kwargs},
			"ExecuteProcess": lambda **kwargs: {"action": "process", **kwargs},
			"TimerAction": lambda **kwargs: {"action": "timer", **kwargs},
			"LogInfo": lambda **kwargs: {"action": "log", **kwargs},
		}.items():
			patcher = patch.object(self.module, name, replacement)
			patcher.start()
			self.addCleanup(patcher.stop)

	def install_executable(self, name):
		path = self.executables / name
		path.write_text("selector fixture, not a runtime executable\n")
		path.chmod(0o755)
		return path

	def actions(self, **configuration):
		self.context.launch_configurations.update(configuration)
		return self.module._launch_adas_vcan(self.context)

	def test_ros2_default_has_exactly_one_planning_source_and_no_network(self):
		actions = self.actions(vsomeip_config_path="")
		planning = [action for action in actions if action.get("name") == "mock_source_planning"]
		self.assertEqual(len(planning), 1)
		self.assertFalse(any("bridge" in action.get("executable", "") for action in actions))

	def test_unimplemented_grpc_and_unknown_selector_fail(self):
		for selector in ("grpc", "mqtt", "unknown"):
			with self.subTest(selector=selector):
				with self.assertRaisesRegex(RuntimeError, "planning_transport must be"):
					self.actions(planning_transport=selector)

	def test_someip_without_installed_bridge_fails_clearly(self):
		with self.assertRaisesRegex(RuntimeError, "CONTROL_LINK_ENABLE_SOMEIP=ON"):
			self.actions(planning_transport="someip")

	def test_someip_non_executable_bridge_fails(self):
		path = self.install_executable("someip_planning_bridge_node")
		path.chmod(0o644)
		with self.assertRaisesRegex(RuntimeError, "missing="):
			self.actions(planning_transport="someip")

	def test_someip_needs_explicit_runtime_config_even_if_environment_has_one(self):
		self.install_executable("someip_planning_bridge_node")
		with patch.dict("os.environ", {"VSOMEIP_CONFIGURATION": str(self.runtime_config)}):
			with self.assertRaisesRegex(RuntimeError, "vsomeip_config_path must explicitly"):
				self.actions(planning_transport="someip", vsomeip_config_path="")

	def test_someip_rejects_missing_or_relative_runtime_config(self):
		self.install_executable("someip_planning_bridge_node")
		for value in ("relative.json", str(self.directory / "missing.json")):
			with self.subTest(value=value):
				with self.assertRaisesRegex(RuntimeError, "vsomeip_config_path must"):
					self.actions(planning_transport="someip", vsomeip_config_path=value)

	def test_someip_default_network_config_and_explicit_json_reach_bridge(self):
		self.install_executable("someip_planning_bridge_node")
		actions = self.actions(planning_transport="someip")
		bridges = [action for action in actions if action.get("executable") == "someip_planning_bridge_node"]
		self.assertEqual(len(bridges), 1)
		parameters = bridges[0]["parameters"][0]
		self.assertEqual(parameters["network_config_path"], str(self.network_config))
		self.assertEqual(parameters["vsomeip_config_path"], str(self.runtime_config))
		self.assertEqual(bridges[0]["additional_env"]["VSOMEIP_CONFIGURATION"], str(self.runtime_config))
		self.assertEqual(parameters["source_id"], "planning")
		self.assertFalse(any(action.get("executable") == "mock_control_source_node" for action in actions))
		self.assertFalse(any(action["action"] == "process" for action in actions))

	def test_someip_explicit_network_config_is_not_silently_replaced(self):
		self.install_executable("someip_planning_bridge_node")
		alternate = self.directory / "alternate-network.yaml"
		alternate.write_text("schema_version: 1\n")
		actions = self.actions(planning_transport="someip", network_config_path=str(alternate))
		bridge = next(action for action in actions if action.get("executable") == "someip_planning_bridge_node")
		self.assertEqual(bridge["parameters"][0]["network_config_path"], str(alternate))
		with self.assertRaisesRegex(RuntimeError, "network_config_path must"):
			self.actions(planning_transport="someip", network_config_path=str(self.directory / "missing.yaml"))

	def test_network_planning_source_identity_cannot_be_changed(self):
		for selector in ("edge", "someip"):
			with self.subTest(selector=selector):
				with self.assertRaisesRegex(RuntimeError, "fixes source binding to planning"):
					self.actions(planning_transport=selector, source_id="teleop")

	def test_network_mode_rejects_second_ros_planning_publisher(self):
		for selector in ("edge", "someip"):
			with self.subTest(selector=selector):
				with self.assertRaisesRegex(RuntimeError, "cannot start a second ROS planning publisher"):
					self.actions(
						planning_transport=selector,
						start_secondary_source="true",
						secondary_source_id="planning")

	def test_external_someip_allows_ros_teleop_without_bundled_source(self):
		self.install_executable("someip_planning_bridge_node")
		actions = self.actions(
			planning_transport="someip", start_source="false", start_secondary_source="true")
		timer = next(action for action in actions if action["action"] == "timer")
		self.assertEqual(timer["actions"][0]["name"], "mock_source_teleop")
		self.assertFalse(any(action.get("name") == "mock_source_planning" for action in actions))

	def test_edge_mode_still_uses_existing_supervisor(self):
		for name in ("edge_planning_bridge_node", "edge_planning_client", "edge_planning_supervisor"):
			self.install_executable(name)
		actions = self.actions(planning_transport="edge", vsomeip_config_path="")
		self.assertEqual(sum(action.get("executable") == "edge_planning_bridge_node" for action in actions), 1)
		process = next(action for action in actions if action["action"] == "process")
		self.assertEqual(process["cmd"][0], str(self.executables / "edge_planning_supervisor"))
		self.assertFalse(any(action.get("name") == "mock_source_planning" for action in actions))


if __name__ == "__main__":
	unittest.main()
