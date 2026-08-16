import json
import os
import shutil
import subprocess
import threading
import time
import unittest

from ament_index_python.packages import get_package_share_directory
from control_link_bringup.profile_bootstrap import load_fastdds_profile_path
from control_link_interfaces.msg import ControlCommand
from diagnostic_msgs.msg import DiagnosticArray
import launch
from launch_testing.actions import ReadyToTest
from launch_ros.actions import Node
import launch_testing
import pytest
import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
import yaml


def write_yaml(path, document):
	with open(path, "w", encoding="utf-8") as output:
		yaml.safe_dump(document, output, sort_keys=False)


def prepare_reconfiguration_tree(package_share):
	installed_root = os.path.join(package_share, "config")
	installed_profile = os.path.join(installed_root, "adas", "adas_profile.yaml")
	source_profile = os.path.realpath(installed_profile)
	source_root = os.path.dirname(os.path.dirname(source_profile))

	generated_root = os.path.join(os.getcwd(), "generated_reconfiguration_config")
	if os.path.exists(generated_root):
		shutil.rmtree(generated_root)
	shutil.copytree(source_root, generated_root)

	base_profile_path = os.path.join(generated_root, "adas", "adas_profile.yaml")
	base_contract_path = os.path.join(
		generated_root, "common", "gateway_contract.yaml")
	with open(base_profile_path, "r", encoding="utf-8") as profile_file:
		base_profile = yaml.safe_load(profile_file)
	with open(base_contract_path, "r", encoding="utf-8") as contract_file:
		base_contract = yaml.safe_load(contract_file)

	paths = {}
	for name, contract_name in (
		("current", "gateway_contract_x8_current.yaml"),
		("candidate", "gateway_contract_x8_candidate.yaml"),
		("bad_qos", "gateway_contract_x8_bad_qos.yaml"),
	):
		profile = dict(base_profile)
		profile["contract"] = "../common/" + contract_name
		profile_path = os.path.join(
			generated_root, "adas", "reconfigure_" + name + ".yaml")
		write_yaml(profile_path, profile)
		paths[name] = os.path.realpath(profile_path)

	current_contract = dict(base_contract)
	candidate_contract = json.loads(json.dumps(base_contract))
	candidate_contract["contract_version"] = 2
	candidate_contract["gateway"]["command_timeout_ms"] = 120
	bad_qos_contract = json.loads(json.dumps(candidate_contract))
	bad_qos_contract["contract_version"] = 3
	bad_qos_contract["qos_profiles"]["canonical_output"][
		"reliability"] = "best_effort"

	write_yaml(
		os.path.join(generated_root, "common", "gateway_contract_x8_current.yaml"),
		current_contract)
	write_yaml(
		os.path.join(generated_root, "common", "gateway_contract_x8_candidate.yaml"),
		candidate_contract)
	write_yaml(
		os.path.join(generated_root, "common", "gateway_contract_x8_bad_qos.yaml"),
		bad_qos_contract)
	return os.path.realpath(generated_root), paths


@pytest.mark.launch_test
def generate_test_description():
	package_share = get_package_share_directory("control_link_bringup")
	config_root, profiles = prepare_reconfiguration_tree(package_share)
	fastdds_profile = load_fastdds_profile_path(profiles["current"], config_root)
	participant_env = {
		"RMW_IMPLEMENTATION": "rmw_fastrtps_cpp",
		"RMW_FASTRTPS_USE_QOS_FROM_XML": "0",
		"FASTRTPS_DEFAULT_PROFILES_FILE": fastdds_profile,
	}
	common_parameters = {
		"profile_path": profiles["current"],
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
	source = Node(
		package="control_link_adapters",
		executable="mock_control_source_node",
		name="mock_source_planning",
		namespace="control_link",
		output="screen",
		emulate_tty=True,
		additional_env=participant_env,
		parameters=[{
			**common_parameters,
			"source_id": "planning",
			"minimum_subscription_count": 1,
		}],
	)
	activator = Node(
		package="control_link_bringup",
		executable="gateway_lifecycle_activator",
		name="x8_gateway_lifecycle_activator",
		output="screen",
		emulate_tty=True,
		additional_env=participant_env,
		parameters=[{
			"target_fqn": "/control_link/gateway",
			"timeout_ms": 30000,
		}],
	)

	return launch.LaunchDescription([
		gateway,
		adapter,
		source,
		activator,
		ReadyToTest(),
	]), {
		"config_root": config_root,
		"profiles": profiles,
		"participant_env": participant_env,
	}


class TestGatewayReconfiguration(unittest.TestCase):
	@classmethod
	def setUpClass(cls):
		rclpy.init()
		cls.node = rclpy.create_node("x8_reconfiguration_launch_test")
		cls.latest_hash = ""
		cls.lifecycle_label = ""
		cls.canonical_count = 0
		cls.diagnostics_subscription = cls.node.create_subscription(
			DiagnosticArray,
			"/diagnostics",
			cls.on_diagnostics,
			QoSProfile(
				depth=20,
				reliability=ReliabilityPolicy.RELIABLE,
				durability=DurabilityPolicy.VOLATILE))
		cls.canonical_subscription = cls.node.create_subscription(
			ControlCommand,
			"/control_link/output/control_cmd",
			cls.on_canonical,
			QoSProfile(
				depth=20,
				reliability=ReliabilityPolicy.RELIABLE,
				durability=DurabilityPolicy.VOLATILE))
		cls.executor = SingleThreadedExecutor()
		cls.executor.add_node(cls.node)
		cls.spin_thread = threading.Thread(
			target=cls.executor.spin,
			name="x8-launch-test-spin",
			daemon=True)
		cls.spin_thread.start()

	@classmethod
	def tearDownClass(cls):
		cls.executor.shutdown()
		cls.spin_thread.join(timeout=5.0)
		cls.node.destroy_node()
		rclpy.shutdown()

	@classmethod
	def on_diagnostics(cls, message):
		for status in message.status:
			if status.name == "gateway/config":
				for value in status.values:
					if value.key == "decision_config_hash":
						cls.latest_hash = value.value
			elif status.name == "gateway/state":
				for value in status.values:
					if value.key == "lifecycle_state":
						cls.lifecycle_label = value.value

	@classmethod
	def on_canonical(cls, _message):
		cls.canonical_count += 1

	def wait_until(self, predicate, timeout, message):
		deadline = time.monotonic() + timeout
		while time.monotonic() < deadline:
			if predicate():
				return
			time.sleep(0.05)
		self.fail(message)

	def gateway_gid(self):
		for endpoint in self.node.get_publishers_info_by_topic(
			"/control_link/output/control_cmd"):
			if endpoint.node_name == "gateway" and endpoint.node_namespace == "/control_link":
				return bytes(endpoint.endpoint_gid)
		return None

	def run_cli(
			self,
			executable,
			candidate,
			expected_hash,
			result_path,
			last_known_good,
			lock_path,
			config_root,
			participant_env,
			expected_exit):
		completed = subprocess.run(
			[
				executable,
				"--candidate-profile", candidate,
				"--candidate-config-root", config_root,
				"--expected-current-hash", expected_hash,
				"--result", result_path,
				"--last-known-good", last_known_good,
				"--lock-file", lock_path,
				"--timeout-ms", "6000",
			],
			check=False,
			capture_output=True,
			text=True,
			timeout=45.0,
			env={**os.environ, **participant_env})
		self.assertEqual(
			completed.returncode,
			expected_exit,
			msg="stdout:\n" + completed.stdout + "\nstderr:\n" + completed.stderr)
		with open(result_path, "r", encoding="utf-8") as result_file:
			return json.load(result_file)

	def test_no_op_commit_and_qos_rollback(
			self,
			config_root,
			profiles,
			participant_env):
		executable = os.environ["CONTROL_LINK_RECONFIGURE_GATEWAY"]
		result_dir = os.path.join(os.getcwd(), "reconfiguration_results")
		os.makedirs(result_dir, exist_ok=True)
		last_known_good = os.path.join(result_dir, "last_known_good.json")
		lock_path = os.path.join(result_dir, "gateway.lock")

		self.wait_until(
			lambda: self.lifecycle_label == "active",
			30.0,
			"Gateway did not reach ACTIVE before X8 transaction")
		self.wait_until(
			lambda: bool(self.latest_hash),
			5.0,
			"Gateway config diagnostics were not observed")
		self.wait_until(
			lambda: self.gateway_gid() is not None,
			5.0,
			"Gateway canonical publisher was not discovered")

		initial_hash = self.latest_hash
		initial_gid = self.gateway_gid()
		canonical_before_no_op = self.canonical_count
		no_op = self.run_cli(
			executable,
			profiles["current"],
			initial_hash,
			os.path.join(result_dir, "no_op.json"),
			last_known_good,
			lock_path,
			config_root,
			participant_env,
			0)
		self.assertEqual(no_op["status"], "NO_OP")
		self.assertEqual(self.lifecycle_label, "active")
		self.assertEqual(self.gateway_gid(), initial_gid)
		self.wait_until(
			lambda: self.canonical_count > canonical_before_no_op,
			1.0,
			"canonical output did not continue across no-op precheck")

		stale = self.run_cli(
			executable,
			profiles["candidate"],
			"0" * 64,
			os.path.join(result_dir, "stale.json"),
			last_known_good,
			lock_path,
			config_root,
			participant_env,
			2)
		self.assertEqual(stale["status"], "REJECTED")
		self.assertEqual(self.lifecycle_label, "active")
		self.assertEqual(self.gateway_gid(), initial_gid)

		committed = self.run_cli(
			executable,
			profiles["candidate"],
			initial_hash,
			os.path.join(result_dir, "committed.json"),
			last_known_good,
			lock_path,
			config_root,
			participant_env,
			0)
		self.assertEqual(committed["status"], "COMMITTED")
		candidate_hash = committed["requested_decision_config_hash"]
		self.assertNotEqual(candidate_hash, initial_hash)
		self.wait_until(
			lambda: self.latest_hash == candidate_hash,
			5.0,
			"candidate config hash was not published")
		self.wait_until(
			lambda: self.gateway_gid() not in (None, initial_gid),
			5.0,
			"candidate canonical publisher GID was not rebuilt")
		candidate_gid = self.gateway_gid()
		self.wait_until(
			lambda: self.lifecycle_label == "active",
			5.0,
			"candidate diagnostics did not report Lifecycle ACTIVE")
		with open(last_known_good, "r", encoding="utf-8") as descriptor_file:
			descriptor = json.load(descriptor_file)
		self.assertEqual(descriptor["decision_config_hash"], candidate_hash)

		rolled_back = self.run_cli(
			executable,
			profiles["bad_qos"],
			candidate_hash,
			os.path.join(result_dir, "rolled_back.json"),
			last_known_good,
			lock_path,
			config_root,
			participant_env,
			3)
		self.assertEqual(rolled_back["status"], "ROLLED_BACK")
		self.assertEqual(rolled_back["failed_step"], "VERIFY_CANDIDATE")
		self.assertEqual(
			rolled_back["final_active_decision_config_hash"], candidate_hash)
		self.wait_until(
			lambda: self.latest_hash == candidate_hash,
			5.0,
			"rollback did not restore the previous config hash")
		self.wait_until(
			lambda: self.gateway_gid() not in (None, candidate_gid),
			5.0,
			"rollback did not rebuild the previous canonical publisher")
		self.wait_until(
			lambda: self.lifecycle_label == "active",
			5.0,
			"rollback diagnostics did not report Lifecycle ACTIVE")
		with open(last_known_good, "r", encoding="utf-8") as descriptor_file:
			post_rollback_descriptor = json.load(descriptor_file)
		self.assertEqual(
			post_rollback_descriptor["decision_config_hash"], candidate_hash)


@launch_testing.post_shutdown_test()
class TestGatewayReconfigurationShutdown(unittest.TestCase):
	def test_processes_exit_cleanly(self, proc_info):
		launch_testing.asserts.assertExitCodes(proc_info)
