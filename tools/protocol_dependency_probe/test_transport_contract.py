#!/usr/bin/env python3
"""P0 静态契约检查，不是尚未实现的运行时配置 parser。"""

import copy
import pathlib
import unittest

import yaml


ROOT = pathlib.Path(__file__).resolve().parents[2]


def check_contract(configuration):
    if configuration.get("schema_version") != 1:
        raise ValueError("unsupported schema")
    if configuration.get("configuration_role") != "contract_only":
        raise ValueError("P0 cannot enable runtime protocol consumers")
    planning = configuration["planning"]
    if planning != {
        "source_id": "planning", "transport": "ros2",
        "allowed_transports": ["ros2", "edge", "someip", "grpc"],
        "runtime_available_transports": ["ros2", "edge"], "exclusive": True,
    }:
        raise ValueError("planning transport selection must stay exclusive and truthful")
    for protocol in ("someip", "grpc", "mqtt"):
        if configuration[protocol]["enabled"] is not False:
            raise ValueError("P0 has no runtime protocol target")
    owners = configuration["safety_owners"]
    for field in ("contract", "source_policy", "profile"):
        if not (ROOT / "config" / owners[field]).is_file():
            raise ValueError("missing immutable safety owner")
    if owners["timestamp_domain"] != "SYSTEM" or owners["reference_frame"] != "body":
        raise ValueError("unsupported time or frame mapping")
    someip = configuration["someip"]
    if someip["command_payload_bytes"] != 64 or not someip["service_discovery"]["enabled"]:
        raise ValueError("fixed codec and Service Discovery are mandatory")
    for name in ("service_id", "instance_id"):
        if not 0 < someip[name] < 0xFFFF:
            raise ValueError("invalid SOME/IP identity")
    methods = list(someip["methods"].values())
    events = list(someip["events"].values())
    if len(set(methods + events)) != len(methods + events):
        raise ValueError("SOME/IP identifiers overlap")
    if not all(0 < value < 0x8000 for value in methods):
        raise ValueError("invalid method id")
    if not all(0x8000 <= value < 0xFFFF for value in events):
        raise ValueError("invalid event id")
    mqtt = configuration["mqtt"]
    if mqtt["control_subscriptions"] or mqtt["disconnected_buffering"]:
        raise ValueError("MQTT is telemetry only without offline control buffering")
    if mqtt["telemetry_retain"] or mqtt["alert_retain"]:
        raise ValueError("live observations cannot use retained messages")
    for protocol in ("someip", "grpc", "mqtt"):
        section = configuration[protocol]
        if section.get("pending_latest_capacity", section.get("latest_capacity_per_group")) != 1:
            raise ValueError("unbounded or FIFO control/status backlog")
        if not 0 < section["reconnect_initial_ms"] <= section["reconnect_max_ms"]:
            raise ValueError("invalid bounded transport backoff")
    forbidden = {"command_timeout_ms", "lease_timeout_ms", "recovery_valid_samples",
                 "max_abs_linear_velocity_mps", "max_abs_angular_velocity_radps"}
    if any(forbidden.intersection(configuration[protocol])
           for protocol in ("someip", "grpc", "mqtt")):
        raise ValueError("network adapter must not duplicate safety thresholds")


class TransportContractTest(unittest.TestCase):
    def setUp(self):
        self.configuration = yaml.safe_load(
            (ROOT / "config/adas/protocol_transports.yaml").read_text())

    def test_actual_configuration(self):
        check_contract(self.configuration)

    def test_rejects_control_or_retained_telemetry(self):
        for field, value in (("control_subscriptions", ["control"]),
                             ("telemetry_retain", True), ("disconnected_buffering", True)):
            configuration = copy.deepcopy(self.configuration)
            configuration["mqtt"][field] = value
            with self.assertRaises(ValueError):
                check_contract(configuration)

    def test_rejects_unimplemented_transport_activation(self):
        self.configuration["planning"]["transport"] = "grpc"
        with self.assertRaises(ValueError):
            check_contract(self.configuration)

    def test_rejects_duplicate_method_event_and_safety_rules(self):
        configuration = copy.deepcopy(self.configuration)
        configuration["someip"]["events"]["gateway_status"] = 1
        with self.assertRaises(ValueError):
            check_contract(configuration)
        self.configuration["grpc"]["command_timeout_ms"] = 1000
        with self.assertRaises(ValueError):
            check_contract(self.configuration)


if __name__ == "__main__":
    unittest.main()
