#!/usr/bin/env python3
"""静态能力与配置 owner 检查；生产 SOME/IP parser 另有 C++ focused 回归。"""

import copy
import pathlib
import unittest

import yaml


ROOT = pathlib.Path(__file__).resolve().parents[2]


def positive_resource(section, field):
    value = section.get(field)
    if type(value) is not int or not 0 < value <= 0xFFFFFFFF:
        raise ValueError(f"{field} must be a positive bounded uint32 resource")
    return value


def check_contract(configuration):
    if configuration.get("schema_version") != 1:
        raise ValueError("unsupported schema")
    if configuration.get("configuration_role") != "runtime_someip_and_future_contracts":
        raise ValueError("unexpected runtime/contract role")
    planning = configuration["planning"]
    if planning != {
        "source_id": "planning", "transport": "ros2",
        "allowed_transports": ["ros2", "edge", "someip", "grpc"],
        "runtime_available_transports": ["ros2", "edge", "someip"], "exclusive": True,
    }:
        raise ValueError("planning transport selection must stay exclusive and truthful")
    for protocol in ("someip", "grpc", "mqtt"):
        if configuration[protocol]["enabled"] is not False:
            raise ValueError("protocols stay disabled unless explicitly selected at launch")
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
    if set(someip["methods"]) != {
            "open_session", "submit", "get_status", "keep_session", "close_session"}:
        raise ValueError("missing observable SOME/IP holder lifecycle method")
    if len(set(methods + events)) != len(methods + events):
        raise ValueError("SOME/IP identifiers overlap")
    if not all(0 < value < 0x8000 for value in methods):
        raise ValueError("invalid method id")
    if not all(0x8000 <= value < 0xFFFF for value in events):
        raise ValueError("invalid event id")
    keepalive_period = positive_resource(someip, "session_keepalive_period_ms")
    idle_timeout = positive_resource(someip, "session_idle_timeout_ms")
    if keepalive_period >= idle_timeout:
        raise ValueError("SOME/IP keep period must leave a bounded idle timeout margin")
    positive_resource(configuration["control_session_resources"], "retired_open_capacity")
    grpc = configuration["grpc"]
    if "max_rpc_deadline_ms" in grpc:
        raise ValueError("unary/control/readonly RPC lifetimes must not share the old deadline")
    deadlines = {field: positive_resource(grpc, field) for field in (
        "max_unary_deadline_ms", "control_open_timeout_ms",
        "max_control_stream_lifetime_ms", "max_readonly_stream_lifetime_ms")}
    if deadlines["max_control_stream_lifetime_ms"] <= max(
            deadlines["max_unary_deadline_ms"], deadlines["control_open_timeout_ms"]):
        raise ValueError("control stream lifetime must outlive its short open/unary budget")
    if deadlines["max_readonly_stream_lifetime_ms"] <= deadlines["max_unary_deadline_ms"]:
        raise ValueError("readonly stream lifetime must be independent of unary deadline")
    for field in ("readonly_inflight_limit", "max_status_subscribers", "max_message_bytes",
                  "shutdown_timeout_ms"):
        positive_resource(grpc, field)
    positive_resource(someip, "readonly_inflight_limit")
    positive_resource(someip, "shutdown_timeout_ms")
    if positive_resource(someip, "max_payload_bytes") < someip["command_payload_bytes"]:
        raise ValueError("SOME/IP payload limit excludes its command codec")
    if grpc["max_message_bytes"] < someip["command_payload_bytes"]:
        raise ValueError("gRPC message resource limit cannot encode a control target")
    mqtt = configuration["mqtt"]
    if mqtt["control_subscriptions"] or mqtt["disconnected_buffering"]:
        raise ValueError("MQTT is telemetry only without offline control buffering")
    if mqtt["telemetry_retain"] or mqtt["alert_retain"]:
        raise ValueError("live observations cannot use retained messages")
    positive_resource(mqtt, "alert_queue_capacity")
    positive_resource(mqtt, "max_message_bytes")
    for protocol in ("someip", "grpc", "mqtt"):
        section = configuration[protocol]
        if section.get("pending_latest_capacity", section.get("latest_capacity_per_group")) != 1:
            raise ValueError("unbounded or FIFO control/status backlog")
        if not positive_resource(section, "reconnect_initial_ms") <= positive_resource(
                section, "reconnect_max_ms"):
            raise ValueError("invalid bounded transport backoff")
    if grpc["subscriber_latest_capacity"] != 1:
        raise ValueError("slow readonly subscriber must have its own latest slot")
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

    def test_rejects_duplicate_lifecycle_method(self):
        self.configuration["someip"]["methods"]["keep_session"] = 1
        with self.assertRaisesRegex(ValueError, "identifiers overlap"):
            check_contract(self.configuration)

    def test_rejects_legacy_combined_deadline(self):
        self.configuration["grpc"]["max_rpc_deadline_ms"] = 2000
        with self.assertRaisesRegex(ValueError, "old deadline"):
            check_contract(self.configuration)

    def test_rejects_missing_deadline_categories(self):
        for field in ("max_unary_deadline_ms", "control_open_timeout_ms",
                      "max_control_stream_lifetime_ms", "max_readonly_stream_lifetime_ms"):
            configuration = copy.deepcopy(self.configuration)
            del configuration["grpc"][field]
            with self.subTest(field=field), self.assertRaises(ValueError):
                check_contract(configuration)

    def test_rejects_short_control_or_readonly_stream_lifetime(self):
        for field in ("max_control_stream_lifetime_ms", "max_readonly_stream_lifetime_ms"):
            configuration = copy.deepcopy(self.configuration)
            configuration["grpc"][field] = configuration["grpc"]["max_unary_deadline_ms"]
            with self.subTest(field=field), self.assertRaises(ValueError):
                check_contract(configuration)

    def test_rejects_keep_period_without_idle_margin(self):
        for value in (0, 5000, 5001, True):
            configuration = copy.deepcopy(self.configuration)
            configuration["someip"]["session_keepalive_period_ms"] = value
            with self.subTest(value=value), self.assertRaises(ValueError):
                check_contract(configuration)

    def test_rejects_missing_lifecycle_method(self):
        for field in ("keep_session", "close_session"):
            configuration = copy.deepcopy(self.configuration)
            del configuration["someip"]["methods"][field]
            with self.subTest(field=field), self.assertRaises(ValueError):
                check_contract(configuration)

    def test_rejects_invalid_session_and_subscriber_resources(self):
        resources = (
            ("control_session_resources", "retired_open_capacity"),
            ("someip", "readonly_inflight_limit"),
            ("grpc", "readonly_inflight_limit"),
            ("grpc", "max_status_subscribers"),
            ("grpc", "shutdown_timeout_ms"),
            ("mqtt", "alert_queue_capacity"),
        )
        for section, field in resources:
            for value in (0, -1, True, 0x100000000):
                configuration = copy.deepcopy(self.configuration)
                configuration[section][field] = value
                with self.subTest(section=section, field=field, value=value), self.assertRaises(ValueError):
                    check_contract(configuration)

    def test_rejects_missing_bounded_grpc_shutdown(self):
        del self.configuration["grpc"]["shutdown_timeout_ms"]
        with self.assertRaisesRegex(ValueError, "shutdown_timeout_ms"):
            check_contract(self.configuration)

    def test_rejects_backlogs_and_unusable_payload_limits(self):
        for section, field, value in (
            ("grpc", "subscriber_latest_capacity", 2),
            ("someip", "pending_latest_capacity", 2),
            ("grpc", "max_message_bytes", 1),
            ("someip", "max_payload_bytes", 63),
        ):
            configuration = copy.deepcopy(self.configuration)
            configuration[section][field] = value
            with self.subTest(section=section, field=field), self.assertRaises(ValueError):
                check_contract(configuration)


if __name__ == "__main__":
    unittest.main()
