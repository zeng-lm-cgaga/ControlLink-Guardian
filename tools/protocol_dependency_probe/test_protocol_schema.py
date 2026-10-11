import argparse
import hashlib
import json
import os
from pathlib import Path
import re
import shlex
import subprocess
import sys
import tempfile
import unittest

from google.protobuf import descriptor_pb2


REPOSITORY = Path(__file__).resolve().parents[2]
SCHEMA = REPOSITORY / "protocols" / "control_link_network_v1.proto"
OUTPUT_DIRECTORY = None
PROTOC = os.environ.get("PROTOC", "protoc")
COMPILER = os.environ.get("CXX", "c++")


CPP_PROBE = r'''
#include "control_link_network_v1.pb.h"

#include <google/protobuf/descriptor.h>
#include <google/protobuf/util/json_util.h>

#include <cmath>
#include <cstdint>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <string>

namespace network = control_link::network::v1;

void require(bool condition, const std::string &description)
{
  if (!condition) {
    throw std::runtime_error(description);
  }
}

template <typename Message>
Message round_trip(const Message &message)
{
  std::string wire;
  require(message.SerializeToString(&wire), "serialize");
  Message parsed;
  require(parsed.ParseFromString(wire), "parse");
  return parsed;
}

std::string schema_fingerprint_from_source()
{
  const std::string hexadecimal = "SCHEMA_FINGERPRINT_HEX";
  require(hexadecimal.size() == 64, "source SHA-256 hexadecimal length");
  std::string fingerprint;
  for (std::size_t offset = 0; offset < hexadecimal.size(); offset += 2) {
    fingerprint.push_back(static_cast<char>(std::stoul(hexadecimal.substr(offset, 2), nullptr, 16)));
  }
  return fingerprint;
}

void command_presence()
{
  network::CommandTarget absent;
  require(absent.schema_version_presence_case() == network::CommandTarget::SCHEMA_VERSION_PRESENCE_NOT_SET, "missing version");
  require(!absent.has_key(), "missing key");
  require(absent.source_stamp_presence_case() == network::CommandTarget::SOURCE_STAMP_PRESENCE_NOT_SET, "missing stamp");
  require(absent.clock_domain_presence_case() == network::CommandTarget::CLOCK_DOMAIN_PRESENCE_NOT_SET, "missing clock");
  require(absent.mode_presence_case() == network::CommandTarget::MODE_PRESENCE_NOT_SET, "missing mode");
  require(absent.linear_velocity_presence_case() == network::CommandTarget::LINEAR_VELOCITY_PRESENCE_NOT_SET, "missing linear");
  require(absent.angular_velocity_presence_case() == network::CommandTarget::ANGULAR_VELOCITY_PRESENCE_NOT_SET, "missing angular");
  network::CommandTarget explicit_zero;
  explicit_zero.set_schema_version(0);
  explicit_zero.mutable_key()->set_session_token(std::string(16, 's'));
  explicit_zero.mutable_key()->set_source_sequence(0);
  explicit_zero.set_source_stamp_ns(0);
  explicit_zero.set_clock_domain(network::CLOCK_DOMAIN_SYSTEM);
  explicit_zero.set_mode(network::COMMAND_MODE_NORMAL);
  explicit_zero.set_linear_velocity_mps(0.0);
  explicit_zero.set_angular_velocity_radps(0.0);
  auto parsed = round_trip(explicit_zero);
  require(parsed.schema_version_presence_case() == network::CommandTarget::kSchemaVersion, "explicit version zero");
  require(parsed.has_key() && parsed.key().source_sequence_presence_case() == network::CommandKey::kSourceSequence, "explicit sequence zero");
  require(parsed.source_stamp_presence_case() == network::CommandTarget::kSourceStampNs, "explicit stamp zero");
  require(parsed.clock_domain_presence_case() == network::CommandTarget::kClockDomain, "explicit clock");
  require(parsed.mode_presence_case() == network::CommandTarget::kMode && parsed.mode() == network::COMMAND_MODE_NORMAL, "explicit NORMAL zero");
  require(parsed.linear_velocity_presence_case() == network::CommandTarget::kLinearVelocityMps, "explicit linear zero");
  require(parsed.angular_velocity_presence_case() == network::CommandTarget::kAngularVelocityRadps, "explicit angular zero");
  require(parsed.ByteSizeLong() != absent.ByteSizeLong(), "missing and explicit zero have distinct wire");
}

void raw_business_values()
{
  network::CommandTarget target;
  target.set_schema_version(1);
  target.mutable_key()->set_session_token(std::string(16, 'a'));
  target.mutable_key()->set_source_sequence(std::numeric_limits<std::uint64_t>::max());
  target.set_source_stamp_ns(std::numeric_limits<std::int64_t>::min());
  target.set_clock_domain(network::CLOCK_DOMAIN_SYSTEM);
  const auto *mode_field = target.GetDescriptor()->FindFieldByName("mode");
  target.GetReflection()->SetEnumValue(&target, mode_field, 253);
  target.set_linear_velocity_mps(std::numeric_limits<double>::quiet_NaN());
  target.set_angular_velocity_radps(std::numeric_limits<double>::infinity());
  auto parsed = round_trip(target);
  require(parsed.key().source_sequence() == std::numeric_limits<std::uint64_t>::max(), "uint64 sequence lossless");
  require(parsed.source_stamp_ns() == std::numeric_limits<std::int64_t>::min(), "raw signed stamp lossless");
  require(parsed.GetReflection()->GetEnumValue(parsed, mode_field) == 253, "unknown enum retained for Gateway");
  require(std::isnan(parsed.linear_velocity_mps()) && std::isinf(parsed.angular_velocity_radps()), "nonfinite not normalized by codec");
  target.set_source_stamp_ns(std::numeric_limits<std::int64_t>::max());
  require(round_trip(target).source_stamp_ns() == std::numeric_limits<std::int64_t>::max(), "maximum raw stamp lossless");
}

void identity_and_receipt()
{
  network::SubmitReceipt receipt;
  require(!receipt.rpc_received(), "missing receipt is not received");
  require(receipt.forwarding_state() == network::FORWARDING_STATE_UNKNOWN, "default forwarding unknown");
  require(!receipt.has_gateway_acceptance(), "no implicit acceptance observation");
  require(receipt.gateway_acceptance().state() == network::GATEWAY_ACCEPTANCE_STATE_UNKNOWN, "initial acceptance unknown");
  receipt.set_rpc_received(true);
  receipt.set_forwarding_state(network::FORWARDING_STATE_FORWARDED);
  receipt.mutable_key()->set_session_token(std::string(16, 's'));
  receipt.mutable_key()->set_source_sequence(42);
  receipt.mutable_forwarded()->mutable_key()->CopyFrom(receipt.key());
  receipt.mutable_forwarded()->mutable_publisher()->set_rmw_implementation("rmw_fastrtps_cpp");
  receipt.mutable_forwarded()->mutable_publisher()->set_gid(std::string(24, 'g'));
  auto parsed = round_trip(receipt);
  require(parsed.rpc_received(), "RPC received survives");
  require(parsed.forwarding_state() == network::FORWARDING_STATE_FORWARDED, "forwarded fact survives");
  require(parsed.gateway_acceptance().state() == network::GATEWAY_ACCEPTANCE_STATE_UNKNOWN, "forwarding cannot mutate acceptance");
  require(parsed.key().session_token().size() == 16 && parsed.forwarded().publisher().gid().size() == 24, "identity sizes independent");
  require(parsed.key().session_token() != parsed.forwarded().publisher().gid(), "session not GID");
}

void independent_observations()
{
  network::StatusEnvelope envelope;
  envelope.set_schema_version(1);
  envelope.set_publisher_instance_token(std::string(16, 'i'));
  envelope.set_publication_sequence(0);
  envelope.set_published_system_ns(1000);
  envelope.set_published_boottime_ns(500);
  auto *gateway = envelope.mutable_status()->mutable_gateway();
  gateway->mutable_observation()->set_observed_system_ns(111);
  gateway->mutable_observation()->set_received_boottime_ns(101);
  gateway->mutable_observation()->set_validity(network::OBSERVATION_VALIDITY_VALID);
  gateway->set_reason_code(65535);
  auto *source = envelope.mutable_status()->add_sources();
  source->set_source_id("planning");
  source->mutable_observation()->set_observed_system_ns(222);
  source->mutable_observation()->set_received_boottime_ns(202);
  source->mutable_observation()->set_validity(network::OBSERVATION_VALIDITY_STALE);
  source->set_last_source_sequence(7);
  source->set_last_reject_reason(65535);
  envelope.mutable_status()->mutable_latest_forwarded()->mutable_key()->set_source_sequence(9);
  auto *vehicle = envelope.mutable_status()->mutable_vehicle();
  vehicle->mutable_observation()->set_observed_system_ns(333);
  vehicle->mutable_observation()->set_received_boottime_ns(303);
  vehicle->mutable_observation()->set_validity(network::OBSERVATION_VALIDITY_INVALID);
  vehicle->set_fault_code(511);
  auto parsed = round_trip(envelope);
  require(parsed.status().gateway().observation().observed_system_ns() == 111, "gateway original time");
  require(parsed.status().sources(0).observation().observed_system_ns() == 222, "source original time");
  require(parsed.status().vehicle().observation().observed_system_ns() == 333, "vehicle original time");
  require(parsed.status().gateway().observation().received_boottime_ns() == 101, "gateway receive domain");
  require(parsed.status().sources(0).observation().received_boottime_ns() == 202, "source receive domain");
  require(parsed.status().vehicle().observation().received_boottime_ns() == 303, "vehicle receive domain");
  require(parsed.status().sources(0).observation().validity() == network::OBSERVATION_VALIDITY_STALE, "source independent stale");
  require(parsed.status().vehicle().observation().validity() == network::OBSERVATION_VALIDITY_INVALID, "vehicle independent invalid");
  require(parsed.status().latest_gateway_acceptance().state() == network::GATEWAY_ACCEPTANCE_STATE_UNKNOWN, "sequence gap cannot imply acceptance");
  require(parsed.publication_sequence_presence_case() == network::StatusEnvelope::kPublicationSequence, "explicit sequence zero retained");
}

void json_mapping()
{
  network::CommandTarget target;
  target.set_schema_version(1);
  target.mutable_key()->set_session_token(std::string(16, 's'));
  target.mutable_key()->set_source_sequence(std::numeric_limits<std::uint64_t>::max());
  target.set_source_stamp_ns(0);
  target.set_clock_domain(network::CLOCK_DOMAIN_SYSTEM);
  target.set_mode(network::COMMAND_MODE_NORMAL);
  target.set_linear_velocity_mps(0);
  target.set_angular_velocity_radps(0);
  google::protobuf::util::JsonPrintOptions options;
  options.preserve_proto_field_names = true;
  std::string json;
  require(google::protobuf::util::MessageToJsonString(target, &json, options).ok(), "JSON encode");
  require(json.find("source_stamp_ns") != std::string::npos, "snake case mapping");
  require(json.find("18446744073709551615") != std::string::npos, "uint64 precision retained");
  require(json.find("COMMAND_MODE_NORMAL") != std::string::npos, "explicit default enum present");
  network::CommandTarget parsed;
  require(google::protobuf::util::JsonStringToMessage(json, &parsed).ok(), "JSON parse");
  require(parsed.key().source_sequence() == target.key().source_sequence(), "JSON sequence exact");
  require(parsed.key().session_token() == target.key().session_token(), "JSON bytes exact");
  require(parsed.source_stamp_presence_case() == network::CommandTarget::kSourceStampNs, "JSON explicit zero stamp presence");
  require(parsed.mode_presence_case() == network::CommandTarget::kMode, "JSON explicit NORMAL presence");
  require(parsed.linear_velocity_presence_case() == network::CommandTarget::kLinearVelocityMps, "JSON explicit zero velocity presence");
  std::cout << json << '\n';
}

void status_json_mapping()
{
  network::StatusEnvelope envelope;
  envelope.set_schema_version(1);
  envelope.set_publisher_instance_token(std::string(16, 'i'));
  envelope.set_publication_sequence(std::numeric_limits<std::uint64_t>::max());
  envelope.set_published_system_ns(0);
  envelope.set_published_boottime_ns(0);
  auto *source = envelope.mutable_status()->add_sources();
  source->set_source_id("planning");
  source->set_last_source_sequence(std::numeric_limits<std::uint64_t>::max());
  source->mutable_observation()->set_observed_system_ns(0);
  source->mutable_observation()->set_received_boottime_ns(0);
  source->mutable_observation()->set_validity(network::OBSERVATION_VALIDITY_STALE);
  source->mutable_endpoint_publisher()->set_rmw_implementation("rmw_fastrtps_cpp");
  source->mutable_endpoint_publisher()->set_gid(std::string(24, 'g'));
  envelope.mutable_status()->mutable_session()->set_control_session_state(network::CONTROL_SESSION_STATE_ADMITTED);
  envelope.mutable_status()->mutable_session()->mutable_ingress_publisher()->set_gid(std::string(24, 'g'));
  envelope.mutable_status()->mutable_latest_forwarded()->mutable_key()->set_source_sequence(7);
  envelope.mutable_status()->mutable_latest_gateway_acceptance()->mutable_key()->set_source_sequence(6);
  google::protobuf::util::JsonPrintOptions options;
  options.preserve_proto_field_names = true;
  std::string json;
  require(google::protobuf::util::MessageToJsonString(envelope, &json, options).ok(), "status JSON encode");
  require(json.find("publication_sequence\":\"18446744073709551615\"") != std::string::npos, "uint64 publication sequence uses precise string");
  require(json.find("OBSERVATION_VALIDITY_STALE") != std::string::npos, "stale fact preserved");
  network::StatusEnvelope parsed;
  require(google::protobuf::util::JsonStringToMessage(json, &parsed).ok(), "status JSON parse");
  require(parsed.publisher_instance_token() == envelope.publisher_instance_token(), "publication instance bytes exact");
  require(parsed.publication_sequence() == envelope.publication_sequence(), "publication sequence exact");
  require(parsed.published_system_presence_case() == network::StatusEnvelope::kPublishedSystemNs, "zero publish SYSTEM presence");
  require(parsed.published_boottime_presence_case() == network::StatusEnvelope::kPublishedBoottimeNs, "zero publish BOOTTIME presence");
  require(parsed.status().sources(0).last_source_sequence() == source->last_source_sequence(), "source sequence exact");
  require(parsed.status().sources(0).endpoint_publisher().gid() == source->endpoint_publisher().gid(), "GID base64 round trip");
  require(parsed.status().sources(0).observation().observed_system_presence_case() == network::ObservationMetadata::kObservedSystemNs, "zero original observation presence");
  require(parsed.status().sources(0).observation().validity() == network::OBSERVATION_VALIDITY_STALE, "retained copy not freshened by JSON");
  require(parsed.status().session().session_token().empty(), "readonly fixture has no control token");
  require(parsed.status().latest_forwarded().key().session_token().empty(), "readonly forwarded key has no token");
  require(parsed.status().latest_gateway_acceptance().key().session_token().empty(), "readonly acceptance key has no token");
  require(parsed.status().latest_forwarded().key().source_sequence() == 7, "redaction retains public sequence fact");
  require(!parsed.status().has_gateway(), "missing gateway remains absent");
  require(parsed.status().gateway().observation().validity() == network::OBSERVATION_VALIDITY_UNKNOWN, "missing gateway not valid");
  std::cout << json << '\n';
}

void schema_fingerprint()
{
  const auto fingerprint = schema_fingerprint_from_source();
  require(fingerprint.size() == 32, "source fingerprint raw bytes length");
  network::StatusEnvelope envelope;
  envelope.set_schema_version(1);
  envelope.set_schema_fingerprint(fingerprint);
  network::Capabilities capabilities;
  capabilities.set_schema_version(1);
  capabilities.set_schema_fingerprint(fingerprint);
  require(round_trip(envelope).schema_fingerprint() == fingerprint, "status fingerprint exact");
  require(round_trip(capabilities).schema_fingerprint() == fingerprint, "capabilities fingerprint exact");
  google::protobuf::util::JsonPrintOptions options;
  options.preserve_proto_field_names = true;
  std::string json;
  require(google::protobuf::util::MessageToJsonString(envelope, &json, options).ok(), "fingerprint JSON encode");
  network::StatusEnvelope parsed;
  require(google::protobuf::util::JsonStringToMessage(json, &parsed).ok(), "fingerprint JSON parse");
  require(parsed.schema_fingerprint() == fingerprint, "fingerprint JSON bytes exact");
  std::cout << json << '\n';
}

void diagnostic_json_mapping()
{
  network::StatusEnvelope envelope;
  envelope.set_schema_version(1);
  envelope.set_schema_fingerprint(schema_fingerprint_from_source());
  auto *first = envelope.mutable_status()->add_alerts();
  first->mutable_observation()->set_observed_system_ns(111);
  first->mutable_observation()->set_received_boottime_ns(101);
  first->mutable_observation()->set_validity(network::OBSERVATION_VALIDITY_STALE);
  first->set_level(3);
  first->set_name("gateway/endpoints");
  first->set_message("last observed diagnostic");
  first->set_hardware_id("demo-vcan0");
  auto *first_value = first->add_values();
  first_value->set_key("endpoint");
  first_value->set_value("planning-previous");
  auto *second_value = first->add_values();
  second_value->set_key("endpoint");
  second_value->set_value("planning-current");
  auto *second = envelope.mutable_status()->add_alerts();
  second->mutable_observation()->set_observed_system_ns(222);
  second->mutable_observation()->set_received_boottime_ns(202);
  second->mutable_observation()->set_validity(network::OBSERVATION_VALIDITY_VALID);
  second->set_level(1);
  second->set_name("gateway/transport");
  auto binary = round_trip(envelope);
  require(binary.status().alerts_size() == 2, "diagnostic binary repeated observations");
  google::protobuf::util::JsonPrintOptions options;
  options.preserve_proto_field_names = true;
  std::string json;
  require(google::protobuf::util::MessageToJsonString(envelope, &json, options).ok(), "diagnostic JSON encode");
  network::StatusEnvelope parsed;
  require(google::protobuf::util::JsonStringToMessage(json, &parsed).ok(), "diagnostic JSON parse");
  require(parsed.status().alerts_size() == 2, "diagnostic JSON repeated observations");
  require(parsed.status().alerts(0).level() == 3 && parsed.status().alerts(1).level() == 1, "diagnostic level raw mapping");
  require(parsed.status().alerts(0).name() == first->name() && parsed.status().alerts(0).message() == first->message(), "diagnostic strings exact");
  require(parsed.status().alerts(0).hardware_id() == first->hardware_id(), "diagnostic hardware exact");
  require(parsed.status().alerts(0).values_size() == 2, "duplicate key values retained");
  require(parsed.status().alerts(0).values(0).key() == parsed.status().alerts(0).values(1).key(), "duplicate key order retained");
  require(parsed.status().alerts(0).values(0).value() == "planning-previous" && parsed.status().alerts(0).values(1).value() == "planning-current", "values not overwritten by map");
  require(parsed.status().alerts(0).observation().observed_system_ns() == 111 && parsed.status().alerts(1).observation().observed_system_ns() == 222, "independent diagnostic SYSTEM times");
  require(parsed.status().alerts(0).observation().received_boottime_ns() == 101 && parsed.status().alerts(1).observation().received_boottime_ns() == 202, "independent diagnostic BOOTTIME times");
  require(parsed.status().alerts(0).observation().validity() == network::OBSERVATION_VALIDITY_STALE, "diagnostic JSON does not freshen stale observation");
  require(!parsed.status().has_gateway(), "diagnostic text does not synthesize gateway state");
  std::cout << json << '\n';
}

void availability_json_mapping()
{
  network::AvailabilityEnvelope absent;
  require(absent.schema_version_presence_case() == network::AvailabilityEnvelope::SCHEMA_VERSION_PRESENCE_NOT_SET, "availability missing version");
  require(absent.presence_hint_presence_case() == network::AvailabilityEnvelope::PRESENCE_HINT_PRESENCE_NOT_SET, "availability missing presence hint");
  require(absent.presence_hint() == network::AVAILABILITY_PRESENCE_HINT_UNSPECIFIED, "missing hint not ONLINE");
  require(absent.published_system_presence_case() == network::AvailabilityEnvelope::PUBLISHED_SYSTEM_PRESENCE_NOT_SET, "availability missing original publish time");
  for (auto hint : {network::AVAILABILITY_PRESENCE_HINT_ONLINE, network::AVAILABILITY_PRESENCE_HINT_OFFLINE}) {
    network::AvailabilityEnvelope envelope;
    envelope.set_schema_version(1);
    envelope.set_schema_fingerprint(schema_fingerprint_from_source());
    envelope.set_publisher_instance_token(std::string(16, 'i'));
    envelope.set_presence_hint(hint);
    envelope.set_published_system_ns(123456789);
    require(round_trip(envelope).presence_hint() == hint, "availability binary presence exact");
    google::protobuf::util::JsonPrintOptions options;
    options.preserve_proto_field_names = true;
    std::string json;
    require(google::protobuf::util::MessageToJsonString(envelope, &json, options).ok(), "availability JSON encode");
    network::AvailabilityEnvelope parsed;
    require(google::protobuf::util::JsonStringToMessage(json, &parsed).ok(), "availability JSON parse");
    require(parsed.schema_version_presence_case() == network::AvailabilityEnvelope::kSchemaVersion, "availability explicit version retained");
    require(parsed.schema_fingerprint() == envelope.schema_fingerprint(), "availability fingerprint bytes exact");
    require(parsed.publisher_instance_token() == envelope.publisher_instance_token(), "availability instance bytes exact");
    require(parsed.presence_hint_presence_case() == network::AvailabilityEnvelope::kPresenceHint && parsed.presence_hint() == hint, "availability explicit hint retained");
    require(parsed.published_system_presence_case() == network::AvailabilityEnvelope::kPublishedSystemNs && parsed.published_system_ns() == 123456789, "availability retains original LWT preparation SYSTEM time");
    std::cout << json << '\n';
  }
}

void metadata_versions()
{
  network::GetStatusRequest query;
  require(query.schema_version_presence_case() == network::GetStatusRequest::SCHEMA_VERSION_PRESENCE_NOT_SET, "missing query schema is distinct");
  query.set_schema_version(1);
  auto parsed_query = round_trip(query);
  require(parsed_query.schema_version_presence_case() == network::GetStatusRequest::kSchemaVersion && parsed_query.schema_version() == 1, "explicit query schema survives wire");
  network::SubmitReceipt receipt;
  receipt.set_schema_version(1);
  auto parsed_receipt = round_trip(receipt);
  require(parsed_receipt.schema_version_presence_case() == network::SubmitReceipt::kSchemaVersion && parsed_receipt.schema_version() == 1, "receipt schema survives wire");
  require(!parsed_receipt.rpc_received(), "schema presence is not received or accepted");
}

void control_operation_presence()
{
  network::ControlSessionRequest request;
  require(request.operation_case() == network::ControlSessionRequest::OPERATION_NOT_SET, "missing operation distinct");
  auto *open = request.mutable_open();
  require(open->schema_version_presence_case() == network::OpenSessionRequest::SCHEMA_VERSION_PRESENCE_NOT_SET, "open missing version distinct");
  open->set_schema_version(0);
  open->set_holder_nonce(std::string(16, 'n'));
  open->set_open_request_id(std::string(16, 'r'));
  auto parsed_open = round_trip(request);
  require(parsed_open.operation_case() == network::ControlSessionRequest::kOpen, "open oneof survives");
  require(parsed_open.open().schema_version_presence_case() == network::OpenSessionRequest::kSchemaVersion, "explicit open version zero presence");
  require(parsed_open.open().holder_nonce() == open->holder_nonce() && parsed_open.open().open_request_id() == open->open_request_id(), "independent 16 byte holder and idempotency identities exact");
  auto *keep = request.mutable_keep();
  require(!request.has_open(), "keep replaces open oneof");
  require(keep->schema_version_presence_case() == network::KeepSessionRequest::SCHEMA_VERSION_PRESENCE_NOT_SET, "keep missing version distinct");
  keep->set_schema_version(1);
  keep->set_holder_nonce(std::string(16, 'n'));
  keep->set_open_request_id(std::string(16, 'r'));
  require(round_trip(request).keep().keepalive_sequence_presence_case() == network::KeepSessionRequest::KEEPALIVE_SEQUENCE_PRESENCE_NOT_SET, "missing keep sequence distinct");
  keep->set_keepalive_sequence(0);
  auto parsed_zero = round_trip(request);
  require(parsed_zero.operation_case() == network::ControlSessionRequest::kKeep, "keep oneof survives");
  require(parsed_zero.keep().schema_version_presence_case() == network::KeepSessionRequest::kSchemaVersion, "keep version presence exact");
  require(parsed_zero.keep().holder_nonce().size() == 16 && parsed_zero.keep().open_request_id().size() == 16, "keep holder identities exact without submission token");
  require(parsed_zero.keep().keepalive_sequence_presence_case() == network::KeepSessionRequest::kKeepaliveSequence && parsed_zero.keep().keepalive_sequence() == 0, "explicit keep sequence zero presence");
  keep->set_keepalive_sequence(std::numeric_limits<std::uint64_t>::max());
  require(round_trip(request).keep().keepalive_sequence() == std::numeric_limits<std::uint64_t>::max(), "keep sequence max exact and independent of business sequence");
  auto *close = request.mutable_close();
  require(!request.has_keep(), "close replaces keep oneof");
  require(close->schema_version_presence_case() == network::CloseSessionRequest::SCHEMA_VERSION_PRESENCE_NOT_SET, "close missing version distinct");
  close->set_schema_version(1);
  close->set_holder_nonce(std::string(16, 'n'));
  close->set_open_request_id(std::string(16, 'r'));
  auto parsed_close = round_trip(request);
  require(parsed_close.operation_case() == network::ControlSessionRequest::kClose, "close oneof survives");
  require(parsed_close.close().schema_version_presence_case() == network::CloseSessionRequest::kSchemaVersion, "close version presence exact");
  require(parsed_close.close().holder_nonce().size() == 16 && parsed_close.close().open_request_id().size() == 16, "close holder identities exact");
  request.mutable_open()->set_holder_nonce(std::string(15, 'n'));
  request.mutable_open()->set_open_request_id(std::string(17, 'r'));
  auto malformed = round_trip(request);
  require(malformed.open().holder_nonce().size() == 15 && malformed.open().open_request_id().size() == 17, "protobuf preserves malformed widths; future boundary must reject, not pad or truncate");
}

void control_response_facts()
{
  network::OpenSessionResponse absent;
  require(!absent.has_session() && absent.result() == network::SESSION_OPERATION_RESULT_UNKNOWN, "missing response never implies admission");
  for (auto state : {network::CONTROL_SESSION_STATE_IDLE, network::CONTROL_SESSION_STATE_OPENING,
      network::CONTROL_SESSION_STATE_ADMITTED, network::CONTROL_SESSION_STATE_REVOKING}) {
    network::SessionInfo info;
    info.set_control_session_state(state);
    info.set_submission_ready(false);
    require(round_trip(info).control_session_state() == state, "control resource state survives independently of Gateway state");
    require(!round_trip(info).submission_ready(), "resource state alone not ready");
  }
  for (auto result : {network::SESSION_OPERATION_RESULT_OPENING, network::SESSION_OPERATION_RESULT_ADMITTED,
      network::SESSION_OPERATION_RESULT_IDEMPOTENT, network::SESSION_OPERATION_RESULT_BUSY,
      network::SESSION_OPERATION_RESULT_REVOKED, network::SESSION_OPERATION_RESULT_REJECTED}) {
    network::OpenSessionResponse response;
    response.set_result(result);
    response.set_open_request_id(std::string(16, 'r'));
    if (result == network::SESSION_OPERATION_RESULT_ADMITTED || result == network::SESSION_OPERATION_RESULT_IDEMPOTENT) {
      auto *session = response.mutable_session();
      session->set_session_token(std::string(16, 't'));
      session->set_control_session_state(network::CONTROL_SESSION_STATE_ADMITTED);
      session->set_submission_ready(true);
      session->mutable_ingress_publisher()->set_gid(std::string(24, 'g'));
    }
    auto parsed = round_trip(response);
    require(parsed.result() == result && parsed.open_request_id() == response.open_request_id(), "result and request echo exact");
    if (result == network::SESSION_OPERATION_RESULT_BUSY || result == network::SESSION_OPERATION_RESULT_REJECTED) {
      require(!parsed.has_session() && parsed.session().session_token().empty(), "BUSY/REJECTED fixture does not disclose incumbent session");
      require(parsed.open_request_id() == std::string(16, 'r'), "only own request key echoed");
    }
  }
}

int main(int argument_count, char **arguments)
{
  try {
    require(argument_count == 2, "expected one case");
    const std::string selected = arguments[1];
    if (selected == "control-operation-presence") {
      control_operation_presence();
    } else if (selected == "control-response-facts") {
      control_response_facts();
    } else if (selected == "metadata-versions") {
      metadata_versions();
    } else if (selected == "command-presence") {
      command_presence();
    } else if (selected == "raw-business-values") {
      raw_business_values();
    } else if (selected == "identity-and-receipt") {
      identity_and_receipt();
    } else if (selected == "independent-observations") {
      independent_observations();
    } else if (selected == "json-mapping") {
      json_mapping();
    } else if (selected == "status-json-mapping") {
      status_json_mapping();
    } else if (selected == "schema-fingerprint") {
      schema_fingerprint();
    } else if (selected == "diagnostic-json-mapping") {
      diagnostic_json_mapping();
    } else if (selected == "availability-json-mapping") {
      availability_json_mapping();
    } else {
      throw std::runtime_error("unknown case");
    }
    std::cout << selected << ": PASS\n";
    return 0;
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
'''


def run_logged(command, directory, name):
    (directory / f"{name}.command.json").write_text(
        json.dumps(command, ensure_ascii=False, indent=2) + "\n", encoding="utf-8"
    )
    result = subprocess.run(command, capture_output=True, text=True, timeout=120)
    (directory / f"{name}.stdout.log").write_text(result.stdout, encoding="utf-8")
    (directory / f"{name}.stderr.log").write_text(result.stderr, encoding="utf-8")
    (directory / f"{name}.exit").write_text(f"{result.returncode}\n", encoding="utf-8")
    if result.returncode != 0:
        raise RuntimeError(f"{name}: exit={result.returncode}\n{result.stderr}")
    return result.stdout


class ProtocolSchemaTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.output = Path(tempfile.mkdtemp(prefix="schema-generation-", dir=OUTPUT_DIRECTORY))
        print(f"schema_artifacts={cls.output}", flush=True)
        version = run_logged([PROTOC, "--version"], cls.output, "protoc-version").strip()
        (cls.output / SCHEMA.name).write_bytes(SCHEMA.read_bytes())
        run_logged(
            [PROTOC, f"--proto_path={SCHEMA.parent}", f"--cpp_out={cls.output}",
             f"--descriptor_set_out={cls.output / 'schema.desc'}", "--include_source_info", str(SCHEMA)],
            cls.output,
            "protoc-generate",
        )
        descriptor_set = descriptor_pb2.FileDescriptorSet()
        descriptor_set.ParseFromString((cls.output / "schema.desc").read_bytes())
        if len(descriptor_set.file) != 1:
            raise RuntimeError("schema must be self-contained")
        cls.descriptor = descriptor_set.file[0]
        cls.messages = {message.name: message for message in cls.descriptor.message_type}
        flags = shlex.split(
            run_logged(["pkg-config", "--cflags", "--libs", "protobuf"], cls.output, "protobuf-link-flags")
        )
        source_digest = hashlib.sha256(SCHEMA.read_bytes()).hexdigest()
        (cls.output / "schema_probe.cpp").write_text(
            CPP_PROBE.replace("SCHEMA_FINGERPRINT_HEX", source_digest), encoding="utf-8"
        )
        cls.executable = cls.output / "schema_probe"
        run_logged(
            [COMPILER, "-std=c++17", "-O0", "-g0", f"-I{cls.output}",
             str(cls.output / "schema_probe.cpp"), str(cls.output / "control_link_network_v1.pb.cc"),
             "-o", str(cls.executable), *flags],
            cls.output,
            "cpp-build",
        )
        run_logged(["ldd", str(cls.executable)], cls.output, "cpp-ldd")
        environment = {
            "source_head": run_logged(["git", "-C", str(REPOSITORY), "rev-parse", "HEAD"], cls.output, "source-head").strip(),
            "schema_sha256": hashlib.sha256(SCHEMA.read_bytes()).hexdigest(),
            "test_sha256": hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
            "protoc_version": version,
            "protobuf_version": run_logged(["pkg-config", "--modversion", "protobuf"], cls.output, "protobuf-version").strip(),
            "compiler": run_logged([COMPILER, "--version"], cls.output, "compiler-version").splitlines()[0],
            "schema_probe_sha256": hashlib.sha256(cls.executable.read_bytes()).hexdigest(),
            "environment": {name: os.environ.get(name) for name in ["AMENT_PREFIX_PATH", "CMAKE_PREFIX_PATH", "PYTHONPATH", "LD_LIBRARY_PATH", "LD_PRELOAD", "MAKEFLAGS", "CMAKE_BUILD_PARALLEL_LEVEL"]},
            "scope": "schema generation/parse only; no gRPC plugin, ROS, EdgeRuntime or protocol node",
        }
        (cls.output / "environment.json").write_text(json.dumps(environment, indent=2) + "\n", encoding="utf-8")

    def test_descriptor_package_and_presence(self):
        self.assertEqual(self.descriptor.package, "control_link.network.v1")
        self.assertEqual(self.descriptor.syntax, "proto3")
        expected = {
            "CommandTarget": ["schema_version", "source_stamp_ns", "clock_domain", "mode", "linear_velocity_mps", "angular_velocity_radps"],
            "CommandKey": ["source_sequence"],
            "ObservationMetadata": ["observed_system_ns", "received_boottime_ns"],
            "StatusEnvelope": ["schema_version", "publication_sequence", "published_system_ns", "published_boottime_ns"],
            "AvailabilityEnvelope": ["schema_version", "presence_hint", "published_system_ns"],
            "OpenSessionRequest": ["schema_version"],
            "KeepSessionRequest": ["schema_version", "keepalive_sequence"],
            "CloseSessionRequest": ["schema_version"],
        }
        for message_name, field_names in expected.items():
            message = self.messages[message_name]
            fields = {field.name: field for field in message.field}
            for field_name in field_names:
                with self.subTest(message=message_name, field=field_name):
                    field = fields[field_name]
                    self.assertTrue(field.HasField("oneof_index"))
                    self.assertEqual(sum(candidate.oneof_index == field.oneof_index for candidate in message.field if candidate.HasField("oneof_index")), 1)
                    self.assertFalse(getattr(field, "proto3_optional", False))

    def test_readonly_and_streaming_service_boundaries(self):
        self.assertEqual(len(self.descriptor.service), 1)
        service = self.descriptor.service[0]
        self.assertEqual(service.name, "PlanningIngress")
        methods = {method.name: method for method in service.method}
        self.assertEqual(set(methods), {"OpenSession", "Submit", "SubscribeStatus", "GetStatus", "GetCapabilities"})
        for name, method in methods.items():
            self.assertEqual(method.client_streaming, name == "OpenSession")
            self.assertEqual(method.server_streaming, name in {"OpenSession", "SubscribeStatus"})
        self.assertEqual(methods["OpenSession"].input_type, ".control_link.network.v1.ControlSessionRequest")
        self.assertEqual(methods["OpenSession"].output_type, ".control_link.network.v1.OpenSessionResponse")
        for request in ["SubscribeStatusRequest", "GetStatusRequest", "GetCapabilitiesRequest"]:
            self.assertEqual({field.name for field in self.messages[request].field}, {"schema_version"})
        for message_name in ["OpenSessionRequest", "CommandTarget"]:
            self.assertTrue({"source_id", "priority"}.issubset(set(self.messages[message_name].reserved_name)))
            self.assertFalse({"source_id", "priority"}.intersection(field.name for field in self.messages[message_name].field))

    def test_business_reason_and_mode_single_owner(self):
        owners = {
            "GatewayObservation": ("GatewayState.msg", {"state", "reason_code"}),
            "SourceStatusObservation": ("SourceStatus.msg", {"last_reject_reason", "endpoint_state"}),
            "VehicleObservation": ("VehicleState.msg", {"state", "fault_code"}),
        }
        for message_name, (owner_file, reason_fields) in owners.items():
            fields = {field.name: field for field in self.messages[message_name].field}
            owner = (REPOSITORY / "ros2_ws/src/control_link_interfaces/msg" / owner_file).read_text(encoding="utf-8")
            for reason_field in reason_fields:
                self.assertRegex(owner, rf"(?m)^uint(?:8|16|32) {reason_field}$")
                self.assertEqual(fields[reason_field].type, descriptor_pb2.FieldDescriptorProto.TYPE_UINT32)
        mode = next(value for value in self.descriptor.enum_type if value.name == "CommandMode")
        values = {value.name: value.number for value in mode.value}
        owner = (REPOSITORY / "ros2_ws/src/control_link_interfaces/msg/ControlCommand.msg").read_text(encoding="utf-8")
        for name in ["NORMAL", "HOLD"]:
            number = int(re.search(rf"(?m)^uint8 MODE_{name}=(\d+)$", owner).group(1))
            self.assertEqual(values[f"COMMAND_MODE_{name}"], number)
        self.assertFalse(any(value.name.startswith(("REJECT_", "REASON_", "FAULT_")) for enumeration in self.descriptor.enum_type for value in enumeration.value))

    def test_receipt_and_readonly_request_versions_are_explicit(self):
        for name in ("SubmitReceipt", "SubscribeStatusRequest", "GetStatusRequest", "GetCapabilitiesRequest"):
            message = next(value for value in self.descriptor.message_type if value.name == name)
            field = next(value for value in message.field if value.name == "schema_version")
            self.assertTrue(field.HasField("oneof_index"))
            self.assertEqual(message.oneof_decl[field.oneof_index].name, "schema_version_presence")
        run_logged([str(self.executable), "metadata-versions"], self.output, "metadata-versions")

    def test_cpp_missing_and_explicit_zero_presence(self):
        run_logged([str(self.executable), "command-presence"], self.output, "command-presence")

    def test_cpp_unknown_and_raw_business_values_survive(self):
        run_logged([str(self.executable), "raw-business-values"], self.output, "raw-business-values")

    def test_cpp_identity_and_receipt_facts_are_separate(self):
        run_logged([str(self.executable), "identity-and-receipt"], self.output, "identity-and-receipt")

    def test_cpp_independent_observation_times_and_validity(self):
        run_logged([str(self.executable), "independent-observations"], self.output, "independent-observations")

    def test_cpp_proto_json_mapping_preserves_presence(self):
        run_logged([str(self.executable), "json-mapping"], self.output, "json-mapping")

    def test_cpp_status_json_mapping_preserves_generation_and_stale(self):
        run_logged([str(self.executable), "status-json-mapping"], self.output, "status-json-mapping")

    def test_descriptor_telemetry_additions_and_session_request(self):
        for message_name in ["StatusEnvelope", "Capabilities", "AvailabilityEnvelope"]:
            fields = {field.name: field for field in self.messages[message_name].field}
            self.assertEqual(fields["schema_fingerprint"].type, descriptor_pb2.FieldDescriptorProto.TYPE_BYTES)
        self.assertEqual({field.name for field in self.messages["OpenSessionRequest"].field}, {"schema_version", "holder_nonce", "open_request_id"})
        diagnostic = self.messages["DiagnosticObservation"]
        self.assertEqual({field.name for field in diagnostic.field}, {"observation", "level", "name", "message", "hardware_id", "values"})
        fields = {field.name: field for field in diagnostic.field}
        self.assertEqual(fields["level"].type, descriptor_pb2.FieldDescriptorProto.TYPE_UINT32)
        self.assertEqual(fields["values"].label, descriptor_pb2.FieldDescriptorProto.LABEL_REPEATED)
        self.assertEqual(fields["values"].type_name, ".control_link.network.v1.DiagnosticKeyValue")
        alerts = next(field for field in self.messages["StatusSnapshot"].field if field.name == "alerts")
        self.assertEqual(alerts.label, descriptor_pb2.FieldDescriptorProto.LABEL_REPEATED)
        self.assertEqual(alerts.type_name, ".control_link.network.v1.DiagnosticObservation")
        availability = self.messages["AvailabilityEnvelope"]
        self.assertEqual({field.name for field in availability.field}, {"schema_version", "schema_fingerprint", "publisher_instance_token", "presence_hint", "published_system_ns"})
        for service in self.descriptor.service:
            self.assertFalse(any(method.input_type.endswith(".AvailabilityEnvelope") or method.output_type.endswith(".AvailabilityEnvelope") for method in service.method))

    def test_cpp_schema_fingerprint_is_original_source_digest(self):
        import base64

        output = run_logged([str(self.executable), "schema-fingerprint"], self.output, "schema-fingerprint")
        encoded = json.loads(output.splitlines()[0])["schema_fingerprint"]
        self.assertEqual(base64.b64decode(encoded, validate=True), hashlib.sha256(SCHEMA.read_bytes()).digest())

    def test_cpp_diagnostic_json_mapping_retains_facts(self):
        run_logged([str(self.executable), "diagnostic-json-mapping"], self.output, "diagnostic-json-mapping")

    def test_cpp_availability_json_is_only_presence_hint(self):
        run_logged([str(self.executable), "availability-json-mapping"], self.output, "availability-json-mapping")

    def test_descriptor_control_session_contract(self):
        request = self.messages["ControlSessionRequest"]
        self.assertEqual([oneof.name for oneof in request.oneof_decl], ["operation"])
        self.assertEqual([(field.name, field.number) for field in request.field],
                         [("open", 1), ("keep", 2), ("close", 3)])
        for field in request.field:
            self.assertEqual(field.oneof_index, 0)
            self.assertEqual(field.type, descriptor_pb2.FieldDescriptorProto.TYPE_MESSAGE)
            self.assertEqual(field.type_name, ".control_link.network.v1." + {
                "open": "OpenSessionRequest", "keep": "KeepSessionRequest",
                "close": "CloseSessionRequest"}[field.name])
        for name in ("OpenSessionRequest", "KeepSessionRequest", "CloseSessionRequest"):
            fields = {field.name: field for field in self.messages[name].field}
            self.assertEqual(set(fields), {"schema_version", "holder_nonce", "open_request_id"} |
                             ({"keepalive_sequence"} if name == "KeepSessionRequest" else set()))
            for identity in ("holder_nonce", "open_request_id"):
                self.assertEqual(fields[identity].type, descriptor_pb2.FieldDescriptorProto.TYPE_BYTES)
                self.assertFalse(fields[identity].HasField("oneof_index"))
        keep_sequence = next(field for field in self.messages["KeepSessionRequest"].field
                             if field.name == "keepalive_sequence")
        self.assertEqual(keep_sequence.type, descriptor_pb2.FieldDescriptorProto.TYPE_UINT64)
        response_fields = {field.name: field for field in self.messages["OpenSessionResponse"].field}
        self.assertEqual(set(response_fields), {"session", "capabilities", "result", "open_request_id"})
        self.assertEqual(response_fields["session"].type_name, ".control_link.network.v1.SessionInfo")
        self.assertEqual(response_fields["result"].type_name, ".control_link.network.v1.SessionOperationResult")
        session_fields = {field.name: field for field in self.messages["SessionInfo"].field}
        self.assertEqual(session_fields["control_session_state"].number, 7)
        self.assertEqual(session_fields["control_session_state"].type_name, ".control_link.network.v1.ControlSessionState")
        enums = {enum.name: {value.name: value.number for value in enum.value}
                 for enum in self.descriptor.enum_type}
        self.assertEqual(enums["ControlSessionState"], {
            "CONTROL_SESSION_STATE_" + name: index for index, name in enumerate(
                ("UNKNOWN", "IDLE", "OPENING", "ADMITTED", "REVOKING"))})
        self.assertEqual(enums["SessionOperationResult"], {
            "SESSION_OPERATION_RESULT_" + name: index for index, name in enumerate(
                ("UNKNOWN", "OPENING", "ADMITTED", "IDEMPOTENT", "BUSY", "REVOKED", "REJECTED"))})

    def test_cpp_control_operation_oneof_and_identity_presence(self):
        run_logged([str(self.executable), "control-operation-presence"], self.output,
                   "control-operation-presence")

    def test_cpp_control_state_result_and_busy_encoding(self):
        run_logged([str(self.executable), "control-response-facts"], self.output,
                   "control-response-facts")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Generate and parse the P0 schema; not a production protocol test")
    parser.add_argument("--output-dir", type=Path, help="Parent of a new unique artifact directory")
    parser.add_argument("--protoc", default=PROTOC)
    parser.add_argument("--compiler", default=COMPILER)
    arguments, unittest_arguments = parser.parse_known_args()
    PROTOC = arguments.protoc
    COMPILER = arguments.compiler
    OUTPUT_DIRECTORY = arguments.output_dir
    if OUTPUT_DIRECTORY is not None:
        OUTPUT_DIRECTORY.mkdir(parents=True, exist_ok=True)
    unittest.main(argv=[sys.argv[0], *unittest_arguments])
