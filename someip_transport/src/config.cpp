#include "control_link_someip_transport/config.hpp"
#include "control_link_edge_protocol/protocol.hpp"
#include "control_link_network_v1.pb.h"

#include <arpa/inet.h>

#include <algorithm>
#include <array>
#include <cctype>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <limits>
#include <set>
#include <sstream>
#include <stdexcept>
#include <string>

#include <boost/property_tree/json_parser.hpp>
#include <yaml-cpp/yaml.h>

namespace control_link::someip {
namespace {

// 656B 只是固定 protobuf 字段的保守配置初筛，不是实际装配可用容量；
// 占 holder/网络启动前还须以真实 Capabilities/RMW 调用 validate_protocol_payload_budget。
constexpr std::uint32_t kMinimumBoundedProtocolPayloadBytes = 656;

[[noreturn]] void invalid(const std::string &field, const std::string &reason) {
  throw std::invalid_argument("SOME/IP config " + field + ": " + reason);
}

// 仅组装本 adapter 必需的有界字段；开放文本使用实际装配身份，不虚构长度上界。
// ROS 字段按原 msg 整数宽度保留，包括标为 INVALID 的原观察；这不是业务合法性判断。
std::size_t mandatory_payload_bytes(const ::control_link::network::v1::Capabilities &capabilities,
                                    const std::string &rmw) {
  namespace wire = ::control_link::network::v1;
  namespace codec = control_link_edge_protocol;
  const auto maximum8 = std::numeric_limits<std::uint8_t>::max();
  const auto maximum16 = std::numeric_limits<std::uint16_t>::max();
  const auto maximum32 = std::numeric_limits<std::uint32_t>::max();
  const auto maximum64 = std::numeric_limits<std::uint64_t>::max();
  wire::PublisherIdentity publisher;
  publisher.set_gid(std::string(24, 'g'));
  publisher.set_rmw_implementation(rmw);
  wire::ObservationMetadata observation;
  observation.set_observed_system_ns(-1);
  observation.set_received_boottime_ns(maximum64);
  observation.set_validity(wire::OBSERVATION_VALIDITY_INVALID);
  wire::OpenSessionResponse response;
  *response.mutable_capabilities() = capabilities;
  wire::OpenAdmissionDomain opening_domain;
  opening_domain.set_service_instance_id(std::string(codec::kSessionTokenSize, 'i'));
  opening_domain.set_generation(maximum64);
  opening_domain.set_domain_version(1);
  *response.mutable_capabilities()->mutable_opening_domain() = opening_domain;
  response.set_result(wire::SESSION_OPERATION_RESULT_ADMITTED);
  response.set_open_request_id(std::string(codec::kSessionTokenSize, 'o'));
  auto *session = response.mutable_session();
  session->set_session_token(std::string(codec::kSessionTokenSize, 't'));
  session->set_planning_transport(wire::PLANNING_TRANSPORT_SOMEIP);
  session->set_connection_state(wire::CONNECTION_STATE_CONNECTED);
  session->set_submission_ready(true);
  *session->mutable_ingress_publisher() = publisher;
  session->set_clock_domain(wire::CLOCK_DOMAIN_SYSTEM);
  session->set_control_session_state(wire::CONTROL_SESSION_STATE_ADMITTED);
  *session->mutable_opening_domain() = opening_domain;
  wire::StatusEnvelope envelope;
  envelope.set_schema_version(1);
  envelope.set_schema_fingerprint(capabilities.schema_fingerprint());
  envelope.set_publisher_instance_token(std::string(codec::kSessionTokenSize, 't'));
  envelope.set_publication_sequence(maximum64);
  envelope.set_published_system_ns(-1);
  envelope.set_published_boottime_ns(maximum64);
  auto *snapshot = envelope.mutable_status();
  *snapshot->mutable_opening_domain() = opening_domain;
  *snapshot->mutable_session() = *session;
  snapshot->mutable_session()->clear_session_token();
  auto *counters = snapshot->mutable_transport();
  counters->set_rpc_received_count(maximum64);
  counters->set_forwarded_count(maximum64);
  counters->set_missed_count(maximum64);
  counters->set_dropped_count(maximum64);
  counters->set_protocol_rejected_count(maximum64);
  counters->set_connection_state(wire::CONNECTION_STATE_CONNECTED);
  auto *forwarded = snapshot->mutable_latest_forwarded();
  forwarded->mutable_key()->set_source_sequence(maximum64);
  *forwarded->mutable_publisher() = publisher;
  forwarded->set_forwarded_boottime_ns(maximum64);
  auto *gateway = snapshot->mutable_gateway();
  *gateway->mutable_observation() = observation;
  gateway->set_state(maximum8);
  gateway->set_reason_code(maximum16);
  // 此处可观测 teleop 等来源；32B grammar 的唯一 owner 为公共 Contract parser。
  gateway->set_active_source_id(std::string(32, 's'));
  gateway->set_active_source_sequence(maximum64);
  gateway->set_active_command_age_ms(1.0);
  gateway->set_recovery_valid_count(maximum16);
  gateway->set_transition_sequence(maximum64);
  auto *source = snapshot->add_sources();
  *source->mutable_observation() = observation;
  source->set_source_id("planning");
  source->set_priority(maximum8);
  source->set_enabled(true);
  source->set_lease_valid(true);
  source->set_command_valid(true);
  source->set_last_reject_reason(maximum16);
  source->set_last_source_sequence(maximum64);
  source->set_command_age_ms(1.0);
  source->set_accepted_count(maximum64);
  source->set_rejected_count(maximum64);
  source->set_endpoint_stable(true);
  source->set_endpoint_state(maximum8);
  source->set_endpoint_publisher_count(maximum32);
  source->set_endpoint_generation_valid(true);
  *source->mutable_endpoint_publisher() = publisher;
  auto *vehicle = snapshot->mutable_vehicle();
  *vehicle->mutable_observation() = observation;
  vehicle->set_state(maximum8);
  vehicle->set_fault_code(maximum16);
  vehicle->set_linear_velocity_mps(1.0);
  vehicle->set_angular_velocity_radps(1.0);
  vehicle->set_rolling_counter(maximum8);
  auto *acceptance = snapshot->mutable_latest_gateway_acceptance();
  acceptance->set_state(wire::GATEWAY_ACCEPTANCE_STATE_LATEST_KEY_OBSERVED);
  acceptance->mutable_key()->set_source_sequence(maximum64);
  *acceptance->mutable_publisher() = publisher;
  *acceptance->mutable_source_observation() = observation;
  acceptance->set_accepted_count(maximum64);
  wire::SubmitReceipt receipt;
  *receipt.mutable_key() = forwarded->key();
  receipt.mutable_key()->set_session_token(session->session_token());
  receipt.set_rpc_received(true);
  receipt.set_forwarding_state(wire::FORWARDING_STATE_FORWARDED);
  *receipt.mutable_forwarded() = *forwarded;
  receipt.mutable_forwarded()->mutable_key()->set_session_token(session->session_token());
  *receipt.mutable_gateway_acceptance() = *acceptance;
  receipt.mutable_gateway_acceptance()->mutable_key()->set_session_token(session->session_token());
  receipt.set_rpc_received_boottime_ns(maximum64);
  receipt.set_schema_version(1);
  auto required = static_cast<std::size_t>(kMinimumBoundedProtocolPayloadBytes);
  for (const auto *message : std::array<const google::protobuf::Message *, 3>{&response, &envelope, &receipt}) {
    if (message->ByteSizeLong() > std::numeric_limits<std::uint32_t>::max()) {
      invalid("max_payload_bytes", "mandatory protocol payload exceeds uint32 capacity");
    }
    std::string serialized;
    if (!message->SerializeToString(&serialized)) {
      invalid("max_payload_bytes", "mandatory protocol payload cannot be serialized");
    }
    required = std::max(required, serialized.size());
  }
  return required;
}

void check_duplicates(const YAML::Node &node, const std::string &path,
                      std::size_t depth, std::size_t &nodes) {
  if (depth > 32 || ++nodes > 4096) invalid(path, "configuration nesting/node budget exceeded");
  if (node.IsMap()) {
    std::set<std::string> keys;
    for (const auto &entry : node) {
      if (!entry.first.IsScalar()) invalid(path, "mapping key must be scalar");
      const auto key = entry.first.Scalar();
      if (!keys.insert(key).second) invalid(path + "." + key, "duplicate field");
      check_duplicates(entry.second, path + "." + key, depth + 1, nodes);
    }
  } else if (node.IsSequence()) {
    for (std::size_t index = 0; index < node.size(); ++index) {
      check_duplicates(node[index], path + "[" + std::to_string(index) + "]", depth + 1, nodes);
    }
  }
}

void check_keys(const YAML::Node &node, const std::string &path,
                std::initializer_list<const char *> fields) {
  if (!node.IsMap()) invalid(path, "required mapping is missing");
  const std::set<std::string> allowed(fields.begin(), fields.end());
  for (const auto &entry : node) {
    const auto key = entry.first.Scalar();
    if (allowed.count(key) == 0) invalid(path + "." + key, "unknown field");
  }
  for (const auto &key : allowed) {
    if (!node[key]) invalid(path + "." + key, "required field is missing");
  }
}

std::string text(const YAML::Node &node, const std::string &path) {
  if (!node.IsScalar() || node.Scalar().empty()) invalid(path, "required nonempty scalar");
  return node.Scalar();
}

std::uint64_t integer(const YAML::Node &node, const std::string &path,
                      std::uint64_t maximum) {
  const auto value = text(node, path);
  std::size_t start = 0;
  unsigned base = 10;
  if (value.size() > 2 && value[0] == '0' && (value[1] == 'x' || value[1] == 'X')) {
    start = 2;
    base = 16;
  }
  std::uint64_t result = 0;
  for (std::size_t index = start; index < value.size(); ++index) {
    const auto character = static_cast<unsigned char>(value[index]);
    unsigned digit = 16;
    if (character >= '0' && character <= '9') digit = character - '0';
    else if (character >= 'a' && character <= 'f') digit = character - 'a' + 10;
    else if (character >= 'A' && character <= 'F') digit = character - 'A' + 10;
    if (digit >= base || digit > maximum || result > (maximum - digit) / base) {
      invalid(path, "expected bounded unsigned integer");
    }
    result = result * base + digit;
  }
  return result;
}

bool boolean(const YAML::Node &node, const std::string &path) {
  const auto value = text(node, path);
  if (value == "true") return true;
  if (value == "false") return false;
  invalid(path, "expected explicit true or false");
}

void exact(const YAML::Node &node, const std::string &path, const std::string &expected) {
  if (text(node, path) != expected) invalid(path, "must equal " + expected);
}

bool identifier(const std::string &value) {
  if (value.empty() || value.size() > 96) return false;
  for (const auto character : value) {
    const auto byte = static_cast<unsigned char>(character);
    if (!(std::isalnum(byte) || byte == '_' || byte == '-')) return false;
  }
  return true;
}

std::uint32_t ipv4(const std::string &value, const std::string &field) {
  in_addr address{};
  if (::inet_pton(AF_INET, value.c_str(), &address) != 1) invalid(field, "expected IPv4 address");
  return ntohl(address.s_addr);
}

std::string hex_id(std::uint16_t value) {
  std::ostringstream output;
  output << "0x" << std::hex << std::setw(4) << std::setfill('0') << value;
  return output.str();
}

std::uint32_t queue_budget(const Config &config) {
  // 控制 latest、握手、两组事件与 readonly 回复都计入有界 endpoint 字节预算。
  // 16B 是 SOME/IP header，不是业务 payload；两项预算同适配器配置推导。
  const auto slots = static_cast<std::uint64_t>(config.readonly_inflight_limit) + 5;
  const auto message_bytes = static_cast<std::uint64_t>(config.max_payload_bytes) + 16;
  if (slots > (std::numeric_limits<std::uint32_t>::max() - 1) / message_bytes) {
    invalid("max_payload_bytes/readonly_inflight_limit", "endpoint queue budget overflows");
  }
  return static_cast<std::uint32_t>(message_bytes * slots);
}

void compare_configuration(const YAML::Node &actual, const YAML::Node &expected,
                           const std::string &path) {
  if (actual.Type() != expected.Type()) invalid(path, "runtime JSON shape differs from generated contract");
  if (expected.IsScalar()) {
    if (actual.Scalar() != expected.Scalar()) invalid(path, "runtime JSON conflicts with configuration owner");
  } else if (expected.IsSequence()) {
    if (actual.size() != expected.size()) invalid(path, "runtime JSON sequence has unexpected members");
    for (std::size_t index = 0; index < expected.size(); ++index) {
      compare_configuration(actual[index], expected[index], path + "[" + std::to_string(index) + "]");
    }
  } else if (expected.IsMap()) {
    if (actual.size() != expected.size()) invalid(path, "runtime JSON has missing/extra fields");
    for (const auto &entry : expected) {
      const auto key = entry.first.Scalar();
      if (!actual[key]) invalid(path + "." + key, "runtime JSON required field is missing");
      compare_configuration(actual[key], entry.second, path + "." + key);
    }
  }
}

}

Config load_config(const std::string &path) {
  try {
    if (!std::filesystem::is_regular_file(path) || std::filesystem::file_size(path) > 65536) {
      invalid("YAML", "explicit regular configuration must fit 64KiB");
    }
    const auto root = YAML::LoadFile(path);
    std::size_t nodes = 0;
    check_duplicates(root, "root", 0, nodes);
    check_keys(root, "root", {"schema_version", "configuration_role", "planning", "safety_owners",
                              "control_session_resources", "someip", "grpc", "mqtt", "rate_limits"});
    if (integer(root["schema_version"], "schema_version", UINT32_MAX) != 1) {
      invalid("schema_version", "only version 1 is supported");
    }
    const auto role = text(root["configuration_role"], "configuration_role");
    if (role != "contract_only" && role != "runtime_someip_and_future_contracts") {
      invalid("configuration_role", "expected contract_only or runtime_someip_and_future_contracts");
    }
    const auto planning = root["planning"];
    check_keys(planning, "planning", {"source_id", "transport", "allowed_transports",
                                      "runtime_available_transports", "exclusive"});
    exact(planning["source_id"], "planning.source_id", "planning");
    if (!boolean(planning["exclusive"], "planning.exclusive")) invalid("planning.exclusive", "must be true");
    const std::set<std::string> transports{"ros2", "edge", "someip", "grpc"};
    const auto selected = text(planning["transport"], "planning.transport");
    if (transports.count(selected) == 0) invalid("planning.transport", "unsupported transport");
    for (const auto *field : {"allowed_transports", "runtime_available_transports"}) {
      const auto list = planning[field];
      if (!list.IsSequence() || list.size() == 0) invalid(std::string("planning.") + field, "required nonempty sequence");
      std::set<std::string> found;
      for (const auto &entry : list) {
        const auto value = text(entry, std::string("planning.") + field);
        if (transports.count(value) == 0 || !found.insert(value).second) {
          invalid(std::string("planning.") + field, "unsupported or duplicate transport");
        }
      }
      if (std::string(field) == "allowed_transports" && found != transports) {
        invalid("planning.allowed_transports", "must declare the four contract transports");
      }
    }
    const auto owners = root["safety_owners"];
    check_keys(owners, "safety_owners", {"contract", "source_policy", "profile", "timestamp_domain",
                                       "reference_frame", "command_codec", "network_schema", "fingerprint_algorithm"});
    exact(owners["contract"], "safety_owners.contract", "common/gateway_contract.yaml");
    exact(owners["source_policy"], "safety_owners.source_policy", "common/source_policy.yaml");
    exact(owners["profile"], "safety_owners.profile", "adas/adas_profile.yaml");
    exact(owners["timestamp_domain"], "safety_owners.timestamp_domain", "SYSTEM");
    exact(owners["reference_frame"], "safety_owners.reference_frame", "body");
    exact(owners["command_codec"], "safety_owners.command_codec", "control_link_edge_protocol/command_v1");
    exact(owners["network_schema"], "safety_owners.network_schema", "control_link_network_v1.proto");
    exact(owners["fingerprint_algorithm"], "safety_owners.fingerprint_algorithm", "sha256_proto_source_bytes");
    const auto resources = root["control_session_resources"];
    check_keys(resources, "control_session_resources", {"retired_open_capacity"});
    const auto someip = root["someip"];
    check_keys(someip, "someip", {"enabled", "application_name", "bind_address", "reliable_port",
                                 "service_discovery", "service_id", "instance_id", "major_version", "minor_version",
                                 "methods", "events", "eventgroups", "command_payload_bytes", "max_payload_bytes",
                                 "pending_latest_capacity", "readonly_inflight_limit", "session_keepalive_period_ms",
                                 "status_event_period_ms",
                                 "session_idle_timeout_ms", "reconnect_initial_ms", "reconnect_max_ms", "shutdown_timeout_ms",
                                 "graph_admission_timeout_ms", "graph_admission_quiet_ms", "graph_admission_poll_ms"});
    boolean(someip["enabled"], "someip.enabled");
    if (integer(someip["command_payload_bytes"], "someip.command_payload_bytes", UINT32_MAX) != 64) {
      invalid("someip.command_payload_bytes", "must use the 64B command codec");
    }
    if (integer(someip["pending_latest_capacity"], "someip.pending_latest_capacity", UINT32_MAX) != 1) {
      invalid("someip.pending_latest_capacity", "control latest capacity must equal 1");
    }
    const auto discovery = someip["service_discovery"];
    check_keys(discovery, "someip.service_discovery", {"enabled", "multicast_address", "port"});
    if (!boolean(discovery["enabled"], "someip.service_discovery.enabled")) {
      invalid("someip.service_discovery.enabled", "Service Discovery is required");
    }
    const auto methods = someip["methods"];
    check_keys(methods, "someip.methods", {"open_session", "submit", "get_status", "keep_session", "close_session"});
    const auto events = someip["events"];
    check_keys(events, "someip.events", {"gateway_status", "vehicle_status"});
    const auto groups = someip["eventgroups"];
    check_keys(groups, "someip.eventgroups", {"gateway_status", "vehicle_status"});
    Config config;
    config.rate_limits = ::control_link::network::parse_rate_limit_policy(root["rate_limits"]);
    config.application_name = text(someip["application_name"], "someip.application_name");
    config.bind_address = text(someip["bind_address"], "someip.bind_address");
    config.sd_multicast_address = text(discovery["multicast_address"], "someip.service_discovery.multicast_address");
    const auto read16 = [](const YAML::Node &section, const char *field) {
      return static_cast<std::uint16_t>(integer(section[field], field, UINT16_MAX));
    };
    const auto read32 = [](const YAML::Node &section, const char *field) {
      return static_cast<std::uint32_t>(integer(section[field], field, UINT32_MAX));
    };
    config.service_id = read16(someip, "service_id");
    config.instance_id = read16(someip, "instance_id");
    config.reliable_port = read16(someip, "reliable_port");
    config.sd_port = read16(discovery, "port");
    config.major_version = static_cast<std::uint8_t>(integer(someip["major_version"], "major_version", UINT8_MAX));
    config.minor_version = read32(someip, "minor_version");
    config.open_method = read16(methods, "open_session");
    config.submit_method = read16(methods, "submit");
    config.get_status_method = read16(methods, "get_status");
    config.keep_method = read16(methods, "keep_session");
    config.close_method = read16(methods, "close_session");
    config.gateway_event = read16(events, "gateway_status");
    config.vehicle_event = read16(events, "vehicle_status");
    config.gateway_eventgroup = read16(groups, "gateway_status");
    config.vehicle_eventgroup = read16(groups, "vehicle_status");
    config.max_payload_bytes = read32(someip, "max_payload_bytes");
    config.readonly_inflight_limit = read32(someip, "readonly_inflight_limit");
    config.status_event_period_ms = read32(someip, "status_event_period_ms");
    config.keep_period_ms = read32(someip, "session_keepalive_period_ms");
    config.idle_timeout_ms = read32(someip, "session_idle_timeout_ms");
    config.retired_open_capacity = read32(resources, "retired_open_capacity");
    config.reconnect_initial_ms = read32(someip, "reconnect_initial_ms");
    config.reconnect_max_ms = read32(someip, "reconnect_max_ms");
    config.shutdown_timeout_ms = read32(someip, "shutdown_timeout_ms");
    config.graph_timeout_ms = read32(someip, "graph_admission_timeout_ms");
    config.graph_quiet_ms = read32(someip, "graph_admission_quiet_ms");
    config.graph_poll_ms = read32(someip, "graph_admission_poll_ms");
    validate_config(config);
    return config;
  } catch (const YAML::Exception &error) {
    invalid(path, error.what());
  }
}

void validate_vsomeip_configuration(const Config &config, const std::string &path,
                                    const std::string &application_name, bool server) {
  try {
    if (!std::filesystem::is_regular_file(path) || std::filesystem::file_size(path) > 65536) {
      invalid("vsomeip JSON", "explicit regular configuration must fit 64KiB");
    }
    std::ifstream input(path, std::ios::binary);
    if (!input) invalid("vsomeip JSON", "cannot read explicit configuration");
    std::string contents(65537, '\0');
    input.read(contents.data(), static_cast<std::streamsize>(contents.size()));
    if (input.bad() || (input.fail() && !input.eof())) {
      invalid("vsomeip JSON", "failed to read explicit configuration");
    }
    if (input.gcount() > 65536) invalid("vsomeip JSON", "explicit regular configuration must fit 64KiB");
    contents.resize(static_cast<std::size_t>(input.gcount()));

    // 同一有界字节快照先限制结构预算；YAML 只用于重复键检查和 owner 比对，不授权语法。
    const auto document = YAML::Load(contents);
    std::size_t nodes = 0;
    check_duplicates(document, "vsomeip JSON", 0, nodes);
    // 与锁定 vsomeip 使用同一 Boost JSON 语法，禁止注释、尾逗号和第二文档退回默认配置。
    std::istringstream json_input(contents);
    boost::property_tree::ptree json_document;
    boost::property_tree::read_json(json_input, json_document);
    if (!document.IsMap() || !document["applications"].IsSequence() || document["applications"].size() != 1) {
      invalid("vsomeip JSON.applications", "one explicit application per routing owner is required");
    }
    const auto application_id = integer(document["applications"][0]["id"], "application id", UINT16_MAX);
    auto assembled = config;
    assembled.bind_address = text(document["unicast"], "vsomeip JSON.unicast");
    const auto network = text(document["network"], "vsomeip JSON.network");
    const auto expected = YAML::Load(vsomeip_configuration_json(assembled, application_name,
        static_cast<std::uint16_t>(application_id), network, server));
    compare_configuration(document, expected, "vsomeip JSON");
  } catch (const boost::property_tree::json_parser_error &error) {
    invalid("vsomeip JSON", "invalid JSON syntax: " + std::string{error.what()});
  } catch (const YAML::Exception &error) {
    invalid("vsomeip JSON", error.what());
  }
}

void validate_config(const Config &config) {
  ::control_link::network::validate_rate_limit_policy(config.rate_limits);
  if (!identifier(config.application_name)) invalid("application_name", "expected bounded alphanumeric identifier");
  const auto unicast = ipv4(config.bind_address, "bind_address");
  if (unicast == 0 || unicast >= 0xe0000000U) invalid("bind_address", "must be a concrete unicast IPv4 address");
  const auto multicast = ipv4(config.sd_multicast_address, "sd_multicast_address");
  if (multicast < 0xe0000000U || multicast >= 0xf0000000U) invalid("sd_multicast_address", "must be IPv4 multicast");
  if (config.service_id == 0 || config.service_id == UINT16_MAX) invalid("service_id", "reserved service identity");
  if (config.instance_id == 0 || config.instance_id == UINT16_MAX) invalid("instance_id", "reserved instance identity");
  if (config.major_version == UINT8_MAX || config.minor_version == UINT32_MAX) invalid("version", "wildcard version is not a service version");
  if (config.reliable_port == 0 || config.sd_port == 0 || config.reliable_port == config.sd_port) invalid("ports", "required distinct nonzero ports");
  const std::array<std::uint16_t, 5> methods{config.open_method, config.submit_method, config.get_status_method,
                                            config.keep_method, config.close_method};
  std::set<std::uint16_t> identifiers;
  for (const auto method : methods) {
    if (method == 0 || method >= 0x8000 || !identifiers.insert(method).second) invalid("methods", "reserved or duplicate method id");
  }
  for (const auto event : {config.gateway_event, config.vehicle_event}) {
    if (event < 0x8000 || event == UINT16_MAX || !identifiers.insert(event).second) invalid("events", "reserved or duplicate event id");
  }
  if (config.gateway_eventgroup == 0 || config.gateway_eventgroup == UINT16_MAX ||
      config.vehicle_eventgroup == 0 || config.vehicle_eventgroup == UINT16_MAX ||
      config.gateway_eventgroup == config.vehicle_eventgroup) invalid("eventgroups", "reserved or duplicate group id");
  if (config.max_payload_bytes < kMinimumBoundedProtocolPayloadBytes) {
    invalid("max_payload_bytes", "must be at least " +
        std::to_string(kMinimumBoundedProtocolPayloadBytes) +
        " bytes for mandatory session/capability/status fields, not just the 64B command codec");
  }
  if (config.readonly_inflight_limit == 0 || config.retired_open_capacity == 0) {
    invalid("resources", "capacities must be positive");
  }
  if (config.keep_period_ms == 0 || config.idle_timeout_ms <= config.keep_period_ms) invalid("keep/idle", "keep must be positive and shorter than idle budget");
  if (config.status_event_period_ms == 0 || config.status_event_period_ms > config.idle_timeout_ms) {
    invalid("status_event_period_ms", "period must be positive and within network idle budget");
  }
  if (config.reconnect_initial_ms == 0 || config.reconnect_initial_ms > config.reconnect_max_ms) invalid("reconnect", "expected positive bounded ordered budgets");
  if (config.shutdown_timeout_ms == 0) invalid("shutdown_timeout_ms", "shutdown budget must be positive");
  if (config.graph_timeout_ms == 0 || config.graph_quiet_ms == 0 || config.graph_poll_ms == 0 ||
      config.graph_poll_ms > config.graph_quiet_ms || config.graph_quiet_ms >= config.graph_timeout_ms) {
    invalid("graph", "positive probe/quiet/readiness budgets must fit the admission timeout");
  }
  queue_budget(config);
}

std::size_t required_protocol_payload_bytes(
    const ::control_link::network::v1::Capabilities &capabilities,
    const std::string &rmw_implementation) {
  if (rmw_implementation.empty()) invalid("rmw_implementation", "actual nonempty runtime identity is required");
  auto normalized = capabilities;
  normalized.set_schema_version(1);
  // PlanningIngress 同样规范到 V1/32B fingerprint；字节内容不影响 wire 长度。
  normalized.set_schema_fingerprint(std::string(32, 'f'));
  return mandatory_payload_bytes(normalized, rmw_implementation);
}

void validate_protocol_payload_budget(
    const Config &config, const ::control_link::network::v1::Capabilities &capabilities,
    const std::string &rmw_implementation) {
  validate_config(config);
  const auto required = required_protocol_payload_bytes(capabilities, rmw_implementation);
  if (config.max_payload_bytes < required) {
    invalid("max_payload_bytes", "PAYLOAD_CAPACITY: actual capabilities/RMW and mandatory "
        "session/status fields require at least " + std::to_string(required) +
        " bytes; configured " + std::to_string(config.max_payload_bytes));
  }
}

std::string vsomeip_configuration_json(const Config &config, const std::string &application_name,
                                      std::uint16_t application_id, const std::string &network_name,
                                      bool server) {
  validate_config(config);
  if (!identifier(application_name) || !identifier(network_name)) invalid("application/network", "invalid routing domain identifier");
  if (application_id == 0 || application_id == UINT16_MAX) invalid("application_id", "explicit nonreserved client id is required");
  std::ostringstream output;
  output << "{\"unicast\":\"" << config.bind_address << "\",\"network\":\"" << network_name
         << "\",\"routing\":\"" << application_name << "\",\"applications\":[{\"name\":\""
         << application_name << "\",\"id\":\"" << hex_id(application_id)
         << "\",\"threads\":\"2\",\"max_dispatchers\":\"1\"}],"
         << "\"logging\":{\"level\":\"warning\",\"console\":\"true\",\"file\":{\"enable\":\"false\"},\"dlt\":\"false\"},"
         << "\"max-payload-size-local\":\"" << config.max_payload_bytes
         << "\",\"max-payload-size-reliable\":\"" << config.max_payload_bytes
         << "\",\"endpoint-queue-limit-external\":\"" << queue_budget(config)
         << "\",\"endpoint-queue-limit-local\":\"" << queue_budget(config)
         << "\",\"shutdown_timeout\":\"" << config.shutdown_timeout_ms << "\","
         << "\"service-discovery\":{\"enable\":\"true\",\"multicast\":\"" << config.sd_multicast_address
         << "\",\"port\":\"" << config.sd_port << "\",\"protocol\":\"udp\"}";
  if (server) {
    output << ",\"services\":[{\"service\":\"" << hex_id(config.service_id) << "\",\"instance\":\""
           << hex_id(config.instance_id) << "\",\"reliable\":{\"port\":\"" << config.reliable_port
           << "\",\"enable-magic-cookies\":\"true\"},\"events\":[{\"event\":\"" << hex_id(config.gateway_event)
           << "\",\"is_field\":\"false\",\"is_reliable\":\"true\"},{\"event\":\"" << hex_id(config.vehicle_event)
           << "\",\"is_field\":\"false\",\"is_reliable\":\"true\"}],\"eventgroups\":[{\"eventgroup\":\""
           << hex_id(config.gateway_eventgroup) << "\",\"events\":[\"" << hex_id(config.gateway_event)
           << "\"]},{\"eventgroup\":\"" << hex_id(config.vehicle_eventgroup) << "\",\"events\":[\""
           << hex_id(config.vehicle_event) << "\"]}]}]";
  }
  output << "}";
  return output.str();
}

}
