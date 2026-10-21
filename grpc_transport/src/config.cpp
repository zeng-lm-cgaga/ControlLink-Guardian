#include "control_link_grpc_transport/config.hpp"

#include <algorithm>
#include <cctype>
#include <filesystem>
#include <fstream>
#include <limits>
#include <set>
#include <stdexcept>
#include <string>

#include <yaml-cpp/yaml.h>

namespace control_link::grpc_transport {
namespace {

[[noreturn]] void invalid(const std::string &field, const std::string &reason) {
  throw std::invalid_argument("gRPC config " + field + ": " + reason);
}

std::string text(const YAML::Node &node, const std::string &field) {
  if (!node.IsScalar() || node.Scalar().empty()) invalid(field, "nonempty scalar required");
  return node.Scalar();
}

void unique_keys(const YAML::Node &node, const std::string &path, std::uint32_t depth = 0) {
  if (depth > 32) invalid(path, "configuration nesting/cyclic alias exceeds local parse budget");
  if (node.IsMap()) {
    std::set<std::string> seen;
    for (const auto &entry : node) {
      const auto key = text(entry.first, path);
      if (!seen.insert(key).second) invalid(path + "." + key, "duplicate field");
      unique_keys(entry.second, path + "." + key, depth + 1);
    }
  } else if (node.IsSequence()) {
    for (const auto &entry : node) unique_keys(entry, path, depth + 1);
  }
}

void keys(const YAML::Node &node, const std::string &path,
          std::initializer_list<const char *> allowed) {
  if (!node.IsMap()) invalid(path, "mapping required");
  const std::set<std::string> fields(allowed.begin(), allowed.end());
  for (const auto &entry : node) {
    const auto key = text(entry.first, path);
    if (!fields.count(key)) invalid(path + "." + key, "unknown field");
  }
  for (const auto &field : fields) {
    if (!node[field]) invalid(path + "." + field, "required field missing");
  }
}

std::uint32_t integer(const YAML::Node &node, const std::string &field) {
  const auto value = text(node, field);
  std::uint64_t result = 0;
  for (const unsigned char character : value) {
    if (character < '0' || character > '9') invalid(field, "unsigned decimal integer required");
    result = result * 10 + character - '0';
    if (result > std::numeric_limits<std::uint32_t>::max()) invalid(field, "uint32 overflow");
  }
  return static_cast<std::uint32_t>(result);
}

bool boolean(const YAML::Node &node, const std::string &field) {
  const auto value = text(node, field);
  if (value == "true") return true;
  if (value == "false") return false;
  invalid(field, "explicit true or false required");
}

void exact(const YAML::Node &node, const std::string &field, const std::string &expected) {
  if (text(node, field) != expected) invalid(field, "must equal " + expected);
}

}

Config load_config(const std::string &path) {
  try {
    if (!std::filesystem::is_regular_file(path) || std::filesystem::file_size(path) > 65536) {
      invalid(path, "explicit regular YAML must fit 64KiB");
    }
    std::ifstream input(path, std::ios::binary);
    if (!input) invalid(path, "cannot read YAML");
    std::string bytes(65537, 0);
    input.read(bytes.data(), static_cast<std::streamsize>(bytes.size()));
    if (input.bad() || (input.fail() && !input.eof()) || input.gcount() > 65536) {
      invalid(path, "cannot read bounded YAML");
    }
    bytes.resize(static_cast<std::size_t>(input.gcount()));
    const auto root = YAML::Load(bytes);
    unique_keys(root, "network");
    if (!root.IsMap() || integer(root["schema_version"], "schema_version") != 1) {
      invalid("schema_version", "must equal 1");
    }
    const auto planning = root["planning"];
    exact(planning["source_id"], "planning.source_id", "planning");
    if (!boolean(planning["exclusive"], "planning.exclusive")) invalid("planning.exclusive", "must be true");
    const auto safety = root["safety_owners"];
    exact(safety["timestamp_domain"], "safety_owners.timestamp_domain", "SYSTEM");
    exact(safety["reference_frame"], "safety_owners.reference_frame", "body");
    exact(safety["network_schema"], "safety_owners.network_schema", "control_link_network_v1.proto");
    exact(safety["command_codec"], "safety_owners.command_codec", "control_link_edge_protocol/command_v1");
    exact(safety["fingerprint_algorithm"], "safety_owners.fingerprint_algorithm", "sha256_proto_source_bytes");
    const auto resources = root["control_session_resources"];
    keys(resources, "control_session_resources", {"retired_open_capacity"});
    const auto grpc = root["grpc"];
    keys(grpc, "grpc", {"enabled", "listen_address", "service", "max_message_bytes",
        "pending_latest_capacity", "readonly_inflight_limit", "client_async_capacity", "client_submit_inflight_limit", "max_status_subscribers",
        "subscriber_latest_capacity", "max_unary_deadline_ms", "control_open_timeout_ms",
        "max_control_stream_lifetime_ms", "max_readonly_stream_lifetime_ms",
        "shutdown_timeout_ms", "reconnect_initial_ms", "reconnect_max_ms"});
    boolean(grpc["enabled"], "grpc.enabled");
    exact(grpc["service"], "grpc.service", "control_link.network.v1.PlanningIngress");
    if (integer(grpc["pending_latest_capacity"], "grpc.pending_latest_capacity") != 1 ||
        integer(grpc["subscriber_latest_capacity"], "grpc.subscriber_latest_capacity") != 1) {
      invalid("latest capacities", "control and subscriber latest must equal 1");
    }
    Config config;
    config.rate_limits = network::parse_rate_limit_policy(root["rate_limits"]);
    config.listen_address = text(grpc["listen_address"], "grpc.listen_address");
    const auto read = [&grpc](const char *field) {
      return integer(grpc[field], std::string("grpc.") + field);
    };
    config.max_message_bytes = read("max_message_bytes");
    config.readonly_inflight_limit = read("readonly_inflight_limit");
    config.client_async_capacity = read("client_async_capacity");
    config.client_submit_inflight_limit = read("client_submit_inflight_limit");
    config.max_status_subscribers = read("max_status_subscribers");
    config.max_unary_deadline_ms = read("max_unary_deadline_ms");
    config.control_open_timeout_ms = read("control_open_timeout_ms");
    config.max_control_stream_lifetime_ms = read("max_control_stream_lifetime_ms");
    config.max_readonly_stream_lifetime_ms = read("max_readonly_stream_lifetime_ms");
    config.shutdown_timeout_ms = read("shutdown_timeout_ms");
    config.reconnect_initial_ms = read("reconnect_initial_ms");
    config.reconnect_max_ms = read("reconnect_max_ms");
    config.ingress.max_payload_bytes = config.max_message_bytes;
    config.ingress.retired_open_capacity = integer(resources["retired_open_capacity"],
        "control_session_resources.retired_open_capacity");
    config.ingress.idle_timeout_ms = 0;
    // 只读共享 Graph/status 预算，不读取 SOME/IP 地址、ID、SD 或启用其 parser/依赖。
    const auto shared = root["someip"];
    config.status_event_period_ms = integer(shared["status_event_period_ms"], "someip.status_event_period_ms");
    config.graph_timeout_ms = integer(shared["graph_admission_timeout_ms"], "someip.graph_admission_timeout_ms");
    config.graph_quiet_ms = integer(shared["graph_admission_quiet_ms"], "someip.graph_admission_quiet_ms");
    config.graph_poll_ms = integer(shared["graph_admission_poll_ms"], "someip.graph_admission_poll_ms");
    validate_config(config);
    return config;
  } catch (const YAML::Exception &failure) {
    invalid(path, failure.what());
  }
}

void validate_config(const Config &config) {
  network::validate_rate_limit_policy(config.rate_limits);
  const auto separator = config.listen_address.rfind(':');
  if (separator == std::string::npos || separator == 0 || separator + 1 == config.listen_address.size()) {
    invalid("listen_address", "explicit loopback host:port required");
  }
  const auto host = config.listen_address.substr(0, separator);
  if (host != "127.0.0.1" && host != "localhost" && host != "[::1]") {
    invalid("listen_address", "this development ingress only permits loopback listeners");
  }
  const auto port = integer(YAML::Node(config.listen_address.substr(separator + 1)), "listen_address.port");
  if (port == 0 || port > 65535) invalid("listen_address.port", "must be in [1,65535]");
  if (config.max_message_bytes > static_cast<std::uint32_t>(std::numeric_limits<int>::max()) ||
      config.ingress.max_payload_bytes != config.max_message_bytes) {
    invalid("max_message_bytes", "must fit gRPC int size and match ingress payload budget");
  }
  ::control_link::network::v1::Capabilities minimum;
  minimum.set_schema_version(1);
  minimum.set_planning_transport(::control_link::network::v1::PLANNING_TRANSPORT_GRPC);
  minimum.set_max_command_bytes(control_link_edge_protocol::kCommandFrameSize);
  minimum.set_max_status_bytes(config.max_message_bytes);
  minimum.set_max_pending_targets(1);
  minimum.set_max_status_subscribers(config.max_status_subscribers);
  const auto minimum_size = ::control_link::network::required_protocol_payload_bytes(minimum, "r");
  if (config.max_message_bytes < minimum_size) {
    invalid("max_message_bytes", "mandatory protobuf groups require at least " +
        std::to_string(minimum_size) + " bytes; actual Contract/RMW checked before admission");
  }
  if (!config.ingress.retired_open_capacity || config.ingress.idle_timeout_ms != 0 ||
      !config.readonly_inflight_limit || !config.max_status_subscribers ||
      !config.client_async_capacity || config.client_async_capacity > 1024 ||
      !config.client_submit_inflight_limit || config.client_submit_inflight_limit > config.client_async_capacity) {
    invalid("resources", "positive bounded capacities required; gRPC has no SOME/IP idle budget");
  }
  const auto call_budget = std::uint64_t{4} * config.readonly_inflight_limit + config.max_status_subscribers + 5;
  if (call_budget > static_cast<std::uint64_t>(std::numeric_limits<int>::max()) ||
      call_budget > std::numeric_limits<std::size_t>::max() /
          ((config.max_message_bytes + std::uint64_t{65536}) * 4)) {
    invalid("resources", "derived transport stream/thread/memory quota is not representable");
  }
  if (!config.max_unary_deadline_ms || !config.control_open_timeout_ms ||
      config.max_control_stream_lifetime_ms <= config.control_open_timeout_ms ||
      !config.max_readonly_stream_lifetime_ms || !config.shutdown_timeout_ms || !config.status_event_period_ms) {
    invalid("deadlines", "independent positive budgets required; control lifetime must exceed first-open budget");
  }
  if (!config.reconnect_initial_ms || config.reconnect_initial_ms > config.reconnect_max_ms ||
      config.reconnect_max_ms > static_cast<std::uint32_t>(std::numeric_limits<int>::max())) {
    invalid("reconnect", "positive ordered budgets fitting gRPC int required");
  }
  if (!config.graph_timeout_ms || !config.graph_quiet_ms || !config.graph_poll_ms ||
      config.graph_poll_ms > config.graph_quiet_ms || config.graph_quiet_ms >= config.graph_timeout_ms) {
    invalid("shared Graph", "positive poll/quiet budgets must fit admission timeout");
  }
}

}
