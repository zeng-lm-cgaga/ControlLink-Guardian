#include "control_link_someip_transport/config.hpp"

#include <arpa/inet.h>

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

[[noreturn]] void invalid(const std::string &field, const std::string &reason) {
  throw std::invalid_argument("SOME/IP config " + field + ": " + reason);
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
                              "control_session_resources", "someip", "grpc", "mqtt"});
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
  if (config.max_payload_bytes < 64 || config.readonly_inflight_limit == 0 || config.retired_open_capacity == 0) {
    invalid("resources", "payload must fit codec; capacities must be positive");
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
