#include "control_link_someip_transport/config.hpp"
#include "control_link_someip_transport/ingress.hpp"

#include <unistd.h>

#include <algorithm>
#include <array>
#include <filesystem>
#include <fstream>
#include <limits>
#include <sstream>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include <gtest/gtest.h>
#include <yaml-cpp/yaml.h>

namespace control_link::someip {
namespace {

std::string source_config_path() {
#ifdef CONTROL_LINK_PROTOCOL_CONFIG_PATH
  return CONTROL_LINK_PROTOCOL_CONFIG_PATH;
#else
  return (std::filesystem::path(__FILE__).parent_path().parent_path().parent_path() /
          "config/adas/protocol_transports.yaml").string();
#endif
}

std::string fixture_yaml() {
  std::ifstream input(source_config_path());
  if (!input) throw std::runtime_error("cannot open protocol configuration fixture");
  std::ostringstream output;
  output << input.rdbuf();
  return output.str();
}

class TemporaryConfig final {
 public:
  explicit TemporaryConfig(const std::string &yaml) {
    std::array<char, 48> pattern{};
    const std::string prefix = "/tmp/control-link-someip-config-XXXXXX";
    std::copy(prefix.begin(), prefix.end(), pattern.begin());
    const auto descriptor = ::mkstemp(pattern.data());
    if (descriptor < 0) throw std::runtime_error("mkstemp failed");
    path_ = pattern.data();
    ::close(descriptor);
    std::ofstream output(path_);
    output << yaml;
    output.flush();
    if (!output) throw std::runtime_error("failed to write temporary configuration");
  }

  ~TemporaryConfig() { std::filesystem::remove(path_); }
  const std::string &path() const { return path_; }

 private:
  std::string path_;
};

void rejects_change(const std::string &needle, const std::string &replacement) {
  auto yaml = fixture_yaml();
  const auto offset = yaml.find(needle);
  ASSERT_NE(offset, std::string::npos) << needle;
  yaml.replace(offset, needle.size(), replacement);
  const TemporaryConfig file(yaml);
  EXPECT_THROW(load_config(file.path()), std::invalid_argument) << needle << " -> " << replacement;
}

wire::Capabilities capacity_capabilities(const Config &config) {
  wire::Capabilities capabilities;
  capabilities.set_schema_version(1);
  const auto contract_path = std::filesystem::path(source_config_path()).parent_path().parent_path() /
      "common/gateway_contract.yaml";
  capabilities.set_contract_id(YAML::LoadFile(contract_path.string())["contract_id"].as<std::string>());
  capabilities.set_contract_hash(std::string(64, 'a'));
  capabilities.set_decision_config_hash(std::string(64, 'b'));
  capabilities.set_planning_transport(wire::PLANNING_TRANSPORT_SOMEIP);
  capabilities.set_max_command_bytes(control_link_edge_protocol::kCommandFrameSize);
  capabilities.set_max_status_bytes(config.max_payload_bytes);
  capabilities.set_max_pending_targets(1);
  capabilities.set_max_status_subscribers(config.readonly_inflight_limit);
  return capabilities;
}

TEST(SomeipConfig, RejectsPayloadBelowMandatoryProtocolFloorBeforeCreatingHolder) {
  auto config = load_config(source_config_path());
  const auto capabilities = capacity_capabilities(config);
  for (const auto capacity : {0U, 63U, 64U, 95U, 143U, 655U}) {
    SCOPED_TRACE(capacity);
    config.max_payload_bytes = capacity;
    EXPECT_THROW(validate_config(config), std::invalid_argument);
    EXPECT_THROW(PlanningIngress(config, capabilities), std::invalid_argument);
    EXPECT_THROW(vsomeip_configuration_json(config, "server", 0x3101, "capacity-test", true),
                 std::invalid_argument);
    rejects_change("  max_payload_bytes: 4096", "  max_payload_bytes: " + std::to_string(capacity));
  }
  try {
    validate_config(config);
    FAIL() << "undersized payload must be rejected before initialization";
  } catch (const std::invalid_argument &error) {
    EXPECT_NE(std::string{error.what()}.find("max_payload_bytes"), std::string::npos);
    EXPECT_NE(std::string{error.what()}.find("656"), std::string::npos);
  }
}

TEST(SomeipConfig, ActualIdentityCapacityUsesSerializedProductionShapeAtExactBoundary) {
  const auto original = load_config(source_config_path());
  auto capabilities = capacity_capabilities(original);
  const auto unchanged = capabilities.SerializeAsString();
  const std::string rmw = "rmw_fastrtps_cpp";
  const auto minimum = required_protocol_payload_bytes(capabilities, rmw);
  ASSERT_GT(minimum, 656U);
  ASSERT_LT(minimum, 4096U);
  RecordProperty("required_payload_bytes", std::to_string(minimum));
  EXPECT_EQ(capabilities.SerializeAsString(), unchanged);
  auto document = YAML::Load(fixture_yaml());
  document["someip"]["max_payload_bytes"] = minimum;
  const TemporaryConfig file(YAML::Dump(document));
  auto config = load_config(file.path());
  capabilities.set_max_status_bytes(config.max_payload_bytes);
  ASSERT_EQ(required_protocol_payload_bytes(capabilities, rmw), minimum);
  EXPECT_EQ(config.max_payload_bytes, minimum);
  EXPECT_NO_THROW(validate_config(config));
  EXPECT_NO_THROW(validate_protocol_payload_budget(config, capabilities, rmw));
  EXPECT_NO_THROW(vsomeip_configuration_json(config, "server", 0x3101, "capacity-test", true));
  config.max_payload_bytes = static_cast<std::uint32_t>(minimum - 1);
  EXPECT_NO_THROW(validate_config(config));
  EXPECT_THROW(validate_protocol_payload_budget(config, capabilities, rmw), std::invalid_argument);
  EXPECT_EQ(load_config(source_config_path()).max_payload_bytes, 4096U);
}

TEST(SomeipConfig, ActualMinimumPayloadAllowsOpenRetryAndAdmittedKeepWithCapabilities) {
  auto config = load_config(source_config_path());
  auto capabilities = capacity_capabilities(config);
  const std::string rmw = "rmw_fastrtps_cpp";
  config.max_payload_bytes = static_cast<std::uint32_t>(required_protocol_payload_bytes(capabilities, rmw));
  capabilities.set_max_status_bytes(config.max_payload_bytes);
  capabilities.set_max_status_subscribers(config.readonly_inflight_limit);
  ASSERT_NO_THROW(validate_protocol_payload_budget(config, capabilities, rmw));
  PlanningIngress ingress(config, capabilities);
  wire::OpenSessionRequest open;
  open.set_schema_version(1);
  open.set_holder_nonce(std::string(16, 'n'));
  open.set_open_request_id(std::string(16, 'o'));
  const auto opening = ingress.open(0x3101, open, 1'000'000'000ULL);
  ASSERT_EQ(opening.result(), wire::SESSION_OPERATION_RESULT_OPENING);
  EXPECT_LE(opening.SerializeAsString().size(), config.max_payload_bytes);
  const auto epoch = ingress.snapshot().epoch;
  const auto retry = ingress.open(0x3101, open, 1'000'000'001ULL);
  ASSERT_EQ(retry.result(), wire::SESSION_OPERATION_RESULT_IDEMPOTENT);
  EXPECT_EQ(ingress.snapshot().epoch, epoch);
  EXPECT_EQ(retry.session().session_token(), opening.session().session_token());
  EXPECT_LE(retry.SerializeAsString().size(), config.max_payload_bytes);
  ASSERT_TRUE(ingress.bind_publisher(epoch, std::string(24, 'g'), rmw));
  ASSERT_TRUE(ingress.set_admitted(epoch, true));
  wire::KeepSessionRequest keep;
  keep.set_schema_version(1);
  keep.set_holder_nonce(open.holder_nonce());
  keep.set_open_request_id(open.open_request_id());
  keep.set_keepalive_sequence(1);
  const auto admitted = ingress.keep(0x3101, keep, 1'000'000'002ULL);
  ASSERT_EQ(admitted.result(), wire::SESSION_OPERATION_RESULT_ADMITTED);
  ASSERT_TRUE(admitted.session().submission_ready());
  EXPECT_EQ(admitted.session().session_token(), opening.session().session_token());
  EXPECT_EQ(admitted.capabilities().schema_fingerprint().size(), 32U);
  EXPECT_EQ(admitted.capabilities().contract_hash().size(), 64U);
  EXPECT_EQ(admitted.capabilities().decision_config_hash().size(), 64U);
  EXPECT_LE(admitted.SerializeAsString().size(), config.max_payload_bytes);
}

TEST(SomeipConfig, BudgetRejectsUnknownRmwAndCodecOnlyOrFixedFloorCapacity) {
  auto config = load_config(source_config_path());
  const auto capabilities = capacity_capabilities(config);
  EXPECT_THROW(required_protocol_payload_bytes(capabilities, ""), std::invalid_argument);
  EXPECT_THROW(validate_protocol_payload_budget(config, capabilities, ""), std::invalid_argument);
  for (const auto capacity : {64U, 656U}) {
    SCOPED_TRACE(capacity);
    config.max_payload_bytes = capacity;
    EXPECT_THROW(validate_protocol_payload_budget(config, capabilities, "rmw_fastrtps_cpp"),
                 std::invalid_argument);
  }
}

TEST(SomeipConfig, BudgetUsesActualUnboundedCapabilityAndRmwTextWithoutTruncation) {
  auto config = load_config(source_config_path());
  auto capabilities = capacity_capabilities(config);
  capabilities.set_contract_id(std::string(4096, 'c'));
  const std::string rmw = "rmw_fastrtps_cpp";
  const auto minimum = required_protocol_payload_bytes(capabilities, rmw);
  ASSERT_GT(minimum, config.max_payload_bytes);
  EXPECT_THROW(validate_protocol_payload_budget(config, capabilities, rmw), std::invalid_argument);
  EXPECT_THROW(PlanningIngress(config, capabilities), std::invalid_argument);
  config.max_payload_bytes = static_cast<std::uint32_t>(minimum);
  capabilities.set_max_status_bytes(config.max_payload_bytes);
  ASSERT_EQ(required_protocol_payload_bytes(capabilities, rmw), minimum);
  EXPECT_NO_THROW(validate_protocol_payload_budget(config, capabilities, rmw));
  EXPECT_NO_THROW(PlanningIngress(config, capabilities));
  EXPECT_EQ(capabilities.contract_id(), std::string(4096, 'c'));
  const std::string longer_rmw(2048, 'r');
  const auto longer_minimum = required_protocol_payload_bytes(capabilities, longer_rmw);
  ASSERT_GT(longer_minimum, minimum);
  EXPECT_THROW(validate_protocol_payload_budget(config, capabilities, longer_rmw), std::invalid_argument);
  config.max_payload_bytes = static_cast<std::uint32_t>(longer_minimum);
  capabilities.set_max_status_bytes(config.max_payload_bytes);
  EXPECT_EQ(required_protocol_payload_bytes(capabilities, longer_rmw), longer_minimum);
  EXPECT_NO_THROW(validate_protocol_payload_budget(config, capabilities, longer_rmw));
  EXPECT_EQ(capabilities.contract_hash(), std::string(64, 'a'));
  EXPECT_EQ(capabilities.decision_config_hash(), std::string(64, 'b'));
}

TEST(SomeipConfig, BindBudgetFailureRevokesBeforeWritingIdentityAndStaleEpochCannotRevokeNewHolder) {
  auto config = load_config(source_config_path());
  config.max_payload_bytes = 656;
  auto capabilities = capacity_capabilities(config);
  PlanningIngress ingress(config, capabilities);
  wire::OpenSessionRequest request;
  request.set_schema_version(1);
  request.set_holder_nonce(std::string(16, 'n'));
  request.set_open_request_id(std::string(16, 'o'));
  ASSERT_EQ(ingress.open(0x3101, request, 1'000'000'000ULL).result(),
            wire::SESSION_OPERATION_RESULT_OPENING);
  const auto old_epoch = ingress.snapshot().epoch;
  ASSERT_FALSE(ingress.bind_publisher(old_epoch, std::string(24, 'g'), "rmw_fastrtps_cpp"));
  const auto revoked = ingress.snapshot();
  EXPECT_EQ(revoked.state, wire::CONTROL_SESSION_STATE_REVOKING);
  EXPECT_FALSE(revoked.session.submission_ready());
  EXPECT_TRUE(revoked.session.ingress_publisher().gid().empty());
  EXPECT_NE(revoked.failure.find("PAYLOAD_CAPACITY"), std::string::npos);
  EXPECT_FALSE(ingress.set_admitted(old_epoch, true));
  ASSERT_TRUE(ingress.finish_release(revoked.epoch));
  request.set_holder_nonce(std::string(16, 'p'));
  request.set_open_request_id(std::string(16, 'q'));
  ASSERT_EQ(ingress.open(0x3101, request, 1'000'000'001ULL).result(),
            wire::SESSION_OPERATION_RESULT_OPENING);
  const auto new_epoch = ingress.snapshot().epoch;
  EXPECT_FALSE(ingress.bind_publisher(old_epoch, std::string(24, 'g'), std::string(2048, 'r')));
  EXPECT_EQ(ingress.snapshot().state, wire::CONTROL_SESSION_STATE_OPENING);
  EXPECT_EQ(ingress.snapshot().epoch, new_epoch);
  ASSERT_TRUE(ingress.bind_publisher(new_epoch, std::string(24, 'g'), "r"));
  EXPECT_TRUE(ingress.set_admitted(new_epoch, true));
}

TEST(SomeipConfig, ReadsOwnerWithoutRequiringEnabledFlagOrNetworkLibraries) {
  const auto config = load_config(source_config_path());
  EXPECT_EQ(config.application_name, "control_link_someip_planning");
  EXPECT_EQ(config.service_id, 0x6c10);
  EXPECT_EQ(config.instance_id, 1);
  EXPECT_EQ(config.major_version, 1);
  EXPECT_EQ(config.minor_version, 0);
  EXPECT_EQ(config.open_method, 1);
  EXPECT_EQ(config.close_method, 5);
  EXPECT_EQ(config.gateway_event, 0x8001);
  EXPECT_EQ(config.vehicle_event, 0x8002);
  EXPECT_EQ(config.reliable_port, 30510);
  EXPECT_EQ(config.sd_port, 30490);
  EXPECT_EQ(config.max_payload_bytes, 4096);
  EXPECT_EQ(config.keep_period_ms, 1000);
  EXPECT_EQ(config.idle_timeout_ms, 5000);
  EXPECT_EQ(config.retired_open_capacity, 64);
  EXPECT_EQ(config.status_event_period_ms, 100);
  EXPECT_EQ(config.graph_timeout_ms, 30000);
  EXPECT_EQ(config.graph_quiet_ms, 250);
  EXPECT_EQ(config.graph_poll_ms, 50);
  EXPECT_EQ(config.shutdown_timeout_ms, 2000);
}

TEST(SomeipConfig, RejectsMissingAndUnknownSchemaFields) {
  rejects_change("schema_version: 1\n", "");
  rejects_change("schema_version: 1", "schema_version: 2");
  const auto role = YAML::Load(fixture_yaml())["configuration_role"].as<std::string>();
  rejects_change("configuration_role: " + role, "configuration_role: arbitrary");
  rejects_change("  graph_admission_timeout_ms: 30000\n", "");
  rejects_change("  graph_admission_poll_ms: 50", "  graph_admission_poll_ms: 50\n  gateway_readiness_timeout_ms: 1000");
  rejects_change("  session_idle_timeout_ms: 5000", "  session_token: abc\n  session_idle_timeout_ms: 5000");
  rejects_change("  retired_open_capacity: 64", "  retired_open_capacity: 64\n  token: abc");
}

TEST(SomeipConfig, RejectsDuplicateKeysIncludingUnusedProtocolSections) {
  rejects_change("schema_version: 1", "schema_version: 1\nschema_version: 1");
  rejects_change("    open_session: 0x0001", "    open_session: 0x0001\n    open_session: 0x0001");
  rejects_change("  max_unary_deadline_ms: 2000", "  max_unary_deadline_ms: 2000\n  max_unary_deadline_ms: 2000");
  rejects_change("  graph_admission_poll_ms: 50", "  graph_admission_poll_ms: 50\n  graph_admission_poll_ms: 50");
}

TEST(SomeipConfig, AcceptsHistoricalAndRuntimeRolesWithoutImplicitEnablement) {
  auto yaml = YAML::Load(fixture_yaml());
  yaml["someip"]["enabled"] = false;
  yaml["configuration_role"] = "contract_only";
  const TemporaryConfig historical(YAML::Dump(yaml));
  EXPECT_NO_THROW(load_config(historical.path()));
  yaml["configuration_role"] = "runtime_someip_and_future_contracts";
  yaml["planning"]["runtime_available_transports"] = YAML::Load("[ros2, edge, someip]");
  const TemporaryConfig current(YAML::Dump(yaml));
  EXPECT_NO_THROW(load_config(current.path()));
}

TEST(SomeipConfig, RejectsNonintegerNegativeOverflowAndPartialNumericScalars) {
  for (const auto *value : {"-1", "1.0", "1e3", "1000x", "0x", "4294967296", "18446744073709551616"}) {
    rejects_change("  graph_admission_timeout_ms: 30000", std::string("  graph_admission_timeout_ms: ") + value);
  }
  rejects_change("  reliable_port: 30510", "  reliable_port: 65536");
  rejects_change("  major_version: 1", "  major_version: 256");
  rejects_change("  service_id: 0x6c10", "  service_id: 0x10000");
}

TEST(SomeipConfig, RejectsIdentifierOverlapAndWildcards) {
  rejects_change("    submit: 0x0002", "    submit: 0x0001");
  rejects_change("    gateway_status: 0x8001", "    gateway_status: 0x0001");
  rejects_change("    vehicle_status: 0x8002", "    vehicle_status: 0x8001");
  rejects_change("  service_id: 0x6c10", "  service_id: 0xffff");
  rejects_change("  instance_id: 0x0001", "  instance_id: 0");
  rejects_change("  major_version: 1", "  major_version: 255");
  rejects_change("  minor_version: 0", "  minor_version: 4294967295");
}

TEST(SomeipConfig, EnforcesCodecCapacityDiscoveryAndSafetyOwners) {
  rejects_change("  command_payload_bytes: 64", "  command_payload_bytes: 63");
  rejects_change("  pending_latest_capacity: 1", "  pending_latest_capacity: 2");
  rejects_change("  max_payload_bytes: 4096", "  max_payload_bytes: 63");
  rejects_change("  readonly_inflight_limit: 4", "  readonly_inflight_limit: 0");
  rejects_change("  retired_open_capacity: 64", "  retired_open_capacity: 0");
  rejects_change("    enabled: true", "    enabled: false");
  rejects_change("  timestamp_domain: SYSTEM", "  timestamp_domain: STEADY");
  rejects_change("  source_id: planning", "  source_id: teleop");
  rejects_change("  exclusive: true", "  exclusive: false");
  rejects_change("  contract: common/gateway_contract.yaml", "  contract: arbitrary.yaml");
}

TEST(SomeipConfig, EnforcesSteadyResourceBudgetsWithoutAddingBusinessThresholds) {
  rejects_change("  session_keepalive_period_ms: 1000", "  session_keepalive_period_ms: 5000");
  rejects_change("  status_event_period_ms: 100", "  status_event_period_ms: 0");
  rejects_change("  status_event_period_ms: 100", "  status_event_period_ms: 5001");
  rejects_change("  reconnect_initial_ms: 100", "  reconnect_initial_ms: 2001");
  rejects_change("  graph_admission_quiet_ms: 250", "  graph_admission_quiet_ms: 30000");
  rejects_change("  graph_admission_poll_ms: 50", "  graph_admission_poll_ms: 251");
  rejects_change("  shutdown_timeout_ms: 2000", "  shutdown_timeout_ms: 0");
  auto config = load_config(source_config_path());
  config.max_payload_bytes = std::numeric_limits<std::uint32_t>::max();
  config.readonly_inflight_limit = std::numeric_limits<std::uint32_t>::max();
  EXPECT_THROW(validate_config(config), std::invalid_argument);
}

TEST(SomeipConfig, RejectsInvalidNetworkAddressAndUnsafeDomainIdentifiers) {
  rejects_change("  bind_address: 127.0.0.1", "  bind_address: 0.0.0.0");
  rejects_change("  bind_address: 127.0.0.1", "  bind_address: 224.0.0.1");
  rejects_change("    multicast_address: 224.224.224.245", "    multicast_address: 127.0.0.1");
  const auto config = load_config(source_config_path());
  EXPECT_THROW(vsomeip_configuration_json(config, "app", 0, "net", true), std::invalid_argument);
  EXPECT_THROW(vsomeip_configuration_json(config, "app", 1, "../../shared", true), std::invalid_argument);
  EXPECT_THROW(vsomeip_configuration_json(config, "app\"x", 1, "net", true), std::invalid_argument);
}

TEST(SomeipConfig, GeneratesSeparateRoutingDomainsAndBoundedExternalLocalQueues) {
  const auto config = load_config(source_config_path());
  const auto server = YAML::Load(vsomeip_configuration_json(config, "server", 0x3101, "p1-server", true));
  const auto client = YAML::Load(vsomeip_configuration_json(config, "client", 0x3201, "p1-client", false));
  EXPECT_EQ(server["network"].as<std::string>(), "p1-server");
  EXPECT_EQ(client["network"].as<std::string>(), "p1-client");
  EXPECT_EQ(server["routing"].as<std::string>(), "server");
  EXPECT_EQ(client["routing"].as<std::string>(), "client");
  EXPECT_TRUE(server["service-discovery"]["enable"].as<bool>());
  EXPECT_EQ(server["service-discovery"]["protocol"].as<std::string>(), "udp");
  EXPECT_EQ(server["services"][0]["service"].as<std::string>(), "0x6c10");
  EXPECT_EQ(server["services"][0]["reliable"]["port"].as<std::uint16_t>(), 30510);
  EXPECT_FALSE(client["services"]);
  for (const auto &json : {server, client}) {
    EXPECT_EQ(json["max-payload-size-reliable"].as<std::uint32_t>(), 4096);
    const auto external = json["endpoint-queue-limit-external"].as<std::uint32_t>();
    const auto local = json["endpoint-queue-limit-local"].as<std::uint32_t>();
    EXPECT_EQ(external, (4096 + 16) * (4 + 5));
    EXPECT_EQ(local, external);
    EXPECT_LT(external, std::numeric_limits<std::uint32_t>::max());
  }
}

TEST(SomeipConfig, RejectsMalformedYamlAndBoundedAliasRecursion) {
  const TemporaryConfig malformed("someip: [");
  EXPECT_THROW(load_config(malformed.path()), std::invalid_argument);
  const TemporaryConfig recursive("root: &root\n  child: *root\n");
  EXPECT_THROW(load_config(recursive.path()), std::invalid_argument);
  EXPECT_THROW(load_config("/nonexistent/control-link-protocol.yaml"), std::invalid_argument);
}

TEST(SomeipConfig, PayloadQueueLimitHasExactHeaderCapacityAndOverflowBoundary) {
  auto config = load_config(source_config_path());
  config.readonly_inflight_limit = 1;
  ASSERT_NO_THROW(validate_config(config));
  const auto minimum = YAML::Load(vsomeip_configuration_json(config, "server", 0x3101, "small-payload", true));
  EXPECT_EQ(minimum["max-payload-size-reliable"].as<std::uint32_t>(), config.max_payload_bytes);
  EXPECT_EQ(minimum["max-payload-size-local"].as<std::uint32_t>(), config.max_payload_bytes);
  const auto queue_bytes = (std::uint64_t{config.max_payload_bytes} + 16) * 6;
  EXPECT_EQ(minimum["endpoint-queue-limit-external"].as<std::uint64_t>(), queue_bytes);
  EXPECT_EQ(minimum["endpoint-queue-limit-local"].as<std::uint64_t>(), queue_bytes);
  config.max_payload_bytes = 715827866;
  ASSERT_NO_THROW(validate_config(config));
  const auto maximum = YAML::Load(vsomeip_configuration_json(config, "server", 0x3101, "max-queue", true));
  EXPECT_EQ(maximum["endpoint-queue-limit-external"].as<std::uint64_t>(), 4294967292ULL);
  ++config.max_payload_bytes;
  EXPECT_THROW(validate_config(config), std::invalid_argument);
  EXPECT_THROW(vsomeip_configuration_json(config, "server", 0x3101, "max-queue", true), std::invalid_argument);
}

TEST(SomeipConfig, ShutdownBudgetDoesNotBorrowHandshakeReconnectBudget) {
  auto config = load_config(source_config_path());
  config.reconnect_initial_ms = 300;
  config.reconnect_max_ms = 4000;
  config.shutdown_timeout_ms = 1700;
  const auto document = YAML::Load(vsomeip_configuration_json(config, "server", 0x3101, "stop-budget", true));
  EXPECT_EQ(document["shutdown_timeout"].as<std::uint32_t>(), 1700);
  config.shutdown_timeout_ms = 0;
  EXPECT_THROW(validate_config(config), std::invalid_argument);
}

TEST(SomeipConfig, RuntimeJsonMatchesOwnerButKeepsExplicitAssemblyIdentity) {
  const auto config = load_config(source_config_path());
  const TemporaryConfig server(vsomeip_configuration_json(config, config.application_name, 0x3101, "server", true));
  EXPECT_NO_THROW(validate_vsomeip_configuration(config, server.path(), config.application_name, true));
  auto client_config = config;
  client_config.bind_address = "127.0.0.3";
  const TemporaryConfig client(vsomeip_configuration_json(client_config, "external", 0x3102, "client", false));
  EXPECT_NO_THROW(validate_vsomeip_configuration(config, client.path(), "external", false));
  EXPECT_THROW(validate_vsomeip_configuration(config, client.path(), "other", false), std::invalid_argument);
  EXPECT_THROW(validate_vsomeip_configuration(config, client.path(), "external", true), std::invalid_argument);
}

TEST(SomeipConfig, RuntimeJsonCannotOverrideSafetyResourceOrDiscoveryOwner) {
  const auto config = load_config(source_config_path());
  const auto original = vsomeip_configuration_json(config, config.application_name, 0x3101, "server", true);
  for (const auto &mutation : std::vector<std::pair<std::string, std::string>>{
      {"\"endpoint-queue-limit-external\":\"37008\"", "\"endpoint-queue-limit-external\":\"0\""},
      {"\"enable\":\"true\"", "\"enable\":\"false\""},
      {"\"max-payload-size-reliable\":\"4096\"", "\"max-payload-size-reliable\":\"1048576\""},
      {"\"service\":\"0x6c10\"", "\"service\":\"0x6c11\""},
      {"\"shutdown_timeout\":\"2000\"", "\"shutdown_timeout\":\"0\""}}) {
    auto mutated = original;
    const auto position = mutated.find(mutation.first);
    ASSERT_NE(position, std::string::npos);
    mutated.replace(position, mutation.first.size(), mutation.second);
    const TemporaryConfig invalid_json(mutated);
    EXPECT_THROW(validate_vsomeip_configuration(config, invalid_json.path(), config.application_name, true),
                 std::invalid_argument);
  }
  auto duplicate = original;
  duplicate.insert(1, "\"endpoint-queue-limit-external\":\"0\",");
  const TemporaryConfig duplicate_json(duplicate);
  EXPECT_THROW(validate_vsomeip_configuration(config, duplicate_json.path(), config.application_name, true),
               std::invalid_argument);
  auto extra = original;
  extra.insert(1, "\"endpoint-queue-limits\":[],");
  const TemporaryConfig extra_json(extra);
  EXPECT_THROW(validate_vsomeip_configuration(config, extra_json.path(), config.application_name, true),
               std::invalid_argument);
}

TEST(SomeipConfig, RuntimeJsonAcceptsStrictSyntaxWhitespaceAndEscapedKeys) {
  const auto config = load_config(source_config_path());
  for (const bool server : {false, true}) {
    const auto generated = vsomeip_configuration_json(config, config.application_name, 0x3101, "json-owner", server);
    const TemporaryConfig original(generated);
    EXPECT_NO_THROW(validate_vsomeip_configuration(config, original.path(), config.application_name, server));
    const TemporaryConfig whitespace(" \t\r\n" + generated + "\r\n\t ");
    EXPECT_NO_THROW(validate_vsomeip_configuration(config, whitespace.path(), config.application_name, server));
    auto escaped = generated;
    const auto position = escaped.find("\"routing\"");
    ASSERT_NE(position, std::string::npos);
    escaped.replace(position, std::string{"\"routing\""}.size(), "\"\\u0072outing\"");
    const TemporaryConfig escaped_key(escaped);
    EXPECT_NO_THROW(validate_vsomeip_configuration(config, escaped_key.path(), config.application_name, server));
  }
}

TEST(SomeipConfig, RuntimeJsonRejectsYamlOnlySyntaxAndTrailingDocuments) {
  const auto config = load_config(source_config_path());
  const auto original = vsomeip_configuration_json(config, config.application_name, 0x3101, "strict-json", true);
  const std::vector<std::pair<std::string, std::string>> mutations{
      {"YAML comment", original + "\n# not JSON\n"},
      {"YAML document marker", "---\n" + original},
      {"YAML block mapping", YAML::Dump(YAML::Load(original))},
      {"second YAML document", original + "\n---\n" + original},
      {"second JSON value", original + "\n{}"},
      {"C++ comment", "// not JSON\n" + original},
      {"C comment", original + "\n/* not JSON */"}};
  for (const auto &mutation : mutations) {
    SCOPED_TRACE(mutation.first);
    const TemporaryConfig file(mutation.second);
    EXPECT_THROW(validate_vsomeip_configuration(config, file.path(), config.application_name, true),
                 std::invalid_argument);
  }
  const std::vector<std::pair<std::string, std::string>> replacements{
      {"\"unicast\":", "unicast: "},
      {"\"routing\":\"" + config.application_name + "\"", "\"routing\":'" + config.application_name + "'"},
      {"\"max_dispatchers\":\"1\"}]", "\"max_dispatchers\":\"1\"},]"}};
  for (const auto &replacement : replacements) {
    SCOPED_TRACE(replacement.second);
    auto mutated = original;
    const auto position = mutated.find(replacement.first);
    ASSERT_NE(position, std::string::npos);
    mutated.replace(position, replacement.first.size(), replacement.second);
    const TemporaryConfig file(mutated);
    EXPECT_THROW(validate_vsomeip_configuration(config, file.path(), config.application_name, true),
                 std::invalid_argument);
  }
  auto trailing_comma = original;
  trailing_comma.insert(trailing_comma.size() - 1, ",");
  const TemporaryConfig object_comma(trailing_comma);
  EXPECT_THROW(validate_vsomeip_configuration(config, object_comma.path(), config.application_name, true),
               std::invalid_argument);
}

TEST(SomeipConfig, RuntimeJsonRejectsYamlCommentWithNativeParserDiagnostic) {
  const auto config = load_config(source_config_path());
  const auto original = vsomeip_configuration_json(config, config.application_name, 0x3101, "json-diagnostic", true);
  const TemporaryConfig commented(original + "\n# valid YAML but invalid JSON\n");
  try {
    validate_vsomeip_configuration(config, commented.path(), config.application_name, true);
    FAIL() << "The actual JSON parser must reject YAML-only comments before vsomeip initialization";
  } catch (const std::invalid_argument &error) {
    EXPECT_NE(std::string{error.what()}.find("invalid JSON syntax"), std::string::npos);
  }
}

TEST(SomeipConfig, RuntimeJsonKeepsDuplicateKeyDepthAndNodeBudgets) {
  const auto config = load_config(source_config_path());
  const auto original = vsomeip_configuration_json(config, config.application_name, 0x3101, "json-budget", true);
  auto duplicate = original;
  duplicate.insert(1, "\"routing\":\"" + config.application_name + "\",");
  auto escaped_duplicate = original;
  escaped_duplicate.insert(1, "\"\\u0072outing\":\"" + config.application_name + "\",");
  auto deep = original;
  deep.insert(1, "\"budget\":" + std::string(34, '[') + "0" + std::string(34, ']') + ",");
  std::string values;
  for (std::size_t index = 0; index < 4097; ++index) {
    if (index != 0) values += ',';
    values += '0';
  }
  auto expanded = original;
  expanded.insert(1, "\"budget\":[" + values + "],");
  const std::vector<std::pair<std::string, std::string>> cases{
      {duplicate, "duplicate field"},
      {escaped_duplicate, "duplicate field"},
      {deep, "configuration nesting/node budget exceeded"},
      {expanded, "configuration nesting/node budget exceeded"}};
  for (const auto &entry : cases) {
    SCOPED_TRACE(entry.second);
    const TemporaryConfig file(entry.first);
    try {
      validate_vsomeip_configuration(config, file.path(), config.application_name, true);
      FAIL() << "JSON syntax validation must not replace configuration resource checks";
    } catch (const std::invalid_argument &error) {
      EXPECT_NE(std::string{error.what()}.find(entry.second), std::string::npos);
    }
  }
}

TEST(SomeipConfig, RuntimeJsonPreserves64KiBInclusiveFileBudget) {
  const auto config = load_config(source_config_path());
  auto padded = vsomeip_configuration_json(config, config.application_name, 0x3101, "json-size", true);
  ASSERT_LT(padded.size(), 65536U);
  padded.append(65536U - padded.size(), ' ');
  const TemporaryConfig maximum(padded);
  EXPECT_NO_THROW(validate_vsomeip_configuration(config, maximum.path(), config.application_name, true));
  padded += ' ';
  const TemporaryConfig excessive(padded);
  EXPECT_THROW(validate_vsomeip_configuration(config, excessive.path(), config.application_name, true),
               std::invalid_argument);
}

TEST(SomeipConfig, GeneratedServiceEventsAndDiscoveryUseChangedOwnerValuesExactly) {
  auto config = load_config(source_config_path());
  config.bind_address = "127.0.0.3";
  config.sd_multicast_address = "239.2.3.4";
  config.service_id = 0x4100;
  config.instance_id = 0x0020;
  config.reliable_port = 40110;
  config.sd_port = 40190;
  config.gateway_event = 0x8101;
  config.vehicle_event = 0x8102;
  config.gateway_eventgroup = 0x0021;
  config.vehicle_eventgroup = 0x0022;
  const auto server = YAML::Load(vsomeip_configuration_json(config, "changed-server", 0x3101, "changed-network", true));
  const auto client = YAML::Load(vsomeip_configuration_json(config, "changed-client", 0x3102, "client-network", false));
  EXPECT_EQ(server["unicast"].as<std::string>(), config.bind_address);
  EXPECT_EQ(server["services"][0]["service"].as<std::string>(), "0x4100");
  EXPECT_EQ(server["services"][0]["instance"].as<std::string>(), "0x0020");
  EXPECT_EQ(server["services"][0]["reliable"]["port"].as<std::uint16_t>(), config.reliable_port);
  const auto events = server["services"][0]["events"];
  ASSERT_EQ(events.size(), 2);
  EXPECT_EQ(events[0]["event"].as<std::string>(), "0x8101");
  EXPECT_EQ(events[1]["event"].as<std::string>(), "0x8102");
  const auto groups = server["services"][0]["eventgroups"];
  ASSERT_EQ(groups.size(), 2);
  EXPECT_EQ(groups[0]["eventgroup"].as<std::string>(), "0x0021");
  EXPECT_EQ(groups[0]["events"][0].as<std::string>(), "0x8101");
  EXPECT_EQ(groups[1]["eventgroup"].as<std::string>(), "0x0022");
  EXPECT_EQ(groups[1]["events"][0].as<std::string>(), "0x8102");
  for (const auto &json : {server, client}) {
    EXPECT_EQ(json["service-discovery"]["multicast"].as<std::string>(), config.sd_multicast_address);
    EXPECT_EQ(json["service-discovery"]["port"].as<std::uint16_t>(), config.sd_port);
    EXPECT_TRUE(json["service-discovery"]["enable"].as<bool>());
  }
  EXPECT_FALSE(client["services"]);
}

TEST(SomeipConfig, RejectsNodeExpansionBudgetWithoutRecursingIndefinitelyThroughAliases) {
  auto yaml = fixture_yaml();
  yaml += "expansion: &leaf [bounded]\nrepeated:\n";
  for (std::size_t index = 0; index < 4097; ++index) yaml += "  - *leaf\n";
  const TemporaryConfig amplified(yaml);
  try {
    load_config(amplified.path());
    FAIL() << "Alias expansion must exhaust the bounded traversal budget";
  } catch (const std::invalid_argument &error) {
    EXPECT_NE(std::string(error.what()).find("configuration nesting/node budget exceeded"), std::string::npos);
  }
}

}
}
