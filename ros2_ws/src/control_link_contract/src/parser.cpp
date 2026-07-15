#include "control_link_contract/parser.hpp"

#include <cmath>
#include <fstream>
#include <initializer_list>
#include <regex>
#include <set>
#include <sstream>
#include <utility>

#include <yaml-cpp/yaml.h>

namespace control_link_contract
{
namespace
{

class GatewayContractParser
{
public:
  explicit GatewayContractParser(std::string source_name)
  : source_name_(std::move(source_name))
  {
  }

  GatewayContractPtr parse(const std::string_view yaml_text) const
  {
    YAML::Node root;
    try {
      root = YAML::Load(std::string(yaml_text));
    } catch (const YAML::Exception & error) {
      fail("root", "invalid YAML", "valid YAML document", error.what(), "检查 YAML 语法");
    }

    require_map(root, "root");
    check_keys(
      root,
      {"schema_version", "contract_id", "gateway", "limits", "qos_profiles", "input",
        "output", "state_topics", "critical_endpoints"},
      "root");

    const auto schema_version = require_uint(root, "schema_version", "schema_version");
    if (schema_version != 1U) {
      fail(
        "schema_version", "unsupported schema version", "1",
        std::to_string(schema_version), "升级 parser 或改用受支持的 schema");
    }

    const auto contract_id = require_string(root, "contract_id", "contract_id");
    if (contract_id.empty()) {
      fail("contract_id", "empty identifier", "non-empty string", "empty", "填写 Contract 标识");
    }

    GatewayContract contract{
      schema_version,
      contract_id,
      parse_gateway(require_field(root, "gateway", "gateway")),
      parse_limits(require_field(root, "limits", "limits")),
      parse_qos_profiles(require_field(root, "qos_profiles", "qos_profiles")),
      parse_input(require_field(root, "input", "input")),
      parse_output(require_field(root, "output", "output")),
      parse_state_topics(require_field(root, "state_topics", "state_topics")),
      parse_critical_endpoints(
        require_field(root, "critical_endpoints", "critical_endpoints")),
    };

    validate_cross_fields(contract);
    return std::make_shared<const GatewayContract>(std::move(contract));
  }

private:
  [[noreturn]] void fail(
    const std::string & path, const std::string & rule, const std::string & expected,
    const std::string & actual, const std::string & hint) const
  {
    throw ContractError(
            source_name_ + ":" + path + ": " + rule + "; expected=" + expected +
            "; actual=" + actual + "; hint=" + hint);
  }

  static std::string child_path(const std::string & parent, const std::string & key)
  {
    return parent == "root" ? key : parent + "." + key;
  }

  static std::string describe(const YAML::Node & node)
  {
    if (!node || node.IsNull()) {
      return "missing";
    }
    if (node.IsMap()) {
      return "map";
    }
    if (node.IsSequence()) {
      return "sequence";
    }
    if (node.IsScalar()) {
      return "scalar(" + node.Scalar() + ")";
    }
    return "unknown";
  }

  static std::string join_keys(const std::set<std::string> & keys)
  {
    std::ostringstream result;
    bool first = true;
    for (const auto & key : keys) {
      if (!first) {
        result << ',';
      }
      result << key;
      first = false;
    }
    return result.str();
  }

  void require_map(const YAML::Node & node, const std::string & path) const
  {
    if (!node || !node.IsMap()) {
      fail(path, "wrong YAML type", "map", describe(node), "使用 key: value 结构");
    }
  }

  void require_sequence(const YAML::Node & node, const std::string & path) const
  {
    if (!node || !node.IsSequence()) {
      fail(path, "wrong YAML type", "sequence", describe(node), "使用 YAML 列表");
    }
  }

  void check_keys(
    const YAML::Node & node, const std::initializer_list<const char *> allowed,
    const std::string & path) const
  {
    require_map(node, path);
    std::set<std::string> allowed_keys;
    for (const auto * key : allowed) {
      allowed_keys.emplace(key);
    }

    std::set<std::string> seen_keys;
    for (const auto & entry : node) {
      if (!entry.first.IsScalar()) {
        fail(path, "non-scalar key", "string key", describe(entry.first), "使用字符串字段名");
      }
      const auto key = entry.first.Scalar();
      if (allowed_keys.count(key) == 0U) {
        fail(
          child_path(path, key), "unknown field", join_keys(allowed_keys), describe(entry.second),
          "删除字段或先更新权威 schema");
      }
      if (!seen_keys.emplace(key).second) {
        fail(
          child_path(path, key), "duplicate field", "unique field", key,
          "删除重复字段，确保配置只有一个明确值");
      }
    }
  }

  YAML::Node require_field(
    const YAML::Node & parent, const std::string & key, const std::string & path) const
  {
    const YAML::Node value = parent[key];
    if (!value || value.IsNull()) {
      fail(path, "missing required field", "present", "missing", "补充必填字段");
    }
    return value;
  }

  std::string scalar_as_string(const YAML::Node & node, const std::string & path) const
  {
    if (!node || !node.IsScalar()) {
      fail(path, "wrong YAML type", "string", describe(node), "填写字符串值");
    }
    try {
      return node.as<std::string>();
    } catch (const YAML::Exception & error) {
      fail(path, "invalid string", "string", error.what(), "检查字段类型");
    }
  }

  std::string require_string(
    const YAML::Node & parent, const std::string & key, const std::string & path) const
  {
    return scalar_as_string(require_field(parent, key, path), path);
  }

  std::uint64_t scalar_as_uint(const YAML::Node & node, const std::string & path) const
  {
    if (!node || !node.IsScalar()) {
      fail(path, "wrong YAML type", "non-negative integer", describe(node), "填写整数值");
    }
    try {
      const auto value = node.as<long long>();
      if (value < 0) {
        fail(
          path, "negative integer", "non-negative integer", std::to_string(value),
          "使用大于或等于 0 的整数");
      }
      return static_cast<std::uint64_t>(value);
    } catch (const YAML::Exception & error) {
      fail(path, "invalid integer", "non-negative integer", error.what(), "检查字段类型");
    }
  }

  std::uint64_t require_uint(
    const YAML::Node & parent, const std::string & key, const std::string & path) const
  {
    return scalar_as_uint(require_field(parent, key, path), path);
  }

  std::optional<std::uint64_t> optional_uint(
    const YAML::Node & parent, const std::string & key, const std::string & path) const
  {
    const YAML::Node value = parent[key];
    if (!value || value.IsNull()) {
      return std::nullopt;
    }
    return scalar_as_uint(value, path);
  }

  double require_double(
    const YAML::Node & parent, const std::string & key, const std::string & path) const
  {
    const YAML::Node node = require_field(parent, key, path);
    if (!node.IsScalar()) {
      fail(path, "wrong YAML type", "finite number", describe(node), "填写数值");
    }
    try {
      const auto value = node.as<double>();
      if (!std::isfinite(value)) {
        fail(path, "non-finite number", "finite number", node.Scalar(), "使用有限数值");
      }
      return value;
    } catch (const YAML::Exception & error) {
      fail(path, "invalid number", "finite number", error.what(), "检查字段类型");
    }
  }

  bool require_bool(
    const YAML::Node & parent, const std::string & key, const std::string & path) const
  {
    const YAML::Node node = require_field(parent, key, path);
    if (!node.IsScalar()) {
      fail(path, "wrong YAML type", "boolean", describe(node), "填写 true 或 false");
    }
    try {
      return node.as<bool>();
    } catch (const YAML::Exception & error) {
      fail(path, "invalid boolean", "true|false", error.what(), "填写 YAML boolean");
    }
  }

  static bool is_valid_node_fqn(const std::string & value)
  {
    static const std::regex pattern(
      R"(^/[A-Za-z_][A-Za-z0-9_]*(/[A-Za-z_][A-Za-z0-9_]*)*$)");
    return std::regex_match(value, pattern);
  }

  static bool is_valid_topic_name(const std::string & value)
  {
    static const std::regex pattern(
      R"(^/[A-Za-z_][A-Za-z0-9_]*(/[A-Za-z_][A-Za-z0-9_]*)*$)");
    return std::regex_match(value, pattern);
  }

  static bool is_valid_interface_type(const std::string & value)
  {
    static const std::regex pattern(
      R"(^[a-z][a-z0-9_]*/(msg|srv)/[A-Za-z][A-Za-z0-9_]*$)");
    return std::regex_match(value, pattern);
  }

  GatewaySettings parse_gateway(const YAML::Node & node) const
  {
    check_keys(
      node,
      {"node_fqn", "output_rate_hz", "command_timeout_ms", "source_switch_hold_ms",
        "recovery_valid_samples", "graph_poll_ms", "graph_stable_window_ms",
        "ros_clock_stall_timeout_ms", "vehicle_state_topic_timeout_ms",
        "output_tick_late_threshold_ms", "consecutive_late_ticks_to_safe_stop"},
      "gateway");

    GatewaySettings result{
      require_string(node, "node_fqn", "gateway.node_fqn"),
      require_double(node, "output_rate_hz", "gateway.output_rate_hz"),
      require_uint(node, "command_timeout_ms", "gateway.command_timeout_ms"),
      require_uint(node, "source_switch_hold_ms", "gateway.source_switch_hold_ms"),
      require_uint(node, "recovery_valid_samples", "gateway.recovery_valid_samples"),
      require_uint(node, "graph_poll_ms", "gateway.graph_poll_ms"),
      require_uint(node, "graph_stable_window_ms", "gateway.graph_stable_window_ms"),
      require_uint(
        node, "ros_clock_stall_timeout_ms", "gateway.ros_clock_stall_timeout_ms"),
      require_uint(
        node, "vehicle_state_topic_timeout_ms", "gateway.vehicle_state_topic_timeout_ms"),
      require_uint(
        node, "output_tick_late_threshold_ms", "gateway.output_tick_late_threshold_ms"),
      require_uint(
        node, "consecutive_late_ticks_to_safe_stop",
        "gateway.consecutive_late_ticks_to_safe_stop"),
    };

    if (!is_valid_node_fqn(result.node_fqn)) {
      fail(
        "gateway.node_fqn", "invalid ROS node FQN", "absolute node FQN",
        result.node_fqn, "例如 /control_link/gateway");
    }
    if (result.output_rate_hz <= 0.0) {
      fail(
        "gateway.output_rate_hz", "non-positive rate", "> 0", std::to_string(result.output_rate_hz),
        "配置正数输出频率");
    }
    if (result.command_timeout_ms == 0U) {
      fail(
        "gateway.command_timeout_ms", "zero timeout", "> 0", "0",
        "timeout 必须覆盖至少一个输出周期");
    }
    if (result.recovery_valid_samples == 0U) {
      fail(
        "gateway.recovery_valid_samples", "invalid recovery count", ">= 1", "0",
        "至少要求一条有效恢复样本");
    }
    if (result.graph_poll_ms == 0U || result.graph_stable_window_ms == 0U ||
      result.ros_clock_stall_timeout_ms == 0U || result.vehicle_state_topic_timeout_ms == 0U)
    {
      fail(
        "gateway", "zero health timeout", "all health periods > 0", "contains zero",
        "Graph、clock 和 VehicleState 周期必须为正数");
    }
    if (result.consecutive_late_ticks_to_safe_stop == 0U) {
      fail(
        "gateway.consecutive_late_ticks_to_safe_stop", "invalid late tick count", ">= 1", "0",
        "至少要求一次连续超限判定");
    }

    const double output_period_ms = 1000.0 / result.output_rate_hz;
    if (static_cast<double>(result.command_timeout_ms) < output_period_ms) {
      fail(
        "gateway.command_timeout_ms", "timeout shorter than output period",
        ">= " + std::to_string(output_period_ms), std::to_string(result.command_timeout_ms),
        "增大 timeout 或降低 output_rate_hz");
    }
    if (static_cast<double>(result.output_tick_late_threshold_ms) < output_period_ms) {
      fail(
        "gateway.output_tick_late_threshold_ms", "late threshold shorter than output period",
        ">= " + std::to_string(output_period_ms),
        std::to_string(result.output_tick_late_threshold_ms), "阈值不能小于目标输出周期");
    }
    return result;
  }

  CommandLimits parse_limits(const YAML::Node & node) const
  {
    check_keys(
      node,
      {"max_abs_linear_velocity_mps", "max_abs_angular_velocity_radps", "reject_non_finite",
        "reject_zero_stamp", "max_future_skew_ms"},
      "limits");

    CommandLimits result{
      require_double(
        node, "max_abs_linear_velocity_mps", "limits.max_abs_linear_velocity_mps"),
      require_double(
        node, "max_abs_angular_velocity_radps", "limits.max_abs_angular_velocity_radps"),
      require_bool(node, "reject_non_finite", "limits.reject_non_finite"),
      require_bool(node, "reject_zero_stamp", "limits.reject_zero_stamp"),
      require_uint(node, "max_future_skew_ms", "limits.max_future_skew_ms"),
    };

    if (result.max_abs_linear_velocity_mps <= 0.0) {
      fail(
        "limits.max_abs_linear_velocity_mps", "non-positive limit", "> 0",
        std::to_string(result.max_abs_linear_velocity_mps), "配置正数速度上限");
    }
    if (result.max_abs_angular_velocity_radps <= 0.0) {
      fail(
        "limits.max_abs_angular_velocity_radps", "non-positive limit", "> 0",
        std::to_string(result.max_abs_angular_velocity_radps), "配置正数角速度上限");
    }
    return result;
  }

  ReliabilityPolicy parse_reliability(
    const YAML::Node & node, const std::string & path) const
  {
    const auto value = scalar_as_string(node, path);
    if (value == "reliable") {
      return ReliabilityPolicy::kReliable;
    }
    if (value == "best_effort") {
      return ReliabilityPolicy::kBestEffort;
    }
    fail(path, "unknown QoS enum", "reliable|best_effort", value, "使用受支持的 reliability");
  }

  DurabilityPolicy parse_durability(
    const YAML::Node & node, const std::string & path) const
  {
    const auto value = scalar_as_string(node, path);
    if (value == "volatile") {
      return DurabilityPolicy::kVolatile;
    }
    if (value == "transient_local") {
      return DurabilityPolicy::kTransientLocal;
    }
    fail(path, "unknown QoS enum", "volatile|transient_local", value, "使用受支持的 durability");
  }

  HistoryPolicy parse_history(const YAML::Node & node, const std::string & path) const
  {
    const auto value = scalar_as_string(node, path);
    if (value == "keep_last") {
      return HistoryPolicy::kKeepLast;
    }
    if (value == "keep_all") {
      return HistoryPolicy::kKeepAll;
    }
    fail(path, "unknown QoS enum", "keep_last|keep_all", value, "使用受支持的 history");
  }

  LivelinessPolicy parse_liveliness(
    const YAML::Node & node, const std::string & path) const
  {
    const auto value = scalar_as_string(node, path);
    if (value == "automatic") {
      return LivelinessPolicy::kAutomatic;
    }
    if (value == "manual_by_topic") {
      return LivelinessPolicy::kManualByTopic;
    }
    fail(
      path, "unknown QoS enum", "automatic|manual_by_topic", value,
      "使用受支持的 liveliness");
  }

  QosProfile parse_qos_profile(const YAML::Node & node, const std::string & path) const
  {
    check_keys(
      node,
      {"reliability", "durability", "history", "depth", "deadline_ms", "lifespan_ms",
        "liveliness", "liveliness_lease_duration_ms"},
      path);

    const YAML::Node liveliness_node = node["liveliness"];
    const auto lease = optional_uint(
      node, "liveliness_lease_duration_ms", path + ".liveliness_lease_duration_ms");
    QosProfile result{
      parse_reliability(
        require_field(node, "reliability", path + ".reliability"), path + ".reliability"),
      parse_durability(
        require_field(node, "durability", path + ".durability"), path + ".durability"),
      parse_history(require_field(node, "history", path + ".history"), path + ".history"),
      require_uint(node, "depth", path + ".depth"),
      optional_uint(node, "deadline_ms", path + ".deadline_ms"),
      optional_uint(node, "lifespan_ms", path + ".lifespan_ms"),
      liveliness_node ? std::optional<LivelinessPolicy>(
        parse_liveliness(liveliness_node, path + ".liveliness")) : std::nullopt,
      lease,
    };

    if (result.history == HistoryPolicy::kKeepLast && result.depth == 0U) {
      fail(path + ".depth", "invalid keep_last depth", ">= 1", "0", "设置至少一个样本槽位");
    }
    if (result.deadline_ms.has_value() && *result.deadline_ms == 0U) {
      fail(path + ".deadline_ms", "zero duration", "> 0", "0", "配置正数 deadline");
    }
    if (result.lifespan_ms.has_value() && *result.lifespan_ms == 0U) {
      fail(path + ".lifespan_ms", "zero duration", "> 0", "0", "配置正数 lifespan");
    }
    if (result.liveliness.has_value() != result.liveliness_lease_duration_ms.has_value()) {
      fail(
        path, "incomplete liveliness policy", "liveliness and lease both present or both absent",
        "partial policy", "同时配置或同时省略 liveliness 字段");
    }
    if (result.liveliness_lease_duration_ms.has_value() &&
      *result.liveliness_lease_duration_ms == 0U)
    {
      fail(
        path + ".liveliness_lease_duration_ms", "zero duration", "> 0", "0",
        "配置正数 liveliness lease");
    }
    return result;
  }

  std::map<std::string, QosProfile> parse_qos_profiles(const YAML::Node & node) const
  {
    require_map(node, "qos_profiles");
    if (node.size() == 0U) {
      fail("qos_profiles", "empty profile map", "at least one profile", "empty", "定义 QoS profile");
    }

    std::map<std::string, QosProfile> profiles;
    for (const auto & entry : node) {
      const auto name = scalar_as_string(entry.first, "qos_profiles");
      if (name.empty()) {
        fail("qos_profiles", "empty profile name", "non-empty string", "empty", "填写 profile 名称");
      }
      const auto [iterator, inserted] = profiles.emplace(
        name, parse_qos_profile(entry.second, "qos_profiles." + name));
      static_cast<void>(iterator);
      if (!inserted) {
        fail(
          "qos_profiles." + name, "duplicate profile", "unique name", name,
          "删除重复的 QoS profile");
      }
    }
    return profiles;
  }

  InputContract parse_input(const YAML::Node & node) const
  {
    check_keys(node, {"topic_prefix", "type", "qos_profile"}, "input");
    InputContract result{
      require_string(node, "topic_prefix", "input.topic_prefix"),
      require_string(node, "type", "input.type"),
      require_string(node, "qos_profile", "input.qos_profile"),
    };
    validate_topic_and_type(result.topic_prefix, result.type, "input.topic_prefix", "input.type");
    return result;
  }

  OutputContract parse_output(const YAML::Node & node) const
  {
    check_keys(node, {"topic", "type", "qos_profile"}, "output");
    OutputContract result{
      require_string(node, "topic", "output.topic"),
      require_string(node, "type", "output.type"),
      require_string(node, "qos_profile", "output.qos_profile"),
    };
    validate_topic_and_type(result.topic, result.type, "output.topic", "output.type");
    return result;
  }

  StateTopicContract parse_state_topic(
    const YAML::Node & node, const std::string & path) const
  {
    check_keys(node, {"topic", "type", "qos_profile"}, path);
    const YAML::Node qos_node = node["qos_profile"];
    StateTopicContract result{
      require_string(node, "topic", path + ".topic"),
      require_string(node, "type", path + ".type"),
      qos_node ? std::optional<std::string>(scalar_as_string(qos_node, path + ".qos_profile")) :
      std::nullopt,
    };
    validate_topic_and_type(result.topic, result.type, path + ".topic", path + ".type");
    return result;
  }

  std::map<std::string, StateTopicContract> parse_state_topics(const YAML::Node & node) const
  {
    check_keys(
      node, {"gateway_state", "source_status", "vehicle_state", "diagnostics"}, "state_topics");
    std::map<std::string, StateTopicContract> topics;
    for (const auto * name : {"gateway_state", "source_status", "vehicle_state", "diagnostics"}) {
      const auto path = std::string("state_topics.") + name;
      topics.emplace(name, parse_state_topic(require_field(node, name, path), path));
    }
    return topics;
  }

  RemoteDirection parse_remote_direction(
    const YAML::Node & node, const std::string & path) const
  {
    const auto value = scalar_as_string(node, path);
    if (value == "publisher") {
      return RemoteDirection::kPublisher;
    }
    if (value == "subscription") {
      return RemoteDirection::kSubscription;
    }
    fail(path, "unknown direction", "publisher|subscription", value, "使用远端 endpoint 方向");
  }

  RuntimeLossAction parse_runtime_loss_action(
    const YAML::Node & node, const std::string & path) const
  {
    const auto value = scalar_as_string(node, path);
    if (value == "safe_stop") {
      return RuntimeLossAction::kSafeStop;
    }
    fail(path, "unknown runtime action", "safe_stop", value, "v1 只支持 safe_stop");
  }

  RuntimeQosMismatchAction parse_runtime_qos_action(
    const YAML::Node & node, const std::string & path) const
  {
    const auto value = scalar_as_string(node, path);
    if (value == "degraded") {
      return RuntimeQosMismatchAction::kDegraded;
    }
    fail(path, "unknown runtime action", "degraded", value, "v1 只支持 degraded");
  }

  CriticalEndpoint parse_critical_endpoint(
    const YAML::Node & node, const std::string & path) const
  {
    check_keys(
      node,
      {"id", "topic", "type", "remote_direction", "remote_node_fqn", "min_count", "max_count",
        "allow_additional_endpoints", "required_for_activation", "exact_qos_required",
        "runtime_loss_action", "runtime_qos_mismatch_action"},
      path);

    const auto max_count = optional_uint(node, "max_count", path + ".max_count");
    CriticalEndpoint result{
      require_string(node, "id", path + ".id"),
      require_string(node, "topic", path + ".topic"),
      require_string(node, "type", path + ".type"),
      parse_remote_direction(
        require_field(node, "remote_direction", path + ".remote_direction"),
        path + ".remote_direction"),
      require_string(node, "remote_node_fqn", path + ".remote_node_fqn"),
      require_uint(node, "min_count", path + ".min_count"),
      max_count,
      require_bool(node, "allow_additional_endpoints", path + ".allow_additional_endpoints"),
      require_bool(node, "required_for_activation", path + ".required_for_activation"),
      require_bool(node, "exact_qos_required", path + ".exact_qos_required"),
      parse_runtime_loss_action(
        require_field(node, "runtime_loss_action", path + ".runtime_loss_action"),
        path + ".runtime_loss_action"),
      parse_runtime_qos_action(
        require_field(
          node, "runtime_qos_mismatch_action", path + ".runtime_qos_mismatch_action"),
        path + ".runtime_qos_mismatch_action"),
    };

    if (result.id.empty()) {
      fail(path + ".id", "empty endpoint id", "non-empty string", "empty", "填写 endpoint id");
    }
    validate_topic_and_type(result.topic, result.type, path + ".topic", path + ".type");
    if (!is_valid_node_fqn(result.remote_node_fqn)) {
      fail(
        path + ".remote_node_fqn", "invalid ROS node FQN", "absolute node FQN",
        result.remote_node_fqn, "例如 /control_link/vehicle_adapter");
    }
    if (result.min_count == 0U) {
      fail(path + ".min_count", "invalid endpoint count", ">= 1", "0", "配置至少一个 endpoint");
    }
    if (result.max_count.has_value() && *result.max_count < result.min_count) {
      fail(
        path + ".max_count", "max_count smaller than min_count",
        ">= " + std::to_string(result.min_count), std::to_string(*result.max_count),
        "修正 endpoint count 区间");
    }
    return result;
  }

  std::vector<CriticalEndpoint> parse_critical_endpoints(const YAML::Node & node) const
  {
    require_sequence(node, "critical_endpoints");
    if (node.size() == 0U) {
      fail(
        "critical_endpoints", "empty endpoint list", "at least one endpoint", "empty",
        "定义关键执行边界");
    }

    std::set<std::string> ids;
    std::vector<CriticalEndpoint> endpoints;
    endpoints.reserve(node.size());
    for (std::size_t index = 0; index < node.size(); ++index) {
      const auto path = "critical_endpoints[" + std::to_string(index) + "]";
      auto endpoint = parse_critical_endpoint(node[index], path);
      if (!ids.emplace(endpoint.id).second) {
        fail(path + ".id", "duplicate endpoint id", "unique id", endpoint.id, "删除重复 endpoint");
      }
      endpoints.push_back(std::move(endpoint));
    }
    return endpoints;
  }

  void validate_topic_and_type(
    const std::string & topic, const std::string & type, const std::string & topic_path,
    const std::string & type_path) const
  {
    if (!is_valid_topic_name(topic)) {
      fail(topic_path, "invalid ROS topic", "absolute topic name", topic, "使用不含通配符的绝对 Topic");
    }
    if (!is_valid_interface_type(type)) {
      fail(
        type_path, "invalid ROS interface type", "package/msg/Type or package/srv/Type", type,
        "填写完整接口类型");
    }
  }

  void validate_qos_reference(
    const std::map<std::string, QosProfile> & profiles, const std::string & reference,
    const std::string & path) const
  {
    if (profiles.count(reference) == 0U) {
      fail(path, "unknown QoS reference", "defined qos_profiles key", reference, "修正 profile 引用");
    }
  }

  void validate_cross_fields(const GatewayContract & contract) const
  {
    if (contract.limits.max_future_skew_ms > contract.gateway.command_timeout_ms) {
      fail(
        "limits.max_future_skew_ms", "future skew exceeds command timeout",
        "<= " + std::to_string(contract.gateway.command_timeout_ms),
        std::to_string(contract.limits.max_future_skew_ms), "缩小 future skew");
    }

    validate_qos_reference(
      contract.qos_profiles, contract.input.qos_profile, "input.qos_profile");
    validate_qos_reference(
      contract.qos_profiles, contract.output.qos_profile, "output.qos_profile");
    for (const auto & [name, topic] : contract.state_topics) {
      if (topic.qos_profile.has_value()) {
        validate_qos_reference(
          contract.qos_profiles, *topic.qos_profile, "state_topics." + name + ".qos_profile");
      }
    }

    for (std::size_t index = 0; index < contract.critical_endpoints.size(); ++index) {
      const auto & endpoint = contract.critical_endpoints[index];
      bool matches_public_interface =
        endpoint.topic == contract.output.topic && endpoint.type == contract.output.type;
      for (const auto & [name, topic] : contract.state_topics) {
        static_cast<void>(name);
        matches_public_interface = matches_public_interface ||
          (endpoint.topic == topic.topic && endpoint.type == topic.type);
      }
      if (!matches_public_interface) {
        const auto path = "critical_endpoints[" + std::to_string(index) + "]";
        fail(
          path + ".topic", "endpoint is outside public Contract", "configured output/state topic+type",
          endpoint.topic + " " + endpoint.type, "关键 endpoint 必须引用公共运行接口");
      }
    }
  }

  std::string source_name_;
};

}  // namespace

GatewayContractPtr parse_gateway_contract_text(
  const std::string_view yaml_text, std::string source_name)
{
  return GatewayContractParser(std::move(source_name)).parse(yaml_text);
}

GatewayContractPtr load_gateway_contract(const std::filesystem::path & path)
{
  std::ifstream input(path);
  if (!input) {
    throw ContractError(
            path.string() +
            ":root: file open failed; expected=readable YAML file; actual=unreadable; "
            "hint=检查路径和权限");
  }
  std::ostringstream buffer;
  buffer << input.rdbuf();
  if (!input.good() && !input.eof()) {
    throw ContractError(
            path.string() +
            ":root: file read failed; expected=complete YAML file; actual=I/O error; "
            "hint=检查文件系统");
  }
  return parse_gateway_contract_text(buffer.str(), path.string());
}

}  // namespace control_link_contract
