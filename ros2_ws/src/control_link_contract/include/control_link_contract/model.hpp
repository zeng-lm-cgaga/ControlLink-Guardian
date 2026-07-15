#pragma once

#include <cstdint>
#include <map>
#include <memory>
#include <optional>
#include <string>
#include <vector>

namespace control_link_contract
{

// 这些枚举保存经过 schema 校验的 QoS 语义，后续由 QosFactory 映射为 ROS2 类型。
enum class ReliabilityPolicy
{
  kReliable,
  kBestEffort,
};

enum class DurabilityPolicy
{
  kVolatile,
  kTransientLocal,
};

enum class HistoryPolicy
{
  kKeepLast,
  kKeepAll,
};

enum class LivelinessPolicy
{
  kAutomatic,
  kManualByTopic,
};

// 方向始终从网关视角描述远端 endpoint，避免 publisher/subscription 语义颠倒。
enum class RemoteDirection
{
  kPublisher,
  kSubscription,
};

enum class RuntimeLossAction
{
  kSafeStop,
};

enum class RuntimeQosMismatchAction
{
  kDegraded,
};

// 所有时间字段单位均为 ms；数值唯一 owner 是 gateway_contract.yaml。
struct GatewaySettings
{
  std::string node_fqn;
  double output_rate_hz;
  std::uint64_t command_timeout_ms;
  std::uint64_t source_switch_hold_ms;
  std::uint64_t recovery_valid_samples;
  std::uint64_t graph_poll_ms;
  std::uint64_t graph_stable_window_ms;
  std::uint64_t ros_clock_stall_timeout_ms;
  std::uint64_t vehicle_state_topic_timeout_ms;
  std::uint64_t output_tick_late_threshold_ms;
  std::uint64_t consecutive_late_ticks_to_safe_stop;
};

struct CommandLimits
{
  double max_abs_linear_velocity_mps;
  double max_abs_angular_velocity_radps;
  bool reject_non_finite;
  bool reject_zero_stamp;
  std::uint64_t max_future_skew_ms;
};

// optional 表示该 QoS policy 未配置，不允许调用方自行补一个隐藏默认值。
struct QosProfile
{
  ReliabilityPolicy reliability;
  DurabilityPolicy durability;
  HistoryPolicy history;
  std::uint64_t depth;
  std::optional<std::uint64_t> deadline_ms;
  std::optional<std::uint64_t> lifespan_ms;
  std::optional<LivelinessPolicy> liveliness;
  std::optional<std::uint64_t> liveliness_lease_duration_ms;
};

struct InputContract
{
  std::string topic_prefix;
  std::string type;
  std::string qos_profile;
};

struct OutputContract
{
  std::string topic;
  std::string type;
  std::string qos_profile;
};

struct StateTopicContract
{
  std::string topic;
  std::string type;
  std::optional<std::string> qos_profile;
};

// min/max 只统计匹配 topic、type、direction 和 remote_node_fqn 的远端 role。
// allow_additional_endpoints 单独决定是否允许其他节点观察同一 Topic。
struct CriticalEndpoint
{
  std::string id;
  std::string topic;
  std::string type;
  RemoteDirection remote_direction;
  std::string remote_node_fqn;
  std::uint64_t min_count;
  std::optional<std::uint64_t> max_count;
  bool allow_additional_endpoints;
  bool required_for_activation;
  bool exact_qos_required;
  RuntimeLossAction runtime_loss_action;
  RuntimeQosMismatchAction runtime_qos_mismatch_action;
};

// parser 完成全部字段和跨字段校验后，才以 shared_ptr<const GatewayContract> 发布。
// 运行时组件只读共享同一份快照，不能绕过配置 owner 修改行为参数。
struct GatewayContract
{
  std::uint64_t schema_version;
  std::string contract_id;
  GatewaySettings gateway;
  CommandLimits limits;
  std::map<std::string, QosProfile> qos_profiles;
  InputContract input;
  OutputContract output;
  std::map<std::string, StateTopicContract> state_topics;
  std::vector<CriticalEndpoint> critical_endpoints;
};

using GatewayContractPtr = std::shared_ptr<const GatewayContract>;

}  // namespace control_link_contract
