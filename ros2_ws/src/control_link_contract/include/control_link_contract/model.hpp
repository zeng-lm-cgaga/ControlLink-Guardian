#pragma once

#include <cstdint>
#include <filesystem>
#include <map>
#include <memory>
#include <optional>
#include <string>
#include <vector>
#include <variant>

namespace control_link_contract
{

	// 这些枚举保存经过 schema 校验的 QoS 语义，后续由 QosFactory 映射为 ROS2 类型
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

	// 方向始终从网关视角描述远端 endpoint，避免 publisher/subscription 语义颠倒
	enum class RemoteDirection
	{
		kPublisher,
		kSubscription,
	};

	// v1 action 是封闭集合，parser 必须拒绝未实现的运行期处理方式
	enum class RuntimeLossAction
	{
		kSafeStop,
	};

	enum class RuntimeQosMismatchAction
	{
		kDegraded,
	};

	enum class ClockMode
	{
		kSim,
		kSystem,
	};

	// 所有时间字段单位均为 ms；数值唯一 owner 是 gateway_contract.yaml
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

	// optional 表示该 QoS policy 未配置，不允许调用方自行补一个隐藏默认值
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

	// min/max 只统计匹配 topic、type、direction 和 remote_node_fqn 的远端 role
	// allow_additional_endpoints 单独决定是否允许其他节点观察同一 Topic
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

	// parser 完成全部字段和跨字段校验后，才以 shared_ptr<const GatewayContract> 发布
	// 运行时组件只读共享同一份快照，不能绕过配置 owner 修改行为参数
	struct GatewayContract
	{
		std::uint64_t schema_version;
		std::string contract_id;
		std::uint64_t contract_version;
		GatewaySettings gateway;
		CommandLimits limits;
		std::map<std::string, QosProfile> qos_profiles;
		InputContract input;
		OutputContract output;
		std::map<std::string, StateTopicContract> state_topics;
		std::vector<CriticalEndpoint> critical_endpoints;
	};

	using GatewayContractPtr = std::shared_ptr<const GatewayContract>;

	// source policy 是来源身份、优先级和 steady-clock lease 的唯一 owner
	struct SourcePolicyEntry
	{
		std::string topic;
		std::string type;
		std::uint8_t priority;
		std::uint64_t lease_timeout_ms;
		bool required_for_activation;
	};

	struct SourcePolicy
	{
		std::uint64_t schema_version;
		std::string policy_id;
		std::map<std::string, SourcePolicyEntry> sources;
	};

	using SourcePolicyPtr = std::shared_ptr<const SourcePolicy>;

	enum class CanByteOrder
	{
		kLittleEndian,
	};

	enum class CanCrcAlgorithm
	{
		kCrc8SaeJ1850,
	};

	struct CanCrcConfig
	{
		CanCrcAlgorithm algorithm;
		std::uint8_t polynomial;
		std::uint8_t initial_value;
		std::uint8_t final_xor;
		bool reflect_input;
		bool reflect_output;
		bool include_can_id_lsb_first;
		std::vector<std::uint8_t> protected_payload_bytes;
	};

	struct CanRollingCounterConfig
	{
		std::uint8_t bit_length;
		std::uint8_t modulo;
		bool accept_any_first_value;
		bool duplicate_is_error;
		bool jump_is_error;
	};

	// 物理量 layout 由 v1 parser 固定校验，model 保存 codec 实际需要的量化规则
	struct CanPhysicalSignalConfig
	{
		double scale;
		double offset;
		double minimum;
		double maximum;
	};

	struct CanControlFrameConfig
	{
		std::uint32_t can_id;
		CanPhysicalSignalConfig target_speed_mps;
		CanPhysicalSignalConfig target_yaw_rate_radps;
	};

	struct CanStateFrameConfig
	{
		std::uint32_t can_id;
		CanPhysicalSignalConfig measured_speed_mps;
		CanPhysicalSignalConfig measured_yaw_rate_radps;
	};

	// can_signal_map.yaml 的规范化只读模型，adapter 与 simulator 必须共享这一配置 owner
	struct CanSignalMap
	{
		std::uint64_t schema_version;
		std::string protocol_id;
		std::string description;
		CanByteOrder byte_order;
		std::uint8_t dlc;
		CanCrcConfig crc;
		CanRollingCounterConfig rolling_counter;
		CanControlFrameConfig control_frame;
		CanStateFrameConfig state_frame;
	};

	using CanSignalMapPtr = std::shared_ptr<const CanSignalMap>;

	// 外部平台 Topic 经 ingress adapter 转换后，进入 source 专属的内部 ControlCommand Topic
	struct IngressBinding
	{
		std::string input_topic;
		std::string input_type;
		std::string output_topic;
	};

	// common 和 adapter 配置引用由 Profile parser 解析为 config_root 内的规范化绝对路径
	struct ProfileCommon
	{
		std::uint64_t schema_version;
		std::filesystem::path contract_path;
		std::filesystem::path source_policy_path;
		std::filesystem::path fastdds_profile_path;
		std::vector<std::string> enabled_sources;
		ClockMode clock_mode;
		bool use_sim_time;
		std::map<std::string, IngressBinding> ingress;
		std::vector<std::string> record_topics;
	};

	struct RobotGeometry
	{
		double wheel_separation_m;
		double wheel_radius_m;
	};

	// adapter 的 config_path 已限制在 config_root 内，所有 timeout 字段单位为 ms
	struct RobotAdapterConfig
	{
		std::filesystem::path config_path;
		std::string canonical_input_topic;
		std::string controller_manager_fqn;
		std::string controller_node_fqn;
		std::string hardware_component_name;
		std::string controller_output_topic;
		std::string controller_command_type;
		std::string odometry_topic;
		std::uint64_t local_watchdog_timeout_ms;
	};

	// resources 路径相对 ROS package share，Profile parser 保留相对路径并拒绝绝对路径和 ..
	struct RobotResources
	{
		std::string package;
		std::filesystem::path robot_description;
		std::filesystem::path world;
		std::filesystem::path map_yaml;
		std::filesystem::path nav2_params;
	};

	struct RobotFrames
	{
		std::string map;
		std::string odom;
		std::string base_footprint;
		std::string base_link;
		std::string laser;
	};

	struct RobotHealth
	{
		std::uint64_t vehicle_state_publish_period_ms;
		std::uint64_t tf_lookup_timeout_ms;
		std::uint64_t tf_max_age_ms;
		std::uint64_t controller_state_timeout_ms;
		std::uint64_t odometry_timeout_ms;
	};

	struct FixedDemoGoal
	{
		std::string frame_id;
		double x_m;
		double y_m;
		double yaw_rad;
	};

	struct RobotProfile
	{
		ProfileCommon common;
		RobotGeometry geometry;
		RobotAdapterConfig adapter;
		RobotResources resources;
		RobotFrames frames;
		RobotHealth health;
		FixedDemoGoal fixed_demo_goal;
	};

	// SocketCAN I/O、状态发布和 watchdog 周期单位均为 ms，跨字段时序约束由 Profile parser 校验
	struct AdasAdapterConfig
	{
		std::filesystem::path config_path;
		std::string canonical_input_topic;
		std::string vehicle_state_output_topic;
		std::string interface;
		std::uint64_t poll_timeout_ms;
		std::uint64_t tx_period_ms;
		std::uint64_t vehicle_state_publish_period_ms;
		std::uint64_t local_watchdog_timeout_ms;
		std::uint64_t can_state_frame_timeout_ms;
		std::uint64_t recovery_valid_frames;
	};

	struct VehicleSimulatorConfig
	{
		std::uint64_t state_period_ms;
		std::uint64_t local_watchdog_timeout_ms;
		std::uint64_t first_order_time_constant_ms;
	};

	struct ReplayConfig
	{
		std::string input_namespace;
	};

	struct AdasProfile
	{
		ProfileCommon common;
		AdasAdapterConfig adapter;
		VehicleSimulatorConfig vehicle_simulator;
		ReplayConfig replay;
	};

	// profile_id 决定 variant 当前分支，调用方使用 std::visit 或 std::get_if 显式处理平台差异
	using ProfileConfig = std::variant<RobotProfile, AdasProfile>;
	using ProfileConfigPtr = std::shared_ptr<const ProfileConfig>;
}  // namespace control_link_contract
