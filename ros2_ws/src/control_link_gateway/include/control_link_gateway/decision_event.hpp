#pragma once

#include <cstdint>
#include <optional>
#include <string>
#include <variant>
#include <vector>

#include "control_link_gateway/model.hpp"
#include "control_link_gateway/state_machine.hpp"
#include "control_link_interfaces/msg/control_command.hpp"

namespace control_link_gateway
{
	constexpr std::uint32_t kDecisionTraceSchemaVersion = 1U;

	// trace 只保存相对 steady offset，禁止把进程内 time_point 当成跨进程时间戳
	struct DecisionTraceHeader
	{
		std::uint32_t trace_schema_version{kDecisionTraceSchemaVersion};
		std::string git_commit;
		bool git_dirty{false};
		std::string build_type;
		std::string profile_id;
		std::string contract_id;
		std::uint64_t contract_version{0U};
		std::string contract_hash;
		std::string decision_config_hash;
		std::string rmw_implementation;
		std::string ros_distro;
		std::string steady_time_origin{"relative_ns_zero"};
	};

	enum class DecisionLifecycleTransition : std::uint8_t
	{
		kConfigure,
		kActivate,
		kDeactivate,
		kCleanup,
		kError,
	};

	enum class DecisionLifecycleResult : std::uint8_t
	{
		kSuccess,
		kFailure,
		kError,
	};

	// 这是 Graph stable window 的规范化输出，不让 replayer 重新实现 ROS Graph 规则
	enum class DecisionSourceEndpointState : std::uint8_t
	{
		kMissing,
		kAmbiguous,
		kUnexpectedDirection,
		kTypeMismatch,
		kQosMismatch,
		kUsable,
	};

	struct DecisionSourceEndpoint
	{
		std::string source_id;
		DecisionSourceEndpointState state;
		std::optional<PublisherGenerationKey> publisher_generation;
	};

	struct DecisionLifecycleEvent
	{
		DecisionLifecycleTransition transition;
		DecisionLifecycleResult result;
	};

	struct DecisionSourceSampleEvent
	{
		std::string expected_source_id;
		PublisherGenerationKey publisher_generation;
		control_link_interfaces::msg::ControlCommand command;
		std::int64_t now_ros_ns{0};
		std::int64_t steady_receive_offset_ns{0};
	};

	struct DecisionHealthSnapshotEvent
	{
		std::uint64_t health_revision{0U};
		std::int64_t steady_observed_offset_ns{0};
		GatewayHealthSnapshot health;
		// 必须按 source_id 升序且覆盖当前 Profile 的全部 enabled source
		std::vector<DecisionSourceEndpoint> source_endpoints;
	};

	struct DecisionOutputTickEvent
	{
		std::int64_t steady_offset_ns{0};
		std::int64_t now_ros_ns{0};
		std::int64_t tick_interval_ns{0};
		std::int64_t tick_lateness_ns{0};
		std::uint64_t health_revision{0U};
	};

	using DecisionEventPayload = std::variant<
		DecisionLifecycleEvent,
		DecisionSourceSampleEvent,
		DecisionHealthSnapshotEvent,
		DecisionOutputTickEvent>;

	struct DecisionEvent
	{
		std::uint64_t event_sequence{0U};
		DecisionEventPayload payload;
	};

	struct DecisionSourceStatus
	{
		std::string source_id;
		std::uint64_t accepted_count{0U};
		std::uint64_t rejected_count{0U};
		RejectReason last_reject_reason{RejectReason::kNone};
		std::optional<std::uint64_t> last_accepted_sequence;
		bool command_valid{false};
		bool lease_valid{false};
		std::int64_t command_age_ns{0};
	};

	// output tick 的完整可比较结果，diagnostics 文本和线程信息不参与 verdict
	struct DecisionResult
	{
		std::uint64_t event_sequence{0U};
		DataState state{DataState::kStandby};
		StateReason reason{StateReason::kNone};
		std::optional<RecoveryCandidateKey> recovery_candidate;
		std::uint16_t recovery_valid_count{0U};
		std::uint64_t transition_sequence{0U};
		std::vector<DecisionSourceStatus> sources;
		control_link_interfaces::msg::ControlCommand canonical_command;
		bool lifecycle_error_requested{false};
	};

	// footer 是完整性边界，缺失 footer 的 partial trace 不能进入 replay verdict
	struct DecisionTraceFooter
	{
		std::uint64_t last_event_sequence{0U};
		std::uint64_t event_count{0U};
		std::uint64_t result_count{0U};
		bool trace_valid{true};
		bool trace_overflow{false};
		std::string error_message;
	};
}  // namespace control_link_gateway
