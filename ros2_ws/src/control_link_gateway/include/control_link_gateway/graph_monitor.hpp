#pragma once

#include <chrono>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <string_view>
#include <vector>

#include "control_link_contract/compatibility.hpp"
#include "control_link_gateway/model.hpp"
#include "rclcpp/node_interfaces/node_graph_interface.hpp"

namespace control_link_gateway
{
	// 单次 ROS Graph 快照的来源准入结论，尚未经过 graph_stable_window_ms 去抖
	enum class SourceEndpointState : std::uint8_t
	{
		kMissing,
		kAmbiguous,
		kUnexpectedDirection,
		kTypeMismatch,
		kQosMismatch,
		kUsable,
	};

	enum class SourceEndpointStabilityEvent : std::uint8_t
	{
		kPending,
		kUnchanged,
		kInitialized,
		kChanged,
	};

	struct SourceEndpointAssessment
	{
		SourceEndpointState state;
		std::size_t publisher_count;
		// 只有唯一 endpoint 的方向、类型和 QoS 全部满足 Contract 时才提供 generation
		std::optional<PublisherGenerationKey> publisher_generation;
		// 只有执行到 QoS 检查时才有报告，缺失、歧义、方向或类型错误时为空
		std::optional<control_link_contract::QosCompatibilityReport> qos_report;

		[[nodiscard]] bool usable() const noexcept;
	};

	struct SourceEndpointStabilityTracker
	{
		// stable_assessment 对外提供已去抖结论，candidate 只保存尚未达到稳定窗口的观察
		std::optional<SourceEndpointAssessment> stable_assessment;
		std::optional<SourceEndpointAssessment> candidate_assessment;
		std::optional<std::chrono::steady_clock::time_point> candidate_since;
		// 只用于拒绝调用方传入的倒退时间，不参与稳定窗口累计
		std::optional<std::chrono::steady_clock::time_point> last_observed_at;
	};

	[[nodiscard]] PublisherGenerationKey
	publisher_generation_from_endpoint_info(
		const rclcpp::TopicEndpointInfo &endpoint,
		std::string_view rmw_implementation);

	// 本函数只分析调用方提供的瞬时 Graph 快照，不查询 Graph，也不确认 Slot generation
	[[nodiscard]] SourceEndpointAssessment assess_source_publishers(
		const std::vector<rclcpp::TopicEndpointInfo> &publishers,
		std::string_view expected_type,
		std::string_view rmw_implementation,
		const rclcpp::QoS &local_qos,
		const control_link_contract::QosProfile &expected_remote_profile);

	[[nodiscard]] SourceEndpointStabilityEvent
	update_source_endpoint_stability(
		SourceEndpointStabilityTracker &tracker,
		const SourceEndpointAssessment &observation,
		std::chrono::steady_clock::time_point observed_at,
		std::chrono::milliseconds stable_window);
} // namespace control_link_gateway
