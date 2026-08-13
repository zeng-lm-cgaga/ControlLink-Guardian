#pragma once

#include <chrono>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <string>
#include <vector>

#include <rclcpp/qos.hpp>
#include "rclcpp/node_interfaces/node_graph_interface.hpp"

#include "control_link_contract/compatibility.hpp"
#include "control_link_contract/model.hpp"

namespace control_link_gateway
{
	// 一次 Graph 查询的身份/数量结论，不代表这个结论已经通过 stable window
	struct CriticalEndpointIdentityAssessment
	{
		std::size_t discovered_count{0};
		std::size_t matching_role_count{0};
		std::size_t additional_endpoint_count{0};
		std::vector<rclcpp::TopicEndpointInfo> matching_role_endpoints;
		bool role_count_healthy{false};
		bool additional_endpoints_healthy{false};

		[[nodiscard]] bool healthy() const noexcept
		{
			return role_count_healthy && additional_endpoints_healthy;
		}
	};

	// ROS Graph 分开返回 node name 与 namespace，FQN 只在这里规范化，避免调用方各自拼接
	[[nodiscard]] std::string endpoint_node_fqn(
		const rclcpp::TopicEndpointInfo &endpoint);

	// 仅匹配 Contract 指定的 direction、type 和远端 FQN，额外 endpoint 不会冒充 role
	[[nodiscard]] CriticalEndpointIdentityAssessment
	assess_critical_endpoint_identity(
		const std::vector<rclcpp::TopicEndpointInfo> &endpoints,
		const control_link_contract::CriticalEndpoint &expected);

	struct CriticalEndpointQosAssessment
	{
		// 报告顺序与 identity.matching_role_endpoints 保持一致，便于 diagnostics 关联 endpoint
		std::vector<control_link_contract::QosCompatibilityReport> endpoint_reports;
		bool dds_compatible{false};
		bool observed_policy_match{false};
		bool observation_complete{false};

		// exact_qos_required 在 Graph gate 中表示所有可观测 Contract policy 不得 mismatch
		// observation_complete 单独说明 RMW 是否暴露了完整字段，不伪装成运行 mismatch
		[[nodiscard]] bool healthy(bool exact_qos_required) const noexcept
		{
			return dds_compatible &&
				(!exact_qos_required || observed_policy_match);
		}
	};

	// QoS 只检查身份匹配的 endpoint，避免观察订阅的 QoS 差异误伤执行 role
	[[nodiscard]] CriticalEndpointQosAssessment assess_critical_endpoint_qos(
		const CriticalEndpointIdentityAssessment &identity,
		const control_link_contract::CriticalEndpoint &expected,
		const rclcpp::QoS &local_qos,
		const control_link_contract::QosProfile &expected_remote_profile);

	struct CriticalEndpointAssessment
	{
		CriticalEndpointIdentityAssessment identity;
		CriticalEndpointQosAssessment qos;
		bool exact_qos_required{false};

		[[nodiscard]] bool healthy() const noexcept
		{
			return identity.healthy() && qos.healthy(exact_qos_required);
		}
	};

	// stable window 只提交控制语义，诊断细节可以在 kUnchanged 时刷新
	enum class CriticalEndpointStabilityEvent : std::uint8_t
	{
		kPending,
		kUnchanged,
		kInitialized,
		kChanged,
	};

	struct CriticalEndpointStabilityTracker
	{
		std::optional<CriticalEndpointAssessment> stable_assessment;
		std::optional<CriticalEndpointAssessment> candidate_assessment;
		std::optional<std::chrono::steady_clock::time_point> candidate_since;
		std::optional<std::chrono::steady_clock::time_point> last_observed_at;
	};

	[[nodiscard]] CriticalEndpointStabilityEvent
	update_critical_endpoint_stability(
		CriticalEndpointStabilityTracker &tracker,
		const CriticalEndpointAssessment &observation,
		std::chrono::steady_clock::time_point observed_at,
		std::chrono::milliseconds stable_window);
} // namespace control_link_gateway
