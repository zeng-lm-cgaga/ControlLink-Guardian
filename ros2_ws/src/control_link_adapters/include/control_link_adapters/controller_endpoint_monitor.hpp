#pragma once

#include <array>
#include <chrono>
#include <cstdint>
#include <optional>
#include <string>
#include <vector>

#include "rclcpp/node_interfaces/node_graph_interface.hpp"
#include "rclcpp/qos.hpp"
#include "rmw/types.h"

namespace control_link_adapters
{
	enum class ControllerEndpointState : std::uint8_t
	{
		// 目标 controller 从未通过稳定窗口确认
		kUnavailable,
		// 新 endpoint generation 尚未持续满足 graph_stable_window_ms
		kStabilizing,
		// 唯一目标 subscription 的 FQN、类型和 generation 已确认
		kHealthy,
		// 曾确认的 controller 消失时间已经超过 controller_state_timeout_ms
		kTimedOut,
		// 同一目标 controller role 存在多个匹配 subscription，generation 无法唯一确认
		kAmbiguous,
	};

	struct ControllerEndpointSnapshot
	{
		ControllerEndpointState state;
		std::optional<std::array<std::uint8_t, RMW_GID_STORAGE_SIZE>> endpoint_gid;

		[[nodiscard]] bool healthy() const noexcept
		{
			return state == ControllerEndpointState::kHealthy;
		}
	};

	// 对 ros2_control command subscription 做身份、唯一性、generation 稳定窗口和失联超时判断
	// 本类只消费调用方提供的 Graph 快照，不在内部发起 ROS Graph 查询
	class ControllerEndpointMonitor final
	{
	public:
		ControllerEndpointMonitor(
			std::string expected_node_fqn,
			std::string expected_type,
			rclcpp::QoS publisher_qos,
			std::chrono::milliseconds stable_window,
			std::chrono::milliseconds state_timeout);

		[[nodiscard]] ControllerEndpointSnapshot observe(
			const std::vector<rclcpp::TopicEndpointInfo> &subscriptions,
			std::chrono::steady_clock::time_point observed_at);

		[[nodiscard]] ControllerEndpointSnapshot current() const noexcept;

	private:
		[[nodiscard]] std::vector<rclcpp::TopicEndpointInfo> matching_endpoints(
			const std::vector<rclcpp::TopicEndpointInfo> &subscriptions) const;

		std::string expected_node_fqn_;
		std::string expected_type_;
		rclcpp::QoS publisher_qos_;
		std::chrono::milliseconds stable_window_;
		std::chrono::milliseconds state_timeout_;
		ControllerEndpointSnapshot current_{
			ControllerEndpointState::kUnavailable,
			std::nullopt};
		std::optional<std::array<std::uint8_t, RMW_GID_STORAGE_SIZE>> stable_gid_;
		std::optional<std::array<std::uint8_t, RMW_GID_STORAGE_SIZE>> candidate_gid_;
		std::optional<std::chrono::steady_clock::time_point> candidate_since_;
		std::optional<std::chrono::steady_clock::time_point> last_confirmed_at_;
		std::optional<std::chrono::steady_clock::time_point> last_observed_at_;
		bool ever_confirmed_{false};
	};
} // namespace control_link_adapters
