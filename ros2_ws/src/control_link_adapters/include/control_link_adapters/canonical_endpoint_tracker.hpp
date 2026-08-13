#pragma once

#include <chrono>
#include <cstdint>
#include <optional>
#include <string_view>
#include <vector>

#include "control_link_adapters/canonical_input_guard.hpp"
#include "rclcpp/node_interfaces/node_graph_interface.hpp"

namespace control_link_adapters
{
	// adapter 侧稳定 Graph 的 publisher 数量、类型、FQN 和 GID generation
	// expected Gateway FQN 与 callback GID 的最终准入仍由 CanonicalInputGuard 负责
	class CanonicalEndpointTracker final
	{
	public:
		CanonicalEndpointTracker(
			std::string expected_type,
			std::string rmw_implementation,
			std::chrono::milliseconds stable_window);

		// 只分析本次 Graph 快照，不在函数内执行 Graph 查询
		[[nodiscard]] CanonicalEndpointSnapshot observe(
			const std::vector<rclcpp::TopicEndpointInfo> &publishers,
			std::chrono::steady_clock::time_point observed_at);

		// 返回已经过稳定窗口确认的结果；有待确认变化时返回 kUnstable
		[[nodiscard]] CanonicalEndpointSnapshot current() const;

	private:
		[[nodiscard]] CanonicalEndpointSnapshot assess(
			const std::vector<rclcpp::TopicEndpointInfo> &publishers) const;

		[[nodiscard]] bool same_snapshot(
			const CanonicalEndpointSnapshot &left,
			const CanonicalEndpointSnapshot &right) const noexcept;

		std::string expected_type_;
		std::string rmw_implementation_;
		std::chrono::milliseconds stable_window_;
		std::optional<CanonicalEndpointSnapshot> stable_snapshot_;
		std::optional<CanonicalEndpointSnapshot> candidate_snapshot_;
		std::optional<std::chrono::steady_clock::time_point> candidate_since_;
		std::optional<std::chrono::steady_clock::time_point> last_observed_at_;
	};
} // namespace control_link_adapters
