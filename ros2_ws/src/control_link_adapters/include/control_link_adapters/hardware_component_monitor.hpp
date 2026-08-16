#pragma once

#include <chrono>
#include <cstdint>
#include <optional>
#include <string>
#include <vector>

#include "controller_manager_msgs/msg/hardware_component_state.hpp"

namespace control_link_adapters
{
	enum class HardwareComponentHealth : std::uint8_t
	{
		kUnavailable,
		kHealthy,
		kInactive,
		kTimedOut,
		kInvalid,
	};

	struct HardwareComponentSnapshot
	{
		HardwareComponentHealth health;
		std::uint8_t lifecycle_state;

		[[nodiscard]] bool healthy() const noexcept
		{
			return health == HardwareComponentHealth::kHealthy;
		}
	};

	// 将 controller manager 的 service 响应转换为确定性健康快照
	// service 调用与超时清理由 ROS node 负责，本类不执行任何 I/O
	class HardwareComponentMonitor final
	{
	public:
		HardwareComponentMonitor(
			std::string expected_component_name,
			std::chrono::milliseconds state_timeout);

		HardwareComponentSnapshot observe(
			const std::vector<controller_manager_msgs::msg::HardwareComponentState> &components,
			std::chrono::steady_clock::time_point observed_at);

		[[nodiscard]] HardwareComponentSnapshot assess(
			std::chrono::steady_clock::time_point now) const;

	private:
		std::string expected_component_name_;
		std::chrono::milliseconds state_timeout_;
		HardwareComponentSnapshot current_{
			HardwareComponentHealth::kUnavailable,
			0U};
		std::optional<std::chrono::steady_clock::time_point> last_response_at_;
	};
}  // namespace control_link_adapters
