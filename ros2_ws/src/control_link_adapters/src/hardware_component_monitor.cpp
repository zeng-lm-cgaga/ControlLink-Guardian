#include "control_link_adapters/hardware_component_monitor.hpp"

#include <algorithm>
#include <stdexcept>
#include <utility>

#include "lifecycle_msgs/msg/state.hpp"

namespace control_link_adapters
{
	HardwareComponentMonitor::HardwareComponentMonitor(
		std::string expected_component_name,
		std::chrono::milliseconds state_timeout)
		: expected_component_name_(std::move(expected_component_name)),
		  state_timeout_(state_timeout)
	{
		if (expected_component_name_.empty() ||
			state_timeout_ <= std::chrono::milliseconds::zero())
		{
			throw std::invalid_argument(
				"HardwareComponentMonitor requires a component name and positive timeout");
		}
	}

	HardwareComponentSnapshot HardwareComponentMonitor::observe(
		const std::vector<controller_manager_msgs::msg::HardwareComponentState> &components,
		std::chrono::steady_clock::time_point observed_at)
	{
		if (last_response_at_.has_value() && observed_at < last_response_at_.value())
		{
			throw std::invalid_argument(
				"hardware component observation time moved backwards");
		}
		last_response_at_ = observed_at;

		const auto matching_count = std::count_if(
			components.begin(),
			components.end(),
			[this](const auto &component)
			{
				return component.name == expected_component_name_;
			});
		if (matching_count != 1)
		{
			current_ = HardwareComponentSnapshot{
				HardwareComponentHealth::kInvalid,
				lifecycle_msgs::msg::State::PRIMARY_STATE_UNKNOWN};
			return current_;
		}

		const auto component = std::find_if(
			components.begin(),
			components.end(),
			[this](const auto &candidate)
			{
				return candidate.name == expected_component_name_;
			});
		current_ = HardwareComponentSnapshot{
			component->state.id == lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE ?
				HardwareComponentHealth::kHealthy :
				HardwareComponentHealth::kInactive,
			component->state.id};
		return current_;
	}

	HardwareComponentSnapshot HardwareComponentMonitor::assess(
		std::chrono::steady_clock::time_point now) const
	{
		if (!last_response_at_.has_value())
		{
			return current_;
		}
		if (now < last_response_at_.value())
		{
			throw std::invalid_argument(
				"hardware component assessment time moved backwards");
		}
		if (now - last_response_at_.value() > state_timeout_)
		{
			return HardwareComponentSnapshot{
				HardwareComponentHealth::kTimedOut,
				current_.lifecycle_state};
		}
		return current_;
	}
}  // namespace control_link_adapters
