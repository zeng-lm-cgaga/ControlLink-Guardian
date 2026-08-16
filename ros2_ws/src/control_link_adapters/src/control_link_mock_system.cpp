#include "control_link_adapters/control_link_mock_system.hpp"

#include <algorithm>
#include <charconv>
#include <cmath>
#include <string>
#include <unordered_set>
#include <utility>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "pluginlib/class_list_macros.hpp"

namespace control_link_adapters
{
	namespace
	{
		constexpr char kLeftWheelParameter[] = "left_wheel_joint";
		constexpr char kRightWheelParameter[] = "right_wheel_joint";
		constexpr char kFailReadParameter[] = "fail_read_after_cycles";
		constexpr char kFailWriteParameter[] = "fail_write_after_cycles";

		bool has_exact_interfaces(
			const std::vector<hardware_interface::InterfaceInfo> & interfaces,
			const std::unordered_set<std::string> & expected)
		{
			if (interfaces.size() != expected.size())
			{
				return false;
			}

			std::unordered_set<std::string> actual;
			for (const auto & interface : interfaces)
			{
				if (interface.name.empty() || !actual.emplace(interface.name).second)
				{
					return false;
				}
			}
			return actual == expected;
		}
	}  // namespace

	ControlLinkMockSystem::CallbackReturn ControlLinkMockSystem::on_init(
		const hardware_interface::HardwareInfo & hardware_info)
	{
		if (hardware_interface::SystemInterface::on_init(hardware_info) != CallbackReturn::SUCCESS)
		{
			return CallbackReturn::ERROR;
		}

		if (!validate_hardware_info() ||
			!parse_failure_threshold(kFailReadParameter, fail_read_after_cycles_) ||
			!parse_failure_threshold(kFailWriteParameter, fail_write_after_cycles_))
		{
			return CallbackReturn::ERROR;
		}

		reset_motion();
		configured_ = false;
		active_ = false;
		return CallbackReturn::SUCCESS;
	}

	ControlLinkMockSystem::CallbackReturn ControlLinkMockSystem::on_configure(
		const rclcpp_lifecycle::State &)
	{
		reset_motion();
		configured_ = true;
		active_ = false;
		return CallbackReturn::SUCCESS;
	}

	ControlLinkMockSystem::CallbackReturn ControlLinkMockSystem::on_cleanup(
		const rclcpp_lifecycle::State &)
	{
		reset_motion();
		configured_ = false;
		active_ = false;
		return CallbackReturn::SUCCESS;
	}

	ControlLinkMockSystem::CallbackReturn ControlLinkMockSystem::on_activate(
		const rclcpp_lifecycle::State &)
	{
		if (!configured_)
		{
			return CallbackReturn::ERROR;
		}

		// 激活不能继承上一次运行留下的速度命令，避免 controller 切换时出现意外运动
		reset_motion();
		active_ = true;
		return CallbackReturn::SUCCESS;
	}

	ControlLinkMockSystem::CallbackReturn ControlLinkMockSystem::on_deactivate(
		const rclcpp_lifecycle::State &)
	{
		reset_motion();
		active_ = false;
		return CallbackReturn::SUCCESS;
	}

	ControlLinkMockSystem::CallbackReturn ControlLinkMockSystem::on_shutdown(
		const rclcpp_lifecycle::State &)
	{
		reset_motion();
		configured_ = false;
		active_ = false;
		return CallbackReturn::SUCCESS;
	}

	std::vector<hardware_interface::StateInterface>
	ControlLinkMockSystem::export_state_interfaces()
	{
		std::vector<hardware_interface::StateInterface> interfaces;
		interfaces.reserve(kWheelCount * 2U);
		for (std::size_t index = 0U; index < kWheelCount; ++index)
		{
			interfaces.emplace_back(
				joint_names_[index], hardware_interface::HW_IF_POSITION, &positions_[index]);
			interfaces.emplace_back(
				joint_names_[index], hardware_interface::HW_IF_VELOCITY,
				&applied_velocities_[index]);
		}
		return interfaces;
	}

	std::vector<hardware_interface::CommandInterface>
	ControlLinkMockSystem::export_command_interfaces()
	{
		std::vector<hardware_interface::CommandInterface> interfaces;
		interfaces.reserve(kWheelCount);
		for (std::size_t index = 0U; index < kWheelCount; ++index)
		{
			interfaces.emplace_back(
				joint_names_[index], hardware_interface::HW_IF_VELOCITY, &commands_[index]);
		}
		return interfaces;
	}

	hardware_interface::return_type ControlLinkMockSystem::read(
		const rclcpp::Time &,
		const rclcpp::Duration & period)
	{
		if (!active_ || period.nanoseconds() < 0)
		{
			return hardware_interface::return_type::ERROR;
		}
		if (fail_read_after_cycles_ != 0U &&
			successful_read_cycles_ >= fail_read_after_cycles_)
		{
			return hardware_interface::return_type::ERROR;
		}

		const double period_seconds = period.seconds();
		if (!std::isfinite(period_seconds))
		{
			return hardware_interface::return_type::ERROR;
		}
		for (std::size_t index = 0U; index < kWheelCount; ++index)
		{
			positions_[index] += applied_velocities_[index] * period_seconds;
			if (!std::isfinite(positions_[index]))
			{
				return hardware_interface::return_type::ERROR;
			}
		}
		++successful_read_cycles_;
		return hardware_interface::return_type::OK;
	}

	hardware_interface::return_type ControlLinkMockSystem::write(
		const rclcpp::Time &,
		const rclcpp::Duration &)
	{
		if (!active_)
		{
			return hardware_interface::return_type::ERROR;
		}
		if (fail_write_after_cycles_ != 0U &&
			successful_write_cycles_ >= fail_write_after_cycles_)
		{
			return hardware_interface::return_type::ERROR;
		}
		if (!std::all_of(commands_.begin(), commands_.end(), [](double command) {
				return std::isfinite(command);
			}))
		{
			return hardware_interface::return_type::ERROR;
		}

		applied_velocities_ = commands_;
		++successful_write_cycles_;
		return hardware_interface::return_type::OK;
	}

	void ControlLinkMockSystem::reset_motion() noexcept
	{
		commands_.fill(0.0);
		applied_velocities_.fill(0.0);
		positions_.fill(0.0);
		successful_read_cycles_ = 0U;
		successful_write_cycles_ = 0U;
	}

	bool ControlLinkMockSystem::validate_hardware_info() noexcept
	{
		const auto left = info_.hardware_parameters.find(kLeftWheelParameter);
		const auto right = info_.hardware_parameters.find(kRightWheelParameter);
		if (left == info_.hardware_parameters.end() ||
			right == info_.hardware_parameters.end() ||
			left->second.empty() || right->second.empty() || left->second == right->second ||
			info_.joints.size() != kWheelCount)
		{
			return false;
		}
		joint_names_ = {left->second, right->second};

		std::unordered_set<std::string> configured_joint_names;
		for (const auto & joint : info_.joints)
		{
			if (!configured_joint_names.emplace(joint.name).second ||
				!has_exact_interfaces(
					joint.command_interfaces, {hardware_interface::HW_IF_VELOCITY}) ||
				!has_exact_interfaces(
					joint.state_interfaces,
					{hardware_interface::HW_IF_POSITION, hardware_interface::HW_IF_VELOCITY}))
			{
				return false;
			}
		}

		return configured_joint_names ==
			std::unordered_set<std::string>{joint_names_[0], joint_names_[1]};
	}

	bool ControlLinkMockSystem::parse_failure_threshold(
		const std::string & parameter_name,
		std::uint64_t & destination) noexcept
	{
		destination = 0U;
		const auto found = info_.hardware_parameters.find(parameter_name);
		if (found == info_.hardware_parameters.end())
		{
			return true;
		}
		if (found->second.empty() || found->second.front() == '-')
		{
			return false;
		}

		const char *begin = found->second.data();
		const char *end = begin + found->second.size();
		std::uint64_t parsed = 0U;
		const auto result = std::from_chars(begin, end, parsed, 10);
		if (result.ec != std::errc{} || result.ptr != end)
		{
			return false;
		}
		destination = parsed;
		return true;
	}
}  // namespace control_link_adapters

PLUGINLIB_EXPORT_CLASS(
	control_link_adapters::ControlLinkMockSystem,
	hardware_interface::SystemInterface)
