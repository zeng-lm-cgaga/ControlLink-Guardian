#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/duration.hpp"
#include "rclcpp/time.hpp"
#include "rclcpp_lifecycle/state.hpp"

namespace control_link_adapters
{
	// ControlLinkMockSystem 只模拟两只驱动轮的执行与反馈，不复制 Guardian 的校验、仲裁或 watchdog
	// 所有状态推进都由 controller manager 传入的 period 驱动，因此测试可以复现完全相同的轨迹
	class ControlLinkMockSystem final : public hardware_interface::SystemInterface
	{
	public:
		using CallbackReturn = hardware_interface::CallbackReturn;

		CallbackReturn on_init(const hardware_interface::HardwareInfo & hardware_info) override;
		CallbackReturn on_configure(const rclcpp_lifecycle::State & previous_state) override;
		CallbackReturn on_cleanup(const rclcpp_lifecycle::State & previous_state) override;
		CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;
		CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;
		CallbackReturn on_shutdown(const rclcpp_lifecycle::State & previous_state) override;

		std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
		std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

		hardware_interface::return_type read(
			const rclcpp::Time & time,
			const rclcpp::Duration & period) override;
		hardware_interface::return_type write(
			const rclcpp::Time & time,
			const rclcpp::Duration & period) override;

	private:
		static constexpr std::size_t kWheelCount = 2U;

		void reset_motion() noexcept;
		[[nodiscard]] bool validate_hardware_info() noexcept;
		[[nodiscard]] bool parse_failure_threshold(
			const std::string & parameter_name,
			std::uint64_t & destination) noexcept;

		std::array<std::string, kWheelCount> joint_names_{};
		std::array<double, kWheelCount> commands_{};
		std::array<double, kWheelCount> applied_velocities_{};
		std::array<double, kWheelCount> positions_{};
		std::uint64_t fail_read_after_cycles_{0U};
		std::uint64_t fail_write_after_cycles_{0U};
		std::uint64_t successful_read_cycles_{0U};
		std::uint64_t successful_write_cycles_{0U};
		bool configured_{false};
		bool active_{false};
	};
}  // namespace control_link_adapters
