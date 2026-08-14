#pragma once

#include <chrono>
#include <cstdint>
#include <memory>
#include <mutex>
#include <optional>
#include <string>

#include "control_link_adapters/can_codec.hpp"
#include "control_link_adapters/canonical_endpoint_tracker.hpp"
#include "control_link_adapters/canonical_input_guard.hpp"
#include "control_link_adapters/local_watchdog.hpp"
#include "control_link_adapters/socketcan_transport.hpp"
#include "control_link_contract/contract_bundle.hpp"
#include "control_link_interfaces/msg/control_command.hpp"
#include "control_link_interfaces/msg/vehicle_state.hpp"
#include "diagnostic_msgs/msg/diagnostic_array.hpp"
#include "rclcpp/rclcpp.hpp"

namespace control_link_adapters
{
	// ADAS Profile 的唯一 ROS2-SocketCAN 执行 adapter
	// 节点只装配 canonical 准入、协议 codec、CAN transport 与 VehicleState，不复制网关仲裁
	class SocketCanAdapter final : public rclcpp::Node
	{
	public:
		explicit SocketCanAdapter(
			const rclcpp::NodeOptions &options = rclcpp::NodeOptions());
		~SocketCanAdapter() override;

	private:
		using ControlCommand = control_link_interfaces::msg::ControlCommand;
		using VehicleState = control_link_interfaces::msg::VehicleState;

		struct ValidCanState final
		{
			CanVehicleStateSignals signals;
			std::chrono::steady_clock::time_point received_at;
		};

		void poll_canonical_endpoint();
		void handle_canonical_command(
			const ControlCommand &command,
			const rclcpp::MessageInfo &message_info);
		void handle_can_frame(
			const CanFrame &frame,
			std::chrono::steady_clock::time_point received_at) noexcept;
		void handle_can_error(const SocketCanIoError &error);
		void publish_vehicle_state();
		void publish_diagnostics();

		[[nodiscard]] std::optional<CanFrame> make_periodic_control_frame();
		[[nodiscard]] CanFrame make_control_frame_locked(
			CanControlMode mode,
			double speed_mps,
			double yaw_rate_radps);
		[[nodiscard]] CanFrame make_safe_hold_frame_locked();
		void confirm_control_frame_transmitted(const CanFrame &frame);
		void request_immediate_hold() noexcept;
		void mark_can_timeout_locked(
			std::chrono::steady_clock::time_point now);
		[[nodiscard]] std::uint16_t canonical_fault_code_locked(
			std::chrono::steady_clock::time_point now) const;
		[[nodiscard]] VehicleState compose_vehicle_state_locked(
			std::int64_t now_ros_ns,
			std::chrono::steady_clock::time_point now_steady) const;

		control_link_contract::ContractBundlePtr bundle_;
		const control_link_contract::AdasProfile *adas_profile_{nullptr};
		control_link_contract::CanSignalMapPtr signal_map_;
		std::string rmw_implementation_;
		std::chrono::milliseconds can_state_timeout_{0};

		std::unique_ptr<CanCodec> can_codec_;
		std::unique_ptr<CanonicalInputGuard> canonical_guard_;
		std::unique_ptr<LocalWatchdog> local_watchdog_;
		std::unique_ptr<CanonicalEndpointTracker> canonical_endpoint_tracker_;
		std::unique_ptr<SocketCanTransport> transport_;

		rclcpp::Subscription<ControlCommand>::SharedPtr canonical_subscription_;
		rclcpp::Publisher<VehicleState>::SharedPtr vehicle_state_publisher_;
		rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr
			diagnostics_publisher_;
		rclcpp::TimerBase::SharedPtr canonical_graph_timer_;
		rclcpp::TimerBase::SharedPtr vehicle_state_timer_;
		rclcpp::TimerBase::SharedPtr diagnostics_timer_;

		// ROS callback 与 CAN I/O thread 只通过该锁内的不可分割快照交换状态
		mutable std::mutex runtime_mutex_;
		CanonicalEndpointState canonical_endpoint_state_{
			CanonicalEndpointState::kUnstable};
		std::optional<CanonicalPublisherKey> confirmed_gateway_publisher_;
		std::optional<ControlCommand> latest_command_;
		CanonicalRejectReason last_canonical_reject_reason_{
			CanonicalRejectReason::kNone};

		RollingCounterChecker state_counter_checker_;
		std::optional<ValidCanState> latest_valid_can_state_;
		std::chrono::steady_clock::time_point can_monitor_started_at_;
		std::uint64_t can_recovery_valid_count_{0U};
		std::uint8_t next_control_counter_{0U};
		bool force_next_hold_{false};
		CanCodecRejectReason last_can_reject_reason_{
			CanCodecRejectReason::kNone};
		CounterObservation last_counter_observation_{
			CounterObservation::kFirstValue};
		std::uint64_t accepted_can_state_frames_{0U};
		std::uint64_t rejected_can_state_frames_{0U};
		bool can_link_healthy_{false};
		bool can_state_timed_out_{false};
		bool fatal_can_io_{false};
		std::optional<SocketCanIoError> last_can_io_error_;
	};
} // namespace control_link_adapters
