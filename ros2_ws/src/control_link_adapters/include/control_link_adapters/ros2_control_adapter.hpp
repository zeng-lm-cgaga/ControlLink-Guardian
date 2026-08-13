#pragma once

#include <chrono>
#include <cstdint>
#include <memory>
#include <optional>
#include <string>

#include "control_link_adapters/canonical_endpoint_tracker.hpp"
#include "control_link_adapters/canonical_input_guard.hpp"
#include "control_link_adapters/controller_endpoint_monitor.hpp"
#include "control_link_adapters/local_watchdog.hpp"
#include "control_link_adapters/tf_health_monitor.hpp"
#include "control_link_contract/contract_bundle.hpp"
#include "control_link_interfaces/msg/control_command.hpp"
#include "control_link_interfaces/msg/vehicle_state.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

namespace control_link_adapters
{
	// Robot Profile 的唯一执行 adapter
	// 只负责 canonical 边界、ros2_control 末端转换、平台健康和 VehicleState，不复制网关仲裁
	class Ros2ControlAdapter final : public rclcpp::Node
	{
	public:
		explicit Ros2ControlAdapter(
			const rclcpp::NodeOptions &options = rclcpp::NodeOptions());

	private:
		using ControlCommand = control_link_interfaces::msg::ControlCommand;
		using VehicleState = control_link_interfaces::msg::VehicleState;

		struct OdomSnapshot
		{
			std::int64_t stamp_ns;
			std::chrono::steady_clock::time_point received_at;
			double linear_velocity_mps;
			double angular_velocity_radps;
		};

		void poll_canonical_endpoint();
		void poll_controller_endpoint();
		void handle_canonical_command(
			const ControlCommand &command,
			const rclcpp::MessageInfo &message_info);
		void handle_odometry(const nav_msgs::msg::Odometry &odometry);
		void publish_controller_command();
		void publish_vehicle_state();

		[[nodiscard]] bool odometry_is_healthy(
			std::chrono::steady_clock::time_point now_steady,
			std::int64_t now_ros_ns) const noexcept;
		[[nodiscard]] std::uint16_t platform_fault_code(
			std::chrono::steady_clock::time_point now_steady,
			std::int64_t now_ros_ns) const noexcept;
		[[nodiscard]] std::uint16_t current_fault_code(
			std::chrono::steady_clock::time_point now_steady,
			std::int64_t now_ros_ns) const;

		control_link_contract::ContractBundlePtr bundle_;
		const control_link_contract::RobotProfile *robot_profile_{nullptr};
		std::string rmw_implementation_;
		std::chrono::nanoseconds odometry_timeout_;
		std::chrono::nanoseconds max_future_skew_;

		std::unique_ptr<CanonicalInputGuard> canonical_guard_;
		std::unique_ptr<LocalWatchdog> local_watchdog_;
		std::unique_ptr<CanonicalEndpointTracker> canonical_endpoint_tracker_;
		std::unique_ptr<ControllerEndpointMonitor> controller_endpoint_monitor_;

		// 声明顺序保证销毁时 monitor -> listener -> buffer，避免悬空 buffer 引用
		std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
		std::unique_ptr<tf2_ros::TransformListener> tf_listener_;
		std::unique_ptr<TfHealthMonitor> tf_health_monitor_;

		rclcpp::Subscription<ControlCommand>::SharedPtr canonical_subscription_;
		rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odometry_subscription_;
		rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr
			controller_command_publisher_;
		rclcpp::Publisher<VehicleState>::SharedPtr vehicle_state_publisher_;
		rclcpp::TimerBase::SharedPtr canonical_graph_timer_;
		rclcpp::TimerBase::SharedPtr controller_graph_timer_;
		rclcpp::TimerBase::SharedPtr controller_output_timer_;
		rclcpp::TimerBase::SharedPtr vehicle_state_timer_;

		std::optional<CanonicalPublisherKey> confirmed_gateway_publisher_;
		std::optional<ControlCommand> latest_command_;
		std::optional<OdomSnapshot> latest_valid_odometry_;
		CanonicalRejectReason last_canonical_reject_reason_{
			CanonicalRejectReason::kNone};
		ControllerEndpointSnapshot controller_endpoint_snapshot_{
			ControllerEndpointState::kUnavailable,
			std::nullopt};
		TfHealthSnapshot tf_health_snapshot_{
			TfHealthState::kUnavailable,
			0,
			0};
		bool latest_odometry_sample_valid_{false};
	};
} // namespace control_link_adapters
