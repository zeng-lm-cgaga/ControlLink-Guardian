#include <chrono>
#include <cmath>
#include <cstdint>
#include <filesystem>
#include <future>
#include <limits>
#include <memory>
#include <optional>
#include <stdexcept>
#include <string>
#include <thread>
#include <variant>

#include "control_link_contract/contract_bundle.hpp"
#include "control_link_contract/qos_factory.hpp"
#include "control_link_interfaces/msg/gateway_state.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/executors/single_threaded_executor.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "tf2/exceptions.hpp"
#include "tf2/time.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

namespace
{
	using namespace std::chrono_literals;
	using GatewayState = control_link_interfaces::msg::GatewayState;
	using NavigateToPose = nav2_msgs::action::NavigateToPose;
	using GoalHandle = rclcpp_action::ClientGoalHandle<NavigateToPose>;

	struct PoseEvidence
	{
		bool available{false};
		std::int64_t stamp_ns{0};
		double x{0.0};
		double y{0.0};
		double yaw{0.0};
	};

	double quaternion_yaw(
		double x,
		double y,
		double z,
		double w) noexcept
	{
		return std::atan2(
			2.0 * (w * z + x * y),
			1.0 - 2.0 * (y * y + z * z));
	}

	double angular_distance(double left, double right) noexcept
	{
		const auto difference = left - right;
		return std::abs(std::atan2(std::sin(difference), std::cos(difference)));
	}

	int run_goal(const rclcpp::Node::SharedPtr & node)
	{
		const auto profile_path = std::filesystem::path{
			node->declare_parameter<std::string>("profile_path", "")};
		const auto config_root = std::filesystem::path{
			node->declare_parameter<std::string>("config_root", "")};
		const auto position_tolerance_m = node->declare_parameter<double>(
			"position_tolerance_m", 0.0);
		const auto yaw_tolerance_rad = node->declare_parameter<double>(
			"yaw_tolerance_rad", 0.0);
		const auto timeout_ms = node->declare_parameter<std::int64_t>(
			"timeout_ms", 120000);
		const auto clock_start_timeout_ms = node->declare_parameter<std::int64_t>(
			"clock_start_timeout_ms", 15000);
		const auto pose_settle_timeout_ms = node->declare_parameter<std::int64_t>(
			"pose_settle_timeout_ms", 5000);
		const auto pose_stable_window_ms = node->declare_parameter<std::int64_t>(
			"pose_stable_window_ms", 500);
		if (profile_path.empty() || config_root.empty())
		{
			throw std::invalid_argument(
				"profile_path and config_root must be non-empty");
		}
		if (!std::isfinite(position_tolerance_m) || position_tolerance_m <= 0.0 ||
			!std::isfinite(yaw_tolerance_rad) || yaw_tolerance_rad <= 0.0)
		{
			throw std::invalid_argument(
				"goal tolerances must be positive finite values");
		}
		if (timeout_ms <= 0 || clock_start_timeout_ms <= 0)
		{
			throw std::invalid_argument(
				"timeout_ms and clock_start_timeout_ms must be positive");
		}
		if (pose_settle_timeout_ms <= 0 || pose_stable_window_ms <= 0 ||
			pose_stable_window_ms > pose_settle_timeout_ms)
		{
			throw std::invalid_argument(
				"pose settle timeout must cover a positive stable window");
		}

		const auto bundle = control_link_contract::load_contract_bundle(
			profile_path,
			config_root);
		const auto * robot_profile = std::get_if<control_link_contract::RobotProfile>(
			bundle->profile.get());
		if (robot_profile == nullptr)
		{
			throw std::invalid_argument("fixed Nav2 goal requires Robot Profile");
		}

		control_link_contract::QosFactory qos_factory{bundle->gateway_contract};
		const auto & state_contract =
			bundle->gateway_contract->state_topics.at("gateway_state");
		if (!state_contract.qos_profile.has_value())
		{
			throw std::logic_error(
				"gateway_state Contract is missing its QoS profile");
		}
		const auto state_qos = qos_factory.make(
			state_contract.qos_profile.value());
		bool saw_active_nav2 = false;
		std::uint64_t active_sequence = 0U;
		auto gateway_state_subscription = node->create_subscription<GatewayState>(
			state_contract.topic,
			state_qos,
			[&saw_active_nav2, &active_sequence](const GatewayState & state)
			{
				if (state.state == GatewayState::ACTIVE &&
					state.active_source_id == "nav2")
				{
					saw_active_nav2 = true;
					active_sequence = state.active_source_sequence;
				}
			});

		auto tf_buffer = std::make_shared<tf2_ros::Buffer>(node->get_clock());
		auto tf_listener = std::make_shared<tf2_ros::TransformListener>(
			*tf_buffer,
			node,
			false);
		PoseEvidence odometry_evidence;
		auto odometry_subscription = node->create_subscription<nav_msgs::msg::Odometry>(
			robot_profile->adapter.odometry_topic,
			rclcpp::QoS{rclcpp::KeepLast{1}}.best_effort(),
			[&odometry_evidence,
			 expected_frame = robot_profile->frames.odom,
			 expected_child_frame = robot_profile->frames.base_footprint](
				const nav_msgs::msg::Odometry & message)
			{
				if (message.header.frame_id != expected_frame ||
					message.child_frame_id != expected_child_frame)
				{
					return;
				}

				const auto & position = message.pose.pose.position;
				const auto & orientation = message.pose.pose.orientation;
				odometry_evidence = PoseEvidence{
					true,
					rclcpp::Time{message.header.stamp}.nanoseconds(),
					position.x,
					position.y,
					quaternion_yaw(
						orientation.x,
						orientation.y,
						orientation.z,
						orientation.w)};
			});
		auto action_client = rclcpp_action::create_client<NavigateToPose>(
			node,
			"/navigate_to_pose");
		rclcpp::executors::SingleThreadedExecutor executor;
		executor.add_node(node);

		// 新启动的 participant 必须在有界 steady-clock 窗口内发现 /clock
		const auto clock_deadline = std::chrono::steady_clock::now() +
			std::chrono::milliseconds{clock_start_timeout_ms};
		while (rclcpp::ok() && std::chrono::steady_clock::now() < clock_deadline &&
			node->get_clock()->now().nanoseconds() <= 0)
		{
			executor.spin_some();
			std::this_thread::sleep_for(20ms);
		}
		if (node->get_clock()->now().nanoseconds() <= 0)
		{
			throw std::runtime_error("ROS clock did not start before goal dispatch");
		}

		const auto server_deadline = std::chrono::steady_clock::now() + 15s;
		while (rclcpp::ok() && std::chrono::steady_clock::now() < server_deadline &&
			!action_client->wait_for_action_server(100ms))
		{
			executor.spin_some();
		}
		if (!action_client->action_server_is_ready())
		{
			throw std::runtime_error("NavigateToPose action server is unavailable");
		}

		const auto now = node->get_clock()->now();
		NavigateToPose::Goal goal;
		goal.pose.header.stamp = now;
		goal.pose.header.frame_id = robot_profile->fixed_demo_goal.frame_id;
		goal.pose.pose.position.x = robot_profile->fixed_demo_goal.x_m;
		goal.pose.pose.position.y = robot_profile->fixed_demo_goal.y_m;
		goal.pose.pose.orientation.z = std::sin(
			robot_profile->fixed_demo_goal.yaw_rad / 2.0);
		goal.pose.pose.orientation.w = std::cos(
			robot_profile->fixed_demo_goal.yaw_rad / 2.0);

		double last_distance_remaining = std::numeric_limits<double>::quiet_NaN();
		rclcpp_action::Client<NavigateToPose>::SendGoalOptions options;
		options.feedback_callback =
			[&last_distance_remaining](
				GoalHandle::SharedPtr,
				const std::shared_ptr<const NavigateToPose::Feedback> feedback)
			{
				last_distance_remaining = feedback->distance_remaining;
			};

		RCLCPP_INFO(
			node->get_logger(),
			"Sending NavigateToPose: frame=%s, x=%.3f, y=%.3f, yaw=%.3f",
			goal.pose.header.frame_id.c_str(),
			goal.pose.pose.position.x,
			goal.pose.pose.position.y,
			robot_profile->fixed_demo_goal.yaw_rad);
		auto goal_handle_future = action_client->async_send_goal(goal, options);
		if (executor.spin_until_future_complete(goal_handle_future, 10s) !=
			rclcpp::FutureReturnCode::SUCCESS)
		{
			throw std::runtime_error("NavigateToPose goal response timed out");
		}
		auto goal_handle = goal_handle_future.get();
		if (!goal_handle)
		{
			throw std::runtime_error("NavigateToPose goal was rejected");
		}

		auto result_future = action_client->async_get_result(goal_handle);
		const auto result_deadline = std::chrono::steady_clock::now() +
			std::chrono::milliseconds{timeout_ms};
		while (rclcpp::ok() && std::chrono::steady_clock::now() < result_deadline &&
			result_future.wait_for(0s) != std::future_status::ready)
		{
			executor.spin_some();
			std::this_thread::sleep_for(20ms);
		}
		if (result_future.wait_for(0s) != std::future_status::ready)
		{
			throw std::runtime_error("NavigateToPose result timed out");
		}
		const auto wrapped_result = result_future.get();
		if (wrapped_result.code != rclcpp_action::ResultCode::SUCCEEDED)
		{
			throw std::runtime_error(
				"NavigateToPose did not succeed; result_code=" +
				std::to_string(static_cast<int>(wrapped_result.code)));
		}
		if (!saw_active_nav2)
		{
			throw std::runtime_error(
				"NavigateToPose succeeded without observing Gateway ACTIVE/nav2");
		}

		// Action result 与 AMCL/TF 回调到达不同步，最终证据必须等待连续稳定的新 TF 样本
		const auto pose_deadline = std::chrono::steady_clock::now() +
			std::chrono::milliseconds{pose_settle_timeout_ms};
		const auto stable_window = std::chrono::milliseconds{pose_stable_window_ms};
		std::optional<std::chrono::steady_clock::time_point> in_tolerance_since;
		std::int64_t stable_first_tf_stamp_ns = 0;
		bool saw_newer_tf_sample = false;
		bool pose_stable = false;
		PoseEvidence final_pose;
		double position_error = std::numeric_limits<double>::infinity();
		double yaw_error = std::numeric_limits<double>::infinity();
		while (rclcpp::ok() && std::chrono::steady_clock::now() < pose_deadline)
		{
			executor.spin_some();
			try
			{
				if (tf_buffer->canTransform(
						robot_profile->fixed_demo_goal.frame_id,
						robot_profile->frames.base_footprint,
						tf2::TimePointZero))
				{
					const auto transform = tf_buffer->lookupTransform(
						robot_profile->fixed_demo_goal.frame_id,
						robot_profile->frames.base_footprint,
						tf2::TimePointZero);
					const auto & translation = transform.transform.translation;
					const auto & rotation = transform.transform.rotation;
					final_pose = PoseEvidence{
						true,
						rclcpp::Time{transform.header.stamp}.nanoseconds(),
						translation.x,
						translation.y,
						quaternion_yaw(
							rotation.x,
							rotation.y,
							rotation.z,
							rotation.w)};
					position_error = std::hypot(
						final_pose.x - robot_profile->fixed_demo_goal.x_m,
						final_pose.y - robot_profile->fixed_demo_goal.y_m);
					yaw_error = angular_distance(
						final_pose.yaw,
						robot_profile->fixed_demo_goal.yaw_rad);

					if (position_error <= position_tolerance_m &&
						yaw_error <= yaw_tolerance_rad)
					{
						const auto now_steady = std::chrono::steady_clock::now();
						if (!in_tolerance_since.has_value())
						{
							in_tolerance_since = now_steady;
							stable_first_tf_stamp_ns = final_pose.stamp_ns;
							saw_newer_tf_sample = false;
						}
						else if (final_pose.stamp_ns > stable_first_tf_stamp_ns)
						{
							saw_newer_tf_sample = true;
						}

						if (saw_newer_tf_sample &&
							now_steady - in_tolerance_since.value() >= stable_window)
						{
							pose_stable = true;
							break;
						}
					}
					else
					{
						in_tolerance_since.reset();
						saw_newer_tf_sample = false;
					}
				}
				else
				{
					in_tolerance_since.reset();
					saw_newer_tf_sample = false;
				}
				}
			catch (const tf2::TransformException &)
			{
				in_tolerance_since.reset();
				saw_newer_tf_sample = false;
			}
			std::this_thread::sleep_for(20ms);
		}
		if (!pose_stable)
		{
			if (!final_pose.available)
			{
				throw std::runtime_error(
					"final map to base transform is unavailable during settle window");
			}
			throw std::runtime_error(
				"final pose did not remain within configured tolerance; final_x=" +
				std::to_string(final_pose.x) + ", final_y=" +
				std::to_string(final_pose.y) + ", final_yaw=" +
				std::to_string(final_pose.yaw) + ", position_error=" +
				std::to_string(position_error) + ", yaw_error=" +
				std::to_string(yaw_error));
		}
		if (!odometry_evidence.available)
		{
			throw std::runtime_error(
				"fixed Nav2 goal completed without observing Profile odometry topic: " +
				robot_profile->adapter.odometry_topic);
		}

		RCLCPP_INFO(
			node->get_logger(),
			"NAV_GOAL_SUCCEEDED final_x=%.4f final_y=%.4f final_yaw=%.4f "
			"position_error=%.4f yaw_error=%.4f tf_stamp_ns=%ld "
			"odom_yaw=%.4f odom_stamp_ns=%ld gateway_sequence=%lu last_distance=%.4f",
			final_pose.x,
			final_pose.y,
			final_pose.yaw,
			position_error,
			yaw_error,
			static_cast<long>(final_pose.stamp_ns),
			odometry_evidence.yaw,
			static_cast<long>(odometry_evidence.stamp_ns),
			static_cast<unsigned long>(active_sequence),
			last_distance_remaining);
		(void)gateway_state_subscription;
		(void)odometry_subscription;
		(void)tf_listener;
		return 0;
	}
}  // namespace

int main(int argc, char ** argv)
{
	rclcpp::init(argc, argv);
	int exit_code = 1;
	try
	{
		auto node = std::make_shared<rclcpp::Node>("send_fixed_nav_goal");
		exit_code = run_goal(node);
	}
	catch (const std::exception & exception)
	{
		RCLCPP_FATAL(
			rclcpp::get_logger("send_fixed_nav_goal"),
			"Fixed Nav2 goal failed: %s",
			exception.what());
	}
	rclcpp::shutdown();
	return exit_code;
}
