#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <filesystem>
#include <memory>
#include <limits>
#include <optional>
#include <stdexcept>
#include <string>
#include <type_traits>
#include <utility>
#include <variant>
#include <vector>

#include "builtin_interfaces/msg/time.hpp"
#include "control_link_adapters/canonical_endpoint_tracker.hpp"
#include "control_link_adapters/canonical_input_guard.hpp"
#include "control_link_adapters/local_watchdog.hpp"
#include "control_link_contract/contract_bundle.hpp"
#include "control_link_contract/fastdds_environment.hpp"
#include "control_link_contract/qos_factory.hpp"
#include "control_link_interfaces/msg/control_command.hpp"
#include "control_link_interfaces/msg/vehicle_state.hpp"
#include "rclcpp/rclcpp.hpp"

namespace
{
	using ControlCommand = control_link_interfaces::msg::ControlCommand;
	using VehicleState = control_link_interfaces::msg::VehicleState;

	std::chrono::milliseconds checked_milliseconds(
		std::uint64_t value,
		const char *field_name)
	{
		if (value == 0U || value > static_cast<std::uint64_t>(
			std::numeric_limits<std::chrono::milliseconds::rep>::max()))
		{
			throw std::invalid_argument(
				std::string(field_name) + " must fit a positive milliseconds duration");
		}
		return std::chrono::milliseconds{
			static_cast<std::chrono::milliseconds::rep>(value)};
	}

	control_link_adapters::CanonicalPublisherKey key_from_message_info(
		const rclcpp::MessageInfo &message_info)
	{
		const auto gid = message_info.get_rmw_message_info().publisher_gid;
		if (gid.implementation_identifier == nullptr)
		{
			throw std::invalid_argument(
				"canonical message has no publisher RMW implementation identifier");
		}

		control_link_adapters::CanonicalPublisherKey result;
		result.rmw_implementation = gid.implementation_identifier;
		std::copy_n(
			gid.data,
			RMW_GID_STORAGE_SIZE,
			result.publisher_gid.begin());
		return result;
	}

	builtin_interfaces::msg::Time ros_time_message(std::int64_t nanoseconds)
	{
		if (nanoseconds < 0)
		{
			throw std::logic_error("mock adapter cannot publish negative ROS time");
		}
		return static_cast<builtin_interfaces::msg::Time>(
			rclcpp::Time(nanoseconds, RCL_ROS_TIME));
	}

	class MockVehicleAdapter final : public rclcpp::Node
	{
	public:
		MockVehicleAdapter()
			: rclcpp::Node("vehicle_adapter")
		{
			declare_parameter<std::string>("profile_path", "");
			declare_parameter<std::string>("config_root", "");

			const auto profile_path = std::filesystem::path{
				get_parameter("profile_path").as_string()};
			const auto config_root = std::filesystem::path{
				get_parameter("config_root").as_string()};
			bundle_ = control_link_contract::load_contract_bundle(
				profile_path,
				config_root);

			const std::string actual_fqn{get_fully_qualified_name()};
			const auto critical_endpoint = std::find_if(
				bundle_->gateway_contract->critical_endpoints.begin(),
				bundle_->gateway_contract->critical_endpoints.end(),
				[](const auto &endpoint)
				{
					return endpoint.id == "canonical_output_consumer";
				});
			if (critical_endpoint ==
				bundle_->gateway_contract->critical_endpoints.end())
			{
				throw std::logic_error(
					"mock vehicle adapter requires canonical_output_consumer endpoint");
			}
			if (actual_fqn != critical_endpoint->remote_node_fqn)
			{
				throw std::runtime_error(
					"mock vehicle adapter FQN must be /control_link/vehicle_adapter; actual=" +
					actual_fqn);
			}

			rmw_implementation_ =
				control_link_contract::validate_fastdds_process_environment(
					*bundle_->profile,
					"MockVehicleAdapter");

			control_link_contract::QosFactory qos_factory{
				bundle_->gateway_contract};
			const auto canonical_qos = qos_factory.make(
				bundle_->gateway_contract->output.qos_profile);
			const auto state_topic =
				bundle_->gateway_contract->state_topics.at("vehicle_state");
			if (!state_topic.qos_profile.has_value())
			{
				throw std::logic_error(
					"mock vehicle adapter requires vehicle_state QoS profile");
			}
			const auto state_qos = qos_factory.make(state_topic.qos_profile.value());

			std::uint64_t watchdog_timeout_ms = 0U;
			std::uint64_t state_period_ms = 0U;
			std::visit(
				[&](const auto &profile)
				{
					watchdog_timeout_ms = profile.adapter.local_watchdog_timeout_ms;
					if constexpr (std::is_same_v<
						decltype(profile), const control_link_contract::RobotProfile &>)
					{
						state_period_ms = profile.health.vehicle_state_publish_period_ms;
					}
					else
					{
						state_period_ms = profile.adapter.vehicle_state_publish_period_ms;
					}
				},
				*bundle_->profile);

			guard_ = std::make_unique<control_link_adapters::CanonicalInputGuard>(
				bundle_->gateway_contract);
			watchdog_ = std::make_unique<control_link_adapters::LocalWatchdog>(
				watchdog_timeout_ms);
			endpoint_tracker_ =
				std::make_unique<control_link_adapters::CanonicalEndpointTracker>(
					bundle_->gateway_contract->output.type,
					rmw_implementation_,
					checked_milliseconds(
						bundle_->gateway_contract->gateway.graph_stable_window_ms,
						"gateway.graph_stable_window_ms"));
			state_publisher_ = create_publisher<VehicleState>(
				state_topic.topic,
				state_qos);
			rclcpp::SubscriptionOptions subscription_options;
			canonical_subscription_ = create_subscription<ControlCommand>(
				bundle_->gateway_contract->output.topic,
				canonical_qos,
				[this](
					ControlCommand::ConstSharedPtr command,
					const rclcpp::MessageInfo &message_info)
				{
					handle_canonical_command(*command, message_info);
				},
				subscription_options);
			graph_timer_ = create_wall_timer(
				checked_milliseconds(
					bundle_->gateway_contract->gateway.graph_poll_ms,
					"gateway.graph_poll_ms"),
				[this]()
				{
					poll_canonical_endpoint();
				});
			state_timer_ = create_wall_timer(
				checked_milliseconds(
					state_period_ms,
					"vehicle_state_publish_period_ms"),
				[this]()
				{
					publish_vehicle_state();
				});

			RCLCPP_INFO(
				get_logger(),
				"Mock vehicle adapter ready: canonical=%s, vehicle_state=%s",
				bundle_->gateway_contract->output.topic.c_str(),
				state_topic.topic.c_str());
		}

	private:
		void poll_canonical_endpoint()
		{
			const auto publishers = get_publishers_info_by_topic(
				bundle_->gateway_contract->output.topic);
			const auto snapshot = endpoint_tracker_->observe(
				publishers,
				std::chrono::steady_clock::now());
			if (snapshot.state ==
				control_link_adapters::CanonicalEndpointState::kConfirmed &&
				snapshot.confirmed_publisher.has_value() &&
				(!confirmed_gateway_publisher_.has_value() ||
					confirmed_gateway_publisher_.value().rmw_implementation !=
						snapshot.confirmed_publisher->rmw_implementation ||
					confirmed_gateway_publisher_->publisher_gid !=
						snapshot.confirmed_publisher->publisher_gid))
			{
				confirmed_gateway_publisher_ = snapshot.confirmed_publisher;
				watchdog_->reset();
				last_command_.reset();
				last_reject_reason_ =
					control_link_adapters::CanonicalRejectReason::kNone;
			}
		}

		void handle_canonical_command(
			const ControlCommand &command,
			const rclcpp::MessageInfo &message_info)
		{
			const auto actual_publisher = key_from_message_info(message_info);
			const auto endpoint = endpoint_tracker_->current();
			const auto now_ros_ns = get_clock()->now().nanoseconds();
			const bool clock_healthy = now_ros_ns > 0;
			const auto result = guard_->validate(
				command,
				actual_publisher,
				endpoint,
				now_ros_ns,
				clock_healthy);
			if (!result.accepted())
			{
				last_reject_reason_ = result.reason;
				last_command_.reset();
				return;
			}

			// 先完成可能抛异常的消息复制，失败时不能把 watchdog 刷新为健康
			last_command_ = command;
			try
			{
				watchdog_->observe_valid_command(std::chrono::steady_clock::now());
			}
			catch (...)
			{
				last_command_.reset();
				throw;
			}
			last_reject_reason_ =
				control_link_adapters::CanonicalRejectReason::kNone;
		}

		void publish_vehicle_state()
		{
			const auto now_steady = std::chrono::steady_clock::now();
			const auto watchdog_state = watchdog_->evaluate(now_steady);
			const auto endpoint = endpoint_tracker_->current();
			VehicleState message;
			message.observed_at = ros_time_message(get_clock()->now().nanoseconds());
			message.linear_velocity_mps = 0.0;
			message.angular_velocity_radps = 0.0;

			if (endpoint.state ==
				control_link_adapters::CanonicalEndpointState::kAmbiguous)
			{
				message.state = VehicleState::SAFE_STOP;
				message.fault_code =
					VehicleState::FAULT_ADAPTER_CANONICAL_SOURCE_AMBIGUOUS;
			}
			else if (last_reject_reason_ !=
				control_link_adapters::CanonicalRejectReason::kNone)
			{
				message.state = VehicleState::SAFE_STOP;
				message.fault_code = VehicleState::FAULT_ADAPTER_CANONICAL_INVALID;
			}
			else if (watchdog_state !=
				control_link_adapters::LocalWatchdogState::kHealthy)
			{
				message.state = VehicleState::SAFE_STOP;
				message.fault_code =
					VehicleState::FAULT_ADAPTER_CANONICAL_TIMEOUT;
			}
			else if (!last_command_.has_value())
			{
				throw std::logic_error(
					"healthy mock adapter watchdog has no accepted canonical command");
			}
			else if (last_command_->mode == ControlCommand::MODE_HOLD)
			{
				message.state = VehicleState::STANDBY;
				message.fault_code = VehicleState::FAULT_NONE;
			}
			else
			{
				message.state = VehicleState::RUNNING;
				message.fault_code = VehicleState::FAULT_NONE;
				message.linear_velocity_mps = last_command_->linear_velocity_mps;
				message.angular_velocity_radps = last_command_->angular_velocity_radps;
			}

			state_publisher_->publish(message);
		}

		control_link_contract::ContractBundlePtr bundle_;
		std::unique_ptr<control_link_adapters::CanonicalInputGuard> guard_;
		std::unique_ptr<control_link_adapters::LocalWatchdog> watchdog_;
		std::unique_ptr<control_link_adapters::CanonicalEndpointTracker>
			endpoint_tracker_;
		rclcpp::Subscription<ControlCommand>::SharedPtr canonical_subscription_;
		rclcpp::Publisher<VehicleState>::SharedPtr state_publisher_;
		rclcpp::TimerBase::SharedPtr graph_timer_;
		rclcpp::TimerBase::SharedPtr state_timer_;
		std::optional<control_link_adapters::CanonicalPublisherKey>
			confirmed_gateway_publisher_;
		std::optional<ControlCommand> last_command_;
		control_link_adapters::CanonicalRejectReason last_reject_reason_{
			control_link_adapters::CanonicalRejectReason::kNone};
		std::string rmw_implementation_;
	};
} // namespace

int main(int argc, char **argv)
{
	rclcpp::init(argc, argv);
	try
	{
		auto node = std::make_shared<MockVehicleAdapter>();
		rclcpp::spin(node);
	}
	catch (const std::exception &exception)
	{
		RCLCPP_FATAL(
			rclcpp::get_logger("mock_vehicle_adapter"),
			"Mock vehicle adapter failed: %s",
			exception.what());
		rclcpp::shutdown();
		return 1;
	}
	rclcpp::shutdown();
	return 0;
}
