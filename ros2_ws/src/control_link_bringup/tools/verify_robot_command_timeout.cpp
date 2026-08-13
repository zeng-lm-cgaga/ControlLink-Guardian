#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <filesystem>
#include <limits>
#include <memory>
#include <stdexcept>
#include <string>
#include <thread>
#include <variant>

#include "control_link_contract/contract_bundle.hpp"
#include "control_link_contract/qos_factory.hpp"
#include "control_link_interfaces/msg/control_command.hpp"
#include "control_link_interfaces/msg/gateway_state.hpp"
#include "control_link_interfaces/msg/source_status.hpp"
#include "control_link_interfaces/msg/vehicle_state.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/executors/single_threaded_executor.hpp"
#include "rclcpp/rclcpp.hpp"

namespace
{
	using namespace std::chrono_literals;
	using ControlCommand = control_link_interfaces::msg::ControlCommand;
	using GatewayState = control_link_interfaces::msg::GatewayState;
	using Odometry = nav_msgs::msg::Odometry;
	using SourceStatus = control_link_interfaces::msg::SourceStatus;
	using SteadyTime = std::chrono::steady_clock::time_point;
	using Twist = geometry_msgs::msg::Twist;
	using TwistStamped = geometry_msgs::msg::TwistStamped;
	using VehicleState = control_link_interfaces::msg::VehicleState;
	constexpr char kNav2SourceId[] = "nav2";
	constexpr std::uint16_t kRequiredStableOutputSamples = 5U;

	enum class ScenarioPhase : std::uint8_t
	{
		kWarmup,
		kFault,
		kRecovery,
	};

	struct ScenarioEvidence
	{
		// 只保存观察事实，Gateway 的 freshness、lease 和恢复规则仍由产品代码判断
		ScenarioPhase phase{ScenarioPhase::kWarmup};
		bool publish_enabled{true};
		bool publication_stopped{false};
		bool publication_resumed{false};
		bool saw_initial_active_nav2{false};
		bool saw_initial_source_healthy{false};
		bool saw_initial_running_vehicle{false};
		bool saw_initial_canonical_motion{false};
		bool saw_initial_controller_motion{false};
		bool saw_initial_odometry_motion{false};
		bool saw_nav2_lease_expire{false};
		bool saw_gateway_no_source_state{false};
		bool saw_gateway_no_source{false};
		bool saw_standby_vehicle{false};
		bool saw_canonical_hold{false};
		bool saw_zero_controller_command{false};
		bool saw_stopped_odometry{false};
		bool saw_nav2_lease_restore{false};
		bool saw_recovering{false};
		bool saw_recovery_complete{false};
		bool saw_running_vehicle_after_recovery{false};
		bool saw_canonical_motion_after_recovery{false};
		bool saw_controller_motion_after_recovery{false};
		bool saw_odometry_motion_after_recovery{false};
		std::uint16_t consecutive_canonical_hold_samples{0U};
		std::uint16_t consecutive_zero_controller_samples{0U};
		std::uint16_t consecutive_stopped_odometry_samples{0U};
		std::uint16_t maximum_recovery_count{0U};
		std::uint64_t raw_publish_count{0U};
		std::uint64_t accepted_count_at_lease_expiry{0U};
		std::uint64_t accepted_count_after_recovery{0U};
		std::uint64_t source_sequence_at_lease_expiry{0U};
		std::uint64_t initial_transition_sequence{0U};
		std::uint64_t fault_transition_sequence{0U};
		std::uint64_t recovery_transition_sequence{0U};
		double command_age_ms_at_lease_expiry{0.0};
		SteadyTime last_raw_publish_at{};
		SteadyTime fault_last_raw_publish_at{};
		SteadyTime gateway_no_source_observed_at{};
		SteadyTime lease_expired_at{};
		SteadyTime publication_resumed_at{};
		SteadyTime lease_restored_at{};
		SteadyTime recovery_completed_at{};
	};

	const control_link_contract::ProfileCommon &profile_common(
		const control_link_contract::ProfileConfig &profile)
	{
		return std::visit(
			[](const auto &concrete_profile)
				-> const control_link_contract::ProfileCommon &
			{
				return concrete_profile.common;
			},
			profile);
	}

	template<typename Predicate>
	bool spin_until(
		rclcpp::executors::SingleThreadedExecutor &executor,
		SteadyTime deadline,
		Predicate predicate)
	{
		while (rclcpp::ok() && std::chrono::steady_clock::now() < deadline)
		{
			executor.spin_some();
			if (predicate())
			{
				return true;
			}
			std::this_thread::sleep_for(5ms);
		}

		executor.spin_some();
		return predicate();
	}

	std::chrono::milliseconds checked_milliseconds(
		std::uint64_t value,
		const char *field_name)
	{
		if (value > static_cast<std::uint64_t>(
				std::numeric_limits<std::int64_t>::max()))
		{
			throw std::overflow_error(
				std::string{field_name} + " exceeds chrono::milliseconds range");
		}
		return std::chrono::milliseconds{static_cast<std::int64_t>(value)};
	}

	std::int64_t elapsed_milliseconds(SteadyTime begin, SteadyTime end)
	{
		if (end < begin)
		{
			throw std::logic_error("steady observation time moved backwards");
		}
		return std::chrono::duration_cast<std::chrono::milliseconds>(
			end - begin).count();
	}

	bool nonzero_motion(double linear, double angular, double epsilon) noexcept
	{
		return std::abs(linear) > epsilon || std::abs(angular) > epsilon;
	}

	bool zero_motion(double linear, double angular, double epsilon) noexcept
	{
		return std::abs(linear) <= epsilon && std::abs(angular) <= epsilon;
	}

	int run_scenario(const rclcpp::Node::SharedPtr &node)
	{
		const auto profile_path = std::filesystem::path{
			node->declare_parameter<std::string>("profile_path", "")};
		const auto config_root = std::filesystem::path{
			node->declare_parameter<std::string>("config_root", "")};
		const auto timeout_ms = node->declare_parameter<std::int64_t>(
			"timeout_ms", 45000);
		const auto nav2_publish_hz = node->declare_parameter<double>(
			"nav2_publish_hz", 20.0);
		const auto nav2_linear_velocity = node->declare_parameter<double>(
			"nav2_linear_velocity_mps", 0.25);
		const auto motion_epsilon = node->declare_parameter<double>(
			"motion_epsilon", 0.01);
		if (profile_path.empty() || config_root.empty())
		{
			throw std::invalid_argument(
				"profile_path and config_root must be non-empty");
		}
		if (timeout_ms <= 0)
		{
			throw std::invalid_argument("timeout_ms must be positive");
		}
		if (!std::isfinite(nav2_publish_hz) || nav2_publish_hz < 20.0)
		{
			throw std::invalid_argument(
				"nav2_publish_hz must be finite and at least 20Hz");
		}
		if (!std::isfinite(motion_epsilon) || motion_epsilon <= 0.0)
		{
			throw std::invalid_argument(
				"motion_epsilon must be positive and finite");
		}

		const auto bundle = control_link_contract::load_contract_bundle(
			profile_path,
			config_root);
		const auto *robot_profile = std::get_if<control_link_contract::RobotProfile>(
			bundle->profile.get());
		if (robot_profile == nullptr)
		{
			throw std::invalid_argument(
				"command timeout scenario requires Robot Profile");
		}
		if (!std::isfinite(nav2_linear_velocity) ||
			std::abs(nav2_linear_velocity) >
			bundle->gateway_contract->limits.max_abs_linear_velocity_mps)
		{
			throw std::invalid_argument(
				"nav2 velocity must be finite and within the Gateway Contract limit");
		}

		const auto &common = profile_common(*bundle->profile);
		if (std::find(
				common.enabled_sources.begin(),
				common.enabled_sources.end(),
				kNav2SourceId) == common.enabled_sources.end())
		{
			throw std::invalid_argument("nav2 is not enabled by Robot Profile");
		}
		const auto nav2_ingress = common.ingress.find(kNav2SourceId);
		if (nav2_ingress == common.ingress.end() ||
			nav2_ingress->second.input_type != "geometry_msgs/msg/Twist")
		{
			throw std::invalid_argument(
				"Robot Profile requires a geometry_msgs/msg/Twist nav2 ingress");
		}
		const auto nav2_policy = bundle->source_policy->sources.find(kNav2SourceId);
		if (nav2_policy == bundle->source_policy->sources.end())
		{
			throw std::invalid_argument("nav2 is absent from SourcePolicy");
		}
		if (bundle->gateway_contract->gateway.recovery_valid_samples == 0U)
		{
			throw std::logic_error(
				"command timeout recovery requires a non-zero recovery sample count");
		}

		control_link_contract::QosFactory qos_factory{
			bundle->gateway_contract};
		const auto &gateway_state_contract =
			bundle->gateway_contract->state_topics.at("gateway_state");
		const auto &source_status_contract =
			bundle->gateway_contract->state_topics.at("source_status");
		const auto &vehicle_state_contract =
			bundle->gateway_contract->state_topics.at("vehicle_state");
		if (!gateway_state_contract.qos_profile.has_value() ||
			!source_status_contract.qos_profile.has_value() ||
			!vehicle_state_contract.qos_profile.has_value())
		{
			throw std::logic_error(
				"command timeout evidence Topics require Contract QoS profiles");
		}

		ScenarioEvidence evidence;
		auto gateway_state_subscription = node->create_subscription<GatewayState>(
			gateway_state_contract.topic,
			qos_factory.make(gateway_state_contract.qos_profile.value()),
			[&evidence](const GatewayState &state)
			{
				if (evidence.phase == ScenarioPhase::kWarmup &&
					state.state == GatewayState::ACTIVE &&
					state.active_source_id == kNav2SourceId)
				{
					evidence.saw_initial_active_nav2 = true;
					evidence.initial_transition_sequence = state.transition_sequence;
				}
				if (evidence.phase == ScenarioPhase::kFault &&
					state.state == GatewayState::SAFE_STOP &&
					state.reason_code == GatewayState::REASON_NO_QUALIFIED_SOURCE)
				{
					if (!evidence.saw_gateway_no_source_state)
					{
						evidence.gateway_no_source_observed_at =
							std::chrono::steady_clock::now();
					}
					evidence.saw_gateway_no_source_state = true;
					if (evidence.saw_nav2_lease_expire)
					{
						// 首次状态可早于 lease 失效，完整场景仍要求两类超时证据同时成立
						evidence.saw_gateway_no_source = true;
						evidence.fault_transition_sequence = state.transition_sequence;
					}
				}
				if (evidence.phase == ScenarioPhase::kRecovery &&
					evidence.saw_nav2_lease_restore &&
					state.state == GatewayState::RECOVERING)
				{
					evidence.saw_recovering = true;
					evidence.maximum_recovery_count = std::max(
						evidence.maximum_recovery_count,
						state.recovery_valid_count);
				}
				if (evidence.phase == ScenarioPhase::kRecovery &&
					evidence.saw_nav2_lease_restore &&
					state.state == GatewayState::ACTIVE &&
					state.reason_code == GatewayState::REASON_RECOVERY_COMPLETE &&
					state.active_source_id == kNav2SourceId)
				{
					if (!evidence.saw_recovery_complete)
					{
						evidence.recovery_completed_at =
							std::chrono::steady_clock::now();
					}
					evidence.saw_recovery_complete = true;
					evidence.recovery_transition_sequence = state.transition_sequence;
				}
			});
		auto source_status_subscription = node->create_subscription<SourceStatus>(
			source_status_contract.topic,
			qos_factory.make(source_status_contract.qos_profile.value()),
			[&evidence](const SourceStatus &status)
			{
				if (status.source_id != kNav2SourceId)
				{
					return;
				}
				if (evidence.phase == ScenarioPhase::kWarmup &&
					status.command_valid && status.lease_valid)
				{
					evidence.saw_initial_source_healthy = true;
				}
				if (evidence.phase == ScenarioPhase::kFault &&
					evidence.publication_stopped && status.command_valid &&
					!status.lease_valid)
				{
					// command_valid 仍为 true，证明 latest-valid snapshot 和 publisher generation 未消失
					if (!evidence.saw_nav2_lease_expire)
					{
						evidence.lease_expired_at = std::chrono::steady_clock::now();
						evidence.accepted_count_at_lease_expiry = status.accepted_count;
						evidence.source_sequence_at_lease_expiry =
							status.last_source_sequence;
						evidence.command_age_ms_at_lease_expiry = status.command_age_ms;
					}
					evidence.saw_nav2_lease_expire = true;
				}
				if (evidence.phase == ScenarioPhase::kRecovery &&
					evidence.publication_resumed && status.command_valid &&
					status.lease_valid &&
					status.accepted_count > evidence.accepted_count_at_lease_expiry)
				{
					if (!evidence.saw_nav2_lease_restore)
					{
						evidence.lease_restored_at = std::chrono::steady_clock::now();
					}
					evidence.saw_nav2_lease_restore = true;
					evidence.accepted_count_after_recovery = std::max(
						evidence.accepted_count_after_recovery,
						status.accepted_count);
				}
			});
		auto vehicle_state_subscription = node->create_subscription<VehicleState>(
			vehicle_state_contract.topic,
			qos_factory.make(vehicle_state_contract.qos_profile.value()),
			[&evidence](const VehicleState &state)
			{
				if (evidence.phase == ScenarioPhase::kWarmup &&
					state.state == VehicleState::RUNNING &&
					state.fault_code == VehicleState::FAULT_NONE)
				{
					evidence.saw_initial_running_vehicle = true;
				}
				if (evidence.phase == ScenarioPhase::kFault &&
					evidence.saw_gateway_no_source &&
					state.state == VehicleState::STANDBY &&
					state.fault_code == VehicleState::FAULT_NONE)
				{
					evidence.saw_standby_vehicle = true;
				}
				if (evidence.phase == ScenarioPhase::kRecovery &&
					evidence.saw_recovery_complete &&
					state.state == VehicleState::RUNNING &&
					state.fault_code == VehicleState::FAULT_NONE)
				{
					evidence.saw_running_vehicle_after_recovery = true;
				}
			});
		auto canonical_subscription = node->create_subscription<ControlCommand>(
			bundle->gateway_contract->output.topic,
			qos_factory.make(bundle->gateway_contract->output.qos_profile),
			[&evidence, motion_epsilon](const ControlCommand &command)
			{
				const bool moving = command.mode == ControlCommand::MODE_NORMAL &&
					command.source_id == kNav2SourceId &&
					nonzero_motion(
						command.linear_velocity_mps,
						command.angular_velocity_radps,
						motion_epsilon);
				if (evidence.phase == ScenarioPhase::kWarmup && moving)
				{
					evidence.saw_initial_canonical_motion = true;
				}
				if (evidence.phase == ScenarioPhase::kFault &&
					evidence.saw_gateway_no_source)
				{
					const bool hold = command.mode == ControlCommand::MODE_HOLD &&
						zero_motion(
							command.linear_velocity_mps,
							command.angular_velocity_radps,
							motion_epsilon);
					evidence.consecutive_canonical_hold_samples = hold ?
						evidence.consecutive_canonical_hold_samples + 1U : 0U;
					evidence.saw_canonical_hold =
						evidence.consecutive_canonical_hold_samples >=
						kRequiredStableOutputSamples;
				}
				if (evidence.phase == ScenarioPhase::kRecovery &&
					evidence.saw_recovery_complete && moving)
				{
					evidence.saw_canonical_motion_after_recovery = true;
				}
			});

		const auto controller_qos = rclcpp::QoS{rclcpp::KeepLast{1}}
			.reliable()
			.durability_volatile();
		auto controller_subscription = node->create_subscription<TwistStamped>(
			robot_profile->adapter.controller_output_topic,
			controller_qos,
			[&evidence, motion_epsilon](const TwistStamped &command)
			{
				const bool moving = nonzero_motion(
					command.twist.linear.x,
					command.twist.angular.z,
					motion_epsilon);
				if (evidence.phase == ScenarioPhase::kWarmup && moving)
				{
					evidence.saw_initial_controller_motion = true;
				}
				if (evidence.phase == ScenarioPhase::kFault &&
					evidence.saw_gateway_no_source)
				{
					evidence.consecutive_zero_controller_samples = !moving ?
						evidence.consecutive_zero_controller_samples + 1U : 0U;
					evidence.saw_zero_controller_command =
						evidence.consecutive_zero_controller_samples >=
						kRequiredStableOutputSamples;
				}
				if (evidence.phase == ScenarioPhase::kRecovery &&
					evidence.saw_recovery_complete && moving)
				{
					evidence.saw_controller_motion_after_recovery = true;
				}
			});
		auto odometry_subscription = node->create_subscription<Odometry>(
			robot_profile->adapter.odometry_topic,
			rclcpp::SensorDataQoS(),
			[&evidence, motion_epsilon](const Odometry &odometry)
			{
				const bool moving = nonzero_motion(
					odometry.twist.twist.linear.x,
					odometry.twist.twist.angular.z,
					motion_epsilon);
				if (evidence.phase == ScenarioPhase::kWarmup && moving)
				{
					evidence.saw_initial_odometry_motion = true;
				}
				if (evidence.phase == ScenarioPhase::kFault &&
					evidence.saw_gateway_no_source)
				{
					evidence.consecutive_stopped_odometry_samples = !moving ?
						evidence.consecutive_stopped_odometry_samples + 1U : 0U;
					evidence.saw_stopped_odometry =
						evidence.consecutive_stopped_odometry_samples >=
						kRequiredStableOutputSamples;
				}
				if (evidence.phase == ScenarioPhase::kRecovery &&
					evidence.saw_recovery_complete && moving)
				{
					evidence.saw_odometry_motion_after_recovery = true;
				}
			});

		const auto raw_input_qos = rclcpp::QoS{rclcpp::KeepLast{10}}
			.reliable()
			.durability_volatile();
		auto nav2_publisher = node->create_publisher<Twist>(
			nav2_ingress->second.input_topic,
			raw_input_qos);
		Twist nav2_twist;
		nav2_twist.linear.x = nav2_linear_velocity;
		auto nav2_timer = node->create_wall_timer(
			std::chrono::duration_cast<std::chrono::nanoseconds>(
				std::chrono::duration<double>{1.0 / nav2_publish_hz}),
			[nav2_publisher, nav2_twist, &evidence]()
			{
				// 故障期仅停止消息，Publisher 与 DDS endpoint 必须始终存活
				if (!evidence.publish_enabled)
				{
					return;
				}

				nav2_publisher->publish(nav2_twist);
				evidence.last_raw_publish_at = std::chrono::steady_clock::now();
				if (evidence.raw_publish_count <
					std::numeric_limits<std::uint64_t>::max())
				{
					evidence.raw_publish_count += 1U;
				}
			});

		rclcpp::executors::SingleThreadedExecutor executor;
		executor.add_node(node);
		const auto scenario_deadline = std::chrono::steady_clock::now() +
			std::chrono::milliseconds{timeout_ms};
		const auto initial_ready = [&evidence, &nav2_publisher]()
		{
			return nav2_publisher->get_subscription_count() > 0U &&
				evidence.raw_publish_count > 0U &&
				evidence.saw_initial_active_nav2 &&
				evidence.saw_initial_source_healthy &&
				evidence.saw_initial_running_vehicle &&
				evidence.saw_initial_canonical_motion &&
				evidence.saw_initial_controller_motion &&
				evidence.saw_initial_odometry_motion;
		};
		if (!spin_until(executor, scenario_deadline, initial_ready))
		{
			throw std::runtime_error(
				"timed out waiting for the moving ACTIVE/nav2 Robot chain");
		}

		evidence.phase = ScenarioPhase::kFault;
		evidence.publish_enabled = false;
		evidence.publication_stopped = true;
		evidence.fault_last_raw_publish_at = evidence.last_raw_publish_at;
		const auto lease_timeout = checked_milliseconds(
			nav2_policy->second.lease_timeout_ms,
			"sources.nav2.lease_timeout_ms");
		const auto fault_deadline = std::min(
			scenario_deadline,
			evidence.fault_last_raw_publish_at + lease_timeout + 3s);
		const auto lease_expired_with_endpoint = [&evidence, &nav2_publisher]()
		{
			return evidence.saw_nav2_lease_expire &&
				nav2_publisher->get_subscription_count() > 0U;
		};
		if (!spin_until(executor, fault_deadline, lease_expired_with_endpoint))
		{
			throw std::runtime_error(
				"raw nav2 publication stop did not expire the source lease while the endpoint remained matched");
		}
		const auto raw_subscription_count_at_fault =
			nav2_publisher->get_subscription_count();

		const auto fault_complete = [&evidence]()
		{
			return evidence.saw_gateway_no_source &&
				evidence.saw_standby_vehicle &&
				evidence.saw_canonical_hold &&
				evidence.saw_zero_controller_command &&
				evidence.saw_stopped_odometry;
		};
		if (!spin_until(executor, fault_deadline, fault_complete))
		{
			throw std::runtime_error(
				"nav2 lease expiry did not close Gateway NO_QUALIFIED_SOURCE, HOLD and stopped Robot evidence");
		}
		if (evidence.fault_transition_sequence <=
			evidence.initial_transition_sequence)
		{
			throw std::runtime_error(
				"Gateway transition_sequence did not advance for command timeout");
		}

		evidence.phase = ScenarioPhase::kRecovery;
		evidence.publication_resumed = true;
		evidence.publication_resumed_at = std::chrono::steady_clock::now();
		evidence.publish_enabled = true;
		const auto recovery_complete = [&evidence, &nav2_publisher]()
		{
			return nav2_publisher->get_subscription_count() > 0U &&
				evidence.saw_nav2_lease_restore &&
				evidence.saw_recovering &&
				evidence.saw_recovery_complete &&
				evidence.saw_running_vehicle_after_recovery &&
				evidence.saw_canonical_motion_after_recovery &&
				evidence.saw_controller_motion_after_recovery &&
				evidence.saw_odometry_motion_after_recovery;
		};
		if (!spin_until(executor, scenario_deadline, recovery_complete))
		{
			throw std::runtime_error(
				"raw nav2 publication resume did not complete the source recovery epoch and Robot motion chain");
		}

		if (evidence.recovery_transition_sequence <=
			evidence.fault_transition_sequence)
		{
			throw std::runtime_error(
				"Gateway transition_sequence did not advance across command recovery");
		}
		const auto expected_maximum_recovery_count = static_cast<std::uint16_t>(
			bundle->gateway_contract->gateway.recovery_valid_samples - 1U);
		if (evidence.maximum_recovery_count != expected_maximum_recovery_count)
		{
			throw std::runtime_error(
				"Gateway RECOVERING evidence did not expose every pre-completion recovery count");
		}
		if (evidence.accepted_count_after_recovery <=
			evidence.accepted_count_at_lease_expiry)
		{
			throw std::runtime_error(
				"nav2 lease recovered without a newly accepted source command");
		}

		const auto lease_expiry_ms = elapsed_milliseconds(
			evidence.fault_last_raw_publish_at,
			evidence.lease_expired_at);
		const auto no_source_observed_ms = elapsed_milliseconds(
			evidence.fault_last_raw_publish_at,
			evidence.gateway_no_source_observed_at);
		const auto lease_restore_ms = elapsed_milliseconds(
			evidence.publication_resumed_at,
			evidence.lease_restored_at);
		const auto recovery_ms = elapsed_milliseconds(
			evidence.publication_resumed_at,
			evidence.recovery_completed_at);
		const auto lease_timeout_ms = static_cast<std::int64_t>(
			nav2_policy->second.lease_timeout_ms);
		const auto command_timeout_ms = static_cast<double>(
			bundle->gateway_contract->gateway.command_timeout_ms);
		if (lease_expiry_ms < lease_timeout_ms ||
			lease_expiry_ms > lease_timeout_ms + 500)
		{
			throw std::runtime_error(
				"nav2 lease observation fell outside the Contract timeout and bounded observation allowance");
		}
		if (evidence.command_age_ms_at_lease_expiry <= command_timeout_ms)
		{
			throw std::runtime_error(
				"nav2 lease expired before the latest command became stale");
		}
		RCLCPP_INFO(
			node->get_logger(),
			"ROBOT_COMMAND_TIMEOUT_SUCCEEDED no_source_observed_ms=%ld lease_expiry_ms=%ld "
			"lease_restore_ms=%ld recovery_ms=%ld command_age_ms=%.1f "
			"raw_subscription_count=%lu raw_publish_count=%lu source_sequence=%lu "
			"accepted_delta=%lu max_recovery_count=%u stable_output_samples=%u "
			"fault_transition=%lu recovery_transition=%lu",
			static_cast<long>(no_source_observed_ms),
			static_cast<long>(lease_expiry_ms),
			static_cast<long>(lease_restore_ms),
			static_cast<long>(recovery_ms),
			evidence.command_age_ms_at_lease_expiry,
			static_cast<unsigned long>(raw_subscription_count_at_fault),
			static_cast<unsigned long>(evidence.raw_publish_count),
			static_cast<unsigned long>(evidence.source_sequence_at_lease_expiry),
			static_cast<unsigned long>(
				evidence.accepted_count_after_recovery -
				evidence.accepted_count_at_lease_expiry),
			static_cast<unsigned int>(evidence.maximum_recovery_count),
			static_cast<unsigned int>(kRequiredStableOutputSamples),
			static_cast<unsigned long>(evidence.fault_transition_sequence),
			static_cast<unsigned long>(evidence.recovery_transition_sequence));

		(void)gateway_state_subscription;
		(void)source_status_subscription;
		(void)vehicle_state_subscription;
		(void)canonical_subscription;
		(void)controller_subscription;
		(void)odometry_subscription;
		(void)nav2_timer;
		return 0;
	}
} // namespace

int main(int argc, char **argv)
{
	rclcpp::init(argc, argv);
	int result = 1;
	try
	{
		auto node = std::make_shared<rclcpp::Node>(
			"verify_robot_command_timeout");
		result = run_scenario(node);
	}
	catch (const std::exception &exception)
	{
		RCLCPP_ERROR(
			rclcpp::get_logger("verify_robot_command_timeout"),
			"Robot command timeout scenario failed: %s",
			exception.what());
	}
	rclcpp::shutdown();
	return result;
}
