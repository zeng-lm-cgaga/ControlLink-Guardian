#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <filesystem>
#include <limits>
#include <memory>
#include <optional>
#include <stdexcept>
#include <string>
#include <thread>
#include <variant>

#include "control_link_contract/contract_bundle.hpp"
#include "control_link_contract/qos_factory.hpp"
#include "control_link_interfaces/msg/control_command.hpp"
#include "control_link_interfaces/msg/gateway_state.hpp"
#include "control_link_interfaces/msg/vehicle_state.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/executors/single_threaded_executor.hpp"
#include "rclcpp/rclcpp.hpp"
#include "ros_gz_interfaces/srv/control_world.hpp"
#include "rosgraph_msgs/msg/clock.hpp"

namespace
{
	using namespace std::chrono_literals;
	using Clock = rosgraph_msgs::msg::Clock;
	using ControlCommand = control_link_interfaces::msg::ControlCommand;
	using ControlWorld = ros_gz_interfaces::srv::ControlWorld;
	using GatewayState = control_link_interfaces::msg::GatewayState;
	using Odometry = nav_msgs::msg::Odometry;
	using SteadyTime = std::chrono::steady_clock::time_point;
	using Twist = geometry_msgs::msg::Twist;
	using TwistStamped = geometry_msgs::msg::TwistStamped;
	using VehicleState = control_link_interfaces::msg::VehicleState;
	constexpr char kNav2SourceId[] = "nav2";
	constexpr std::uint16_t kRequiredStableOutputSamples = 5U;
	constexpr std::int64_t kNanosecondsPerSecond = 1'000'000'000LL;

	enum class ScenarioPhase : std::uint8_t
	{
		kWarmup,
		kFault,
		kRecovery,
	};

	struct ScenarioEvidence
	{
		ScenarioPhase phase{ScenarioPhase::kWarmup};
		bool saw_clock_progress{false};
		bool clock_moved_backward{false};
		bool saw_initial_active_nav2{false};
		bool saw_initial_running_vehicle{false};
		bool saw_initial_canonical_motion{false};
		bool saw_initial_controller_motion{false};
		bool saw_initial_odometry_motion{false};
		bool saw_clock_stall{false};
		bool saw_gateway_clock_fault{false};
		bool saw_canonical_hold{false};
		bool saw_zero_controller_command{false};
		bool saw_clock_resumed{false};
		bool saw_healthy_vehicle_after_resume{false};
		bool saw_recovering{false};
		bool saw_recovery_complete{false};
		bool saw_canonical_motion_after_recovery{false};
		bool saw_controller_motion_after_recovery{false};
		bool saw_odometry_motion_after_recovery{false};
		std::uint16_t consecutive_canonical_hold_samples{0U};
		std::uint16_t consecutive_zero_controller_samples{0U};
		std::uint16_t maximum_recovery_count{0U};
		std::uint64_t clock_sample_count{0U};
		std::uint64_t initial_transition_sequence{0U};
		std::uint64_t fault_transition_sequence{0U};
		std::uint64_t recovery_transition_sequence{0U};
		std::optional<std::int64_t> latest_clock_ns;
		std::optional<std::int64_t> paused_clock_ns;
		SteadyTime last_clock_progress_at{};
		SteadyTime pause_acknowledged_at{};
		SteadyTime clock_stall_observed_at{};
		SteadyTime gateway_fault_observed_at{};
		SteadyTime unpause_acknowledged_at{};
		SteadyTime clock_resumed_at{};
		SteadyTime recovery_completed_at{};
	};

	const control_link_contract::ProfileCommon & profile_common(
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

	template<typename FutureT>
	void require_future(
		rclcpp::executors::SingleThreadedExecutor &executor,
		const FutureT &future,
		std::chrono::milliseconds timeout,
		const char *failure_message)
	{
		if (executor.spin_until_future_complete(future, timeout) !=
			rclcpp::FutureReturnCode::SUCCESS)
		{
			throw std::runtime_error(failure_message);
		}
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

	std::optional<std::int64_t> clock_nanoseconds(const Clock &clock) noexcept
	{
		if (clock.clock.nanosec >= static_cast<std::uint32_t>(kNanosecondsPerSecond) ||
			clock.clock.sec < 0)
		{
			return std::nullopt;
		}
		return static_cast<std::int64_t>(clock.clock.sec) *
			kNanosecondsPerSecond + static_cast<std::int64_t>(clock.clock.nanosec);
	}

	bool nonzero_motion(double linear, double angular, double epsilon) noexcept
	{
		return std::abs(linear) > epsilon || std::abs(angular) > epsilon;
	}

	bool zero_motion(double linear, double angular, double epsilon) noexcept
	{
		return std::abs(linear) <= epsilon && std::abs(angular) <= epsilon;
	}

	void set_world_paused(
		const rclcpp::Client<ControlWorld>::SharedPtr &client,
		rclcpp::executors::SingleThreadedExecutor &executor,
		bool paused)
	{
		auto request = std::make_shared<ControlWorld::Request>();
		request->world_control.pause = paused;
		auto future = client->async_send_request(request);
		require_future(
			executor,
			future,
			5s,
			paused ?
				"Gazebo world pause request timed out" :
				"Gazebo world unpause request timed out");
		if (!future.get()->success)
		{
			throw std::runtime_error(
				paused ?
					"Gazebo rejected the world pause request" :
					"Gazebo rejected the world unpause request");
		}
	}

	int run_scenario(const rclcpp::Node::SharedPtr &node)
	{
		const auto profile_path = std::filesystem::path{
			node->declare_parameter<std::string>("profile_path", "")};
		const auto config_root = std::filesystem::path{
			node->declare_parameter<std::string>("config_root", "")};
		const auto world_control_service = node->declare_parameter<std::string>(
			"world_control_service", "/world/control_link_flat/control");
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
		if (world_control_service.empty() || world_control_service.front() != '/')
		{
			throw std::invalid_argument(
				"world_control_service must be an absolute ROS service name");
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
			throw std::invalid_argument("Clock stall scenario requires Robot Profile");
		}
		if (!std::isfinite(nav2_linear_velocity) ||
			std::abs(nav2_linear_velocity) >
			bundle->gateway_contract->limits.max_abs_linear_velocity_mps)
		{
			throw std::invalid_argument(
				"nav2 velocity must be finite and within the Gateway Contract limit");
		}

		const auto &common = profile_common(*bundle->profile);
		if (common.clock_mode != control_link_contract::ClockMode::kSim ||
			!common.use_sim_time)
		{
			throw std::invalid_argument(
				"Clock stall scenario requires sim clock and use_sim_time=true");
		}
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

		control_link_contract::QosFactory qos_factory{
			bundle->gateway_contract};
		const auto &gateway_state_contract =
			bundle->gateway_contract->state_topics.at("gateway_state");
		const auto &vehicle_state_contract =
			bundle->gateway_contract->state_topics.at("vehicle_state");
		if (!gateway_state_contract.qos_profile.has_value() ||
			!vehicle_state_contract.qos_profile.has_value())
		{
			throw std::logic_error(
				"Clock stall evidence Topics require Contract QoS profiles");
		}

		const auto clock_stall_window = checked_milliseconds(
			bundle->gateway_contract->gateway.ros_clock_stall_timeout_ms,
			"gateway.ros_clock_stall_timeout_ms") + 100ms;
		ScenarioEvidence evidence;
		auto clock_subscription = node->create_subscription<Clock>(
			"/clock",
			rclcpp::ClockQoS(),
			[&evidence](const Clock &clock)
			{
				const auto sample_ns = clock_nanoseconds(clock);
				if (!sample_ns.has_value())
				{
					evidence.clock_moved_backward = true;
					return;
				}

				const auto now = std::chrono::steady_clock::now();
				if (!evidence.latest_clock_ns.has_value() ||
					sample_ns.value() > evidence.latest_clock_ns.value())
				{
					if (evidence.latest_clock_ns.has_value())
					{
						evidence.saw_clock_progress = true;
					}
					evidence.last_clock_progress_at = now;
				}
				else if (sample_ns.value() < evidence.latest_clock_ns.value())
				{
					evidence.clock_moved_backward = true;
				}

				evidence.latest_clock_ns = sample_ns;
				if (evidence.clock_sample_count <
					std::numeric_limits<std::uint64_t>::max())
				{
					evidence.clock_sample_count += 1U;
				}
				if (evidence.phase == ScenarioPhase::kRecovery &&
					evidence.paused_clock_ns.has_value() &&
					sample_ns.value() > evidence.paused_clock_ns.value())
				{
					if (!evidence.saw_clock_resumed)
					{
						evidence.clock_resumed_at = now;
					}
					evidence.saw_clock_resumed = true;
				}
			});
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
					evidence.saw_clock_stall &&
					state.state == GatewayState::SAFE_STOP &&
					state.reason_code == GatewayState::REASON_CLOCK_INVALID)
				{
					if (!evidence.saw_gateway_clock_fault)
					{
						evidence.gateway_fault_observed_at =
							std::chrono::steady_clock::now();
					}
					evidence.saw_gateway_clock_fault = true;
					evidence.fault_transition_sequence = state.transition_sequence;
				}
				if (evidence.phase == ScenarioPhase::kRecovery &&
					evidence.saw_clock_resumed &&
					state.state == GatewayState::RECOVERING)
				{
					evidence.saw_recovering = true;
					evidence.maximum_recovery_count = std::max(
						evidence.maximum_recovery_count,
						state.recovery_valid_count);
				}
				if (evidence.phase == ScenarioPhase::kRecovery &&
					evidence.saw_clock_resumed &&
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
				if (evidence.phase == ScenarioPhase::kRecovery &&
					evidence.saw_clock_resumed &&
					state.fault_code == VehicleState::FAULT_NONE &&
					(state.state == VehicleState::STANDBY ||
					state.state == VehicleState::RUNNING))
				{
					evidence.saw_healthy_vehicle_after_resume = true;
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
					evidence.saw_gateway_clock_fault)
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
					evidence.saw_gateway_clock_fault)
				{
					const bool zero = !moving;
					evidence.consecutive_zero_controller_samples = zero ?
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
			[nav2_publisher, nav2_twist]()
			{
				nav2_publisher->publish(nav2_twist);
			});

		auto world_control_client = node->create_client<ControlWorld>(
			world_control_service);
		rclcpp::executors::SingleThreadedExecutor executor;
		executor.add_node(node);
		const auto scenario_deadline = std::chrono::steady_clock::now() +
			std::chrono::milliseconds{timeout_ms};

		const auto initial_ready = [&evidence, &nav2_publisher,
			&world_control_client]()
		{
			return nav2_publisher->get_subscription_count() > 0U &&
				world_control_client->service_is_ready() &&
				evidence.saw_clock_progress &&
				evidence.saw_initial_active_nav2 &&
				evidence.saw_initial_running_vehicle &&
				evidence.saw_initial_canonical_motion &&
				evidence.saw_initial_controller_motion &&
				evidence.saw_initial_odometry_motion;
		};
		if (!spin_until(executor, scenario_deadline, initial_ready))
		{
			throw std::runtime_error(
				"timed out waiting for moving ACTIVE/nav2 Robot chain, /clock and Gazebo control service");
		}
		if (evidence.clock_moved_backward || !evidence.latest_clock_ns.has_value())
		{
			throw std::runtime_error(
				"/clock was invalid before the Clock stall injection");
		}

		bool world_may_be_paused = false;
		try
		{
			evidence.phase = ScenarioPhase::kFault;
			world_may_be_paused = true;
			set_world_paused(world_control_client, executor, true);
			evidence.pause_acknowledged_at = std::chrono::steady_clock::now();

			const auto clock_stalled = [&evidence, clock_stall_window]()
			{
				if (!evidence.latest_clock_ns.has_value() ||
					evidence.clock_moved_backward)
				{
					return false;
				}
				const auto now = std::chrono::steady_clock::now();
				const auto no_progress_since = std::max(
					evidence.pause_acknowledged_at,
					evidence.last_clock_progress_at);
				if (now - no_progress_since < clock_stall_window)
				{
					return false;
				}

				evidence.saw_clock_stall = true;
				evidence.paused_clock_ns = evidence.latest_clock_ns;
				evidence.clock_stall_observed_at = now;
				return true;
			};
			const auto fault_deadline = std::min(
				scenario_deadline,
				evidence.pause_acknowledged_at + 3s);
			if (!spin_until(executor, fault_deadline, clock_stalled))
			{
				throw std::runtime_error(
					"Gazebo pause did not produce a stable /clock stall window");
			}

			const auto fault_complete = [&evidence]()
			{
				return evidence.saw_gateway_clock_fault &&
					evidence.saw_canonical_hold &&
					evidence.saw_zero_controller_command;
			};
			if (!spin_until(executor, fault_deadline, fault_complete))
			{
				throw std::runtime_error(
					"Clock stall did not close Gateway CLOCK_INVALID, canonical HOLD and controller zero evidence");
			}
			if (evidence.fault_transition_sequence <=
				evidence.initial_transition_sequence)
			{
				throw std::runtime_error(
					"Gateway transition_sequence did not advance for the Clock fault");
			}

			evidence.phase = ScenarioPhase::kRecovery;
			set_world_paused(world_control_client, executor, false);
			world_may_be_paused = false;
			evidence.unpause_acknowledged_at = std::chrono::steady_clock::now();

			const auto recovery_complete = [&evidence]()
			{
				return evidence.saw_clock_resumed &&
					evidence.saw_healthy_vehicle_after_resume &&
					evidence.saw_recovering &&
					evidence.saw_recovery_complete &&
					evidence.saw_canonical_motion_after_recovery &&
					evidence.saw_controller_motion_after_recovery &&
					evidence.saw_odometry_motion_after_recovery;
			};
			if (!spin_until(executor, scenario_deadline, recovery_complete))
			{
				throw std::runtime_error(
					"Clock resume did not complete the healthy VehicleState and Gateway recovery epoch");
			}
		}
		catch (...)
		{
			// service 结果不确定时也尝试 unpause，避免失败场景把共享 Gazebo world 留在暂停态
			if (world_may_be_paused && rclcpp::ok())
			{
				try
				{
					set_world_paused(world_control_client, executor, false);
				}
				catch (const std::exception &exception)
				{
					RCLCPP_ERROR(
						node->get_logger(),
						"Failed to unpause Gazebo after scenario error: %s",
						exception.what());
				}
			}
			throw;
		}

		if (evidence.clock_moved_backward)
		{
			throw std::runtime_error(
				"/clock moved backwards during the Clock stall scenario");
		}
		if (evidence.recovery_transition_sequence <=
			evidence.fault_transition_sequence)
		{
			throw std::runtime_error(
				"Gateway transition_sequence did not advance across Clock recovery");
		}

		const auto clock_stall_ms = elapsed_milliseconds(
			evidence.pause_acknowledged_at,
			evidence.clock_stall_observed_at);
		const auto gateway_fault_ms = elapsed_milliseconds(
			evidence.pause_acknowledged_at,
			evidence.gateway_fault_observed_at);
		const auto clock_resume_ms = elapsed_milliseconds(
			evidence.unpause_acknowledged_at,
			evidence.clock_resumed_at);
		const auto recovery_ms = elapsed_milliseconds(
			evidence.unpause_acknowledged_at,
			evidence.recovery_completed_at);
		RCLCPP_INFO(
			node->get_logger(),
			"ROBOT_CLOCK_STALL_SUCCEEDED clock_stall_ms=%ld gateway_fault_ms=%ld "
			"clock_resume_ms=%ld recovery_ms=%ld max_recovery_count=%u "
			"stable_output_samples=%u clock_samples=%lu stopped_clock_ns=%ld "
			"fault_transition=%lu recovery_transition=%lu",
			static_cast<long>(clock_stall_ms),
			static_cast<long>(gateway_fault_ms),
			static_cast<long>(clock_resume_ms),
			static_cast<long>(recovery_ms),
			static_cast<unsigned int>(evidence.maximum_recovery_count),
			static_cast<unsigned int>(kRequiredStableOutputSamples),
			static_cast<unsigned long>(evidence.clock_sample_count),
			static_cast<long>(evidence.paused_clock_ns.value()),
			static_cast<unsigned long>(evidence.fault_transition_sequence),
			static_cast<unsigned long>(evidence.recovery_transition_sequence));

		(void)clock_subscription;
		(void)gateway_state_subscription;
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
		auto node = std::make_shared<rclcpp::Node>("verify_robot_clock_stall");
		result = run_scenario(node);
	}
	catch (const std::exception &exception)
	{
		RCLCPP_ERROR(
			rclcpp::get_logger("verify_robot_clock_stall"),
			"Robot Clock stall scenario failed: %s",
			exception.what());
	}
	rclcpp::shutdown();
	return result;
}
