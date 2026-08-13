#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <filesystem>
#include <memory>
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
#include "lifecycle_msgs/msg/state.hpp"
#include "lifecycle_msgs/srv/get_state.hpp"
#include "nav2_msgs/srv/manage_lifecycle_nodes.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/executors/single_threaded_executor.hpp"
#include "rclcpp/rclcpp.hpp"

namespace
{
	using namespace std::chrono_literals;
	using ControlCommand = control_link_interfaces::msg::ControlCommand;
	using GatewayState = control_link_interfaces::msg::GatewayState;
	using GetState = lifecycle_msgs::srv::GetState;
	using ManageLifecycleNodes = nav2_msgs::srv::ManageLifecycleNodes;
	using Odometry = nav_msgs::msg::Odometry;
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
		ScenarioPhase phase{ScenarioPhase::kWarmup};
		bool saw_initial_active_nav2{false};
		bool saw_initial_running_vehicle{false};
		bool saw_initial_canonical_motion{false};
		bool saw_initial_controller_motion{false};
		bool saw_initial_odometry_motion{false};
		bool saw_tf_stale_vehicle_fault{false};
		bool saw_gateway_vehicle_fault{false};
		bool saw_canonical_hold{false};
		bool saw_zero_controller_command{false};
		bool saw_stopped_odometry{false};
		bool saw_healthy_vehicle_after_reactivation{false};
		bool saw_recovering{false};
		bool saw_recovery_complete{false};
		bool saw_canonical_motion_after_recovery{false};
		bool saw_controller_motion_after_recovery{false};
		bool saw_odometry_motion_after_recovery{false};
		std::uint16_t consecutive_canonical_hold_samples{0U};
		std::uint16_t consecutive_zero_controller_samples{0U};
		std::uint16_t consecutive_stopped_odometry_samples{0U};
		std::uint16_t maximum_recovery_count{0U};
		std::uint64_t fault_transition_sequence{0U};
		std::uint64_t recovery_transition_sequence{0U};
		SteadyTime fault_started_at{};
		SteadyTime vehicle_fault_observed_at{};
		SteadyTime gateway_fault_observed_at{};
		SteadyTime odometry_stopped_at{};
		SteadyTime recovery_started_at{};
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

	std::uint8_t read_lifecycle_state(
		const rclcpp::Client<GetState>::SharedPtr &client,
		rclcpp::executors::SingleThreadedExecutor &executor)
	{
		auto future = client->async_send_request(
			std::make_shared<GetState::Request>());
		require_future(
			executor,
			future,
			2s,
			"AMCL GetState request timed out");
		return future.get()->current_state.id;
	}

	void request_lifecycle_manager_command(
		const rclcpp::Client<ManageLifecycleNodes>::SharedPtr &client,
		rclcpp::executors::SingleThreadedExecutor &executor,
		std::uint8_t command)
	{
		auto request = std::make_shared<ManageLifecycleNodes::Request>();
		request->command = command;
		auto future = client->async_send_request(request);
		require_future(
			executor,
			future,
			5s,
			"localization LifecycleManager request timed out");
		if (!future.get()->success)
		{
			throw std::runtime_error(
				"localization LifecycleManager rejected the requested command");
		}
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
		const auto amcl_fqn = node->declare_parameter<std::string>(
			"amcl_fqn", "/amcl");
		const auto localization_manager_fqn = node->declare_parameter<std::string>(
			"localization_manager_fqn", "/lifecycle_manager_localization");
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
		if (amcl_fqn.empty() || amcl_fqn.front() != '/')
		{
			throw std::invalid_argument("amcl_fqn must be an absolute node FQN");
		}
		if (localization_manager_fqn.empty() ||
			localization_manager_fqn.front() != '/')
		{
			throw std::invalid_argument(
				"localization_manager_fqn must be an absolute node FQN");
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
			throw std::invalid_argument("motion_epsilon must be positive and finite");
		}

		const auto bundle = control_link_contract::load_contract_bundle(
			profile_path,
			config_root);
		const auto *robot_profile = std::get_if<control_link_contract::RobotProfile>(
			bundle->profile.get());
		if (robot_profile == nullptr)
		{
			throw std::invalid_argument("TF stale scenario requires Robot Profile");
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
				"TF stale evidence Topics require Contract QoS profiles");
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
				}
				if (evidence.phase == ScenarioPhase::kFault &&
					evidence.saw_tf_stale_vehicle_fault &&
					state.state == GatewayState::SAFE_STOP &&
					state.reason_code == GatewayState::REASON_VEHICLE_FAULT)
				{
					if (!evidence.saw_gateway_vehicle_fault)
					{
						evidence.gateway_fault_observed_at =
							std::chrono::steady_clock::now();
					}
					evidence.saw_gateway_vehicle_fault = true;
					evidence.fault_transition_sequence = state.transition_sequence;
				}
				if (evidence.phase == ScenarioPhase::kRecovery &&
					state.state == GatewayState::RECOVERING)
				{
					evidence.saw_recovering = true;
					evidence.maximum_recovery_count = std::max(
						evidence.maximum_recovery_count,
						state.recovery_valid_count);
				}
				if (evidence.phase == ScenarioPhase::kRecovery &&
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
				if (evidence.phase == ScenarioPhase::kFault &&
					state.state == VehicleState::SAFE_STOP &&
					state.fault_code == VehicleState::FAULT_ROBOT_TF_STALE)
				{
					if (!evidence.saw_tf_stale_vehicle_fault)
					{
						evidence.vehicle_fault_observed_at =
							std::chrono::steady_clock::now();
					}
					evidence.saw_tf_stale_vehicle_fault = true;
				}
				if (evidence.phase == ScenarioPhase::kRecovery &&
					state.fault_code == VehicleState::FAULT_NONE &&
					(state.state == VehicleState::STANDBY ||
					state.state == VehicleState::RUNNING))
				{
					evidence.saw_healthy_vehicle_after_reactivation = true;
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
					evidence.saw_gateway_vehicle_fault)
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
					evidence.saw_tf_stale_vehicle_fault)
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
					evidence.saw_tf_stale_vehicle_fault)
				{
					const bool was_stopped = evidence.saw_stopped_odometry;
					evidence.consecutive_stopped_odometry_samples = !moving ?
						evidence.consecutive_stopped_odometry_samples + 1U : 0U;
					evidence.saw_stopped_odometry =
						evidence.consecutive_stopped_odometry_samples >=
						kRequiredStableOutputSamples;
					if (!was_stopped && evidence.saw_stopped_odometry)
					{
						evidence.odometry_stopped_at =
							std::chrono::steady_clock::now();
					}
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

		auto get_state_client = node->create_client<GetState>(
			amcl_fqn + "/get_state");
		auto manage_localization_client = node->create_client<ManageLifecycleNodes>(
			localization_manager_fqn + "/manage_nodes");
		rclcpp::executors::SingleThreadedExecutor executor;
		executor.add_node(node);
		const auto scenario_deadline = std::chrono::steady_clock::now() +
			std::chrono::milliseconds{timeout_ms};

		const auto initial_ready = [&evidence, &nav2_publisher,
			&get_state_client, &manage_localization_client]()
		{
			return nav2_publisher->get_subscription_count() > 0U &&
				get_state_client->service_is_ready() &&
				manage_localization_client->service_is_ready() &&
				evidence.saw_initial_active_nav2 &&
				evidence.saw_initial_running_vehicle &&
				evidence.saw_initial_canonical_motion &&
				evidence.saw_initial_controller_motion &&
				evidence.saw_initial_odometry_motion;
		};
		if (!spin_until(executor, scenario_deadline, initial_ready))
		{
			throw std::runtime_error(
				"timed out waiting for moving ACTIVE/nav2 Robot chain and AMCL services");
		}
		if (read_lifecycle_state(get_state_client, executor) !=
			lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE)
		{
			throw std::runtime_error("AMCL is not ACTIVE before TF stale injection");
		}

		bool localization_paused = false;
		try
		{
			evidence.phase = ScenarioPhase::kFault;
			evidence.fault_started_at = std::chrono::steady_clock::now();
			request_lifecycle_manager_command(
				manage_localization_client,
				executor,
				ManageLifecycleNodes::Request::PAUSE);
			localization_paused = true;
			if (read_lifecycle_state(get_state_client, executor) !=
				lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE)
			{
				throw std::runtime_error(
					"AMCL did not enter INACTIVE after fault injection");
			}

			const auto fault_complete = [&evidence]()
			{
				return evidence.saw_tf_stale_vehicle_fault &&
					evidence.saw_gateway_vehicle_fault &&
					evidence.saw_canonical_hold &&
					evidence.saw_zero_controller_command &&
					evidence.saw_stopped_odometry;
			};
			const auto fault_deadline = std::min(
				scenario_deadline,
				evidence.fault_started_at + 3s);
			if (!spin_until(executor, fault_deadline, fault_complete))
			{
				throw std::runtime_error(
					"TF stale did not close VehicleState, Gateway HOLD and odometry stop evidence");
			}

			evidence.phase = ScenarioPhase::kRecovery;
			evidence.recovery_started_at = std::chrono::steady_clock::now();
			request_lifecycle_manager_command(
				manage_localization_client,
				executor,
				ManageLifecycleNodes::Request::RESUME);
			localization_paused = false;
			if (read_lifecycle_state(get_state_client, executor) !=
				lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE)
			{
				throw std::runtime_error("AMCL did not return to ACTIVE");
			}

			const auto recovery_complete = [&evidence]()
			{
				return evidence.saw_healthy_vehicle_after_reactivation &&
					evidence.saw_recovering &&
					evidence.saw_recovery_complete &&
					evidence.saw_canonical_motion_after_recovery &&
					evidence.saw_controller_motion_after_recovery &&
					evidence.saw_odometry_motion_after_recovery;
			};
			if (!spin_until(executor, scenario_deadline, recovery_complete))
			{
				throw std::runtime_error(
					"TF recovery did not complete healthy VehicleState and Gateway recovery epoch");
			}
		}
		catch (...)
		{
			// 场景失败也通过原 Lifecycle owner 恢复 localization 节点与 bond
			if (localization_paused && rclcpp::ok())
			{
				try
				{
					request_lifecycle_manager_command(
						manage_localization_client,
						executor,
						ManageLifecycleNodes::Request::RESUME);
				}
				catch (const std::exception &exception)
				{
					RCLCPP_ERROR(
						node->get_logger(),
						"Failed to resume localization after scenario error: %s",
						exception.what());
				}
			}
			throw;
		}

		const auto vehicle_fault_ms = elapsed_milliseconds(
			evidence.fault_started_at,
			evidence.vehicle_fault_observed_at);
		const auto gateway_fault_ms = elapsed_milliseconds(
			evidence.fault_started_at,
			evidence.gateway_fault_observed_at);
		const auto stop_ms = elapsed_milliseconds(
			evidence.fault_started_at,
			evidence.odometry_stopped_at);
		const auto recovery_ms = elapsed_milliseconds(
			evidence.recovery_started_at,
			evidence.recovery_completed_at);
		if (evidence.recovery_transition_sequence <=
			evidence.fault_transition_sequence)
		{
			throw std::runtime_error(
				"Gateway transition_sequence did not advance across TF recovery");
		}

		RCLCPP_INFO(
			node->get_logger(),
			"ROBOT_TF_STALE_SUCCEEDED vehicle_fault_ms=%ld gateway_fault_ms=%ld "
			"odom_stop_ms=%ld recovery_ms=%ld max_recovery_count=%u "
			"stable_output_samples=%u fault_transition=%lu recovery_transition=%lu",
			static_cast<long>(vehicle_fault_ms),
			static_cast<long>(gateway_fault_ms),
			static_cast<long>(stop_ms),
			static_cast<long>(recovery_ms),
			static_cast<unsigned int>(evidence.maximum_recovery_count),
			static_cast<unsigned int>(kRequiredStableOutputSamples),
			static_cast<unsigned long>(evidence.fault_transition_sequence),
			static_cast<unsigned long>(evidence.recovery_transition_sequence));

		(void)gateway_state_subscription;
		(void)vehicle_state_subscription;
		(void)canonical_subscription;
		(void)controller_subscription;
		(void)odometry_subscription;
		(void)nav2_timer;
		return 0;
	}
}  // namespace

int main(int argc, char **argv)
{
	rclcpp::init(argc, argv);
	int result = 1;
	try
	{
		auto node = std::make_shared<rclcpp::Node>("verify_robot_tf_stale");
		result = run_scenario(node);
	}
	catch (const std::exception &exception)
	{
		RCLCPP_ERROR(
			rclcpp::get_logger("verify_robot_tf_stale"),
			"Robot TF stale scenario failed: %s",
			exception.what());
	}
	rclcpp::shutdown();
	return result;
}
