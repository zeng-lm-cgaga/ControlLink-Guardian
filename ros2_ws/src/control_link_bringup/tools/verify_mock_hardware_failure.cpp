#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <filesystem>
#include <future>
#include <limits>
#include <map>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <string>
#include <thread>
#include <variant>

#include "control_link_contract/contract_bundle.hpp"
#include "control_link_contract/qos_factory.hpp"
#include "control_link_interfaces/msg/control_command.hpp"
#include "control_link_interfaces/msg/gateway_state.hpp"
#include "control_link_interfaces/msg/vehicle_state.hpp"
#include "controller_manager_msgs/srv/list_hardware_components.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "lifecycle_msgs/msg/state.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/executors/single_threaded_executor.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

namespace
{
	using namespace std::chrono_literals;
	using ControlCommand = control_link_interfaces::msg::ControlCommand;
	using GatewayState = control_link_interfaces::msg::GatewayState;
	using ListHardwareComponents =
		controller_manager_msgs::srv::ListHardwareComponents;
	using Odometry = nav_msgs::msg::Odometry;
	using Twist = geometry_msgs::msg::Twist;
	using TwistStamped = geometry_msgs::msg::TwistStamped;
	using VehicleState = control_link_interfaces::msg::VehicleState;
	constexpr char kNav2SourceId[] = "nav2";
	constexpr std::uint16_t kRequiredStableOutputSamples = 5U;

	struct ScenarioEvidence
	{
		// 本结构只保存 ROS Graph 上的观察事实，不重新实现 Guardian 或 adapter 判断
		bool initial_ready{false};
		bool saw_initial_active_nav2{false};
		bool saw_initial_running_vehicle{false};
		bool saw_initial_canonical_motion{false};
		bool saw_initial_controller_motion{false};
		bool saw_joint_position_advance{false};
		bool saw_initial_odometry_motion{false};
		bool saw_vehicle_platform_fault{false};
		bool saw_gateway_safe_stop{false};
		bool saw_canonical_hold{false};
		bool saw_zero_controller_command{false};
		std::uint16_t consecutive_canonical_hold_samples{0U};
		std::uint16_t consecutive_zero_controller_samples{0U};
		std::uint16_t observed_vehicle_fault{VehicleState::FAULT_NONE};
		std::uint64_t raw_publish_count{0U};
		std::uint64_t initial_transition_sequence{0U};
		std::uint64_t fault_transition_sequence{0U};
		std::map<std::string, double> initial_joint_positions;
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
		std::chrono::steady_clock::time_point deadline,
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

	bool nonzero_motion(double linear, double angular, double epsilon) noexcept
	{
		return std::abs(linear) > epsilon || std::abs(angular) > epsilon;
	}

	bool zero_motion(double linear, double angular, double epsilon) noexcept
	{
		return std::abs(linear) <= epsilon && std::abs(angular) <= epsilon;
	}

	bool is_robot_platform_fault(std::uint16_t fault_code) noexcept
	{
		return fault_code == VehicleState::FAULT_ROBOT_HARDWARE_INACTIVE;
	}

	std::string initial_chain_observation(
		const ScenarioEvidence &evidence,
		std::size_t raw_subscription_count)
	{
		std::ostringstream stream;
		stream << std::boolalpha
			<< "raw_subscription_count=" << raw_subscription_count
			<< ", raw_publish_count=" << evidence.raw_publish_count
			<< ", gateway_active_nav2=" << evidence.saw_initial_active_nav2
			<< ", vehicle_running=" << evidence.saw_initial_running_vehicle
			<< ", canonical_motion=" << evidence.saw_initial_canonical_motion
			<< ", controller_motion=" << evidence.saw_initial_controller_motion
			<< ", joint_position_advance=" << evidence.saw_joint_position_advance
			<< ", odometry_motion=" << evidence.saw_initial_odometry_motion;
		return stream.str();
	}

	std::string fault_chain_observation(const ScenarioEvidence &evidence)
	{
		std::ostringstream stream;
		stream << std::boolalpha
			<< "vehicle_platform_fault=" << evidence.saw_vehicle_platform_fault
			<< ", gateway_safe_stop=" << evidence.saw_gateway_safe_stop
			<< ", canonical_hold=" << evidence.saw_canonical_hold
			<< ", controller_zero=" << evidence.saw_zero_controller_command
			<< ", vehicle_fault=" << evidence.observed_vehicle_fault
			<< ", canonical_hold_samples="
			<< evidence.consecutive_canonical_hold_samples
			<< ", controller_zero_samples="
			<< evidence.consecutive_zero_controller_samples
			<< ", initial_transition=" << evidence.initial_transition_sequence
			<< ", fault_transition=" << evidence.fault_transition_sequence;
		return stream.str();
	}

	std::uint8_t read_hardware_state(
		const rclcpp::Client<ListHardwareComponents>::SharedPtr &client,
		rclcpp::executors::SingleThreadedExecutor &executor,
		const std::string &component_name)
	{
		if (!client->wait_for_service(5s))
		{
			throw std::runtime_error(
				"controller manager list_hardware_components service is unavailable");
		}

		auto future = client->async_send_request(
			std::make_shared<ListHardwareComponents::Request>());
		if (executor.spin_until_future_complete(future, 1s) !=
			rclcpp::FutureReturnCode::SUCCESS)
		{
			throw std::runtime_error(
				"list_hardware_components request timed out");
		}

		const auto response = future.get();
		const auto component = std::find_if(
			response->component.begin(),
			response->component.end(),
			[&component_name](const auto &candidate)
			{
				return candidate.name == component_name;
			});
		if (component == response->component.end())
		{
			throw std::runtime_error(
				"mock hardware component is absent from controller manager");
		}
		return component->state.id;
	}

	int run_scenario(const rclcpp::Node::SharedPtr &node)
	{
		const auto profile_path = std::filesystem::path{
			node->declare_parameter<std::string>("profile_path", "")};
		const auto config_root = std::filesystem::path{
			node->declare_parameter<std::string>("config_root", "")};
		const auto failure_mode = node->declare_parameter<std::string>(
			"failure_mode", "");
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
		if (failure_mode != "read" && failure_mode != "write")
		{
			throw std::invalid_argument("failure_mode must be read or write");
		}
		if (timeout_ms <= 0 || !std::isfinite(nav2_publish_hz) ||
			nav2_publish_hz < 20.0 || !std::isfinite(motion_epsilon) ||
			motion_epsilon <= 0.0)
		{
			throw std::invalid_argument(
				"timeout and motion observation parameters are invalid");
		}

		const auto bundle = control_link_contract::load_contract_bundle(
			profile_path,
			config_root);
		const auto *robot_profile = std::get_if<control_link_contract::RobotProfile>(
			bundle->profile.get());
		if (robot_profile == nullptr)
		{
			throw std::invalid_argument(
				"mock hardware failure scenario requires Robot Profile");
		}
		if (!std::isfinite(nav2_linear_velocity) ||
			std::abs(nav2_linear_velocity) >
			bundle->gateway_contract->limits.max_abs_linear_velocity_mps)
		{
			throw std::invalid_argument(
				"nav2 velocity must be finite and within the Gateway Contract limit");
		}

		const auto &common = profile_common(*bundle->profile);
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
				"mock hardware evidence Topics require Contract QoS profiles");
		}

		ScenarioEvidence evidence;
		auto gateway_state_subscription = node->create_subscription<GatewayState>(
			gateway_state_contract.topic,
			qos_factory.make(gateway_state_contract.qos_profile.value()),
			[&evidence](const GatewayState &state)
			{
				if (state.state == GatewayState::ACTIVE &&
					state.active_source_id == kNav2SourceId)
				{
					evidence.saw_initial_active_nav2 = true;
					evidence.initial_transition_sequence = state.transition_sequence;
				}
					if (evidence.initial_ready &&
						state.state == GatewayState::SAFE_STOP &&
						state.reason_code == GatewayState::REASON_VEHICLE_FAULT)
					{
						evidence.saw_gateway_safe_stop = true;
						evidence.fault_transition_sequence = state.transition_sequence;
					}
			});
		auto vehicle_state_subscription = node->create_subscription<VehicleState>(
			vehicle_state_contract.topic,
			qos_factory.make(vehicle_state_contract.qos_profile.value()),
			[&evidence](const VehicleState &state)
			{
				if (state.state == VehicleState::RUNNING &&
					state.fault_code == VehicleState::FAULT_NONE)
				{
					evidence.saw_initial_running_vehicle = true;
				}
				if (evidence.initial_ready &&
					state.state == VehicleState::SAFE_STOP &&
					is_robot_platform_fault(state.fault_code))
				{
					evidence.saw_vehicle_platform_fault = true;
					evidence.observed_vehicle_fault = state.fault_code;
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
				if (moving)
				{
					evidence.saw_initial_canonical_motion = true;
				}
				if (evidence.initial_ready && evidence.saw_gateway_safe_stop)
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
				if (moving)
				{
					evidence.saw_initial_controller_motion = true;
				}
				if (evidence.initial_ready && evidence.saw_gateway_safe_stop)
				{
					evidence.consecutive_zero_controller_samples = !moving ?
						evidence.consecutive_zero_controller_samples + 1U : 0U;
					evidence.saw_zero_controller_command =
						evidence.consecutive_zero_controller_samples >=
						kRequiredStableOutputSamples;
				}
			});
		auto odometry_subscription = node->create_subscription<Odometry>(
			robot_profile->adapter.odometry_topic,
			rclcpp::SensorDataQoS(),
			[&evidence, motion_epsilon](const Odometry &odometry)
			{
				if (nonzero_motion(
						odometry.twist.twist.linear.x,
						odometry.twist.twist.angular.z,
						motion_epsilon))
				{
					evidence.saw_initial_odometry_motion = true;
				}
			});
		auto joint_state_subscription =
			node->create_subscription<sensor_msgs::msg::JointState>(
				"/joint_states",
				rclcpp::SensorDataQoS(),
				[&evidence, motion_epsilon](
					const sensor_msgs::msg::JointState &state)
				{
					const auto count = std::min(state.name.size(), state.position.size());
					for (std::size_t index = 0U; index < count; ++index)
					{
						if (!std::isfinite(state.position[index]))
						{
							continue;
						}
						const auto initial = evidence.initial_joint_positions.find(
							state.name[index]);
						if (initial == evidence.initial_joint_positions.end())
						{
							evidence.initial_joint_positions.emplace(
								state.name[index], state.position[index]);
							continue;
						}
						evidence.saw_joint_position_advance =
							evidence.saw_joint_position_advance ||
							std::abs(state.position[index] - initial->second) > motion_epsilon;
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
				nav2_publisher->publish(nav2_twist);
				if (evidence.raw_publish_count <
					std::numeric_limits<std::uint64_t>::max())
				{
					evidence.raw_publish_count += 1U;
				}
			});
		auto hardware_client = node->create_client<ListHardwareComponents>(
			robot_profile->adapter.controller_manager_fqn +
			"/list_hardware_components");

		rclcpp::executors::SingleThreadedExecutor executor;
		executor.add_node(node);
		const auto scenario_deadline = std::chrono::steady_clock::now() +
			std::chrono::milliseconds{timeout_ms};
		const auto initial_hardware_state = read_hardware_state(
			hardware_client,
			executor,
			robot_profile->adapter.hardware_component_name);
		if (initial_hardware_state != lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE)
		{
			throw std::runtime_error(
				"mock hardware was not ACTIVE when the failure verifier started");
		}

		const auto initial_ready = [&evidence, &nav2_publisher]()
		{
			return nav2_publisher->get_subscription_count() > 0U &&
				evidence.raw_publish_count > 0U &&
				evidence.saw_initial_active_nav2 &&
				evidence.saw_initial_running_vehicle &&
				evidence.saw_initial_canonical_motion &&
				evidence.saw_initial_controller_motion &&
				evidence.saw_joint_position_advance &&
				evidence.saw_initial_odometry_motion;
		};
		if (!spin_until(executor, scenario_deadline, initial_ready))
		{
			throw std::runtime_error(
				"timed out waiting for the moving ACTIVE mock execution chain: " +
				initial_chain_observation(
					evidence,
					nav2_publisher->get_subscription_count()));
		}
		evidence.initial_ready = true;

		std::uint8_t failed_hardware_state = initial_hardware_state;
		while (rclcpp::ok() && std::chrono::steady_clock::now() < scenario_deadline)
		{
			failed_hardware_state = read_hardware_state(
				hardware_client,
				executor,
				robot_profile->adapter.hardware_component_name);
			if (failed_hardware_state !=
				lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE)
			{
				break;
			}
			std::this_thread::sleep_for(20ms);
		}
		if (failed_hardware_state ==
			lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE)
		{
			throw std::runtime_error(
				"injected mock hardware failure did not leave ACTIVE state");
		}

		const auto fault_complete = [&evidence]()
		{
			return evidence.saw_vehicle_platform_fault &&
				evidence.saw_gateway_safe_stop &&
				evidence.saw_canonical_hold &&
				evidence.saw_zero_controller_command;
		};
		if (!spin_until(executor, scenario_deadline, fault_complete))
		{
			throw std::runtime_error(
				"mock hardware failure did not close VehicleState, Gateway SAFE_STOP and HOLD evidence: " +
				fault_chain_observation(evidence));
		}
		if (evidence.fault_transition_sequence <=
			evidence.initial_transition_sequence)
		{
			throw std::runtime_error(
				"Gateway transition_sequence did not advance for mock hardware failure");
		}

		RCLCPP_INFO(
			node->get_logger(),
			"MOCK_HARDWARE_FAILURE_SUCCEEDED mode=%s hardware_state=%u vehicle_fault=%u "
			"raw_publish_count=%lu stable_output_samples=%u initial_transition=%lu "
			"fault_transition=%lu",
			failure_mode.c_str(),
			static_cast<unsigned int>(failed_hardware_state),
			static_cast<unsigned int>(evidence.observed_vehicle_fault),
			static_cast<unsigned long>(evidence.raw_publish_count),
			static_cast<unsigned int>(kRequiredStableOutputSamples),
			static_cast<unsigned long>(evidence.initial_transition_sequence),
			static_cast<unsigned long>(evidence.fault_transition_sequence));

		(void)gateway_state_subscription;
		(void)vehicle_state_subscription;
		(void)canonical_subscription;
		(void)controller_subscription;
		(void)odometry_subscription;
		(void)joint_state_subscription;
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
		auto node = std::make_shared<rclcpp::Node>(
			"verify_mock_hardware_failure");
		result = run_scenario(node);
	}
	catch (const std::exception &exception)
	{
		RCLCPP_ERROR(
			rclcpp::get_logger("verify_mock_hardware_failure"),
			"Mock hardware failure scenario failed: %s",
			exception.what());
	}
	rclcpp::shutdown();
	return result;
}
