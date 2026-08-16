#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <filesystem>
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
#include "controller_manager_msgs/srv/configure_controller.hpp"
#include "controller_manager_msgs/srv/list_controllers.hpp"
#include "controller_manager_msgs/srv/load_controller.hpp"
#include "controller_manager_msgs/srv/switch_controller.hpp"
#include "controller_manager_msgs/srv/unload_controller.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/executors/single_threaded_executor.hpp"
#include "rclcpp/rclcpp.hpp"

namespace
{
	using namespace std::chrono_literals;
	using ControlCommand = control_link_interfaces::msg::ControlCommand;
	using ConfigureController = controller_manager_msgs::srv::ConfigureController;
	using GatewayState = control_link_interfaces::msg::GatewayState;
	using ListControllers = controller_manager_msgs::srv::ListControllers;
	using LoadController = controller_manager_msgs::srv::LoadController;
	using Odometry = nav_msgs::msg::Odometry;
	using SteadyTime = std::chrono::steady_clock::time_point;
	using SwitchController = controller_manager_msgs::srv::SwitchController;
	using Twist = geometry_msgs::msg::Twist;
	using TwistStamped = geometry_msgs::msg::TwistStamped;
	using UnloadController = controller_manager_msgs::srv::UnloadController;
	using VehicleState = control_link_interfaces::msg::VehicleState;
	using EndpointGid = std::array<std::uint8_t, RMW_GID_STORAGE_SIZE>;
	constexpr char kNav2SourceId[] = "nav2";
	constexpr std::uint16_t kRequiredStableOutputSamples = 5U;

	enum class ScenarioPhase : std::uint8_t
	{
		kWarmup,
		kFault,
		kRecovery,
	};

	enum class ManagedControllerState : std::uint8_t
	{
		kActive,
		kInactive,
		kUnconfigured,
		kUnloaded,
	};

	struct ScenarioEvidence
	{
		ScenarioPhase phase{ScenarioPhase::kWarmup};
		bool controller_graph_loss_confirmed{false};
		bool controller_active_confirmed{false};
		bool saw_initial_active_nav2{false};
		bool saw_initial_running_vehicle{false};
		bool saw_initial_canonical_motion{false};
		bool saw_initial_controller_motion{false};
		bool saw_initial_odometry_motion{false};
		bool saw_controller_timeout_fault{false};
		bool saw_gateway_vehicle_fault{false};
		bool saw_canonical_hold{false};
		bool saw_zero_adapter_output{false};
		bool saw_healthy_vehicle_after_reactivation{false};
		bool saw_recovering{false};
		bool saw_recovery_complete{false};
		bool saw_canonical_motion_after_recovery{false};
		bool saw_controller_motion_after_recovery{false};
		bool saw_odometry_motion_after_recovery{false};
		std::uint16_t consecutive_canonical_hold_samples{0U};
		std::uint16_t consecutive_zero_adapter_samples{0U};
		std::uint16_t maximum_recovery_count{0U};
		std::uint64_t fault_transition_sequence{0U};
		std::uint64_t recovery_transition_sequence{0U};
		SteadyTime controller_lost_at{};
		SteadyTime vehicle_fault_observed_at{};
		SteadyTime gateway_fault_observed_at{};
		SteadyTime reactivation_started_at{};
		SteadyTime new_generation_observed_at{};
		SteadyTime healthy_vehicle_observed_at{};
		SteadyTime recovery_completed_at{};
	};

	std::string endpoint_fqn(const rclcpp::TopicEndpointInfo &endpoint)
	{
		const auto node_namespace = endpoint.node_namespace();
		if (node_namespace.empty() || node_namespace == "/")
		{
			return "/" + endpoint.node_name();
		}
		if (node_namespace.back() == '/')
		{
			return node_namespace + endpoint.node_name();
		}
		return node_namespace + "/" + endpoint.node_name();
	}

	std::optional<EndpointGid> controller_endpoint_gid(
		const rclcpp::Node::SharedPtr &node,
		const std::string &topic,
		const std::string &expected_fqn,
		const std::string &expected_type)
	{
		std::optional<EndpointGid> result;
		for (const auto &endpoint : node->get_subscriptions_info_by_topic(topic))
		{
			if (endpoint.endpoint_type() != rclcpp::EndpointType::Subscription ||
				endpoint_fqn(endpoint) != expected_fqn ||
				endpoint.topic_type() != expected_type)
			{
				continue;
			}
			if (result.has_value())
			{
				throw std::runtime_error(
					"controller Graph contains multiple matching subscriptions");
			}
			result = endpoint.endpoint_gid();
		}
		return result;
	}

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

	void switch_controller(
		const rclcpp::Client<SwitchController>::SharedPtr &client,
		rclcpp::executors::SingleThreadedExecutor &executor,
		const std::string &controller_name,
		bool activate)
	{
		auto request = std::make_shared<SwitchController::Request>();
		if (activate)
		{
			request->activate_controllers = {controller_name};
		}
		else
		{
			request->deactivate_controllers = {controller_name};
		}
		request->strictness = SwitchController::Request::STRICT;
		request->activate_asap = true;
		request->timeout.sec = 2;

		auto future = client->async_send_request(request);
		require_future(
			executor,
			future,
			5s,
			"controller switch request timed out");
		if (!future.get()->ok)
		{
			throw std::runtime_error(
				activate ?
				"controller manager rejected activation" :
				"controller manager rejected deactivation");
		}
	}

	std::optional<std::string> read_controller_state(
		const rclcpp::Client<ListControllers>::SharedPtr &client,
		rclcpp::executors::SingleThreadedExecutor &executor,
		const std::string &controller_name)
	{
		auto future = client->async_send_request(
			std::make_shared<ListControllers::Request>());
		require_future(
			executor,
			future,
			2s,
			"ListControllers request timed out");

		// Response 必须由局部 shared_ptr 持有，range-for 不能引用 future.get() 的临时对象
		const auto response = future.get();
		const auto controller = std::find_if(
			response->controller.begin(),
			response->controller.end(),
			[&controller_name](const auto &candidate)
			{
				return candidate.name == controller_name;
			});
		if (controller == response->controller.end())
		{
			return std::nullopt;
		}
		return controller->state;
	}

	template<typename ServiceT>
	void request_controller_operation(
		const typename rclcpp::Client<ServiceT>::SharedPtr &client,
		rclcpp::executors::SingleThreadedExecutor &executor,
		const std::string &controller_name,
		const char *timeout_message,
		const char *rejected_message)
	{
		auto request = std::make_shared<typename ServiceT::Request>();
		request->name = controller_name;
		auto future = client->async_send_request(request);
		require_future(executor, future, 5s, timeout_message);
		if (!future.get()->ok)
		{
			throw std::runtime_error(rejected_message);
		}
	}

	void restore_controller(
		const rclcpp::Client<LoadController>::SharedPtr &load_client,
		const rclcpp::Client<ConfigureController>::SharedPtr &configure_client,
		const rclcpp::Client<SwitchController>::SharedPtr &switch_client,
		rclcpp::executors::SingleThreadedExecutor &executor,
		const std::string &controller_name,
		ManagedControllerState &state)
	{
		if (state == ManagedControllerState::kUnloaded)
		{
			request_controller_operation<LoadController>(
				load_client,
				executor,
				controller_name,
				"LoadController request timed out",
				"controller manager rejected loading controller");
			state = ManagedControllerState::kUnconfigured;
		}
		if (state == ManagedControllerState::kUnconfigured)
		{
			request_controller_operation<ConfigureController>(
				configure_client,
				executor,
				controller_name,
				"ConfigureController request timed out",
				"controller manager rejected configuring controller");
			state = ManagedControllerState::kInactive;
		}
		if (state == ManagedControllerState::kInactive)
		{
			switch_controller(
				switch_client,
				executor,
				controller_name,
				true);
			state = ManagedControllerState::kActive;
		}
	}

	std::string controller_name_from_fqn(const std::string &controller_fqn)
	{
		if (controller_fqn.empty() || controller_fqn.front() != '/')
		{
			throw std::invalid_argument(
				"controller_node_fqn must be an absolute node FQN");
		}
		const auto separator = controller_fqn.find_last_of('/');
		const auto name = controller_fqn.substr(separator + 1U);
		if (name.empty())
		{
			throw std::invalid_argument(
				"controller_node_fqn must end with a controller name");
		}
		return name;
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
			throw std::invalid_argument("motion_epsilon must be positive and finite");
		}

		const auto bundle = control_link_contract::load_contract_bundle(
			profile_path,
			config_root);
		const auto *robot_profile = std::get_if<control_link_contract::RobotProfile>(
			bundle->profile.get());
		if (robot_profile == nullptr)
		{
			throw std::invalid_argument(
				"controller loss scenario requires Robot Profile");
		}
		const auto &controller_manager_fqn =
			robot_profile->adapter.controller_manager_fqn;
		if (!std::isfinite(nav2_linear_velocity) ||
			std::abs(nav2_linear_velocity) >
			bundle->gateway_contract->limits.max_abs_linear_velocity_mps)
		{
			throw std::invalid_argument(
				"nav2 velocity must be finite and within the Gateway Contract limit");
		}

		const auto controller_name = controller_name_from_fqn(
			robot_profile->adapter.controller_node_fqn);
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
				"controller loss evidence Topics require Contract QoS profiles");
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
					evidence.saw_controller_timeout_fault &&
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
					evidence.saw_healthy_vehicle_after_reactivation &&
					state.state == GatewayState::RECOVERING)
				{
					evidence.saw_recovering = true;
					evidence.maximum_recovery_count = std::max(
						evidence.maximum_recovery_count,
						state.recovery_valid_count);
				}
				if (evidence.phase == ScenarioPhase::kRecovery &&
					evidence.saw_recovering &&
					state.state == GatewayState::ACTIVE &&
					state.reason_code == GatewayState::REASON_RECOVERY_COMPLETE &&
					state.active_source_id == kNav2SourceId &&
					state.transition_sequence > evidence.fault_transition_sequence)
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
				evidence.controller_graph_loss_confirmed &&
					state.state == VehicleState::SAFE_STOP &&
					state.fault_code ==
						VehicleState::FAULT_ROBOT_CONTROLLER_STATE_TIMEOUT)
				{
					if (!evidence.saw_controller_timeout_fault)
					{
						evidence.vehicle_fault_observed_at =
							std::chrono::steady_clock::now();
					}
					evidence.saw_controller_timeout_fault = true;
				}
				if (evidence.phase == ScenarioPhase::kRecovery &&
					evidence.controller_active_confirmed &&
					state.fault_code == VehicleState::FAULT_NONE &&
					(state.state == VehicleState::STANDBY ||
					state.state == VehicleState::RUNNING))
				{
					if (!evidence.saw_healthy_vehicle_after_reactivation)
					{
						evidence.healthy_vehicle_observed_at =
							std::chrono::steady_clock::now();
					}
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
					evidence.saw_controller_timeout_fault)
				{
					evidence.consecutive_zero_adapter_samples = !moving ?
						evidence.consecutive_zero_adapter_samples + 1U : 0U;
					evidence.saw_zero_adapter_output =
						evidence.consecutive_zero_adapter_samples >=
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

		auto switch_client = node->create_client<SwitchController>(
			controller_manager_fqn + "/switch_controller");
		auto list_client = node->create_client<ListControllers>(
			controller_manager_fqn + "/list_controllers");
		auto load_client = node->create_client<LoadController>(
			controller_manager_fqn + "/load_controller");
		auto configure_client = node->create_client<ConfigureController>(
			controller_manager_fqn + "/configure_controller");
		auto unload_client = node->create_client<UnloadController>(
			controller_manager_fqn + "/unload_controller");
		rclcpp::executors::SingleThreadedExecutor executor;
		executor.add_node(node);
		const auto scenario_deadline = std::chrono::steady_clock::now() +
			std::chrono::milliseconds{timeout_ms};

		const auto initial_ready = [&evidence, &nav2_publisher,
			&switch_client, &list_client, &load_client, &configure_client,
			&unload_client]()
		{
			return nav2_publisher->get_subscription_count() > 0U &&
				switch_client->service_is_ready() &&
				list_client->service_is_ready() &&
				load_client->service_is_ready() &&
				configure_client->service_is_ready() &&
				unload_client->service_is_ready() &&
				evidence.saw_initial_active_nav2 &&
				evidence.saw_initial_running_vehicle &&
				evidence.saw_initial_canonical_motion &&
				evidence.saw_initial_controller_motion &&
				evidence.saw_initial_odometry_motion;
		};
		if (!spin_until(executor, scenario_deadline, initial_ready))
		{
			throw std::runtime_error(
				"timed out waiting for moving ACTIVE/nav2 Robot chain and controller services");
		}
		if (read_controller_state(list_client, executor, controller_name) !=
			std::optional<std::string>{"active"})
		{
			throw std::runtime_error(
				"diff drive controller is not active before fault injection");
		}
		const auto initial_controller_gid = controller_endpoint_gid(
			node,
			robot_profile->adapter.controller_output_topic,
			robot_profile->adapter.controller_node_fqn,
			robot_profile->adapter.controller_command_type);
		if (!initial_controller_gid.has_value())
		{
			throw std::runtime_error(
				"active controller has no matching command subscription in ROS Graph");
		}

		ManagedControllerState managed_state{ManagedControllerState::kActive};
		try
		{
			evidence.phase = ScenarioPhase::kFault;
			switch_controller(
				switch_client,
				executor,
				controller_name,
				false);
			managed_state = ManagedControllerState::kInactive;
			if (read_controller_state(list_client, executor, controller_name) !=
				std::optional<std::string>{"inactive"})
			{
				throw std::runtime_error(
					"diff drive controller did not enter inactive state");
			}
			request_controller_operation<UnloadController>(
				unload_client,
				executor,
				controller_name,
				"UnloadController request timed out",
				"controller manager rejected unloading controller");
			managed_state = ManagedControllerState::kUnloaded;
			if (read_controller_state(list_client, executor, controller_name).has_value())
			{
				throw std::runtime_error(
					"controller manager still lists controller after unload");
			}
			const auto graph_removed = [&node, &robot_profile]()
			{
				return !controller_endpoint_gid(
					node,
					robot_profile->adapter.controller_output_topic,
					robot_profile->adapter.controller_node_fqn,
					robot_profile->adapter.controller_command_type).has_value();
			};
			if (!spin_until(executor, std::min(scenario_deadline,
				std::chrono::steady_clock::now() + 2s), graph_removed))
			{
				throw std::runtime_error(
					"controller subscription remained visible after unload");
			}
			evidence.controller_graph_loss_confirmed = true;
			evidence.controller_lost_at = std::chrono::steady_clock::now();

			const auto fault_complete = [&evidence]()
			{
				return evidence.saw_controller_timeout_fault &&
					evidence.saw_gateway_vehicle_fault &&
					evidence.saw_canonical_hold &&
					evidence.saw_zero_adapter_output;
			};
			const auto fault_deadline = std::min(
				scenario_deadline,
				evidence.controller_lost_at + 3s);
			if (!spin_until(executor, fault_deadline, fault_complete))
			{
				throw std::runtime_error(
					"controller loss did not close VehicleState timeout, Gateway HOLD and adapter zero output evidence");
			}

			evidence.phase = ScenarioPhase::kRecovery;
			evidence.reactivation_started_at = std::chrono::steady_clock::now();
			request_controller_operation<LoadController>(
				load_client,
				executor,
				controller_name,
				"LoadController request timed out",
				"controller manager rejected loading controller");
			managed_state = ManagedControllerState::kUnconfigured;
			request_controller_operation<ConfigureController>(
				configure_client,
				executor,
				controller_name,
				"ConfigureController request timed out",
				"controller manager rejected configuring controller");
			managed_state = ManagedControllerState::kInactive;
			const auto new_generation_visible = [&node, &robot_profile,
				&initial_controller_gid]()
			{
				const auto gid = controller_endpoint_gid(
					node,
					robot_profile->adapter.controller_output_topic,
					robot_profile->adapter.controller_node_fqn,
					robot_profile->adapter.controller_command_type);
				return gid.has_value() &&
					gid.value() != initial_controller_gid.value();
			};
			if (!spin_until(
				executor,
				std::min(scenario_deadline, std::chrono::steady_clock::now() + 2s),
				new_generation_visible))
			{
				throw std::runtime_error(
					"controller reload did not create a new Graph endpoint generation");
			}
			evidence.new_generation_observed_at = std::chrono::steady_clock::now();
			switch_controller(
				switch_client,
				executor,
				controller_name,
				true);
			if (read_controller_state(list_client, executor, controller_name) != "active")
			{
				throw std::runtime_error(
					"diff drive controller did not return to active state");
			}
			managed_state = ManagedControllerState::kActive;
			evidence.controller_active_confirmed = true;

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
					"controller recovery did not complete Graph stabilization and Gateway recovery epoch");
			}
		}
		catch (...)
		{
			// 故障场景不能把共享的 Robot demo 留在 controller inactive 状态
			if (managed_state != ManagedControllerState::kActive && rclcpp::ok())
			{
				try
				{
					restore_controller(
						load_client,
						configure_client,
						switch_client,
						executor,
						controller_name,
						managed_state);
				}
				catch (const std::exception &exception)
				{
					RCLCPP_ERROR(
						node->get_logger(),
						"Failed to reactivate controller after scenario error: %s",
						exception.what());
				}
			}
			throw;
		}

		const auto vehicle_fault_ms = elapsed_milliseconds(
			evidence.controller_lost_at,
			evidence.vehicle_fault_observed_at);
		const auto gateway_fault_ms = elapsed_milliseconds(
			evidence.controller_lost_at,
			evidence.gateway_fault_observed_at);
		const auto reload_to_health_ms = elapsed_milliseconds(
			evidence.reactivation_started_at,
			evidence.healthy_vehicle_observed_at);
		const auto generation_to_health_ms = elapsed_milliseconds(
			evidence.new_generation_observed_at,
			evidence.healthy_vehicle_observed_at);
		const auto recovery_ms = elapsed_milliseconds(
			evidence.reactivation_started_at,
			evidence.recovery_completed_at);
		if (generation_to_health_ms < static_cast<std::int64_t>(
			bundle->gateway_contract->gateway.graph_stable_window_ms))
		{
			throw std::runtime_error(
				"controller health recovered before the Contract Graph stable window");
		}
		if (evidence.recovery_transition_sequence <=
			evidence.fault_transition_sequence)
		{
			throw std::runtime_error(
				"Gateway transition_sequence did not advance across controller recovery");
		}

		RCLCPP_INFO(
			node->get_logger(),
			"ROBOT_CONTROLLER_LOSS_SUCCEEDED vehicle_fault_ms=%ld gateway_fault_ms=%ld "
			"reload_to_health_ms=%ld generation_to_health_ms=%ld recovery_ms=%ld "
			"max_recovery_count=%u "
			"stable_output_samples=%u fault_transition=%lu recovery_transition=%lu",
			static_cast<long>(vehicle_fault_ms),
			static_cast<long>(gateway_fault_ms),
			static_cast<long>(reload_to_health_ms),
			static_cast<long>(generation_to_health_ms),
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
		auto node = std::make_shared<rclcpp::Node>(
			"verify_robot_controller_loss");
		result = run_scenario(node);
	}
	catch (const std::exception &exception)
	{
		RCLCPP_ERROR(
			rclcpp::get_logger("verify_robot_controller_loss"),
			"Robot controller loss scenario failed: %s",
			exception.what());
	}
	rclcpp::shutdown();
	return result;
}
