#include <algorithm>
#include <charconv>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <filesystem>
#include <functional>
#include <limits>
#include <optional>
#include <stdexcept>
#include <string>
#include <system_error>
#include <thread>
#include <utility>
#include <vector>

#include "control_link_contract/contract_bundle.hpp"
#include "control_link_contract/parser.hpp"
#include "control_link_contract/qos_factory.hpp"
#include "control_link_interfaces/msg/control_command.hpp"
#include "control_link_interfaces/msg/gateway_state.hpp"
#include "control_link_interfaces/msg/vehicle_state.hpp"
#include "diagnostic_msgs/msg/diagnostic_array.hpp"
#include "diagnostic_msgs/msg/diagnostic_status.hpp"
#include "rclcpp/parameter_client.hpp"
#include "rclcpp/rclcpp.hpp"

namespace
{
	using namespace std::chrono_literals;
	using ControlCommand = control_link_interfaces::msg::ControlCommand;
	using GatewayState = control_link_interfaces::msg::GatewayState;
	using VehicleState = control_link_interfaces::msg::VehicleState;
	using SteadyTime = std::chrono::steady_clock::time_point;

	constexpr char kAdapterDiagnosticName[] =
		"control_link/vehicle_adapter/socketcan";
	constexpr char kPlanningSourceId[] = "planning";
	constexpr char kSimulatorFqn[] = "/vehicle_simulator";
	constexpr std::size_t kRequiredHoldSamples = 5U;

	template<typename Message>
	struct TimedMessage final
	{
		Message value;
		SteadyTime received_at;
	};

	struct DiagnosticSnapshot final
	{
		std::uint8_t level;
		std::string message;
		std::vector<diagnostic_msgs::msg::KeyValue> values;
		SteadyTime received_at;
	};

	struct Observations final
	{
		std::vector<TimedMessage<GatewayState>> gateway_states;
		std::vector<TimedMessage<VehicleState>> vehicle_states;
		std::vector<TimedMessage<ControlCommand>> canonical_commands;
		std::vector<DiagnosticSnapshot> adapter_diagnostics;
	};

	struct Cursor final
	{
		std::size_t gateway_states;
		std::size_t vehicle_states;
		std::size_t canonical_commands;
		std::size_t adapter_diagnostics;
	};

	Cursor cursor(const Observations &observations)
	{
		return Cursor{
			observations.gateway_states.size(),
			observations.vehicle_states.size(),
			observations.canonical_commands.size(),
			observations.adapter_diagnostics.size()};
	}

	std::chrono::milliseconds checked_timeout(std::int64_t timeout_ms)
	{
		if (timeout_ms <= 0 ||
			timeout_ms > std::chrono::milliseconds::max().count())
		{
			throw std::out_of_range(
				"timeout_ms must fit a positive milliseconds duration");
		}
		return std::chrono::milliseconds{timeout_ms};
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
			std::this_thread::sleep_for(2ms);
		}
		executor.spin_some();
		return predicate();
	}

	std::optional<std::string> diagnostic_value(
		const DiagnosticSnapshot &diagnostic,
		const std::string &key)
	{
		const auto found = std::find_if(
			diagnostic.values.begin(),
			diagnostic.values.end(),
			[&key](const auto &value)
			{
				return value.key == key;
			});
		return found == diagnostic.values.end() ? std::nullopt :
			std::optional<std::string>{found->value};
	}

	std::string required_diagnostic_value(
		const DiagnosticSnapshot &diagnostic,
		const std::string &key)
	{
		const auto value = diagnostic_value(diagnostic, key);
		if (!value.has_value())
		{
			throw std::runtime_error(
				"SocketCAN diagnostic is missing key: " + key);
		}
		return value.value();
	}

	std::uint64_t required_unsigned(
		const DiagnosticSnapshot &diagnostic,
		const std::string &key)
	{
		const auto text = required_diagnostic_value(diagnostic, key);
		std::uint64_t result = 0U;
		const auto conversion = std::from_chars(
			text.data(),
			text.data() + text.size(),
			result);
		if (conversion.ec != std::errc{} ||
			conversion.ptr != text.data() + text.size())
		{
			throw std::runtime_error(
				"SocketCAN diagnostic key is not an unsigned integer: " + key);
		}
		return result;
	}

	bool required_bool(
		const DiagnosticSnapshot &diagnostic,
		const std::string &key)
	{
		const auto text = required_diagnostic_value(diagnostic, key);
		if (text == "true")
		{
			return true;
		}
		if (text == "false")
		{
			return false;
		}
		throw std::runtime_error(
			"SocketCAN diagnostic key is not boolean: " + key);
	}

	bool transport_errors_are_zero(const DiagnosticSnapshot &diagnostic)
	{
		return required_unsigned(diagnostic, "transport_poll_errors") == 0U &&
			required_unsigned(diagnostic, "transport_read_errors") == 0U &&
			required_unsigned(diagnostic, "transport_write_errors") == 0U &&
			required_diagnostic_value(diagnostic, "last_io_error_kind") == "none" &&
			required_unsigned(diagnostic, "last_io_errno") == 0U &&
			required_diagnostic_value(diagnostic, "last_io_detail") == "none";
	}

	bool healthy_adapter_diagnostic(
		const DiagnosticSnapshot &diagnostic,
		std::uint64_t recovery_required)
	{
		return diagnostic.level == diagnostic_msgs::msg::DiagnosticStatus::OK &&
			required_bool(diagnostic, "can_link_healthy") &&
			!required_bool(diagnostic, "can_state_timed_out") &&
			required_unsigned(diagnostic, "can_recovery_valid_count") >=
				recovery_required &&
			required_unsigned(diagnostic, "can_recovery_required") ==
				recovery_required &&
			required_diagnostic_value(diagnostic, "last_codec_reason") == "none" &&
			transport_errors_are_zero(diagnostic);
	}

	bool canonical_is_motion(const ControlCommand &command)
	{
		return command.mode == ControlCommand::MODE_NORMAL &&
			command.source_id == kPlanningSourceId &&
			(command.linear_velocity_mps != 0.0 ||
			command.angular_velocity_radps != 0.0);
	}

	bool canonical_is_hold(const ControlCommand &command)
	{
		return command.mode == ControlCommand::MODE_HOLD &&
			command.linear_velocity_mps == 0.0 &&
			command.angular_velocity_radps == 0.0;
	}

	bool saw_consecutive_hold(
		const Observations &observations,
		const Cursor &begin)
	{
		std::size_t consecutive = 0U;
		for (std::size_t index = begin.canonical_commands;
			index < observations.canonical_commands.size();
			++index)
		{
			consecutive = canonical_is_hold(
				observations.canonical_commands[index].value) ?
				consecutive + 1U : 0U;
			if (consecutive >= kRequiredHoldSamples)
			{
				return true;
			}
		}
		return false;
	}

	template<typename Predicate>
	bool saw_diagnostic(
		const Observations &observations,
		const Cursor &begin,
		Predicate predicate)
	{
		return std::any_of(
			observations.adapter_diagnostics.begin() +
				static_cast<std::ptrdiff_t>(begin.adapter_diagnostics),
			observations.adapter_diagnostics.end(),
			predicate);
	}

	bool saw_vehicle_fault(
		const Observations &observations,
		const Cursor &begin,
		std::uint8_t expected_state,
		std::uint16_t expected_fault)
	{
		return std::any_of(
			observations.vehicle_states.begin() +
				static_cast<std::ptrdiff_t>(begin.vehicle_states),
			observations.vehicle_states.end(),
			[expected_state, expected_fault](const auto &observation)
			{
				return observation.value.state == expected_state &&
					observation.value.fault_code == expected_fault;
			});
	}

	bool saw_gateway_vehicle_fault(
		const Observations &observations,
		const Cursor &begin)
	{
		return std::any_of(
			observations.gateway_states.begin() +
				static_cast<std::ptrdiff_t>(begin.gateway_states),
			observations.gateway_states.end(),
			[](const auto &observation)
			{
				return observation.value.state == GatewayState::SAFE_STOP &&
					observation.value.reason_code == GatewayState::REASON_VEHICLE_FAULT;
			});
	}

	bool initial_chain_is_healthy(
		const Observations &observations,
		std::uint64_t recovery_required)
	{
		const bool gateway_active = std::any_of(
			observations.gateway_states.begin(),
			observations.gateway_states.end(),
			[](const auto &observation)
			{
				return observation.value.state == GatewayState::ACTIVE &&
					observation.value.active_source_id == kPlanningSourceId;
			});
		const bool vehicle_running = std::any_of(
			observations.vehicle_states.begin(),
			observations.vehicle_states.end(),
			[](const auto &observation)
			{
				return observation.value.state == VehicleState::RUNNING &&
					observation.value.fault_code == VehicleState::FAULT_NONE;
			});
		const bool canonical_motion = std::any_of(
			observations.canonical_commands.begin(),
			observations.canonical_commands.end(),
			[](const auto &observation)
			{
				return canonical_is_motion(observation.value);
			});
		const bool diagnostic_healthy = std::any_of(
			observations.adapter_diagnostics.begin(),
			observations.adapter_diagnostics.end(),
			[recovery_required](const auto &diagnostic)
			{
				return healthy_adapter_diagnostic(diagnostic, recovery_required) &&
					required_unsigned(
						diagnostic,
						"accepted_can_state_frames") >= recovery_required;
			});
		return gateway_active && vehicle_running && canonical_motion &&
			diagnostic_healthy;
	}

	bool recovery_is_complete(
		const Observations &observations,
		const Cursor &begin,
		std::uint64_t recovery_required)
	{
		std::optional<SteadyTime> vehicle_healthy_at;
		for (std::size_t index = begin.vehicle_states;
			index < observations.vehicle_states.size();
			++index)
		{
			const auto &observation = observations.vehicle_states[index];
			if (observation.value.fault_code == VehicleState::FAULT_NONE &&
				(observation.value.state == VehicleState::STANDBY ||
				observation.value.state == VehicleState::RUNNING))
			{
				vehicle_healthy_at = observation.received_at;
				break;
			}
		}
		if (!vehicle_healthy_at.has_value())
		{
			return false;
		}

		std::optional<SteadyTime> gateway_recovering_at;
		for (std::size_t index = begin.gateway_states;
			index < observations.gateway_states.size();
			++index)
		{
			const auto &observation = observations.gateway_states[index];
			if (observation.received_at >= vehicle_healthy_at.value() &&
				observation.value.state == GatewayState::RECOVERING)
			{
				gateway_recovering_at = observation.received_at;
				break;
			}
		}
		if (!gateway_recovering_at.has_value())
		{
			return false;
		}

		std::optional<SteadyTime> gateway_active_at;
		for (std::size_t index = begin.gateway_states;
			index < observations.gateway_states.size();
			++index)
		{
			const auto &observation = observations.gateway_states[index];
			if (observation.received_at >= gateway_recovering_at.value() &&
				observation.value.state == GatewayState::ACTIVE &&
				observation.value.reason_code == GatewayState::REASON_RECOVERY_COMPLETE &&
				observation.value.active_source_id == kPlanningSourceId)
			{
				gateway_active_at = observation.received_at;
				break;
			}
		}
		if (!gateway_active_at.has_value())
		{
			return false;
		}

		const bool motion_resumed = std::any_of(
			observations.canonical_commands.begin() +
				static_cast<std::ptrdiff_t>(begin.canonical_commands),
			observations.canonical_commands.end(),
			[&gateway_active_at](const auto &observation)
			{
				return observation.received_at >= gateway_active_at.value() &&
					canonical_is_motion(observation.value);
			});
		const bool diagnostic_healthy = saw_diagnostic(
			observations,
			begin,
			[recovery_required](const auto &diagnostic)
			{
				return healthy_adapter_diagnostic(diagnostic, recovery_required);
			});
		return motion_resumed && diagnostic_healthy;
	}

	void set_simulator_parameter(
		const rclcpp::AsyncParametersClient::SharedPtr &client,
		rclcpp::executors::SingleThreadedExecutor &executor,
		rclcpp::Parameter parameter)
	{
		auto future = client->set_parameters({std::move(parameter)});
		if (executor.spin_until_future_complete(future, 3s) !=
			rclcpp::FutureReturnCode::SUCCESS)
		{
			throw std::runtime_error("VehicleSimulator parameter request timed out");
		}
		const auto results = future.get();
		if (results.size() != 1U || !results.front().successful)
		{
			throw std::runtime_error(
				"VehicleSimulator rejected parameter update" +
				(results.empty() || results.front().reason.empty() ?
					std::string{} : ": " + results.front().reason));
		}
	}

	template<typename FaultDiagnosticPredicate>
	void require_can_fault_and_recovery(
		const rclcpp::Node::SharedPtr &node,
		rclcpp::executors::SingleThreadedExecutor &executor,
		const rclcpp::AsyncParametersClient::SharedPtr &parameter_client,
		Observations &observations,
		SteadyTime scenario_deadline,
		std::uint64_t recovery_required,
		const std::string &parameter_name,
		FaultDiagnosticPredicate fault_diagnostic,
		const std::string &scenario_name)
	{
		const auto fault_begin = cursor(observations);
		set_simulator_parameter(
			parameter_client,
			executor,
			rclcpp::Parameter{parameter_name, true});
		const auto fault_complete = [&observations, &fault_begin, &fault_diagnostic]()
		{
			return saw_vehicle_fault(
				observations,
				fault_begin,
				VehicleState::SAFE_STOP,
				VehicleState::FAULT_ADAS_CAN_STATE_TIMEOUT) &&
				saw_gateway_vehicle_fault(observations, fault_begin) &&
				saw_consecutive_hold(observations, fault_begin) &&
				saw_diagnostic(observations, fault_begin, fault_diagnostic);
		};
		if (!spin_until(
			executor,
			std::min(scenario_deadline, std::chrono::steady_clock::now() + 6s),
			fault_complete))
		{
			throw std::runtime_error(
				scenario_name +
				" did not close CAN diagnostic, VehicleState, GatewayState and HOLD evidence");
		}

		const auto recovery_begin = cursor(observations);
		set_simulator_parameter(
			parameter_client,
			executor,
			rclcpp::Parameter{parameter_name, false});
		if (!spin_until(
			executor,
			scenario_deadline,
			[&observations, &recovery_begin, recovery_required]()
			{
				return recovery_is_complete(
					observations,
					recovery_begin,
					recovery_required);
			}))
		{
			throw std::runtime_error(
				scenario_name +
				" recovery did not preserve CAN-before-Gateway recovery ordering");
		}

		RCLCPP_INFO(node->get_logger(), "%s scenario verified", scenario_name.c_str());
	}

	int run_scenario(const rclcpp::Node::SharedPtr &node)
	{
		const auto profile_path = std::filesystem::path{
			node->declare_parameter<std::string>("profile_path", "")};
		const auto config_root = std::filesystem::path{
			node->declare_parameter<std::string>("config_root", "")};
		const auto timeout = checked_timeout(
			node->declare_parameter<std::int64_t>("timeout_ms", 120000));
		const auto bundle = control_link_contract::load_contract_bundle(
			profile_path,
			config_root);
		const auto *adas_profile = std::get_if<control_link_contract::AdasProfile>(
			bundle->profile.get());
		if (adas_profile == nullptr)
		{
			throw std::invalid_argument("ADAS vcan verifier requires profile_id=adas");
		}
		const auto recovery_required =
			adas_profile->adapter.recovery_valid_frames;
		if (recovery_required == 0U)
		{
			throw std::logic_error("ADAS CAN recovery count must be positive");
		}

		control_link_contract::QosFactory qos_factory{bundle->gateway_contract};
		const auto &gateway_state_contract =
			bundle->gateway_contract->state_topics.at("gateway_state");
		const auto &vehicle_state_contract =
			bundle->gateway_contract->state_topics.at("vehicle_state");
		if (!gateway_state_contract.qos_profile.has_value() ||
			!vehicle_state_contract.qos_profile.has_value())
		{
			throw std::logic_error(
				"ADAS verifier state Topics require Contract QoS profiles");
		}

		Observations observations;
		auto gateway_subscription = node->create_subscription<GatewayState>(
			gateway_state_contract.topic,
			qos_factory.make(gateway_state_contract.qos_profile.value()),
			[&observations](const GatewayState &state)
			{
				observations.gateway_states.push_back(
					TimedMessage<GatewayState>{state, std::chrono::steady_clock::now()});
			});
		auto vehicle_subscription = node->create_subscription<VehicleState>(
			vehicle_state_contract.topic,
			qos_factory.make(vehicle_state_contract.qos_profile.value()),
			[&observations](const VehicleState &state)
			{
				observations.vehicle_states.push_back(
					TimedMessage<VehicleState>{state, std::chrono::steady_clock::now()});
			});
		auto canonical_subscription = node->create_subscription<ControlCommand>(
			bundle->gateway_contract->output.topic,
			qos_factory.make(bundle->gateway_contract->output.qos_profile),
			[&observations](const ControlCommand &command)
			{
				observations.canonical_commands.push_back(
					TimedMessage<ControlCommand>{command, std::chrono::steady_clock::now()});
			});
		auto diagnostic_subscription =
			node->create_subscription<diagnostic_msgs::msg::DiagnosticArray>(
				"/diagnostics",
				rclcpp::QoS{rclcpp::KeepLast{10U}},
				[&observations](const diagnostic_msgs::msg::DiagnosticArray &array)
				{
					for (const auto &status : array.status)
					{
						if (status.name == kAdapterDiagnosticName)
						{
							observations.adapter_diagnostics.push_back(
								DiagnosticSnapshot{
									status.level,
									status.message,
									status.values,
									std::chrono::steady_clock::now()});
						}
					}
				});
		auto parameter_client =
			std::make_shared<rclcpp::AsyncParametersClient>(node, kSimulatorFqn);

		rclcpp::executors::SingleThreadedExecutor executor;
		executor.add_node(node);
		const auto scenario_deadline = std::chrono::steady_clock::now() + timeout;
		if (!spin_until(
			executor,
			scenario_deadline,
			[&observations, &parameter_client, recovery_required]()
			{
				return parameter_client->service_is_ready() &&
					initial_chain_is_healthy(observations, recovery_required);
			}))
		{
			throw std::runtime_error(
				"timed out waiting for ACTIVE planning -> CAN -> VehicleState baseline");
		}

		const auto initial_rejected = required_unsigned(
			observations.adapter_diagnostics.back(),
			"rejected_can_state_frames");
		require_can_fault_and_recovery(
			node,
			executor,
			parameter_client,
			observations,
			scenario_deadline,
			recovery_required,
			"corrupt_crc",
			[initial_rejected](const DiagnosticSnapshot &diagnostic)
			{
				return !required_bool(diagnostic, "can_link_healthy") &&
					required_bool(diagnostic, "can_state_timed_out") &&
					required_diagnostic_value(
						diagnostic,
						"last_codec_reason") == "crc_mismatch" &&
					required_unsigned(
						diagnostic,
						"rejected_can_state_frames") > initial_rejected &&
					transport_errors_are_zero(diagnostic);
			},
			"CRC corruption");

		const auto rejected_after_crc = required_unsigned(
			observations.adapter_diagnostics.back(),
			"rejected_can_state_frames");
		require_can_fault_and_recovery(
			node,
			executor,
			parameter_client,
			observations,
			scenario_deadline,
			recovery_required,
			"freeze_counter",
			[rejected_after_crc](const DiagnosticSnapshot &diagnostic)
			{
				return !required_bool(diagnostic, "can_link_healthy") &&
					required_bool(diagnostic, "can_state_timed_out") &&
					required_diagnostic_value(
						diagnostic,
						"last_counter_observation") == "duplicate" &&
					required_unsigned(
						diagnostic,
						"rejected_can_state_frames") > rejected_after_crc &&
					transport_errors_are_zero(diagnostic);
			},
			"state counter freeze");

		const auto rejected_before_dropout = required_unsigned(
			observations.adapter_diagnostics.back(),
			"rejected_can_state_frames");
		require_can_fault_and_recovery(
			node,
			executor,
			parameter_client,
			observations,
			scenario_deadline,
			recovery_required,
			"drop_state",
			[rejected_before_dropout](const DiagnosticSnapshot &diagnostic)
			{
				return !required_bool(diagnostic, "can_link_healthy") &&
					required_bool(diagnostic, "can_state_timed_out") &&
					required_unsigned(
						diagnostic,
						"rejected_can_state_frames") == rejected_before_dropout &&
					transport_errors_are_zero(diagnostic);
			},
			"CAN state dropout");

		const auto vehicle_fault_begin = cursor(observations);
		const auto rejected_before_vehicle_fault = required_unsigned(
			observations.adapter_diagnostics.back(),
			"rejected_can_state_frames");
		constexpr std::int64_t kInjectedVehicleFault = 7;
		set_simulator_parameter(
			parameter_client,
			executor,
			rclcpp::Parameter{"fault_code", kInjectedVehicleFault});
		const auto vehicle_fault_code = static_cast<std::uint16_t>(
			VehicleState::FAULT_ADAS_VEHICLE_REPORTED_BASE +
			kInjectedVehicleFault);
		const auto vehicle_fault_complete = [&observations,
			&vehicle_fault_begin, rejected_before_vehicle_fault, vehicle_fault_code]()
		{
			return saw_vehicle_fault(
				observations,
				vehicle_fault_begin,
				VehicleState::FAULT,
				vehicle_fault_code) &&
				saw_gateway_vehicle_fault(observations, vehicle_fault_begin) &&
				saw_consecutive_hold(observations, vehicle_fault_begin) &&
				saw_diagnostic(
					observations,
					vehicle_fault_begin,
					[rejected_before_vehicle_fault](const auto &diagnostic)
					{
						return required_bool(diagnostic, "can_link_healthy") &&
							!required_bool(diagnostic, "can_state_timed_out") &&
							required_unsigned(
								diagnostic,
								"rejected_can_state_frames") ==
								rejected_before_vehicle_fault &&
							transport_errors_are_zero(diagnostic);
					});
		};
		if (!spin_until(
			executor,
			std::min(scenario_deadline, std::chrono::steady_clock::now() + 6s),
			vehicle_fault_complete))
		{
			throw std::runtime_error(
				"vehicle raw fault did not preserve healthy CAN while closing Gateway HOLD");
		}

		const auto vehicle_recovery_begin = cursor(observations);
		set_simulator_parameter(
			parameter_client,
			executor,
			rclcpp::Parameter{"fault_code", std::int64_t{0}});
		if (!spin_until(
			executor,
			scenario_deadline,
			[&observations, &vehicle_recovery_begin, recovery_required]()
			{
				return recovery_is_complete(
					observations,
					vehicle_recovery_begin,
					recovery_required);
			}))
		{
			throw std::runtime_error(
				"vehicle raw fault recovery did not return to ACTIVE planning");
		}

		const auto &final_diagnostic = observations.adapter_diagnostics.back();
		RCLCPP_INFO(
			node->get_logger(),
			"E10_ADAS_VCAN_VERIFIED accepted=%llu rejected=%llu tx=%llu rx=%llu",
			static_cast<unsigned long long>(required_unsigned(
				final_diagnostic,
				"accepted_can_state_frames")),
			static_cast<unsigned long long>(required_unsigned(
				final_diagnostic,
				"rejected_can_state_frames")),
			static_cast<unsigned long long>(required_unsigned(
				final_diagnostic,
				"transport_transmitted_frames")),
			static_cast<unsigned long long>(required_unsigned(
				final_diagnostic,
				"transport_received_frames")));

		(void)gateway_subscription;
		(void)vehicle_subscription;
		(void)canonical_subscription;
		(void)diagnostic_subscription;
		return 0;
	}
}  // namespace

int main(int argc, char **argv)
{
	rclcpp::init(argc, argv);
	try
	{
		auto node = std::make_shared<rclcpp::Node>("adas_vcan_verifier");
		const int result = run_scenario(node);
		rclcpp::shutdown();
		return result;
	}
	catch (const std::exception &exception)
	{
		RCLCPP_FATAL(
			rclcpp::get_logger("adas_vcan_verifier"),
			"ADAS vcan verification failed: %s",
			exception.what());
		rclcpp::shutdown();
		return 1;
	}
	catch (...)
	{
		RCLCPP_FATAL(
			rclcpp::get_logger("adas_vcan_verifier"),
			"ADAS vcan verification failed with an unknown exception");
		rclcpp::shutdown();
		return 1;
	}
}
