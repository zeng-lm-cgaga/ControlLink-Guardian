#include <chrono>
#include <cstdint>
#include <filesystem>
#include <functional>
#include <limits>
#include <map>
#include <memory>
#include <optional>
#include <stdexcept>
#include <string>
#include <thread>
#include <utility>

#include "control_link_contract/contract_bundle.hpp"
#include "control_link_contract/qos_factory.hpp"
#include "control_link_interfaces/msg/control_command.hpp"
#include "control_link_interfaces/msg/gateway_state.hpp"
#include "control_link_interfaces/msg/source_status.hpp"
#include "control_link_interfaces/msg/vehicle_state.hpp"
#include "diagnostic_msgs/msg/diagnostic_array.hpp"
#include "diagnostic_msgs/msg/diagnostic_status.hpp"
#include "lifecycle_msgs/msg/state.hpp"
#include "lifecycle_msgs/msg/transition.hpp"
#include "lifecycle_msgs/srv/change_state.hpp"
#include "lifecycle_msgs/srv/get_state.hpp"
#include "rclcpp/executors/single_threaded_executor.hpp"
#include "rclcpp/rclcpp.hpp"

namespace
{
	using namespace std::chrono_literals;
	using ChangeState = lifecycle_msgs::srv::ChangeState;
	using ControlCommand = control_link_interfaces::msg::ControlCommand;
	using DiagnosticArray = diagnostic_msgs::msg::DiagnosticArray;
	using DiagnosticStatus = diagnostic_msgs::msg::DiagnosticStatus;
	using GatewayState = control_link_interfaces::msg::GatewayState;
	using GetState = lifecycle_msgs::srv::GetState;
	using SourceStatus = control_link_interfaces::msg::SourceStatus;
	using VehicleState = control_link_interfaces::msg::VehicleState;

	class GatewayLifecycleVerifier final : public rclcpp::Node
	{
	public:
		GatewayLifecycleVerifier()
		: rclcpp::Node("gateway_lifecycle_verifier")
		{
			declare_parameter<std::string>("profile_path", "");
			declare_parameter<std::string>("config_root", "");
			declare_parameter<std::string>("target_fqn", "/control_link/gateway");

			const auto profile_path = std::filesystem::path{
				get_parameter("profile_path").as_string()};
			const auto config_root = std::filesystem::path{
				get_parameter("config_root").as_string()};
			target_fqn_ = get_parameter("target_fqn").as_string();
			if (target_fqn_.empty() || target_fqn_.front() != '/')
			{
				throw std::invalid_argument("target_fqn must be an absolute node FQN");
			}

			bundle_ = control_link_contract::load_contract_bundle(
				profile_path,
				config_root);
			if (!std::holds_alternative<control_link_contract::AdasProfile>(
					*bundle_->profile))
			{
				throw std::invalid_argument(
					"Gateway lifecycle verification requires the ADAS system-clock Profile");
			}

			control_link_contract::QosFactory qos_factory{
				bundle_->gateway_contract};
			input_qos_ = qos_factory.make(
				bundle_->gateway_contract->input.qos_profile);
			canonical_qos_ = qos_factory.make(
				bundle_->gateway_contract->output.qos_profile);
			const auto runtime_qos = qos_factory.make("runtime_state");

			state_subscription_ = create_subscription<GatewayState>(
				bundle_->gateway_contract->state_topics.at("gateway_state").topic,
				runtime_qos,
				[this](GatewayState::ConstSharedPtr message)
				{
					latest_gateway_state_ = *message;
				});
			source_status_subscription_ = create_subscription<SourceStatus>(
				bundle_->gateway_contract->state_topics.at("source_status").topic,
				runtime_qos,
				[this](SourceStatus::ConstSharedPtr message)
				{
					latest_source_status_[message->source_id] = *message;
				});
			vehicle_state_subscription_ = create_subscription<VehicleState>(
				bundle_->gateway_contract->state_topics.at("vehicle_state").topic,
				runtime_qos,
				[this](VehicleState::ConstSharedPtr message)
				{
					latest_vehicle_state_ = *message;
				});
			canonical_subscription_ = create_subscription<ControlCommand>(
				bundle_->gateway_contract->output.topic,
				canonical_qos_.value(),
				[this](ControlCommand::ConstSharedPtr message)
				{
					latest_canonical_ = *message;
					canonical_count_ += 1U;
					if (message->mode == ControlCommand::MODE_HOLD &&
						message->linear_velocity_mps == 0.0 &&
						message->angular_velocity_radps == 0.0)
					{
						hold_count_ += 1U;
					}
				});
			diagnostics_subscription_ = create_subscription<DiagnosticArray>(
				bundle_->gateway_contract->state_topics.at("diagnostics").topic,
				rclcpp::QoS(rclcpp::KeepLast(10U)),
				[this](DiagnosticArray::ConstSharedPtr message)
				{
					for (const auto &status : message->status)
					{
						if (status.name == "gateway/config")
						{
							config_identity_observed_ = config_identity_matches(status);
						}
					}
				});

			get_state_client_ = create_client<GetState>(target_fqn_ + "/get_state");
			change_state_client_ =
				create_client<ChangeState>(target_fqn_ + "/change_state");
		}

		void run(rclcpp::executors::SingleThreadedExecutor &executor)
		{
			wait_for_lifecycle_services(executor);
			request_transition(
				executor,
				lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE,
				true);
			wait_for_lifecycle_state(
				executor,
				lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE,
				5s);
			wait_until(
				executor,
				[this]()
				{
					return config_identity_observed_;
				},
				3s,
				"Gateway diagnostics did not expose the validated config identity");

			// 配置后 Gateway 的 Lifecycle publisher 已进入 Graph，第二个 publisher 会让 adapter fail closed
			rogue_canonical_publisher_ = create_publisher<ControlCommand>(
				bundle_->gateway_contract->output.topic,
				canonical_qos_.value());
			wait_until(
				executor,
				[this]()
				{
					return latest_vehicle_state_.has_value() &&
						latest_vehicle_state_->state == VehicleState::SAFE_STOP &&
						latest_vehicle_state_->fault_code ==
							VehicleState::FAULT_ADAPTER_CANONICAL_SOURCE_AMBIGUOUS;
				},
				5s,
				"adapter did not report canonical publisher ambiguity");
			request_transition(
				executor,
				lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE,
				false);
			wait_for_lifecycle_state(
				executor,
				lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE,
				3s);

			rogue_canonical_publisher_.reset();
			wait_until(
				executor,
				[this]()
				{
					return latest_vehicle_state_.has_value() &&
						latest_vehicle_state_->state == VehicleState::SAFE_STOP &&
						latest_vehicle_state_->fault_code ==
							VehicleState::FAULT_ADAPTER_CANONICAL_TIMEOUT;
				},
				5s,
				"adapter did not return to the canonical-timeout bootstrap state");
			activate_when_ready(executor);

			const auto hold_start = hold_count_;
			wait_until(
				executor,
				[this, hold_start]()
				{
					return hold_count_ >= hold_start + 3U &&
						latest_vehicle_state_.has_value() &&
						latest_vehicle_state_->state == VehicleState::STANDBY &&
						latest_vehicle_state_->fault_code == VehicleState::FAULT_NONE;
				},
				5s,
				"Gateway bootstrap HOLD did not release the adapter watchdog");

			create_source("planning");
			wait_for_gateway_state(
				executor,
				GatewayState::ACTIVE,
				"planning",
				GatewayState::REASON_RECOVERY_COMPLETE,
				8s);
			wait_until(
				executor,
				[this]()
				{
					return latest_canonical_.has_value() &&
						latest_canonical_->mode == ControlCommand::MODE_NORMAL &&
						latest_canonical_->source_id == "planning";
				},
				3s,
				"canonical output did not carry the selected planning command");

			create_source("teleop");
			wait_for_gateway_state(
				executor,
				GatewayState::ACTIVE,
				"teleop",
				GatewayState::REASON_SOURCE_SWITCH,
				8s);

			destroy_source("teleop");
			wait_for_gateway_state(
				executor,
				GatewayState::ACTIVE,
				"planning",
				GatewayState::REASON_SOURCE_FALLBACK,
				5s);

			create_ambiguous_planning_publisher();
			wait_until(
				executor,
				[this]()
				{
					const auto iterator = latest_source_status_.find("planning");
					return iterator != latest_source_status_.end() &&
						!iterator->second.command_valid &&
						iterator->second.last_reject_reason ==
							SourceStatus::REJECT_SOURCE_ENDPOINT_AMBIGUOUS;
				},
				8s,
				"planning ambiguity was not exposed through SourceStatus");
			wait_for_gateway_state(
				executor,
				GatewayState::SAFE_STOP,
				"",
				GatewayState::REASON_NO_QUALIFIED_SOURCE,
				5s);

			const auto safe_stop_hold_start = hold_count_;
			wait_until(
				executor,
				[this, safe_stop_hold_start]()
				{
					return hold_count_ >= safe_stop_hold_start + 5U;
				},
				3s,
				"SAFE_STOP did not continuously publish canonical HOLD");

			destroy_source("planning");
			ambiguous_planning_publisher_.reset();
			wait_until(
				executor,
				[this]()
				{
					return get_publishers_info_by_topic(planning_topic()).empty();
				},
				3s,
				"planning publishers remained in the ROS Graph after destruction");
			const auto graph_absent_since = std::chrono::steady_clock::now();
			wait_until(
				executor,
				[graph_absent_since]()
				{
					return std::chrono::steady_clock::now() - graph_absent_since >= 800ms;
				},
				2s,
				"planning absence did not span the Graph stable window");

			create_source("planning");
			wait_until(
				executor,
				[this]()
				{
					return latest_gateway_state_.has_value() &&
						latest_gateway_state_->state == GatewayState::RECOVERING &&
						latest_gateway_state_->active_source_id == "planning" &&
						latest_gateway_state_->recovery_valid_count > 0U &&
						latest_gateway_state_->recovery_valid_count < 5U;
				},
				8s,
				"new planning generation did not enter RECOVERING with partial evidence");
			wait_for_gateway_state(
				executor,
				GatewayState::ACTIVE,
				"planning",
				GatewayState::REASON_RECOVERY_COMPLETE,
				5s);

			RCLCPP_INFO(
				get_logger(),
				"E4_GATEWAY_LIFECYCLE_VERIFIED canonical=%lu hold=%lu",
				static_cast<unsigned long>(canonical_count_),
				static_cast<unsigned long>(hold_count_));
		}

	private:
		struct ManagedSource
		{
			rclcpp::Publisher<ControlCommand>::SharedPtr publisher;
			std::uint64_t sequence{0U};
		};

		std::optional<std::string> diagnostic_value(
			const DiagnosticStatus &status,
			const std::string &key) const
		{
			for (const auto &value : status.values)
			{
				if (value.key == key)
				{
					return value.value;
				}
			}
			return std::nullopt;
		}

		bool config_identity_matches(const DiagnosticStatus &status) const
		{
			const auto &identity = bundle_->identity;
			return status.level == DiagnosticStatus::OK &&
				diagnostic_value(status, "contract_id") == identity.contract.contract_id &&
				diagnostic_value(status, "contract_version") ==
					std::to_string(identity.contract.contract_version) &&
				diagnostic_value(status, "contract_hash") == identity.contract.contract_hash &&
				diagnostic_value(status, "profile_id") ==
					identity.decision_config.profile_id &&
				diagnostic_value(status, "decision_config_hash") ==
					identity.decision_config.decision_config_hash;
		}

		void wait_for_lifecycle_services(
			rclcpp::executors::SingleThreadedExecutor &executor)
		{
			wait_until(
				executor,
				[this]()
				{
					return get_state_client_->service_is_ready() &&
						change_state_client_->service_is_ready();
				},
				10s,
				"Gateway Lifecycle services are unavailable");
		}

		void request_transition(
			rclcpp::executors::SingleThreadedExecutor &executor,
			std::uint8_t transition,
			bool expected_success)
		{
			auto request = std::make_shared<ChangeState::Request>();
			request->transition.id = transition;
			auto future = change_state_client_->async_send_request(request);
			if (executor.spin_until_future_complete(future, 8s) !=
				rclcpp::FutureReturnCode::SUCCESS)
			{
				throw std::runtime_error("Gateway Lifecycle transition response timed out");
			}
			if (future.get()->success != expected_success)
			{
				throw std::runtime_error(
					"Gateway Lifecycle transition returned an unexpected success flag");
			}
		}

		std::uint8_t read_lifecycle_state(
			rclcpp::executors::SingleThreadedExecutor &executor)
		{
			auto future = get_state_client_->async_send_request(
				std::make_shared<GetState::Request>());
			if (executor.spin_until_future_complete(future, 3s) !=
				rclcpp::FutureReturnCode::SUCCESS)
			{
				throw std::runtime_error("Gateway GetState response timed out");
			}
			return future.get()->current_state.id;
		}

		void wait_for_lifecycle_state(
			rclcpp::executors::SingleThreadedExecutor &executor,
			std::uint8_t expected,
			std::chrono::seconds timeout)
		{
			wait_until(
				executor,
				[this, &executor, expected]()
				{
					return read_lifecycle_state(executor) == expected;
				},
				timeout,
				"Gateway Lifecycle state did not reach the expected value");
		}

		void activate_when_ready(rclcpp::executors::SingleThreadedExecutor &executor)
		{
			const auto deadline = std::chrono::steady_clock::now() + 12s;
			while (rclcpp::ok() && std::chrono::steady_clock::now() < deadline)
			{
				auto request = std::make_shared<ChangeState::Request>();
				request->transition.id =
					lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE;
				auto future = change_state_client_->async_send_request(request);
				if (executor.spin_until_future_complete(future, 3s) ==
					rclcpp::FutureReturnCode::SUCCESS && future.get()->success)
				{
					wait_for_lifecycle_state(
						executor,
						lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE,
						3s);
					return;
				}
				std::this_thread::sleep_for(100ms);
			}
			throw std::runtime_error("Gateway did not activate after bootstrap became healthy");
		}

		void wait_for_gateway_state(
			rclcpp::executors::SingleThreadedExecutor &executor,
			std::uint8_t expected_state,
			const std::string &expected_source,
			std::uint16_t expected_reason,
			std::chrono::seconds timeout)
		{
			wait_until(
				executor,
				[this, expected_state, expected_source, expected_reason]()
				{
					return latest_gateway_state_.has_value() &&
						latest_gateway_state_->state == expected_state &&
						latest_gateway_state_->active_source_id == expected_source &&
						latest_gateway_state_->reason_code == expected_reason;
				},
				timeout,
				"GatewayState did not reach the expected state/source/reason tuple");
		}

		void wait_until(
			rclcpp::executors::SingleThreadedExecutor &executor,
			const std::function<bool()> &predicate,
			std::chrono::steady_clock::duration timeout,
			const std::string &failure)
		{
			const auto deadline = std::chrono::steady_clock::now() + timeout;
			while (rclcpp::ok() && std::chrono::steady_clock::now() < deadline)
			{
				executor.spin_some();
				publish_sources_if_due();
				if (predicate())
				{
					return;
				}
				std::this_thread::sleep_for(2ms);
			}
			throw std::runtime_error(failure);
		}

		void create_source(const std::string &source_id)
		{
			if (sources_.count(source_id) != 0U)
			{
				throw std::logic_error("source already exists: " + source_id);
			}
			const auto policy = bundle_->source_policy->sources.find(source_id);
			if (policy == bundle_->source_policy->sources.end())
			{
				throw std::out_of_range("source is absent from SourcePolicy: " + source_id);
			}
			sources_.emplace(
				source_id,
				ManagedSource{
					create_publisher<ControlCommand>(policy->second.topic, input_qos_.value()),
					0U});
		}

		void destroy_source(const std::string &source_id)
		{
			if (sources_.erase(source_id) != 1U)
			{
				throw std::logic_error("source does not exist: " + source_id);
			}
		}

		void create_ambiguous_planning_publisher()
		{
			if (ambiguous_planning_publisher_)
			{
				throw std::logic_error("ambiguous planning publisher already exists");
			}
			ambiguous_planning_sequence_ = 0U;
			ambiguous_planning_publisher_ = create_publisher<ControlCommand>(
				planning_topic(),
				input_qos_.value());
		}

		std::string planning_topic() const
		{
			return bundle_->source_policy->sources.at("planning").topic;
		}

		void publish_sources_if_due()
		{
			const auto now_steady = std::chrono::steady_clock::now();
			if (now_steady < next_source_publish_at_)
			{
				return;
			}
			next_source_publish_at_ = now_steady + 40ms;
			for (auto &[source_id, source] : sources_)
			{
				publish_source(source.publisher, source_id, source.sequence);
			}
			if (ambiguous_planning_publisher_)
			{
				publish_source(
					ambiguous_planning_publisher_,
					"planning",
					ambiguous_planning_sequence_);
			}
		}

		void publish_source(
			const rclcpp::Publisher<ControlCommand>::SharedPtr &publisher,
			const std::string &source_id,
			std::uint64_t &sequence)
		{
			if (sequence == std::numeric_limits<std::uint64_t>::max())
			{
				throw std::overflow_error("verification source sequence exhausted");
			}
			ControlCommand command;
			command.source_stamp = static_cast<builtin_interfaces::msg::Time>(
				get_clock()->now());
			command.source_id = source_id;
			command.source_sequence = ++sequence;
			command.mode = ControlCommand::MODE_NORMAL;
			command.linear_velocity_mps = source_id == "teleop" ? 0.35 : 0.20;
			command.angular_velocity_radps = source_id == "teleop" ? -0.10 : 0.05;
			publisher->publish(command);
		}

		control_link_contract::ContractBundlePtr bundle_;
		std::optional<rclcpp::QoS> input_qos_;
		std::optional<rclcpp::QoS> canonical_qos_;
		rclcpp::Subscription<GatewayState>::SharedPtr state_subscription_;
		rclcpp::Subscription<SourceStatus>::SharedPtr source_status_subscription_;
		rclcpp::Subscription<VehicleState>::SharedPtr vehicle_state_subscription_;
		rclcpp::Subscription<ControlCommand>::SharedPtr canonical_subscription_;
		rclcpp::Subscription<DiagnosticArray>::SharedPtr diagnostics_subscription_;
		rclcpp::Client<GetState>::SharedPtr get_state_client_;
		rclcpp::Client<ChangeState>::SharedPtr change_state_client_;
		rclcpp::Publisher<ControlCommand>::SharedPtr rogue_canonical_publisher_;
		rclcpp::Publisher<ControlCommand>::SharedPtr ambiguous_planning_publisher_;
		std::map<std::string, ManagedSource> sources_;
		std::map<std::string, SourceStatus> latest_source_status_;
		std::optional<GatewayState> latest_gateway_state_;
		std::optional<VehicleState> latest_vehicle_state_;
		std::optional<ControlCommand> latest_canonical_;
		std::chrono::steady_clock::time_point next_source_publish_at_{};
		std::uint64_t ambiguous_planning_sequence_{0U};
		std::uint64_t canonical_count_{0U};
		std::uint64_t hold_count_{0U};
		bool config_identity_observed_{false};
		std::string target_fqn_;
	};
}  // namespace

int main(int argc, char **argv)
{
	rclcpp::init(argc, argv);
	int exit_code = 1;
	try
	{
		auto verifier = std::make_shared<GatewayLifecycleVerifier>();
		rclcpp::executors::SingleThreadedExecutor executor;
		executor.add_node(verifier);
		verifier->run(executor);
		executor.remove_node(verifier);
		exit_code = 0;
	}
	catch (const std::exception &exception)
	{
		RCLCPP_FATAL(
			rclcpp::get_logger("gateway_lifecycle_verifier"),
			"E4 verification failed: %s",
			exception.what());
	}
	rclcpp::shutdown();
	return exit_code;
}
