#include <algorithm>
#include <chrono>
#include <cstdint>
#include <filesystem>
#include <memory>
#include <stdexcept>
#include <string>
#include <variant>

#include "control_link_adapters/graph_admission_policy.hpp"
#include "control_link_contract/contract_bundle.hpp"
#include "control_link_contract/qos_factory.hpp"
#include "control_link_interfaces/msg/control_command.hpp"
#include "control_link_interfaces/msg/gateway_state.hpp"
#include "control_link_interfaces/msg/source_status.hpp"
#include "control_link_interfaces/msg/vehicle_state.hpp"
#include "control_link_someip_transport/ingress.hpp"
#include "control_link_someip_transport/server.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rmw/rmw.h"

namespace control_link_adapters
{
namespace net = control_link::someip;
namespace wire = control_link::network::v1;
using ControlCommand = control_link_interfaces::msg::ControlCommand;
using GatewayState = control_link_interfaces::msg::GatewayState;
using SourceStatus = control_link_interfaces::msg::SourceStatus;
using VehicleState = control_link_interfaces::msg::VehicleState;
using SteadyClock = std::chrono::steady_clock;

static std::int64_t system_stamp_ns(const builtin_interfaces::msg::Time &stamp)
{
	return static_cast<std::int64_t>(stamp.sec) * 1'000'000'000LL + stamp.nanosec;
}

static void set_observation(
	wire::ObservationMetadata *observation,
	const builtin_interfaces::msg::Time &stamp,
	std::uint64_t received)
{
	observation->set_observed_system_ns(system_stamp_ns(stamp));
	observation->set_received_boottime_ns(received);
	observation->set_validity(stamp.nanosec < 1'000'000'000U &&
		system_stamp_ns(stamp) > 0 && received > 0 ?
		wire::OBSERVATION_VALIDITY_VALID : wire::OBSERVATION_VALIDITY_INVALID);
}

class SomeipPlanningBridge final : public rclcpp::Node
{
public:
	SomeipPlanningBridge() : Node("someip_planning_bridge")
	{
		const auto config_root = declare_parameter<std::string>("config_root", "");
		const auto profile_path = declare_parameter<std::string>("profile_path", "");
		const auto network_path = declare_parameter<std::string>("network_config_path", "");
		const auto vsomeip_path = declare_parameter<std::string>("vsomeip_config_path", "");
		if (config_root.empty() || profile_path.empty() || network_path.empty() || vsomeip_path.empty())
		{
			throw std::invalid_argument("SOME/IP requires explicit config_root/profile/network/vsomeip configuration paths");
		}
		bundle_ = control_link_contract::load_contract_bundle(profile_path, config_root);
		const auto &profile = std::get<control_link_contract::AdasProfile>(*bundle_->profile);
		if (profile.common.clock_mode != control_link_contract::ClockMode::kSystem)
		{
			throw std::invalid_argument("SOME/IP planning ingress requires SYSTEM clock");
		}
		const auto expected_network_path = std::filesystem::canonical(
			std::filesystem::path{config_root} / "adas/protocol_transports.yaml");
		if (std::filesystem::canonical(network_path) != expected_network_path)
		{
			throw std::invalid_argument("network configuration must be the config_root/adas owner");
		}
		config_ = net::load_config(network_path);
		const auto &policy = bundle_->source_policy->sources.at("planning");
		if (policy.type != "control_link_interfaces/msg/ControlCommand" ||
			std::find(profile.common.enabled_sources.begin(), profile.common.enabled_sources.end(),
				"planning") == profile.common.enabled_sources.end())
		{
			throw std::invalid_argument("SOME/IP planning requires enabled ControlCommand source");
		}
		source_topic_ = policy.topic;
		wire::Capabilities capabilities;
		capabilities.set_schema_version(1);
		capabilities.set_contract_id(bundle_->identity.contract.contract_id);
		capabilities.set_contract_hash(bundle_->identity.contract.contract_hash);
		capabilities.set_decision_config_hash(bundle_->identity.decision_config.decision_config_hash);
		capabilities.set_planning_transport(wire::PLANNING_TRANSPORT_SOMEIP);
		capabilities.set_max_command_bytes(control_link_edge_protocol::kCommandFrameSize);
		capabilities.set_max_status_bytes(config_.max_payload_bytes);
		capabilities.set_max_pending_targets(1);
		capabilities.set_max_status_subscribers(config_.readonly_inflight_limit);
		ingress_ = std::make_shared<net::PlanningIngress>(config_, capabilities);
		resource_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
		feedback_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
		create_feedback_subscriptions();
		server_ = std::make_unique<net::SomeipServer>(config_, ingress_, vsomeip_path);
		server_->start();
		timer_ = create_wall_timer(std::chrono::milliseconds{5},
			[this]() { resource_tick(); }, resource_group_);
		RCLCPP_INFO(get_logger(), "SOME/IP planning ingress running; network offer is not Gateway readiness");
	}

	~SomeipPlanningBridge() override
	{
		ingress_->revoke("STOP");
		timer_.reset();
		server_->stop();
		publisher_.reset();
		ingress_->finish_release(ingress_->snapshot().epoch);
	}

private:
	void create_feedback_subscriptions()
	{
		control_link_contract::QosFactory factory{bundle_->gateway_contract};
		rclcpp::SubscriptionOptions options;
		options.callback_group = feedback_group_;
		const auto &topics = bundle_->gateway_contract->state_topics;
		const auto &gateway_topic = topics.at("gateway_state");
		const auto &source_topic = topics.at("source_status");
		const auto &vehicle_topic = topics.at("vehicle_state");
		gateway_subscription_ = create_subscription<GatewayState>(gateway_topic.topic,
			factory.make(gateway_topic.qos_profile.value()),
			[this](GatewayState::ConstSharedPtr message)
			{
				const auto received = net::boottime_now_ns();
				ingress_->update_observations([&](wire::StatusSnapshot &status)
				{
					auto *target = status.mutable_gateway();
					set_observation(target->mutable_observation(), message->observed_at, received);
					target->set_state(message->state);
					target->set_reason_code(message->reason_code);
					target->set_active_source_id(message->active_source_id);
					target->set_active_source_sequence(message->active_source_sequence);
					target->set_active_command_age_ms(message->active_command_age_ms);
					target->set_recovery_valid_count(message->recovery_valid_count);
					target->set_transition_sequence(message->transition_sequence);
				});
			}, options);
		source_subscription_ = create_subscription<SourceStatus>(source_topic.topic,
			factory.make(source_topic.qos_profile.value()),
			[this](SourceStatus::ConstSharedPtr message)
			{
				if (message->source_id != "planning") return;
				const auto received = net::boottime_now_ns();
				ingress_->update_observations([&](wire::StatusSnapshot &status)
				{
					status.clear_sources();
					auto *target = status.add_sources();
					set_observation(target->mutable_observation(), message->observed_at, received);
					target->set_source_id(message->source_id);
					target->set_priority(message->priority);
					target->set_enabled(message->enabled);
					target->set_lease_valid(message->lease_valid);
					target->set_command_valid(message->command_valid);
					target->set_last_reject_reason(message->last_reject_reason);
					target->set_last_source_sequence(message->last_source_sequence);
					target->set_command_age_ms(message->command_age_ms);
					target->set_accepted_count(message->accepted_count);
					target->set_rejected_count(message->rejected_count);
					target->set_endpoint_stable(message->endpoint_stable);
					target->set_endpoint_state(message->endpoint_state);
					target->set_endpoint_publisher_count(message->endpoint_publisher_count);
					target->set_endpoint_generation_valid(message->endpoint_generation_valid);
					auto *identity = target->mutable_endpoint_publisher();
					identity->set_gid(message->endpoint_publisher_gid.data(), message->endpoint_publisher_gid.size());
					identity->set_rmw_implementation(rmw_get_implementation_identifier());
				});
			}, options);
		vehicle_subscription_ = create_subscription<VehicleState>(vehicle_topic.topic,
			factory.make(vehicle_topic.qos_profile.value()),
			[this](VehicleState::ConstSharedPtr message)
			{
				const auto received = net::boottime_now_ns();
				ingress_->update_observations([&](wire::StatusSnapshot &status)
				{
					auto *target = status.mutable_vehicle();
					set_observation(target->mutable_observation(), message->observed_at, received);
					target->set_state(message->state);
					target->set_fault_code(message->fault_code);
					target->set_linear_velocity_mps(message->linear_velocity_mps);
					target->set_angular_velocity_radps(message->angular_velocity_radps);
					target->set_rolling_counter(message->rolling_counter);
				});
			}, options);
	}

	bool source_ready(const wire::StatusSnapshot &status, const std::string &gid) const
	{
		if (status.sources_size() != 1) return false;
		const auto &source = status.sources(0);
		const auto now_boot = net::boottime_now_ns();
		const auto now_system = net::system_now_ns();
		const auto &observation = source.observation();
		const auto &topic = bundle_->gateway_contract->state_topics.at("source_status");
		const auto budget_ns = bundle_->gateway_contract->qos_profiles.at(
			topic.qos_profile.value()).liveliness_lease_duration_ms.value() * 1'000'000ULL;
		if (observation.received_boottime_ns() == 0 || now_boot < observation.received_boottime_ns() ||
			now_boot - observation.received_boottime_ns() > budget_ns || now_system <= 0 ||
			observation.observed_system_ns() <= 0 ||
			(observation.observed_system_ns() > now_system &&
				static_cast<std::uint64_t>(observation.observed_system_ns() - now_system) >
				bundle_->gateway_contract->limits.max_future_skew_ms * 1'000'000ULL) ||
			(now_system >= observation.observed_system_ns() &&
				static_cast<std::uint64_t>(now_system - observation.observed_system_ns()) > budget_ns)) return false;
		return source.observation().validity() == wire::OBSERVATION_VALIDITY_VALID &&
			source.endpoint_stable() && source.endpoint_state() == SourceStatus::ENDPOINT_USABLE &&
			source.endpoint_publisher_count() == 1 && source.endpoint_generation_valid() &&
			source.endpoint_publisher().gid() == gid;
	}

	void expire_observations()
	{
		ingress_->update_observations([&](wire::StatusSnapshot &status)
		{
			const auto boot = net::boottime_now_ns();
			const auto system = net::system_now_ns();
			auto expire = [&](wire::ObservationMetadata *metadata, std::uint64_t timeout_ms)
			{
				if (metadata->validity() != wire::OBSERVATION_VALIDITY_VALID) return;
				const auto received = metadata->received_boottime_ns();
				const auto observed = metadata->observed_system_ns();
				if (boot == 0 || boot < received || system <= 0)
				{
					metadata->set_validity(wire::OBSERVATION_VALIDITY_UNKNOWN);
				}
				else if (observed > system && static_cast<std::uint64_t>(observed - system) >
					bundle_->gateway_contract->limits.max_future_skew_ms * 1'000'000ULL)
				{
					metadata->set_validity(wire::OBSERVATION_VALIDITY_INVALID);
				}
				else if (boot - received > timeout_ms * 1'000'000ULL ||
					(system >= observed && static_cast<std::uint64_t>(system - observed) > timeout_ms * 1'000'000ULL))
				{
					metadata->set_validity(wire::OBSERVATION_VALIDITY_STALE);
				}
			};
			const auto &contract = *bundle_->gateway_contract;
			const auto state_budget = [&](const std::string &group)
			{
				const auto &topic = contract.state_topics.at(group);
				return contract.qos_profiles.at(topic.qos_profile.value()).liveliness_lease_duration_ms.value();
			};
			if (status.has_gateway()) expire(status.mutable_gateway()->mutable_observation(), state_budget("gateway_state"));
			for (auto &source : *status.mutable_sources()) expire(source.mutable_observation(), state_budget("source_status"));
			if (status.has_vehicle()) expire(status.mutable_vehicle()->mutable_observation(), contract.gateway.vehicle_state_topic_timeout_ms);
		});
	}

	void resource_tick()
	{
		if (!rclcpp::ok())
		{
			ingress_->revoke("ROS_STOP");
			return;
		}
		expire_observations();
		const auto current = ingress_->snapshot();
		if (current.state == wire::CONTROL_SESSION_STATE_REVOKING)
		{
			publisher_.reset();
			publisher_gid_.clear();
			admission_pending_ = false;
			ingress_->finish_release(current.epoch);
			return;
		}
		if (current.state == wire::CONTROL_SESSION_STATE_IDLE) return;
		const auto now = SteadyClock::now();
		if (admission_epoch_ != current.epoch)
		{
			publisher_.reset();
			publisher_gid_.clear();
			admission_epoch_ = current.epoch;
			admission_pending_ = true;
			deadline_ = now + std::chrono::milliseconds{config_.graph_timeout_ms};
			quiet_since_ = SteadyClock::time_point::min();
			next_probe_ = now;
			RCLCPP_INFO(get_logger(), "Graph admission opening epoch=%llu deadline_ms=%u",
				static_cast<unsigned long long>(current.epoch), config_.graph_timeout_ms);
		}
		if (admission_pending_)
		{
			const auto result = graph_admission_probe_failure_result(now, deadline_, true,
				current.epoch == admission_epoch_, !rclcpp::ok());
			if (result == GraphAdmissionProbeResult::kTimedOut)
			{
				if (ingress_->revoke_if_current(admission_epoch_, "GRAPH_ADMISSION_TIMEOUT"))
				{
					RCLCPP_ERROR(get_logger(), "Graph admission timed out epoch=%llu; fail-closed",
						static_cast<unsigned long long>(admission_epoch_));
				}
				return;
			}
			if (result != GraphAdmissionProbeResult::kWaiting) return;
			if (now < next_probe_) return;
			next_probe_ = now + std::chrono::milliseconds{config_.graph_poll_ms};
			if (!publisher_)
			{
				try
				{
					const auto peers = get_publishers_info_by_topic(source_topic_);
					if (!peers.empty())
					{
						quiet_since_ = SteadyClock::time_point::min();
						RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
							"Graph admission waiting; planning publisher count=%zu", peers.size());
						return;
					}
					if (quiet_since_ == SteadyClock::time_point::min()) quiet_since_ = now;
					const auto quiet_ms = std::max<std::uint64_t>(config_.graph_quiet_ms,
						bundle_->gateway_contract->gateway.graph_stable_window_ms +
						bundle_->gateway_contract->gateway.graph_poll_ms);
					if (!graph_admission_quiet_window_complete(true, quiet_since_, SteadyClock::now(),
						std::chrono::milliseconds{quiet_ms}, deadline_)) return;
					if (!get_publishers_info_by_topic(source_topic_).empty())
					{
						quiet_since_ = SteadyClock::time_point::min();
						return;
					}
					control_link_contract::QosFactory factory{bundle_->gateway_contract};
					publisher_ = create_publisher<ControlCommand>(source_topic_,
						factory.make(bundle_->gateway_contract->input.qos_profile));
					const auto gid = publisher_->get_gid();
					publisher_gid_.assign(reinterpret_cast<const char *>(gid.data), RMW_GID_STORAGE_SIZE);
					if (!ingress_->bind_publisher(admission_epoch_, publisher_gid_, rmw_get_implementation_identifier()))
					{
						publisher_.reset();
						publisher_gid_.clear();
					}
				}
				catch (const std::exception &error)
				{
					quiet_since_ = SteadyClock::time_point::min();
					RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000, "Graph probe/create failed: %s", error.what());
				}
				return;
			}
			if (source_ready(ingress_->readonly_status(), publisher_gid_))
			{
				if (ingress_->set_admitted(admission_epoch_, true,
					[this](const wire::StatusSnapshot &latest)
					{
						return rclcpp::ok() && SteadyClock::now() < deadline_ &&
							source_ready(latest, publisher_gid_);
					}))
				{
					admission_pending_ = false;
					RCLCPP_INFO(get_logger(), "Gateway admitted SOME/IP epoch=%llu stable USABLE count=1 actual GID",
						static_cast<unsigned long long>(admission_epoch_));
				}
			}
			return;
		}
		const bool ready = publisher_ && source_ready(ingress_->readonly_status(), publisher_gid_);
		ingress_->set_admitted(current.epoch, ready,
			[this](const wire::StatusSnapshot &latest)
			{
				return rclcpp::ok() && source_ready(latest, publisher_gid_);
			});
		if (!ready) return;
		try
		{
			ingress_->publish_pending(current.epoch, net::boottime_now_ns(),
				[this](const control_link_edge_protocol::CommandFrame &frame)
				{
					ControlCommand command;
					auto seconds = frame.source_stamp_ns / 1'000'000'000LL;
					auto remainder = frame.source_stamp_ns % 1'000'000'000LL;
					if (remainder < 0) { --seconds; remainder += 1'000'000'000LL; }
					command.source_stamp.sec = static_cast<std::int32_t>(seconds);
					command.source_stamp.nanosec = static_cast<std::uint32_t>(remainder);
					command.source_id = "planning";
					command.source_sequence = frame.command_sequence;
					command.mode = static_cast<std::uint8_t>(frame.mode);
					command.linear_velocity_mps = frame.linear_velocity_mps;
					command.angular_velocity_radps = frame.angular_velocity_radps;
					publisher_->publish(command);
				}, [this](const wire::StatusSnapshot &latest)
				{
					return rclcpp::ok() && source_ready(latest, publisher_gid_);
				});
		}
		catch (const std::exception &error)
		{
			ingress_->revoke_if_current(current.epoch, "ROS_PUBLISH_FAILURE");
			RCLCPP_ERROR(get_logger(), "SOME/IP ROS publish failed: %s", error.what());
		}
	}

	control_link_contract::ContractBundlePtr bundle_;
	net::Config config_;
	std::shared_ptr<net::PlanningIngress> ingress_;
	std::unique_ptr<net::SomeipServer> server_;
	std::string source_topic_;
	rclcpp::CallbackGroup::SharedPtr resource_group_;
	rclcpp::CallbackGroup::SharedPtr feedback_group_;
	rclcpp::Publisher<ControlCommand>::SharedPtr publisher_;
	rclcpp::Subscription<GatewayState>::SharedPtr gateway_subscription_;
	rclcpp::Subscription<SourceStatus>::SharedPtr source_subscription_;
	rclcpp::Subscription<VehicleState>::SharedPtr vehicle_subscription_;
	rclcpp::TimerBase::SharedPtr timer_;
	std::string publisher_gid_;
	std::uint64_t admission_epoch_{0};
	bool admission_pending_{false};
	SteadyClock::time_point deadline_{};
	SteadyClock::time_point quiet_since_{SteadyClock::time_point::min()};
	SteadyClock::time_point next_probe_{};
};
}

int main(int argc, char **argv)
{
	rclcpp::init(argc, argv);
	try
	{
		auto node = std::make_shared<control_link_adapters::SomeipPlanningBridge>();
		rclcpp::executors::MultiThreadedExecutor executor{rclcpp::ExecutorOptions{}, 2};
		executor.add_node(node);
		executor.spin();
		executor.remove_node(node);
		node.reset();
		rclcpp::shutdown();
		return 0;
	}
	catch (const std::exception &error)
	{
		RCLCPP_FATAL(rclcpp::get_logger("someip_planning_bridge"), "%s", error.what());
		rclcpp::shutdown();
		return 1;
	}
}
