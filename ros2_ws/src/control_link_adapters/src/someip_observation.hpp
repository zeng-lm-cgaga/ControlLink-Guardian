#pragma once

#include <cmath>
#include <cstdint>
#include <stdexcept>
#include <string>

#include "builtin_interfaces/msg/time.hpp"
#include "control_link_contract/model.hpp"
#include "control_link_interfaces/msg/vehicle_state.hpp"
#include "control_link_network_v1.pb.h"

namespace control_link_adapters
{

// 只投影原Contract的观察预算；SOME/IP需要显式预算，不改变通用Contract的optional语义。
struct SomeipObservationBudgets
{
	std::uint64_t gateway_ms;
	std::uint64_t source_ms;
	std::uint64_t vehicle_ms;
};

inline SomeipObservationBudgets someip_observation_budgets(
	const control_link_contract::GatewayContract &contract)
{
	const auto required_lease = [&](const std::string &group)
	{
		const auto &topic = contract.state_topics.at(group);
		if (!topic.qos_profile.has_value())
		{
			throw std::invalid_argument("SOME/IP observation requires a QoS profile for " + group);
		}
		const auto &lease = contract.qos_profiles.at(*topic.qos_profile).liveliness_lease_duration_ms;
		if (!lease.has_value() || *lease == 0)
		{
			throw std::invalid_argument("SOME/IP observation requires explicit positive liveliness_lease_duration_ms for " + group);
		}
		return *lease;
	};
	return {required_lease("gateway_state"), required_lease("source_status"),
		contract.gateway.vehicle_state_topic_timeout_ms};
}

inline std::int64_t someip_system_stamp_ns(const builtin_interfaces::msg::Time &stamp)
{
	return static_cast<std::int64_t>(stamp.sec) * 1'000'000'000LL + stamp.nanosec;
}

inline void set_someip_observation(
	control_link::network::v1::ObservationMetadata *observation,
	const builtin_interfaces::msg::Time &stamp, std::uint64_t received)
{
	namespace wire = control_link::network::v1;
	observation->set_observed_system_ns(someip_system_stamp_ns(stamp));
	observation->set_received_boottime_ns(received);
	observation->set_validity(stamp.nanosec < 1'000'000'000U &&
		someip_system_stamp_ns(stamp) > 0 && received > 0 ?
		wire::OBSERVATION_VALIDITY_VALID : wire::OBSERVATION_VALIDITY_INVALID);
}

// 仅检查VehicleState.msg的结构不变量并保留坏原值；准入、freshness和健康决策仍由Gateway负责。
inline void set_someip_vehicle_observation(
	control_link::network::v1::VehicleObservation *target,
	const control_link_interfaces::msg::VehicleState &message, std::uint64_t received)
{
	using VehicleState = control_link_interfaces::msg::VehicleState;
	namespace wire = control_link::network::v1;
	set_someip_observation(target->mutable_observation(), message.observed_at, received);
	const bool known_state = message.state == VehicleState::STANDBY ||
		message.state == VehicleState::RUNNING || message.state == VehicleState::SAFE_STOP ||
		message.state == VehicleState::FAULT;
	if (!known_state || !std::isfinite(message.linear_velocity_mps) ||
		!std::isfinite(message.angular_velocity_radps))
	{
		target->mutable_observation()->set_validity(wire::OBSERVATION_VALIDITY_INVALID);
	}
	target->set_state(message.state);
	target->set_fault_code(message.fault_code);
	target->set_linear_velocity_mps(message.linear_velocity_mps);
	target->set_angular_velocity_radps(message.angular_velocity_radps);
	target->set_rolling_counter(message.rolling_counter);
}

}
