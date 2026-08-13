#include "control_link_gateway/vehicle_state_validator.hpp"

#include <cmath>
#include <cstdint>
#include <limits>
#include <optional>
#include <utility>
#include <stdexcept>

#include "builtin_interfaces/msg/time.hpp"

namespace control_link_gateway
{
	namespace
	{
		constexpr std::uint32_t kNanosecondsPerSecond = 1'000'000'000U;
		constexpr std::uint64_t kNanosecondsPerMillisecond = 1'000'000ULL;

		VehicleStateValidationResult reject(
			VehicleStateRejectReason reason)
		{
			return VehicleStateValidationResult{
				reason,
				std::nullopt};
		}

		bool known_state(std::uint8_t raw_state) noexcept
		{
			switch (raw_state)
			{
			case control_link_interfaces::msg::VehicleState::STANDBY:
			case control_link_interfaces::msg::VehicleState::RUNNING:
			case control_link_interfaces::msg::VehicleState::SAFE_STOP:
			case control_link_interfaces::msg::VehicleState::FAULT:
				return true;

			default:
				return false;
			}
		}

		bool is_zero_stamp(
			const builtin_interfaces::msg::Time &stamp) noexcept
		{
			return stamp.sec == 0 && stamp.nanosec == 0U;
		}

		bool is_normalized_stamp(
			const builtin_interfaces::msg::Time &stamp) noexcept
		{
			return stamp.nanosec < kNanosecondsPerSecond;
		}

		std::int64_t to_nanoseconds(
			const builtin_interfaces::msg::Time &stamp) noexcept
		{
			return static_cast<std::int64_t>(stamp.sec) *
					   kNanosecondsPerSecond +
				   static_cast<std::int64_t>(stamp.nanosec);
		}

		bool exceeds_milliseconds(
			std::uint64_t duration_ns,
			std::uint64_t limit_ms) noexcept
		{
			const auto maximum = std::numeric_limits<std::uint64_t>::max();
			if (limit_ms > maximum / kNanosecondsPerMillisecond)
			{
				return false;
			}

			return duration_ns > limit_ms * kNanosecondsPerMillisecond;
		}

		VehicleStateSnapshot make_snapshot(
			const control_link_interfaces::msg::VehicleState &state,
			const VehicleStateValidationContext &context,
			std::int64_t observed_at_ns)
		{
			return VehicleStateSnapshot{
				context.publisher_generation,
				observed_at_ns,
				state.state,
				state.fault_code,
				state.linear_velocity_mps,
				state.angular_velocity_radps,
				state.rolling_counter,
				context.received_at};
		}
	} // namespace

	bool VehicleStateValidationResult::accepted() const noexcept
	{
		return reason == VehicleStateRejectReason::kNone && snapshot.has_value();
	}

	VehicleStateValidator::VehicleStateValidator(control_link_contract::GatewayContractPtr contract)
		: contract_(std::move(contract))
	{
		if (!contract_)
		{
			throw std::invalid_argument(
				"VehicleStateValidator requires a non-null GatewayContract");
		}
	}

	VehicleStateValidationResult VehicleStateValidator::validate(
		const control_link_interfaces::msg::VehicleState &state,
		const VehicleStateValidationContext &context) const
	{
		// 只有枚举、数值、时间结构和 freshness 全部通过后才创建可信快照
		if (!known_state(state.state))
		{
			return reject(VehicleStateRejectReason::kUnknownState);
		}

		if (!std::isfinite(state.linear_velocity_mps) ||
			!std::isfinite(state.angular_velocity_radps))
		{
			return reject(VehicleStateRejectReason::kNonFinite);
		}

		if (is_zero_stamp(state.observed_at))
		{
			return reject(VehicleStateRejectReason::kZeroStamp);
		}

		if (!is_normalized_stamp(state.observed_at))
		{
			return reject(VehicleStateRejectReason::kInvalidStamp);
		}

		if (!context.ros_clock_valid || context.now_ros_ns < 0)
		{
			return reject(VehicleStateRejectReason::kClockInvalid);
		}

		const auto observed_at_ns = to_nanoseconds(state.observed_at);
		if (observed_at_ns < 0)
		{
			return reject(VehicleStateRejectReason::kStale);
		}

		if (observed_at_ns > context.now_ros_ns)
		{
			const auto future_ns = static_cast<std::uint64_t>(
				observed_at_ns - context.now_ros_ns);
			if (exceeds_milliseconds(
					future_ns,
					contract_->limits.max_future_skew_ms))
			{
				return reject(VehicleStateRejectReason::kFutureStamp);
			}
		}
		else
		{
			const auto age_ns = static_cast<std::uint64_t>(
				context.now_ros_ns - observed_at_ns);
			if (exceeds_milliseconds(
					age_ns,
					contract_->gateway.vehicle_state_topic_timeout_ms))
			{
				return reject(VehicleStateRejectReason::kStale);
			}
		}

		return VehicleStateValidationResult{
			VehicleStateRejectReason::kNone,
			make_snapshot(state, context, observed_at_ns)};
	}
} // namespace control_link_gateway
