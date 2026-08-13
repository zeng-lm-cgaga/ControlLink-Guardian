#include "control_link_gateway/command_validator.hpp"

#include <cmath>
#include <cstdint>
#include <limits>
#include <optional>
#include <stdexcept>
#include <utility>

#include "builtin_interfaces/msg/time.hpp"

namespace control_link_gateway
{
	namespace
	{
		CommandValidationResult reject(RejectReason reason)
		{
			return CommandValidationResult{
				reason,
				std::nullopt};
		}

		std::optional<CommandMode> parse_mode(std::uint8_t raw_mode) noexcept
		{
			switch (raw_mode)
			{
			case control_link_interfaces::msg::ControlCommand::MODE_NORMAL:
				return CommandMode::kNormal;

			case control_link_interfaces::msg::ControlCommand::MODE_HOLD:
				return CommandMode::kHold;

			default:
				return std::nullopt;
			}
		}

		constexpr std::uint32_t kNanosecondsPerSecond = 1'000'000'000U;
		constexpr std::uint64_t kNanosecondsPerMillisecond = 1'000'000ULL;

		std::int64_t to_nanoseconds(const builtin_interfaces::msg::Time &stamp) noexcept
		{
			return static_cast<std::int64_t>(stamp.sec) * kNanosecondsPerSecond +
				   static_cast<std::int64_t>(stamp.nanosec);
		}

		bool is_zero_stamp(const builtin_interfaces::msg::Time &stamp) noexcept
		{
			return stamp.sec == 0 && stamp.nanosec == 0U;
		}

		bool is_normalized_stamp(const builtin_interfaces::msg::Time &stamp) noexcept
		{
			// builtin_interfaces/Time 要求 nanosec 位于 [0, 1e9)，rosidl 类型本身不会强制该范围
			return stamp.nanosec < kNanosecondsPerSecond;
		}

		bool exceeds_milliseconds(std::uint64_t duration_ns, std::uint64_t limit_ms) noexcept
		{
			const auto max_value = std::numeric_limits<std::uint64_t>::max();

			// 阈值已经大于 duration_ns 的可表示范围，任何 duration 都不可能超过它
			if (limit_ms > max_value / kNanosecondsPerMillisecond)
				return false;

			const auto limit_ns = limit_ms * kNanosecondsPerMillisecond;
			return duration_ns > limit_ns;
		}

		std::optional<RejectReason> validate_motion(
			const control_link_interfaces::msg::ControlCommand &command,
			CommandMode mode,
			const control_link_contract::CommandLimits &limits) noexcept
		{
			if (
				!std::isfinite(command.linear_velocity_mps) ||
				!std::isfinite(command.angular_velocity_radps))
			{
				return RejectReason::kNonFinite;
			}

			if (mode == CommandMode::kHold)
			{
				if (
					command.linear_velocity_mps != 0.0 ||
					command.angular_velocity_radps != 0.0)
				{
					return RejectReason::kHoldNonzero;
				}

				return std::nullopt;
			}

			if (
				std::abs(command.linear_velocity_mps) > limits.max_abs_linear_velocity_mps ||
				std::abs(command.angular_velocity_radps) > limits.max_abs_angular_velocity_radps)
			{
				return RejectReason::kOutOfRange;
			}

			return std::nullopt;
		}

		SourceSnapshot make_snapshot(
			const control_link_interfaces::msg::ControlCommand &command,
			const CommandValidationContext &context,
			std::int64_t source_stamp_ns,
			CommandMode mode)
		{
			return SourceSnapshot{
				command.source_id,
				context.priority,
				context.publisher_generation,
				command.source_sequence,
				source_stamp_ns,
				mode,
				command.linear_velocity_mps,
				command.angular_velocity_radps,
				context.received_at};
		}

	} // namespace

	CommandValidationResult CommandValidator::validate(
		const control_link_interfaces::msg::ControlCommand &command,
		const CommandValidationContext &context) const
	{
		// 顺序有业务含义，只有身份、内容、时间和 sequence 全部通过后才创建快照
		if (std::string_view(command.source_id) != context.expected_source_id)
		{
			return reject(RejectReason::kSourceIdMismatch);
		}

		const auto command_mode = parse_mode(command.mode);
		if (command_mode == std::nullopt)
		{
			return reject(RejectReason::kUnknownMode);
		}

		const auto command_result = validate_motion(command, command_mode.value(), contract_->limits);
		if (command_result != std::nullopt)
		{
			return reject(command_result.value());
		}

		if (is_zero_stamp(command.source_stamp))
		{
			return reject(RejectReason::kZeroStamp);
		}

		if (!is_normalized_stamp(command.source_stamp))
		{
			return reject(RejectReason::kInvalidStamp);
		}

		if (!context.ros_clock_valid)
		{
			return reject(RejectReason::kClockInvalid);
		}

		if (context.now_ros_ns < 0)
		{
			return reject(RejectReason::kClockInvalid);
		}

		const auto source_stamp_ns = to_nanoseconds(command.source_stamp);
		if (source_stamp_ns < 0)
		{
			return reject(RejectReason::kStale);
		}

		if (source_stamp_ns > context.now_ros_ns)
		{
			const auto future_ns = static_cast<uint64_t>(source_stamp_ns - context.now_ros_ns);
			if (exceeds_milliseconds(future_ns, contract_->limits.max_future_skew_ms))
			{
				return reject(RejectReason::kFutureStamp);
			}
		}
		else
		{
			const auto age_ns = static_cast<uint64_t>(context.now_ros_ns - source_stamp_ns);
			if (exceeds_milliseconds(age_ns, contract_->gateway.command_timeout_ms))
			{
				return reject(RejectReason::kStale);
			}
		}

		if (context.last_accepted_sequence.has_value())
		{
			if (command.source_sequence <= context.last_accepted_sequence.value())
			{
				return reject(RejectReason::kSequenceNotIncreasing);
			}
		}

		const auto snapshot = make_snapshot(command, context, source_stamp_ns, command_mode.value());

		return CommandValidationResult{RejectReason::kNone, snapshot};
	}

	CommandValidator::CommandValidator(control_link_contract::GatewayContractPtr contract)
		: contract_(std::move(contract))
	{
		if (!contract_)
		{
			throw std::invalid_argument("CommandValidator requires a non-null GatewayContract");
		}
	}

	bool CommandValidationResult::accepted() const noexcept
	{
		return reason == RejectReason::kNone &&
			   snapshot.has_value();
	}
} // namespace control_link_gateway
