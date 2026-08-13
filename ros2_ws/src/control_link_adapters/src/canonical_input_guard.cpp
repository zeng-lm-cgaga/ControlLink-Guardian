#include "control_link_adapters/canonical_input_guard.hpp"

#include <cmath>
#include <cstdint>
#include <optional>
#include <stdexcept>
#include <utility>
#include <limits>

#include "builtin_interfaces/msg/time.hpp"

namespace control_link_adapters
{
	namespace
	{
		CanonicalValidationResult reject(
			CanonicalRejectReason reason)
		{
			return CanonicalValidationResult{reason};
		}

		bool same_publisher(
			const CanonicalPublisherKey &left,
			const CanonicalPublisherKey &right) noexcept
		{
			return left.rmw_implementation == right.rmw_implementation && left.publisher_gid == right.publisher_gid;
		}

		std::optional<CanonicalRejectReason> validate_endpoint(
			const CanonicalPublisherKey &actual_publisher,
			const CanonicalEndpointSnapshot &endpoint,
			const std::string &expected_node_fqn)
		{
			if (endpoint.state == CanonicalEndpointState::kUnavailable)
			{
				return CanonicalRejectReason::kEndpointUnavailable;
			}

			if (endpoint.state == CanonicalEndpointState::kUnstable)
			{
				return CanonicalRejectReason::kEndpointUnstable;
			}

			if (endpoint.state == CanonicalEndpointState::kAmbiguous)
			{
				return CanonicalRejectReason::kEndpointAmbiguous;
			}

			if (endpoint.state == CanonicalEndpointState::kConfirmed)
			{
				if (!endpoint.confirmed_publisher.has_value())
				{
					throw std::logic_error("confirmed canonical endpoint has no publisher key");
				}

				if (endpoint.node_fqn != expected_node_fqn)
				{
					return CanonicalRejectReason::kPublisherMismatch;
				}

				if (!same_publisher(actual_publisher, endpoint.confirmed_publisher.value()))
				{
					return CanonicalRejectReason::kPublisherMismatch;
				}

				return std::nullopt;
			}

			throw std::logic_error(
				"CanonicalInputGuard received an unknown endpoint state");
		}

		enum class CanonicalCommandMode : std::uint8_t
		{
			kNormal,
			kHold,
		};

		constexpr std::uint32_t kNanosecondsPerSecond = 1'000'000'000U;
		constexpr std::uint64_t kNanosecondsPerMillisecond = 1'000'000ULL;

		std::optional<CanonicalCommandMode> parse_mode(
			std::uint8_t raw_mode) noexcept
		{
			switch (raw_mode)
			{
			case control_link_interfaces::msg::ControlCommand::MODE_NORMAL:
				return CanonicalCommandMode::kNormal;

			case control_link_interfaces::msg::ControlCommand::MODE_HOLD:
				return CanonicalCommandMode::kHold;

			default:
				return std::nullopt;
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
			// rosidl 生成类型只保存字段，不会强制 nanosec 的合法范围
			return stamp.nanosec < kNanosecondsPerSecond;
		}

		std::optional<CanonicalRejectReason> validate_motion(
			const control_link_interfaces::msg::ControlCommand &command,
			CanonicalCommandMode mode,
			const control_link_contract::CommandLimits &limits) noexcept
		{
			// NaN 与范围阈值比较会得到 false，必须先完成 finite 校验
			if (
				!std::isfinite(command.linear_velocity_mps) ||
				!std::isfinite(command.angular_velocity_radps))
			{
				return CanonicalRejectReason::kNonFinite;
			}

			if (mode == CanonicalCommandMode::kHold)
			{
				if (
					command.linear_velocity_mps != 0.0 ||
					command.angular_velocity_radps != 0.0)
				{
					return CanonicalRejectReason::kHoldNonzero;
				}

				return std::nullopt;
			}

			if (
				std::abs(command.linear_velocity_mps) > limits.max_abs_linear_velocity_mps ||
				std::abs(command.angular_velocity_radps) > limits.max_abs_angular_velocity_radps)
			{
				return CanonicalRejectReason::kOutOfRange;
			}

			return std::nullopt;
		}

		std::optional<CanonicalRejectReason> validate_metadata(
			const control_link_interfaces::msg::ControlCommand &command,
			CanonicalCommandMode mode) noexcept
		{
			if (!is_normalized_stamp(command.source_stamp))
			{
				return CanonicalRejectReason::kInvalidStamp;
			}

			const bool has_source = !command.source_id.empty();
			const bool zero_stamp = is_zero_stamp(command.source_stamp);

			if (mode == CanonicalCommandMode::kNormal)
			{
				if (!has_source || zero_stamp)
				{
					return CanonicalRejectReason::kInvalidMetadata;
				}

				return std::nullopt;
			}

			// HOLD 只能保留完整来源元数据，或使用 Gateway 生成的全空来源形式
			if (has_source)
			{
				if (zero_stamp)
				{
					return CanonicalRejectReason::kInvalidMetadata;
				}
				if (command.source_stamp.sec < 0)
				{
					// 来源型 HOLD 可以过期，但不可能来自 Gateway 的合法负时间快照
					return CanonicalRejectReason::kInvalidStamp;
				}

				return std::nullopt;
			}

			if (command.source_sequence != 0U || !zero_stamp)
			{
				return CanonicalRejectReason::kInvalidMetadata;
			}

			return std::nullopt;
		}

		std::int64_t to_nanoseconds(
			const builtin_interfaces::msg::Time &stamp) noexcept
		{
			return static_cast<std::int64_t>(stamp.sec) * kNanosecondsPerSecond + stamp.nanosec;
		}

		bool exceeds_milliseconds(
			std::uint64_t duration_ns,
			std::uint64_t limit_ms) noexcept
		{
			if (limit_ms > std::numeric_limits<std::uint64_t>::max() / kNanosecondsPerMillisecond)
			{
				return false;
			}

			const auto limit_ns = limit_ms * kNanosecondsPerMillisecond;
			return duration_ns > limit_ns;
		}

		std::optional<CanonicalRejectReason> validate_normal_time(
			const control_link_interfaces::msg::ControlCommand &command,
			std::int64_t now_ros_ns,
			bool ros_clock_healthy,
			const control_link_contract::CommandLimits &limits,
			std::uint64_t command_timeout_ms) noexcept
		{
			if (!ros_clock_healthy)
			{
				return CanonicalRejectReason::kClockInvalid;
			}

			if (now_ros_ns < 0)
			{
				return CanonicalRejectReason::kClockInvalid;
			}

			const auto source_stamp_ns = to_nanoseconds(command.source_stamp);
			if (source_stamp_ns < 0)
			{
				return CanonicalRejectReason::kStale;
			}

			if (source_stamp_ns > now_ros_ns)
			{
				const auto future_ns = source_stamp_ns - now_ros_ns;
				if (exceeds_milliseconds(future_ns, limits.max_future_skew_ms))
				{
					return CanonicalRejectReason::kFutureStamp;
				}
			}
			else
			{
				const auto age_ns = now_ros_ns - source_stamp_ns;
				if (exceeds_milliseconds(age_ns, command_timeout_ms))
				{
					return CanonicalRejectReason::kStale;
				}
			}

			return std::nullopt;
		}
	} // namespace

	bool CanonicalValidationResult::accepted() const noexcept
	{
		return reason == CanonicalRejectReason::kNone;
	}

	CanonicalInputGuard::CanonicalInputGuard(
		control_link_contract::GatewayContractPtr contract)
		: contract_(std::move(contract))
	{
		if (contract_ == nullptr)
		{
			throw std::invalid_argument("CanonicalInputGuard requires a non-null GatewayContract");
		}
	}

	CanonicalValidationResult CanonicalInputGuard::validate(
		const control_link_interfaces::msg::ControlCommand &command,
		const CanonicalPublisherKey &actual_publisher,
		const CanonicalEndpointSnapshot &endpoint,
		std::int64_t now_ros_ns,
		bool ros_clock_healthy) const
	{
		const auto endpoint_reject =
			validate_endpoint(
				actual_publisher,
				endpoint,
				contract_->gateway.node_fqn);
		if(endpoint_reject.has_value())
		{
			return reject(endpoint_reject.value());
		}

		const auto mode = parse_mode(command.mode);
		if(!mode.has_value())
		{
			return reject(CanonicalRejectReason::kUnknownMode);

		}

			const auto motion_reject =
			validate_motion(
				command,
				mode.value(),
				contract_->limits);
		if(motion_reject.has_value())
		{
			return reject(motion_reject.value());
		}

		const auto metadata_reject =
			validate_metadata(command, mode.value());

		if (metadata_reject.has_value())
		{
			return reject(metadata_reject.value());
		}

		if(mode.value() == CanonicalCommandMode::kNormal)
		{
			const auto time_reject =
				validate_normal_time(
					command,
					now_ros_ns,
					ros_clock_healthy,
					contract_->limits,
					contract_->gateway.command_timeout_ms);
			if(time_reject.has_value())
			{
				return reject(time_reject.value());
			}
		}

		return CanonicalValidationResult{CanonicalRejectReason::kNone};
	}
} // namespace control_link_adapters
