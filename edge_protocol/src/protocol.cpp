#include "control_link_edge_protocol/protocol.hpp"

#include <cerrno>
#include <cmath>
#include <cstring>
#include <limits>

#include <sys/random.h>
#include <time.h>

namespace control_link_edge_protocol
{
	namespace
	{
		constexpr std::size_t kCommandReservedOffset = 42U;
		constexpr std::size_t kCommandReservedSize = 6U;
		constexpr std::size_t kFeedbackReservedOffset = 50U;
		constexpr std::size_t kFeedbackReservedSize = 2U;
		constexpr std::uint32_t kKnownFeedbackFlags =
			kGatewayStateValid | kPlanningStatusValid | kVehicleStateValid | kSessionReady;

		void set_error(DecodeError *error, DecodeError value) noexcept
		{
			if (error != nullptr)
			{
				*error = value;
			}
		}

		void put_u16(std::byte *output, std::size_t offset, std::uint16_t value) noexcept
		{
			output[offset] = static_cast<std::byte>(value & 0xFFU);
			output[offset + 1U] = static_cast<std::byte>((value >> 8U) & 0xFFU);
		}

		void put_u32(std::byte *output, std::size_t offset, std::uint32_t value) noexcept
		{
			for (std::size_t index = 0U; index < 4U; ++index)
			{
				output[offset + index] =
					static_cast<std::byte>((value >> (index * 8U)) & 0xFFU);
			}
		}

		void put_u64(std::byte *output, std::size_t offset, std::uint64_t value) noexcept
		{
			for (std::size_t index = 0U; index < 8U; ++index)
			{
				output[offset + index] =
					static_cast<std::byte>((value >> (index * 8U)) & 0xFFU);
			}
		}

		std::uint16_t get_u16(const std::byte *input, std::size_t offset) noexcept
		{
			return static_cast<std::uint16_t>(
				std::to_integer<std::uint8_t>(input[offset])) |
				static_cast<std::uint16_t>(
					std::to_integer<std::uint8_t>(input[offset + 1U]) << 8U);
		}

		std::uint32_t get_u32(const std::byte *input, std::size_t offset) noexcept
		{
			std::uint32_t value = 0U;
			for (std::size_t index = 0U; index < 4U; ++index)
			{
				value |= static_cast<std::uint32_t>(
					std::to_integer<std::uint8_t>(input[offset + index])) << (index * 8U);
			}
			return value;
		}

		std::uint64_t get_u64(const std::byte *input, std::size_t offset) noexcept
		{
			std::uint64_t value = 0U;
			for (std::size_t index = 0U; index < 8U; ++index)
			{
				value |= static_cast<std::uint64_t>(
					std::to_integer<std::uint8_t>(input[offset + index])) << (index * 8U);
			}
			return value;
		}

		void put_double(std::byte *output, std::size_t offset, double value) noexcept
		{
			std::uint64_t bits = 0U;
			static_assert(sizeof(bits) == sizeof(value), "unexpected double width");
			std::memcpy(&bits, &value, sizeof(bits));
			put_u64(output, offset, bits);
		}

		double get_double(const std::byte *input, std::size_t offset) noexcept
		{
			const auto bits = get_u64(input, offset);
			double value = 0.0;
			static_assert(sizeof(bits) == sizeof(value), "unexpected double width");
			std::memcpy(&value, &bits, sizeof(value));
			return value;
		}

		bool bytes_zero(
			const std::byte *input,
			std::size_t offset,
			std::size_t size) noexcept
		{
			for (std::size_t index = 0U; index < size; ++index)
			{
				if (input[offset + index] != std::byte{0}) return false;
			}
			return true;
		}

		void copy_token(
			std::byte *output,
			std::size_t offset,
			const SessionToken &token) noexcept
		{
			std::memcpy(output + offset, token.bytes.data(), token.bytes.size());
		}

		void read_token(
			const std::byte *input,
			std::size_t offset,
			SessionToken *token) noexcept
		{
			std::memcpy(token->bytes.data(), input + offset, token->bytes.size());
		}

		bool valid_command(const CommandFrame &frame) noexcept
		{
			return !is_zero(frame.session_token) && frame.command_sequence != 0U &&
				frame.source_stamp_ns > 0 &&
				frame.clock_domain == ClockDomain::kSystem &&
				(frame.mode == CommandMode::kNormal || frame.mode == CommandMode::kHold) &&
				std::isfinite(frame.linear_velocity_mps) &&
				std::isfinite(frame.angular_velocity_radps);
		}

		bool valid_feedback(const FeedbackFrame &frame) noexcept
		{
			return !is_zero(frame.session_token) && frame.feedback_sequence != 0U &&
			frame.bridge_publish_boot_ns != 0U &&
			frame.transport_state <= TransportState::kProtocolError &&
			frame.gateway_state <= 5U &&
			frame.validity_flags <= kKnownFeedbackFlags &&
			frame.active_source <= ActiveSource::kNav2 &&
			frame.vehicle_state <= 3U &&
			std::isfinite(frame.vehicle_linear_velocity_mps) &&
			std::isfinite(frame.vehicle_angular_velocity_radps);
		}

		bool check_frame_header(
			const std::byte *input,
			std::size_t input_size,
			std::size_t expected_size,
			std::uint32_t expected_magic,
			DecodeError *error) noexcept
		{
			if (input == nullptr)
			{
				set_error(error, DecodeError::kNullOutput);
				return false;
			}
			if (input_size != expected_size)
			{
				set_error(error, DecodeError::kSize);
				return false;
			}
			if (get_u32(input, 0U) != expected_magic)
			{
				set_error(error, DecodeError::kMagic);
				return false;
			}
			if (get_u16(input, 4U) != kProtocolVersion)
			{
				set_error(error, DecodeError::kVersion);
				return false;
			}
			if (get_u16(input, 6U) != expected_size)
			{
				set_error(error, DecodeError::kLength);
				return false;
			}
			return true;
		}

		bool decode_command_impl(
			const std::byte *input,
			std::size_t input_size,
			CommandFrame *output,
			DecodeError *error,
			bool strict_business_values) noexcept
		{
			set_error(error, DecodeError::kNone);
			if (output == nullptr)
			{
				set_error(error, DecodeError::kNullOutput);
				return false;
			}
			if (!check_frame_header(input, input_size, kCommandFrameSize, kCommandMagic, error))
			{
				return false;
			}
			if (!bytes_zero(input, kCommandReservedOffset, kCommandReservedSize))
			{
				set_error(error, DecodeError::kReserved);
				return false;
			}
			CommandFrame frame;
			read_token(input, 8U, &frame.session_token);
			frame.command_sequence = get_u64(input, 24U);
			frame.source_stamp_ns = static_cast<std::int64_t>(get_u64(input, 32U));
			frame.clock_domain = static_cast<ClockDomain>(
				std::to_integer<std::uint8_t>(input[40U]));
			frame.mode = static_cast<CommandMode>(
				std::to_integer<std::uint8_t>(input[41U]));
			frame.linear_velocity_mps = get_double(input, 48U);
			frame.angular_velocity_radps = get_double(input, 56U);
			if (is_zero(frame.session_token))
			{
				set_error(error, DecodeError::kToken);
				return false;
			}
			if (strict_business_values && frame.command_sequence == 0U)
			{
				set_error(error, DecodeError::kSequence);
				return false;
			}
			if (strict_business_values && frame.source_stamp_ns <= 0)
			{
				set_error(error, DecodeError::kTimestamp);
				return false;
			}
			if (frame.clock_domain != ClockDomain::kSystem)
			{
				set_error(error, DecodeError::kClockDomain);
				return false;
			}
			if (strict_business_values &&
				frame.mode != CommandMode::kNormal && frame.mode != CommandMode::kHold)
			{
				set_error(error, DecodeError::kMode);
				return false;
			}
			if (strict_business_values &&
				(!std::isfinite(frame.linear_velocity_mps) ||
				 !std::isfinite(frame.angular_velocity_radps)))
			{
				set_error(error, DecodeError::kNonFinite);
				return false;
			}
			*output = frame;
			return true;
		}

		bool read_clock(clockid_t clock_id, std::uint64_t *value) noexcept
		{
			if (value == nullptr) return false;
			timespec time{};
			if (::clock_gettime(clock_id, &time) != 0 || time.tv_sec < 0 || time.tv_nsec < 0)
			{
				return false;
			}
			const auto seconds = static_cast<std::uint64_t>(time.tv_sec);
			const auto nanoseconds = static_cast<std::uint64_t>(time.tv_nsec);
			if (seconds > (std::numeric_limits<std::uint64_t>::max() - nanoseconds) / 1'000'000'000U)
			{
				return false;
			}
			*value = seconds * 1'000'000'000U + nanoseconds;
			return true;
		}
	}

	const char *to_string(DecodeError error) noexcept
	{
		switch (error)
		{
		case DecodeError::kNone: return "none";
		case DecodeError::kNullOutput: return "null_output";
		case DecodeError::kSize: return "size";
		case DecodeError::kMagic: return "magic";
		case DecodeError::kVersion: return "version";
		case DecodeError::kLength: return "length";
		case DecodeError::kReserved: return "reserved";
		case DecodeError::kToken: return "token";
		case DecodeError::kSequence: return "sequence";
		case DecodeError::kTimestamp: return "timestamp";
		case DecodeError::kClockDomain: return "clock_domain";
		case DecodeError::kMode: return "mode";
		case DecodeError::kEnum: return "enum";
		case DecodeError::kFlags: return "flags";
		case DecodeError::kNonFinite: return "non_finite";
		}
		return "unknown";
	}

	bool is_zero(const SessionToken &token) noexcept
	{
		for (const auto byte : token.bytes)
		{
			if (byte != std::byte{0}) return false;
		}
		return true;
	}

	bool encode_command_layout(
		const CommandFrame &frame,
		std::byte *output,
		std::size_t output_size) noexcept
	{
		if (output == nullptr || output_size != kCommandFrameSize ||
			is_zero(frame.session_token) || frame.clock_domain != ClockDomain::kSystem)
		{
			return false;
		}
		std::memset(output, 0, output_size);
		put_u32(output, 0U, kCommandMagic);
		put_u16(output, 4U, kProtocolVersion);
		put_u16(output, 6U, static_cast<std::uint16_t>(kCommandFrameSize));
		copy_token(output, 8U, frame.session_token);
		put_u64(output, 24U, frame.command_sequence);
		put_u64(output, 32U, static_cast<std::uint64_t>(frame.source_stamp_ns));
		output[40U] = static_cast<std::byte>(frame.clock_domain);
		output[41U] = static_cast<std::byte>(frame.mode);
		put_double(output, 48U, frame.linear_velocity_mps);
		put_double(output, 56U, frame.angular_velocity_radps);
		return true;
	}

	bool encode_command(
		const CommandFrame &frame,
		std::byte *output,
		std::size_t output_size) noexcept
	{
		if (output == nullptr || output_size != kCommandFrameSize || !valid_command(frame))
		{
			return false;
		}
		return encode_command_layout(frame, output, output_size);
	}

	bool decode_command_layout(
		const std::byte *input,
		std::size_t input_size,
		CommandFrame *output,
		DecodeError *error) noexcept
	{
		return decode_command_impl(input, input_size, output, error, false);
	}

	bool decode_command(
		const std::byte *input,
		std::size_t input_size,
		CommandFrame *output,
		DecodeError *error) noexcept
	{
		return decode_command_impl(input, input_size, output, error, true);
	}

	bool encode_feedback(
		const FeedbackFrame &frame,
		std::byte *output,
		std::size_t output_size) noexcept
	{
		if (output == nullptr || output_size != kFeedbackFrameSize || !valid_feedback(frame))
		{
			return false;
		}
		std::memset(output, 0, output_size);
		put_u32(output, 0U, kFeedbackMagic);
		put_u16(output, 4U, kProtocolVersion);
		put_u16(output, 6U, static_cast<std::uint16_t>(kFeedbackFrameSize));
		copy_token(output, 8U, frame.session_token);
		put_u64(output, 24U, frame.feedback_sequence);
		put_u64(output, 32U, frame.bridge_publish_boot_ns);
		output[40U] = static_cast<std::byte>(frame.transport_state);
		output[41U] = static_cast<std::byte>(frame.gateway_state);
		put_u16(output, 42U, frame.gateway_reason_code);
		put_u32(output, 44U, frame.validity_flags);
		output[48U] = static_cast<std::byte>(frame.active_source);
		output[49U] = static_cast<std::byte>(frame.vehicle_state);
		put_u16(output, 52U, frame.vehicle_fault_code);
		put_u16(output, 54U, frame.planning_reject_reason);
		put_u64(output, 56U, frame.active_source_sequence);
		put_u64(output, 64U, frame.planning_accepted_count);
		put_u64(output, 72U, frame.planning_rejected_count);
		put_u64(output, 80U, frame.gateway_observed_ros_ns);
		put_u64(output, 88U, frame.gateway_receive_boot_ns);
		put_u64(output, 96U, frame.planning_observed_ros_ns);
		put_u64(output, 104U, frame.planning_receive_boot_ns);
		put_u64(output, 112U, frame.vehicle_observed_ros_ns);
		put_u64(output, 120U, frame.vehicle_receive_boot_ns);
		put_double(output, 128U, frame.vehicle_linear_velocity_mps);
		put_double(output, 136U, frame.vehicle_angular_velocity_radps);
		put_u64(output, 144U, frame.transport_generation);
		std::memcpy(output + 152U, frame.transport_instance_nonce.data(),
			frame.transport_instance_nonce.size());
		put_u64(output, 168U, frame.command_edge_sequence);
		return true;
	}

	bool decode_feedback(
		const std::byte *input,
		std::size_t input_size,
		FeedbackFrame *output,
		DecodeError *error) noexcept
	{
		set_error(error, DecodeError::kNone);
		if (output == nullptr)
		{
			set_error(error, DecodeError::kNullOutput);
			return false;
		}
		if (!check_frame_header(input, input_size, kFeedbackFrameSize, kFeedbackMagic, error))
		{
			return false;
		}
		if (!bytes_zero(input, kFeedbackReservedOffset, kFeedbackReservedSize))
		{
			set_error(error, DecodeError::kReserved);
			return false;
		}
		FeedbackFrame frame;
		read_token(input, 8U, &frame.session_token);
		frame.feedback_sequence = get_u64(input, 24U);
		frame.bridge_publish_boot_ns = get_u64(input, 32U);
		frame.transport_state = static_cast<TransportState>(
			std::to_integer<std::uint8_t>(input[40U]));
		frame.gateway_state = std::to_integer<std::uint8_t>(input[41U]);
		frame.gateway_reason_code = get_u16(input, 42U);
		frame.validity_flags = get_u32(input, 44U);
		frame.active_source = static_cast<ActiveSource>(
			std::to_integer<std::uint8_t>(input[48U]));
		frame.vehicle_state = std::to_integer<std::uint8_t>(input[49U]);
		frame.vehicle_fault_code = get_u16(input, 52U);
		frame.planning_reject_reason = get_u16(input, 54U);
		frame.active_source_sequence = get_u64(input, 56U);
		frame.planning_accepted_count = get_u64(input, 64U);
		frame.planning_rejected_count = get_u64(input, 72U);
		frame.gateway_observed_ros_ns = get_u64(input, 80U);
		frame.gateway_receive_boot_ns = get_u64(input, 88U);
		frame.planning_observed_ros_ns = get_u64(input, 96U);
		frame.planning_receive_boot_ns = get_u64(input, 104U);
		frame.vehicle_observed_ros_ns = get_u64(input, 112U);
		frame.vehicle_receive_boot_ns = get_u64(input, 120U);
		frame.vehicle_linear_velocity_mps = get_double(input, 128U);
		frame.vehicle_angular_velocity_radps = get_double(input, 136U);
		frame.transport_generation = get_u64(input, 144U);
		std::memcpy(frame.transport_instance_nonce.data(), input + 152U,
			frame.transport_instance_nonce.size());
		frame.command_edge_sequence = get_u64(input, 168U);
		if (is_zero(frame.session_token))
		{
			set_error(error, DecodeError::kToken);
			return false;
		}
		if (frame.feedback_sequence == 0U || frame.bridge_publish_boot_ns == 0U)
		{
			set_error(error, DecodeError::kSequence);
			return false;
		}
		if (frame.transport_state > TransportState::kProtocolError ||
			frame.active_source > ActiveSource::kNav2 || frame.gateway_state > 5U ||
			frame.vehicle_state > 3U)
		{
			set_error(error, DecodeError::kEnum);
			return false;
		}
		if (frame.validity_flags > kKnownFeedbackFlags)
		{
			set_error(error, DecodeError::kFlags);
			return false;
		}
		if (!std::isfinite(frame.vehicle_linear_velocity_mps) ||
			!std::isfinite(frame.vehicle_angular_velocity_radps))
		{
			set_error(error, DecodeError::kNonFinite);
			return false;
		}
		*output = frame;
		return true;
	}

	bool generate_session_token(SessionToken *token) noexcept
	{
		if (token == nullptr) return false;
		std::size_t offset = 0U;
		while (offset < token->bytes.size())
		{
			const auto result = ::getrandom(
				token->bytes.data() + offset,
				token->bytes.size() - offset,
				0U);
			if (result < 0)
			{
				if (errno == EINTR) continue;
				return false;
			}
			if (result == 0) return false;
			offset += static_cast<std::size_t>(result);
		}
		return !is_zero(*token);
	}

	std::uint64_t boottime_now_ns() noexcept
	{
		std::uint64_t value = 0U;
		return read_clock(CLOCK_BOOTTIME, &value) ? value : 0U;
	}

	std::int64_t system_time_now_ns() noexcept
	{
		std::uint64_t value = 0U;
		if (!read_clock(CLOCK_REALTIME, &value) ||
			value > static_cast<std::uint64_t>(std::numeric_limits<std::int64_t>::max()))
		{
			return 0;
		}
		return static_cast<std::int64_t>(value);
	}
} // namespace control_link_edge_protocol
