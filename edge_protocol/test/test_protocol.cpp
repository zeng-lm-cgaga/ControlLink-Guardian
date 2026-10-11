#include "control_link_edge_protocol/protocol.hpp"

#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <cstdlib>
#include <iostream>
#include <limits>

namespace
{
	using namespace control_link_edge_protocol;

	void check(bool condition, const char *expression, int line)
	{
		if (!condition)
		{
			std::cerr << "check failed at line " << line << ": " << expression << '\n';
			std::exit(1);
		}
	}

#define CHECK(expression) check((expression), #expression, __LINE__)

	SessionToken token()
	{
		SessionToken result;
		result.bytes[0] = std::byte{0x01};
		result.bytes[15] = std::byte{0xFE};
		return result;
	}

	CommandFrame command()
	{
		CommandFrame result;
		result.session_token = token();
		result.command_sequence = 42U;
		result.source_stamp_ns = 1'723'456'789'123'456'789LL;
		result.linear_velocity_mps = -1.25;
		result.angular_velocity_radps = 0.75;
		return result;
	}

	std::uint64_t double_bits(double value)
	{
		std::uint64_t bits = 0U;
		static_assert(sizeof(bits) == sizeof(value), "unexpected double width");
		std::memcpy(&bits, &value, sizeof(bits));
		return bits;
	}

	bool same_command(const CommandFrame &left, const CommandFrame &right)
	{
		return left.session_token == right.session_token &&
			left.command_sequence == right.command_sequence &&
			left.source_stamp_ns == right.source_stamp_ns &&
			left.clock_domain == right.clock_domain && left.mode == right.mode &&
			double_bits(left.linear_velocity_mps) == double_bits(right.linear_velocity_mps) &&
			double_bits(left.angular_velocity_radps) == double_bits(right.angular_velocity_radps);
	}

	void check_command_layout_round_trip(const CommandFrame &input, DecodeError strict_error)
	{
		std::array<std::byte, kCommandFrameSize> bytes{};
		CHECK(encode_command_layout(input, bytes.data(), bytes.size()));
		CommandFrame output;
		DecodeError error = DecodeError::kEnum;
		CHECK(decode_command_layout(bytes.data(), bytes.size(), &output, &error));
		CHECK(error == DecodeError::kNone);
		CHECK(same_command(output, input));
		for (std::size_t index = 0U; index < 8U; ++index)
		{
			CHECK(std::to_integer<std::uint8_t>(bytes[24U + index]) ==
				((input.command_sequence >> (index * 8U)) & 0xFFU));
			CHECK(std::to_integer<std::uint8_t>(bytes[32U + index]) ==
				((static_cast<std::uint64_t>(input.source_stamp_ns) >> (index * 8U)) & 0xFFU));
			CHECK(std::to_integer<std::uint8_t>(bytes[48U + index]) ==
				((double_bits(input.linear_velocity_mps) >> (index * 8U)) & 0xFFU));
			CHECK(std::to_integer<std::uint8_t>(bytes[56U + index]) ==
				((double_bits(input.angular_velocity_radps) >> (index * 8U)) & 0xFFU));
		}
		std::array<std::byte, kCommandFrameSize> reencoded{};
		CHECK(encode_command_layout(output, reencoded.data(), reencoded.size()));
		CHECK(reencoded == bytes);
		if (strict_error == DecodeError::kNone)
		{
			CHECK(encode_command(input, reencoded.data(), reencoded.size()));
			CHECK(reencoded == bytes);
			CHECK(decode_command(bytes.data(), bytes.size(), &output, &error));
			CHECK(error == DecodeError::kNone);
			CHECK(same_command(output, input));
		}
		else
		{
			reencoded.fill(std::byte{0xA5});
			const auto original_bytes = reencoded;
			CHECK(!encode_command(input, reencoded.data(), reencoded.size()));
			CHECK(reencoded == original_bytes);
			output = command();
			const auto original_output = output;
			CHECK(!decode_command(bytes.data(), bytes.size(), &output, &error));
			CHECK(error == strict_error);
			CHECK(same_command(output, original_output));
		}
	}

	void check_command_structure_rejection(
		const std::byte *input,
		std::size_t size,
		DecodeError expected_error)
	{
		const auto original_output = command();
		CommandFrame output = original_output;
		DecodeError error = DecodeError::kNone;
		CHECK(!decode_command_layout(input, size, &output, &error));
		CHECK(error == expected_error);
		CHECK(same_command(output, original_output));
		CHECK(!decode_command(input, size, &output, &error));
		CHECK(error == expected_error);
		CHECK(same_command(output, original_output));
		CHECK(!decode_command_layout(input, size, &output));
		CHECK(same_command(output, original_output));
	}

	void command_round_trip()
	{
		CommandFrame input;
		input.session_token = token();
		input.command_sequence = 42U;
		input.source_stamp_ns = 1'723'456'789'123'456'789LL;
		input.mode = CommandMode::kNormal;
		input.linear_velocity_mps = -1.25;
		input.angular_velocity_radps = 0.75;
		std::array<std::byte, kCommandFrameSize> bytes{};
		CHECK(encode_command(input, bytes.data(), bytes.size()));
		CHECK(std::to_integer<std::uint8_t>(bytes[0]) == 0x43U);
		CHECK(std::to_integer<std::uint8_t>(bytes[1]) == 0x4DU);
		CHECK(std::to_integer<std::uint8_t>(bytes[2]) == 0x44U);
		CHECK(std::to_integer<std::uint8_t>(bytes[3]) == 0x31U);
		CHECK(std::to_integer<std::uint8_t>(bytes[24]) == 42U);
		CommandFrame output;
		DecodeError error = DecodeError::kNone;
		CHECK(decode_command(bytes.data(), bytes.size(), &output, &error));
		CHECK(error == DecodeError::kNone);
		CHECK(output.session_token == input.session_token);
		CHECK(output.command_sequence == input.command_sequence);
		CHECK(output.source_stamp_ns == input.source_stamp_ns);
		CHECK(output.linear_velocity_mps == input.linear_velocity_mps);
		CHECK(output.angular_velocity_radps == input.angular_velocity_radps);
		check_command_layout_round_trip(input, DecodeError::kNone);
	}

	void command_layout_preserves_business_values()
	{
		CommandFrame input = command();
		input.command_sequence = 0U;
		check_command_layout_round_trip(input, DecodeError::kSequence);
		input = command();
		for (const auto stamp : std::array<std::int64_t, 3U>{
			0, -1, std::numeric_limits<std::int64_t>::min()})
		{
			input.source_stamp_ns = stamp;
			check_command_layout_round_trip(input, DecodeError::kTimestamp);
		}
		input = command();
		for (const auto mode : std::array<std::uint8_t, 2U>{2U, 255U})
		{
			input.mode = static_cast<CommandMode>(mode);
			check_command_layout_round_trip(input, DecodeError::kMode);
		}
		input = command();
		input.command_sequence = std::numeric_limits<std::uint64_t>::max();
		input.source_stamp_ns = std::numeric_limits<std::int64_t>::max();
		input.mode = CommandMode::kHold;
		input.linear_velocity_mps = -0.0;
		input.angular_velocity_radps = std::numeric_limits<double>::denorm_min();
		check_command_layout_round_trip(input, DecodeError::kNone);
		input.command_sequence = 0U;
		input.source_stamp_ns = -1;
		input.mode = static_cast<CommandMode>(255U);
		input.linear_velocity_mps = std::numeric_limits<double>::quiet_NaN();
		input.angular_velocity_radps = std::numeric_limits<double>::infinity();
		check_command_layout_round_trip(input, DecodeError::kSequence);
	}

	void command_layout_preserves_nonfinite_bits()
	{
		for (const auto bits : std::array<std::uint64_t, 6U>{
			0x7FF8000000000042ULL, 0xFFF8000000001234ULL,
			0x7FF0000000000001ULL, 0xFFF0000000000042ULL,
			0x7FF0000000000000ULL, 0xFFF0000000000000ULL})
		{
			CommandFrame input = command();
			std::memcpy(&input.linear_velocity_mps, &bits, sizeof(bits));
			check_command_layout_round_trip(input, DecodeError::kNonFinite);
			input = command();
			std::memcpy(&input.angular_velocity_radps, &bits, sizeof(bits));
			check_command_layout_round_trip(input, DecodeError::kNonFinite);
		}
	}

	void command_layout_rejects_structure()
	{
		std::array<std::byte, kCommandFrameSize> bytes{};
		CHECK(encode_command(command(), bytes.data(), bytes.size()));
		const auto original_bytes = bytes;
		check_command_structure_rejection(nullptr, bytes.size(), DecodeError::kNullOutput);
		for (const auto size : std::array<std::size_t, 3U>{0U, 63U, 65U})
		{
			check_command_structure_rejection(bytes.data(), size, DecodeError::kSize);
		}
		DecodeError error = DecodeError::kNone;
		CHECK(!decode_command_layout(bytes.data(), bytes.size(), nullptr, &error));
		CHECK(error == DecodeError::kNullOutput);
		CHECK(!decode_command(bytes.data(), bytes.size(), nullptr, &error));
		CHECK(error == DecodeError::kNullOutput);
		for (std::size_t offset = 0U; offset < 8U; ++offset)
		{
			bytes = original_bytes;
			bytes[offset] ^= std::byte{0x01};
			const auto expected_error = offset < 4U ? DecodeError::kMagic :
				(offset < 6U ? DecodeError::kVersion : DecodeError::kLength);
			check_command_structure_rejection(bytes.data(), bytes.size(), expected_error);
		}
		for (std::size_t offset = 42U; offset < 48U; ++offset)
		{
			bytes = original_bytes;
			bytes[offset] = std::byte{0x01};
			check_command_structure_rejection(bytes.data(), bytes.size(), DecodeError::kReserved);
		}
		bytes = original_bytes;
		for (std::size_t offset = 8U; offset < 24U; ++offset)
		{
			bytes[offset] = std::byte{0};
		}
		check_command_structure_rejection(bytes.data(), bytes.size(), DecodeError::kToken);
		for (const auto domain : std::array<std::byte, 2U>{std::byte{1}, std::byte{255}})
		{
			bytes = original_bytes;
			bytes[40U] = domain;
			check_command_structure_rejection(bytes.data(), bytes.size(), DecodeError::kClockDomain);
		}
	}

	void command_layout_encode_rejects_structure()
	{
		std::array<std::byte, kCommandFrameSize> bytes{};
		bytes.fill(std::byte{0xA5});
		const auto original_bytes = bytes;
		CHECK(!encode_command_layout(command(), nullptr, bytes.size()));
		for (const auto size : std::array<std::size_t, 3U>{0U, 63U, 65U})
		{
			CHECK(!encode_command_layout(command(), bytes.data(), size));
			CHECK(bytes == original_bytes);
		}
		CommandFrame input = command();
		input.session_token = SessionToken{};
		CHECK(!encode_command_layout(input, bytes.data(), bytes.size()));
		CHECK(bytes == original_bytes);
		input = command();
		input.clock_domain = static_cast<ClockDomain>(1U);
		CHECK(!encode_command_layout(input, bytes.data(), bytes.size()));
		CHECK(bytes == original_bytes);
	}

	void strict_decode_error_priority_is_unchanged()
	{
		CommandFrame input = command();
		input.command_sequence = 0U;
		std::array<std::byte, kCommandFrameSize> bytes{};
		CHECK(encode_command_layout(input, bytes.data(), bytes.size()));
		bytes[40U] = std::byte{1};
		CommandFrame output = command();
		const auto original_output = output;
		DecodeError error = DecodeError::kNone;
		CHECK(!decode_command(bytes.data(), bytes.size(), &output, &error));
		CHECK(error == DecodeError::kSequence);
		CHECK(!decode_command_layout(bytes.data(), bytes.size(), &output, &error));
		CHECK(error == DecodeError::kClockDomain);
		CHECK(same_command(output, original_output));
		input = command();
		input.source_stamp_ns = 0;
		CHECK(encode_command_layout(input, bytes.data(), bytes.size()));
		bytes[40U] = std::byte{1};
		CHECK(!decode_command(bytes.data(), bytes.size(), &output, &error));
		CHECK(error == DecodeError::kTimestamp);
		CHECK(!decode_command_layout(bytes.data(), bytes.size(), &output, &error));
		CHECK(error == DecodeError::kClockDomain);
		CHECK(same_command(output, original_output));
		bytes[8U] = std::byte{0};
		bytes[23U] = std::byte{0};
		check_command_structure_rejection(bytes.data(), bytes.size(), DecodeError::kToken);
		input = command();
		input.mode = static_cast<CommandMode>(2U);
		input.linear_velocity_mps = std::numeric_limits<double>::quiet_NaN();
		check_command_layout_round_trip(input, DecodeError::kMode);
	}

	void command_rejects_replay_and_reserved_bytes()
	{
		CommandFrame input;
		input.session_token = token();
		input.command_sequence = 1U;
		input.source_stamp_ns = 1U;
		std::array<std::byte, kCommandFrameSize> bytes{};
		CHECK(encode_command(input, bytes.data(), bytes.size()));
		bytes[42] = std::byte{0x01};
		DecodeError error = DecodeError::kNone;
		CommandFrame output;
		CHECK(!decode_command(bytes.data(), bytes.size(), &output, &error));
		CHECK(error == DecodeError::kReserved);
		bytes[42] = std::byte{0};
		bytes[40] = std::byte{0x01};
		CHECK(!decode_command(bytes.data(), bytes.size(), &output, &error));
		CHECK(error == DecodeError::kClockDomain);
	}

	void feedback_round_trip()
	{
		FeedbackFrame input;
		input.session_token = token();
		input.feedback_sequence = 8U;
		input.bridge_publish_boot_ns = 99U;
		input.transport_state = TransportState::kForwarding;
		input.gateway_state = 1U;
		input.gateway_reason_code = 13U;
		input.validity_flags = kGatewayStateValid | kPlanningStatusValid |
			kVehicleStateValid | kSessionReady;
		input.active_source = ActiveSource::kPlanning;
		input.vehicle_state = 1U;
		input.vehicle_fault_code = 0U;
		input.planning_reject_reason = 10U;
		input.active_source_sequence = 11U;
		input.planning_accepted_count = 12U;
		input.planning_rejected_count = 13U;
		input.gateway_observed_ros_ns = 14U;
		input.gateway_receive_boot_ns = 15U;
		input.planning_observed_ros_ns = 16U;
		input.planning_receive_boot_ns = 17U;
		input.vehicle_observed_ros_ns = 18U;
		input.vehicle_receive_boot_ns = 19U;
		input.vehicle_linear_velocity_mps = 0.125;
		input.vehicle_angular_velocity_radps = -0.25;
		input.transport_generation = 20U;
		input.transport_instance_nonce[3] = std::byte{0xA5};
		input.command_edge_sequence = 21U;
		std::array<std::byte, kFeedbackFrameSize> bytes{};
		CHECK(encode_feedback(input, bytes.data(), bytes.size()));
		FeedbackFrame output;
		DecodeError error = DecodeError::kNone;
		CHECK(decode_feedback(bytes.data(), bytes.size(), &output, &error));
		CHECK(error == DecodeError::kNone);
		CHECK(output.session_token == input.session_token);
		CHECK(output.feedback_sequence == input.feedback_sequence);
		CHECK(output.validity_flags == input.validity_flags);
		CHECK(output.planning_observed_ros_ns == input.planning_observed_ros_ns);
		CHECK(output.planning_receive_boot_ns == input.planning_receive_boot_ns);
		CHECK(output.vehicle_linear_velocity_mps == input.vehicle_linear_velocity_mps);
		CHECK(output.vehicle_angular_velocity_radps == input.vehicle_angular_velocity_radps);
		CHECK(output.transport_instance_nonce == input.transport_instance_nonce);
	}

	void invalid_float_is_rejected()
	{
		CommandFrame input;
		input.session_token = token();
		input.command_sequence = 1U;
		input.source_stamp_ns = 1U;
		input.linear_velocity_mps = std::numeric_limits<double>::quiet_NaN();
		std::array<std::byte, kCommandFrameSize> bytes{};
		CHECK(!encode_command(input, bytes.data(), bytes.size()));
	}
}

int main()
{
	command_round_trip();
	command_layout_preserves_business_values();
	command_layout_preserves_nonfinite_bits();
	command_layout_rejects_structure();
	command_layout_encode_rejects_structure();
	strict_decode_error_priority_is_unchanged();
	command_rejects_replay_and_reserved_bytes();
	feedback_round_trip();
	invalid_float_is_rejected();
	std::cout << "edge protocol tests passed\n";
	return 0;
}
