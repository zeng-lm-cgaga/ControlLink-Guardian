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
	command_rejects_replay_and_reserved_bytes();
	feedback_round_trip();
	invalid_float_is_rejected();
	std::cout << "edge protocol tests passed\n";
	return 0;
}
