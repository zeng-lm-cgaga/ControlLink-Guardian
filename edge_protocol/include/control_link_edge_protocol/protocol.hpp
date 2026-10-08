#pragma once

#include <array>
#include <cstddef>
#include <cstdint>

namespace control_link_edge_protocol
{
	// 协议只描述固定字节布局；session token 用于接入会话隔离，不承担认证。
	inline constexpr std::uint32_t kCommandMagic = 0x31444D43U;
	inline constexpr std::uint32_t kFeedbackMagic = 0x31424446U;
	inline constexpr std::uint16_t kProtocolVersion = 1U;
	inline constexpr std::size_t kSessionTokenSize = 16U;
	inline constexpr std::size_t kCommandFrameSize = 64U;
	inline constexpr std::size_t kFeedbackFrameSize = 176U;

	struct SessionToken
	{
		std::array<std::byte, kSessionTokenSize> bytes{};

		bool operator==(const SessionToken &other) const noexcept
		{
			return bytes == other.bytes;
		}
		bool operator!=(const SessionToken &other) const noexcept
		{
			return !(*this == other);
		}
	};

	enum class ClockDomain : std::uint8_t
	{
		kSystem = 0U,
	};

	enum class CommandMode : std::uint8_t
	{
		kNormal = 0U,
		kHold = 1U,
	};

	enum class TransportState : std::uint8_t
	{
		kStopped = 0U,
		kWaitingForCommand = 1U,
		kForwarding = 2U,
		kProtocolError = 3U,
	};

	enum class ActiveSource : std::uint8_t
	{
		kNone = 0U,
		kPlanning = 1U,
		kTeleop = 2U,
		kNav2 = 3U,
	};

	inline constexpr std::uint32_t kGatewayStateValid = 1U << 0U;
	inline constexpr std::uint32_t kPlanningStatusValid = 1U << 1U;
	inline constexpr std::uint32_t kVehicleStateValid = 1U << 2U;
	inline constexpr std::uint32_t kSessionReady = 1U << 3U;

	struct CommandFrame
	{
		// source_stamp_ns 保留客户端的 SYSTEM 时间，bridge 不以接收时间重盖章。
		SessionToken session_token{};
		std::uint64_t command_sequence{0U};
		std::int64_t source_stamp_ns{0};
		ClockDomain clock_domain{ClockDomain::kSystem};
		CommandMode mode{CommandMode::kNormal};
		double linear_velocity_mps{0.0};
		double angular_velocity_radps{0.0};
	};

	struct FeedbackFrame
	{
		// validity_flags 逐组描述观测有效性，反馈不是同一时刻的原子状态或任务 ACK。
		SessionToken session_token{};
		std::uint64_t feedback_sequence{0U};
		std::uint64_t bridge_publish_boot_ns{0U};
		TransportState transport_state{TransportState::kStopped};
		std::uint8_t gateway_state{0U};
		std::uint16_t gateway_reason_code{0U};
		std::uint32_t validity_flags{0U};
		ActiveSource active_source{ActiveSource::kNone};
		std::uint8_t vehicle_state{0U};
		std::uint16_t vehicle_fault_code{0U};
		std::uint16_t planning_reject_reason{0U};
		std::uint64_t active_source_sequence{0U};
		std::uint64_t planning_accepted_count{0U};
		std::uint64_t planning_rejected_count{0U};
		std::uint64_t gateway_observed_ros_ns{0U};
		std::uint64_t gateway_receive_boot_ns{0U};
		std::uint64_t planning_observed_ros_ns{0U};
		std::uint64_t planning_receive_boot_ns{0U};
		std::uint64_t vehicle_observed_ros_ns{0U};
		std::uint64_t vehicle_receive_boot_ns{0U};
		double vehicle_linear_velocity_mps{0.0};
		double vehicle_angular_velocity_radps{0.0};
		std::uint64_t transport_generation{0U};
		std::array<std::byte, kSessionTokenSize> transport_instance_nonce{};
		std::uint64_t command_edge_sequence{0U};
	};

	enum class DecodeError : std::uint8_t
	{
		kNone = 0U,
		kNullOutput,
		kSize,
		kMagic,
		kVersion,
		kLength,
		kReserved,
		kToken,
		kSequence,
		kTimestamp,
		kClockDomain,
		kMode,
		kEnum,
		kFlags,
		kNonFinite,
	};

	const char *to_string(DecodeError error) noexcept;

	// 编解码拒绝非法 token、sequence、时间域、保留位和非 finite 数值。
	bool encode_command(
		const CommandFrame &frame,
		std::byte *output,
		std::size_t output_size) noexcept;
	bool decode_command(
		const std::byte *input,
		std::size_t input_size,
		CommandFrame *output,
		DecodeError *error = nullptr) noexcept;

	bool encode_feedback(
		const FeedbackFrame &frame,
		std::byte *output,
		std::size_t output_size) noexcept;
	bool decode_feedback(
		const std::byte *input,
		std::size_t input_size,
		FeedbackFrame *output,
		DecodeError *error = nullptr) noexcept;

	// 只能从操作系统随机源生成新会话；随机源失败时调用方必须 fail-closed。
	bool generate_session_token(SessionToken *token) noexcept;
	bool is_zero(const SessionToken &token) noexcept;
	std::uint64_t boottime_now_ns() noexcept;
	std::int64_t system_time_now_ns() noexcept;
} // namespace control_link_edge_protocol
