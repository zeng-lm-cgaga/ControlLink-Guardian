#pragma once

#include <array>
#include <cstddef>
#include <cstdint>

#include "control_link_edge_protocol/protocol.hpp"
#include "edge_runtime/common/schema.hpp"

namespace edge_runtime
{
	// EdgeRuntime 只承载这些固定字节 payload，不引入 ROS 消息、指针或隐式 padding。
	template <>
	struct PayloadCodec<control_link_edge_protocol::CommandFrame>
	{
		static constexpr bool kDefined = true;
		static constexpr std::uint32_t kEncodedSize =
			static_cast<std::uint32_t>(control_link_edge_protocol::kCommandFrameSize);
		using EncodedBuffer = std::array<std::byte, kEncodedSize>;

		static bool encode(
			const control_link_edge_protocol::CommandFrame &value,
			std::byte *output,
			std::size_t output_size) noexcept
		{
			return control_link_edge_protocol::encode_command(
				value, output, output_size);
		}

		static bool decode(
			const std::byte *input,
			std::size_t input_size,
			control_link_edge_protocol::CommandFrame *output) noexcept
		{
			return control_link_edge_protocol::decode_command(
				input, input_size, output);
		}
	};

	template <>
	struct PayloadCodec<control_link_edge_protocol::FeedbackFrame>
	{
		static constexpr bool kDefined = true;
		static constexpr std::uint32_t kEncodedSize =
			static_cast<std::uint32_t>(control_link_edge_protocol::kFeedbackFrameSize);
		using EncodedBuffer = std::array<std::byte, kEncodedSize>;

		static bool encode(
			const control_link_edge_protocol::FeedbackFrame &value,
			std::byte *output,
			std::size_t output_size) noexcept
		{
			return control_link_edge_protocol::encode_feedback(
				value, output, output_size);
		}

		static bool decode(
			const std::byte *input,
			std::size_t input_size,
			control_link_edge_protocol::FeedbackFrame *output) noexcept
		{
			return control_link_edge_protocol::decode_feedback(
				input, input_size, output);
		}
	};

	inline SchemaDescriptor command_schema() noexcept
	{
		SchemaDescriptor schema;
		const std::array<std::uint8_t, 32U> fingerprint{
			0x43U, 0x4CU, 0x2DU, 0x45U, 0x44U, 0x47U, 0x45U, 0x2DU,
			0x43U, 0x4DU, 0x44U, 0x2DU, 0x56U, 0x31U, 0x00U, 0x01U,
			0x8BU, 0x9AU, 0x17U, 0xE2U, 0x61U, 0x40U, 0xCCU, 0x52U,
			0x30U, 0xA1U, 0x75U, 0x7EU, 0xE8U, 0x11U, 0x04U, 0xD9U};
		for (std::size_t index = 0U; index < fingerprint.size(); ++index)
		{
			schema.fingerprint[index] = static_cast<std::byte>(fingerprint[index]);
		}
		schema.version = control_link_edge_protocol::kProtocolVersion;
		schema.debug_name = "control_link.edge.command.v1";
		return schema;
	}

	inline SchemaDescriptor feedback_schema() noexcept
	{
		SchemaDescriptor schema;
		const std::array<std::uint8_t, 32U> fingerprint{
			0x43U, 0x4CU, 0x2DU, 0x45U, 0x44U, 0x47U, 0x45U, 0x2DU,
			0x46U, 0x42U, 0x4BU, 0x2DU, 0x56U, 0x31U, 0x00U, 0x01U,
			0x4AU, 0x2FU, 0x91U, 0xD8U, 0x6BU, 0x10U, 0x55U, 0xA3U,
			0x72U, 0x19U, 0xC4U, 0x0EU, 0xB6U, 0x3DU, 0x88U, 0xF0U};
		for (std::size_t index = 0U; index < fingerprint.size(); ++index)
		{
			schema.fingerprint[index] = static_cast<std::byte>(fingerprint[index]);
		}
		schema.version = control_link_edge_protocol::kProtocolVersion;
		schema.debug_name = "control_link.edge.feedback.v1";
		return schema;
	}
} // namespace edge_runtime
