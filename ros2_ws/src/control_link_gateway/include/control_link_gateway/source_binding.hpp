#pragma once

#include <cstdint>

#include "control_link_gateway/model.hpp"
#include "rclcpp/message_info.hpp"

namespace control_link_gateway
{
	// 将 callback 的非 owning RMW 元数据复制为可跨 callback 保存的 generation identity
	[[nodiscard]] PublisherGenerationKey
	publisher_generation_from_message_info(
		const rclcpp::MessageInfo &message_info);

	// 描述 Graph 稳定确认相对于 Slot 当前 generation 的身份变化
	enum class PublisherGenerationUpdate : std::uint8_t
	{
		kFirstConfirmation,
		kUnchanged,
		kChanged,
	};

	// 只允许 Graph stable window 的确认路径调用，确认新 generation 时使旧命令与 sequence 基线失效
	[[nodiscard]] PublisherGenerationUpdate confirm_publisher_generation(
		SourceRuntimeSlot &slot,
		PublisherGenerationKey generation);

	// Graph 稳定确认来源不可用时只撤销可仲裁快照，同 GID 的 sequence 防回放基线必须保留
	void invalidate_source_endpoint_snapshot(SourceRuntimeSlot &slot) noexcept;

	// Message callback 只读核对实际发布者，不得借此自行确认陌生 GID
	[[nodiscard]] bool message_matches_confirmed_generation(
		const SourceRuntimeSlot &slot,
		const PublisherGenerationKey &actual_generation) noexcept;
} // namespace control_link_gateway
