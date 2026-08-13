#pragma once

#include <array>
#include <chrono>
#include <cstdint>
#include <optional>
#include <string>

#include <rmw/types.h>

#include "control_link_interfaces/msg/control_command.hpp"
#include "control_link_interfaces/msg/source_status.hpp"

namespace control_link_gateway
{
	// 内部枚举直接绑定公共消息常量，避免运行逻辑与 rosidl 接口形成两套数值
	enum class CommandMode : std::uint8_t
	{
		kNormal =
			control_link_interfaces::msg::ControlCommand::MODE_NORMAL,
		kHold =
			control_link_interfaces::msg::ControlCommand::MODE_HOLD,
	};

	enum class RejectReason : std::uint16_t
	{
		kNone = control_link_interfaces::msg::SourceStatus::REJECT_NONE,
		kSourceIdMismatch =
			control_link_interfaces::msg::SourceStatus::REJECT_SOURCE_ID_MISMATCH,
		kSourceEndpointAmbiguous =
			control_link_interfaces::msg::SourceStatus::REJECT_SOURCE_ENDPOINT_AMBIGUOUS,
		kPublisherGenerationUnstable =
			control_link_interfaces::msg::SourceStatus::REJECT_PUBLISHER_GENERATION_UNSTABLE,
		kUnknownMode = control_link_interfaces::msg::SourceStatus::REJECT_UNKNOWN_MODE,
		kNonFinite = control_link_interfaces::msg::SourceStatus::REJECT_NON_FINITE,
		kOutOfRange = control_link_interfaces::msg::SourceStatus::REJECT_OUT_OF_RANGE,
		kZeroStamp = control_link_interfaces::msg::SourceStatus::REJECT_ZERO_STAMP,
		kFutureStamp = control_link_interfaces::msg::SourceStatus::REJECT_FUTURE_STAMP,
		kStale = control_link_interfaces::msg::SourceStatus::REJECT_STALE,
		kSequenceNotIncreasing =
			control_link_interfaces::msg::SourceStatus::REJECT_SEQUENCE_NOT_INCREASING,
		kClockInvalid = control_link_interfaces::msg::SourceStatus::REJECT_CLOCK_INVALID,
		kHoldNonzero = control_link_interfaces::msg::SourceStatus::REJECT_HOLD_NONZERO,
		kInvalidStamp = control_link_interfaces::msg::SourceStatus::REJECT_INVALID_STAMP,
	};

	// GID 只能在同一个 RMW implementation 语义下比较，二者共同标识 publisher generation
	struct PublisherGenerationKey
	{
		std::string rmw_implementation;
		std::array<std::uint8_t, RMW_GID_STORAGE_SIZE> publisher_gid;

		[[nodiscard]] bool operator==(
			const PublisherGenerationKey &other) const noexcept
		{
			return this->publisher_gid == other.publisher_gid && this->rmw_implementation == other.rmw_implementation;
		}
	};

	// 仅在 CommandValidator 完整通过后创建，每个 source 在 Gateway 中最多保存一个 latest-valid 快照
	// source_stamp_ns 属于 ROS clock，received_at 属于本地 steady clock，禁止混算两个时间域
	struct SourceSnapshot
	{
		std::string source_id;
		std::uint8_t priority;
		PublisherGenerationKey publisher_generation;
		std::uint64_t sequence;
		std::int64_t source_stamp_ns;
		CommandMode mode;
		double linear_velocity_mps;
		double angular_velocity_radps;
		std::chrono::steady_clock::time_point received_at;
	};

	// 每个 enabled source 只有一个运行时槽位，非法样本不得覆盖 latest-valid snapshot
	// sequence 基线与 snapshot 分开保存，使 snapshot 失效时仍可保留同 generation 防回放状态
	struct SourceRuntimeSlot
	{
		std::optional<PublisherGenerationKey> confirmed_publisher_generation;
		std::optional<std::uint64_t> last_accepted_sequence;
		std::optional<SourceSnapshot> latest_valid_snapshot;
		RejectReason last_reject_reason{RejectReason::kNone};
		std::uint64_t accepted_count{0};
		std::uint64_t rejected_count{0};
	};
} // namespace control_link_gateway
