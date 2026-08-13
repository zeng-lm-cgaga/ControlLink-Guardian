#pragma once

#include <array>
#include <cstdint>
#include <optional>
#include <rmw/types.h>
#include <string>

#include "control_link_contract/model.hpp"
#include "control_link_interfaces/msg/control_command.hpp"

namespace control_link_adapters
{
	// RMW implementation 与 publisher GID 共同标识一个 canonical publisher generation
	// GID 只在同一 RMW implementation 的语义下可比较，不作为身份认证凭据
	struct CanonicalPublisherKey
	{
		std::string rmw_implementation;
		std::array<std::uint8_t, RMW_GID_STORAGE_SIZE> publisher_gid;
	};

	// Graph monitor 在 callback 外完成稳定窗口判断，Guard 只消费其不可变结果
	enum class CanonicalEndpointState : std::uint8_t
	{
		// 当前没有符合 Gateway role 的 canonical publisher
		kUnavailable,
		// publisher generation 正在变化，尚未经过 graph_stable_window_ms
		kUnstable,
		// node FQN、唯一性和 generation 已确认，可以继续校验 callback GID
		kConfirmed,
		// 同 Topic 存在多个候选 publisher，canonical authority 无法唯一确定
		kAmbiguous,
	};

	// 单次 validate 使用的 Graph 快照，不在 command callback 中发起 Graph 查询
	// kConfirmed 必须同时提供 node_fqn 与 confirmed_publisher，其他状态不得接受命令
	struct CanonicalEndpointSnapshot
	{
		CanonicalEndpointState state;
		std::string node_fqn;
		std::optional<CanonicalPublisherKey> confirmed_publisher;
	};

	// Guard 的内部结构化拒绝原因，adapter 再将其映射为平台零输出和公共 fault code
	enum class CanonicalRejectReason : std::uint8_t
	{
		kNone,
		kEndpointUnavailable,
		kEndpointUnstable,
		kEndpointAmbiguous,
		kPublisherMismatch,
		kUnknownMode,
		kNonFinite,
		kOutOfRange,
		kInvalidMetadata,
		kInvalidStamp,
		kClockInvalid,
		kFutureStamp,
		kStale,
		kHoldNonzero,
	};

	// 输入对象由调用方持有，结果只表达是否可消费以及失败发生在哪一层
	struct CanonicalValidationResult
	{
		CanonicalRejectReason reason;

		[[nodiscard]] bool accepted() const noexcept;
	};

	// Robot 与 ADAS 执行 adapter 共用的 canonical 边界校验器
	// 不查询 Graph、不维护 watchdog、不执行平台 I/O，也不要求重复 canonical sequence 递增
	class CanonicalInputGuard final
	{
	public:
		// Guard 只读共享 parser 与 Bundle 已校验的 GatewayContract
		explicit CanonicalInputGuard(
			control_link_contract::GatewayContractPtr contract);

		// 固定执行 endpoint state、Gateway role/GID、mode、数值、元数据和 ROS time 校验
		// 只有 accepted 结果才允许刷新 LocalWatchdog 或写入平台 command mailbox
		[[nodiscard]] CanonicalValidationResult validate(
			const control_link_interfaces::msg::ControlCommand &command,
			const CanonicalPublisherKey &actual_publisher,
			const CanonicalEndpointSnapshot &endpoint,
			std::int64_t now_ros_ns,
			bool ros_clock_healthy) const;

	private:
		control_link_contract::GatewayContractPtr contract_;
	};

} // namespace control_link_adapters
