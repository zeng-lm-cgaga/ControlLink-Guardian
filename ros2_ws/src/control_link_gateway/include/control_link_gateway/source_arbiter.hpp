#pragma once

#include <cstdint>
#include <chrono>
#include <map>
#include <optional>
#include <string>

#include "control_link_contract/model.hpp"
#include "control_link_gateway/model.hpp"

namespace control_link_gateway
{
	// 仲裁输入由输出 tick 每次组装；仲裁器不缓存指针，不持有快照容器
	struct ArbitrationInput
	{
		// 非空，且 Gateway 已排除 disabled 与 generation-invalid 来源
		// 每来源单槽（key = source_id），仅在本次 evaluate 调用期间有效
		const std::map<std::string, SourceSnapshot> *snapshots;
		std::int64_t now_ros_ns;						  // freshness 用（统一 ROS clock）
		std::chrono::steady_clock::time_point now_steady; // lease/switch hold 用（本地 steady clock）
	};

	// 仲裁器和 SourceStatus 共享同一份 freshness/lease 结论，避免 Node 复制超时规则
	struct SourceSnapshotAssessment
	{
		bool command_fresh;
		bool lease_valid;
		std::int64_t command_age_ns;

		[[nodiscard]] bool qualified() const noexcept
		{
			return command_fresh && lease_valid;
		}
	};

	[[nodiscard]] SourceSnapshotAssessment assess_source_snapshot(
		const std::string &map_source_id,
		const SourceSnapshot &snapshot,
		const control_link_contract::GatewayContract &contract,
		const control_link_contract::SourcePolicy &policy,
		std::int64_t now_ros_ns,
		std::chrono::steady_clock::time_point now_steady);

	// event 只描述本次 evaluate 相对上一次的转变（edge）；
	// 持续状态由 selected 表达（level）：持续无合格来源时为 kNoChange + 空 selected
	enum class ArbitrationEvent : std::uint8_t
	{
		kNoChange,
		kFirstSelection,	// 从无 active 到选出来源（含失去全部来源后的重新选出）
		kSwitch,			// active 仍合格，但 challenger 连续更优达 source_switch_hold_ms
		kFallback,			// active 失效，立即改选其他合格来源，不受 hold 约束
		kNoQualifiedSource, // 本 tick 起没有任何合格来源
	};

	struct ArbitrationDecision
	{
		// 值拷贝：决策结果不依赖快照容器在本 tick 之后的生命周期；空 = 无合格来源
		std::optional<SourceSnapshot> selected;
		ArbitrationEvent event;
	};

	// 来源仲裁器：纯 C++ 决策类，只消费已通过 CommandValidator 的快照。
	// 合格 = 命令年龄未超 command_timeout_ms（ROS clock）且槽位未超该来源 lease_timeout_ms（steady clock）；
	// 选优固定为 priority 降序 -> 命令年龄升序 -> source_id 升序。
	// 命令校验归 CommandValidator，恢复计数与 GatewayState 转换归状态机，此类不重复实现
	class SourceArbiter final
	{
	public:
		// 两个配置快照均不可为空，空指针抛 std::invalid_argument
		SourceArbiter(
			control_link_contract::GatewayContractPtr contract,
			control_link_contract::SourcePolicyPtr policy);

		// 非 const：更新 active/challenger 决策历史；input.snapshots 为空指针抛 std::invalid_argument
		[[nodiscard]] ArbitrationDecision evaluate(const ArbitrationInput &input);

	private:
		// challenger 身份与其“开始持续更优”的时刻必须同生共死，合并为一个 optional
		struct ChallengerTracking
		{
			std::string source_id;
			std::chrono::steady_clock::time_point better_since;
		};

		control_link_contract::GatewayContractPtr contract_;
		control_link_contract::SourcePolicyPtr policy_;
		// 上一次 evaluate 的已选来源，下一 tick 用它判断保持、switch 或 fallback
		std::optional<std::string> active_source_id_;
		// 仅跟踪当前持续优于 active 的来源，不代表它已经获得控制权
		std::optional<ChallengerTracking> challenger_;
	};
} // namespace control_link_gateway
