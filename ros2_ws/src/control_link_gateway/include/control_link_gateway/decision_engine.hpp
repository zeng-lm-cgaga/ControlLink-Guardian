#pragma once

#include <cstdint>
#include <map>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include "control_link_contract/contract_bundle.hpp"
#include "control_link_gateway/command_validator.hpp"
#include "control_link_gateway/decision_event.hpp"
#include "control_link_gateway/source_arbiter.hpp"
#include "control_link_gateway/state_machine.hpp"

namespace control_link_gateway
{
	// live Gateway 与 offline replayer 的唯一确定性编排入口
	// ROS Graph、Clock 与文件 I/O 在类外完成，本类只消费已规范化且有序的 DecisionEvent
	class DecisionEngine final
	{
	public:
		explicit DecisionEngine(
			control_link_contract::ContractBundlePtr contract_bundle);

		// 只有 output_tick 返回 DecisionResult，其他事件只推进确定性运行状态
		[[nodiscard]] std::optional<DecisionResult> apply_event(
			const DecisionEvent &event);

		[[nodiscard]] std::uint64_t next_event_sequence() const;
		[[nodiscard]] std::uint64_t next_health_revision() const;
		[[nodiscard]] bool configured() const noexcept;
		[[nodiscard]] bool active() const noexcept;

		// live Node 使用本函数构造 timing 字段，replayer 会在 apply 时重新计算并核对
		[[nodiscard]] DecisionOutputTickEvent describe_output_tick(
			std::int64_t steady_offset_ns,
			std::int64_t now_ros_ns) const;

		[[nodiscard]] const GatewayHealthSnapshot &health_snapshot() const noexcept;
		[[nodiscard]] const std::map<std::string, SourceRuntimeSlot> &source_slots() const noexcept;
		[[nodiscard]] std::uint64_t consecutive_late_output_ticks() const noexcept;

	private:
		void apply_lifecycle_event(const DecisionLifecycleEvent &event);
		void apply_health_event(const DecisionHealthSnapshotEvent &event);
		void apply_source_sample(const DecisionSourceSampleEvent &event);
		[[nodiscard]] DecisionResult apply_output_tick(
			std::uint64_t event_sequence,
			const DecisionOutputTickEvent &event);

		void reset_active_decision_state();
		void update_health_epoch();
		void validate_steady_offset(std::int64_t offset_ns) const;
		void commit_steady_offset(std::int64_t offset_ns) noexcept;

		control_link_contract::ContractBundlePtr contract_bundle_;
		std::unique_ptr<CommandValidator> command_validator_;
		std::unique_ptr<SourceArbiter> source_arbiter_;
		std::unique_ptr<GatewayStateMachine> state_machine_;
		std::map<std::string, SourceRuntimeSlot> source_slots_;
		std::map<std::string, DecisionSourceEndpoint> source_endpoints_;
		GatewayHealthSnapshot health_snapshot_;
		std::vector<RecoveryEvidence> pending_recovery_evidences_;
		std::uint64_t health_revision_{0U};
		std::uint64_t health_epoch_{0U};
		bool recovery_health_healthy_{false};
		std::uint64_t last_event_sequence_{0U};
		std::optional<std::int64_t> last_steady_offset_ns_;
		std::optional<std::int64_t> last_output_tick_offset_ns_;
		std::uint64_t consecutive_late_output_ticks_{0U};
		bool configured_{false};
		bool active_{false};
	};
}  // namespace control_link_gateway
