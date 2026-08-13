#include "control_link_gateway/state_machine.hpp"

#include <limits>
#include <stdexcept>
#include <utility>

namespace control_link_gateway
{
	namespace
	{
		struct HealthOverride
		{
			DataState state;
			StateReason reason;
		};

		std::optional<HealthOverride> assess_health(const GatewayHealthSnapshot &health)
		{
			// 命中第一项即锁定本 tick 的最高优先级故障，QoS 降级必须让位于安全停止条件
			if (!health.internal_invariants_healthy)
			{
				return HealthOverride{
					DataState::kError,
					StateReason::kInternalInvariant};
			}

			if (!health.output_tick_healthy)
			{
				return HealthOverride{
					DataState::kSafeStop,
					StateReason::kOutputTickOverrun};
			}

			if (!health.critical_endpoints_healthy)
			{
				return HealthOverride{
					DataState::kSafeStop,
					StateReason::kCriticalEndpointUnhealthy};
			}

			if (!health.ros_clock_healthy)
			{
				return HealthOverride{
					DataState::kSafeStop,
					StateReason::kClockInvalid};
			}

			if (!health.vehicle_state_valid)
			{
				return HealthOverride{
					DataState::kSafeStop,
					StateReason::kVehicleStateInvalid};
			}

			if (!health.vehicle_state_fresh)
			{
				return HealthOverride{
					DataState::kSafeStop,
					StateReason::kVehicleStateTimeout};
			}

			if (health.vehicle_reports_fault)
			{
				return HealthOverride{
					DataState::kSafeStop,
					StateReason::kVehicleFault};
			}

			if (health.vehicle_reports_safe_stop)
			{
				return HealthOverride{
					DataState::kSafeStop,
					StateReason::kVehicleSafeStop};
			}

			if (!health.critical_qos_compatible)
			{
				return HealthOverride{
					DataState::kDegraded,
					StateReason::kCriticalQosMismatch};
			}

			return std::nullopt;
		}

		bool same_candidate(
			const RecoveryCandidateKey &left,
			const RecoveryCandidateKey &right) noexcept
		{
			return left.source_id == right.source_id && left.publisher_generation == right.publisher_generation;
		}

		RecoveryCandidateKey candidate_from_snapshot(
			const SourceSnapshot &snapshot)
		{
			return RecoveryCandidateKey{
				snapshot.source_id,
				snapshot.publisher_generation};
		}

	} // namespace
	bool RecoveryEvidence::accepted() const noexcept
	{
		return result == RejectReason::kNone;
	}

	GatewayStateMachine::GatewayStateMachine(control_link_contract::GatewayContractPtr contract)
		: contract_(std::move(contract))
	{
		if (contract_ == nullptr)
		{
			throw std::invalid_argument("GatewayStateMachine requires a non-null GatewayContract");
		}
	}

	StateMachineDecision GatewayStateMachine::commit_decision(
		DataState next_state,
		StateReason transition_reason,
		const std::optional<SourceSnapshot> &selected)
	{
		std::optional<std::string> next_selected_source_id;
		if (selected.has_value())
		{
			next_selected_source_id = selected.value().source_id;
		}

		if (state_ != next_state || last_selected_source_id_ != next_selected_source_id)
		{
			state_ = next_state;
			reason_ = transition_reason;
			last_selected_source_id_ = next_selected_source_id;
			transition_sequence_ += 1;
		}

		if (next_state != DataState::kRecovering)
		{
			recovery_candidate_.reset();
			recovery_valid_count_ = 0;
		}

		return StateMachineDecision{
			state_,
			reason_,
			selected,
			recovery_valid_count_,
			transition_sequence_};
	}

	std::optional<StateMachineDecision> GatewayStateMachine::evaluate_safety_gate(
		const StateMachineInput &input)
	{
		const auto health_override = assess_health(input.health_snapshot);
		if (health_override.has_value() &&
			health_override->state != DataState::kDegraded)
		{
			return commit_decision(
				health_override->state,
				health_override->reason,
				input.arbitration.selected);
		}

		if (!input.arbitration.selected.has_value() &&
			state_ != DataState::kStandby)
		{
			// DEGRADED 仍表示存在可追踪来源，无来源必须提升为 SAFE_STOP
			return commit_decision(
				DataState::kSafeStop,
				StateReason::kNoQualifiedSource,
				input.arbitration.selected);
		}

		if (health_override.has_value())
		{
			return commit_decision(
				health_override->state,
				health_override->reason,
				input.arbitration.selected);
		}

		if (!input.arbitration.selected.has_value())
		{
			return commit_decision(
				DataState::kStandby,
				reason_,
				input.arbitration.selected);
		}

		return std::nullopt;
	}

	StateMachineDecision GatewayStateMachine::evaluate_active(
		const ArbitrationDecision &arbitration)
	{
		switch (arbitration.event)
		{
		case ArbitrationEvent::kNoChange:
			return commit_decision(
				DataState::kActive,
				reason_,
				arbitration.selected);

		case ArbitrationEvent::kSwitch:
			return commit_decision(
				DataState::kActive,
				StateReason::kSourceSwitch,
				arbitration.selected);

		case ArbitrationEvent::kFallback:
			return commit_decision(
				DataState::kActive,
				StateReason::kSourceFallback,
				arbitration.selected);

		case ArbitrationEvent::kFirstSelection:
		case ArbitrationEvent::kNoQualifiedSource:
			// ACTIVE 下出现首次选择或无来源边沿，说明 Arbiter 与状态机历史已经失配
			return commit_decision(
				DataState::kError,
				StateReason::kInternalInvariant,
				arbitration.selected);
		}

		return commit_decision(
			DataState::kError,
			StateReason::kInternalInvariant,
			arbitration.selected);
	}

	StateMachineDecision GatewayStateMachine::evaluate_recovery(
		const StateMachineInput &input)
	{
		if (!recovery_health_epoch_.has_value() ||
			recovery_health_epoch_.value() != input.health_epoch)
		{
			// 健康边界发生过变化后，旧代次的候选和证据都不能继续证明恢复
			recovery_health_epoch_ = input.health_epoch;
			recovery_candidate_.reset();
			recovery_valid_count_ = 0;
		}

		std::optional<StateReason> recovery_reason;
		switch (input.arbitration.event)
		{
		case ArbitrationEvent::kNoChange:
			recovery_reason = reason_;
			break;

		case ArbitrationEvent::kFirstSelection:
			recovery_reason = StateReason::kFirstValidCommand;
			break;

		case ArbitrationEvent::kSwitch:
			recovery_reason = StateReason::kSourceSwitch;
			break;

		case ArbitrationEvent::kFallback:
			recovery_reason = StateReason::kSourceFallback;
			break;

		case ArbitrationEvent::kNoQualifiedSource:
			// safety gate 已保证 selected 非空，此组合只能来自内部决策历史失配
			return commit_decision(
				DataState::kError,
				StateReason::kInternalInvariant,
				input.arbitration.selected);
		}

		if (!recovery_reason.has_value())
		{
			return commit_decision(
				DataState::kError,
				StateReason::kInternalInvariant,
				input.arbitration.selected);
		}

		const auto candidate = candidate_from_snapshot(
			input.arbitration.selected.value());
		if (!recovery_candidate_.has_value() ||
			!same_candidate(recovery_candidate_.value(), candidate))
		{
			// source 或 publisher generation 改变后，连续恢复证据必须从零重新累计
			recovery_candidate_ = candidate;
			recovery_valid_count_ = 0;
		}

		for (const auto &evidence : input.evidences)
		{
			if (evidence.health_epoch != input.health_epoch)
			{
				continue;
			}

			if (!same_candidate(recovery_candidate_.value(), evidence.candidate))
			{
				// 其他来源的好坏样本都不能改变当前候选的连续恢复计数
				continue;
			}

			if (!evidence.accepted())
			{
				recovery_valid_count_ = 0;
				continue;
			}

			if (recovery_valid_count_ <
				std::numeric_limits<std::uint16_t>::max())
			{
				recovery_valid_count_ += 1;
			}
		}

		if (recovery_valid_count_ >=
			contract_->gateway.recovery_valid_samples)
		{
			return commit_decision(
				DataState::kActive,
				StateReason::kRecoveryComplete,
				input.arbitration.selected);
		}

		return commit_decision(
			DataState::kRecovering,
			recovery_reason.value(),
			input.arbitration.selected);
	}

	StateMachineDecision GatewayStateMachine::evaluate(
		const StateMachineInput &input)
	{
		if (state_ == DataState::kError)
		{
			return commit_decision(
				state_,
				reason_,
				input.arbitration.selected);
		}

		const auto decision = evaluate_safety_gate(input);
		if (decision.has_value())
		{
			return decision.value();
		}

		if (state_ == DataState::kActive)
		{
			return evaluate_active(input.arbitration);
		}

		return evaluate_recovery(input);
	}
} // namespace control_link_gateway
