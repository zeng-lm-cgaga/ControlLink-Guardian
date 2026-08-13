#pragma once

#include <cstdint>
#include <optional>
#include <string>
#include <vector>

#include "control_link_gateway/model.hpp"
#include "control_link_gateway/source_arbiter.hpp"
#include "control_link_interfaces/msg/gateway_state.hpp"

namespace control_link_gateway
{
	// 数据面状态只决定控制输出行为，不代替 ROS2 Lifecycle 资源状态
	enum class DataState : std::uint8_t
	{
		// Lifecycle ACTIVE 后尚未建立可用控制来源，固定输出 canonical HOLD
		kStandby = control_link_interfaces::msg::GatewayState::STANDBY,
		// 健康条件满足且来源恢复完成，输出仲裁器选中的 canonical command
		kActive = control_link_interfaces::msg::GatewayState::ACTIVE,
		// 仍可运行但关键 QoS 等条件降级，固定输出 canonical HOLD
		kDegraded = control_link_interfaces::msg::GatewayState::DEGRADED,
		// 来源、执行反馈或关键 endpoint 不安全，固定输出 canonical HOLD
		kSafeStop = control_link_interfaces::msg::GatewayState::SAFE_STOP,
		// 健康恢复后累计同一候选的连续有效样本，达标前固定输出 canonical HOLD
		kRecovering = control_link_interfaces::msg::GatewayState::RECOVERING,
		// 仅用于网关内部不变量破坏，并触发 ROS2 Lifecycle error transition
		kError = control_link_interfaces::msg::GatewayState::ERROR,
	};

	// 原因描述最近一次状态或 active source 变化的触发因素，本身不是状态
	// 内部值直接绑定公共消息常量，避免形成第二套持久化数值
	enum class StateReason : std::uint16_t
	{
		kNone = control_link_interfaces::msg::GatewayState::REASON_NONE,
		// 以下四项由 SourceArbiter 的边沿事件映射产生
		kFirstValidCommand =
			control_link_interfaces::msg::GatewayState::REASON_FIRST_VALID_COMMAND,
		kSourceSwitch = control_link_interfaces::msg::GatewayState::REASON_SOURCE_SWITCH,
		kSourceFallback = control_link_interfaces::msg::GatewayState::REASON_SOURCE_FALLBACK,
		kNoQualifiedSource =
			control_link_interfaces::msg::GatewayState::REASON_NO_QUALIFIED_SOURCE,
		// 以下两项由 ROS Graph 与 QoS 监测结果产生
		kCriticalEndpointUnhealthy =
			control_link_interfaces::msg::GatewayState::REASON_CRITICAL_ENDPOINT_UNHEALTHY,
		kCriticalQosMismatch =
			control_link_interfaces::msg::GatewayState::REASON_CRITICAL_QOS_MISMATCH,
		// 以下四项由 VehicleState 内容校验与接收超时监测产生
		kVehicleStateTimeout =
			control_link_interfaces::msg::GatewayState::REASON_VEHICLE_STATE_TIMEOUT,
		kVehicleSafeStop =
			control_link_interfaces::msg::GatewayState::REASON_VEHICLE_SAFE_STOP,
		kVehicleFault = control_link_interfaces::msg::GatewayState::REASON_VEHICLE_FAULT,
		kVehicleStateInvalid =
			control_link_interfaces::msg::GatewayState::REASON_VEHICLE_STATE_INVALID,
		// 时钟、调度和恢复逻辑分别产生自己的结构化原因
		kClockInvalid = control_link_interfaces::msg::GatewayState::REASON_CLOCK_INVALID,
		kOutputTickOverrun =
			control_link_interfaces::msg::GatewayState::REASON_OUTPUT_TICK_OVERRUN,
		kRecoveryComplete =
			control_link_interfaces::msg::GatewayState::REASON_RECOVERY_COMPLETE,
		kInternalInvariant =
			control_link_interfaces::msg::GatewayState::REASON_INTERNAL_INVARIANT,
	};

	// output tick 消费的一致健康快照，各字段由对应监测组件预先结构化判断
	// 状态机不查询 ROS Graph、不解析 diagnostics 文本，也不执行平台 I/O
	// 默认快照将外部健康条件设为 false，从而 fail closed 到非 ACTIVE 状态
	// 本地 invariant 与 output tick 在监测器报告故障前默认为健康
	struct GatewayHealthSnapshot
	{
		// 关键远端 role 的 endpoint 数量、方向、node FQN 与唯一性均满足 Contract
		bool critical_endpoints_healthy{false};
		// 关键 endpoint 同时满足 DDS compatibility 与 Contract 要求的 exact policy match
		bool critical_qos_compatible{false};
		// ROS time 正常推进且没有未处理的 backward jump
		bool ros_clock_healthy{false};
		// 最近观测的 VehicleState 枚举、时间戳和数值结构合法
		bool vehicle_state_valid{false};
		// 最近合法 VehicleState 没有超过 Contract 的 Topic timeout
		bool vehicle_state_fresh{false};
		// 执行端明确报告 SAFE_STOP
		bool vehicle_reports_safe_stop{false};
		// 执行端报告 FAULT 或携带非零 fault_code
		bool vehicle_reports_fault{false};
		// output tick 未达到 Contract 的连续超限条件
		bool output_tick_healthy{true};
		// Gateway 自身的数据结构与状态约束没有遭到破坏
		bool internal_invariants_healthy{true};
	};

	struct RecoveryCandidateKey
	{
		// source_id 只能标识逻辑来源，RMW implementation + GID 才能区分发布进程 generation
		std::string source_id;
		PublisherGenerationKey publisher_generation;
	};

	// Validator 对上一个 output tick 后真实到达事件的判定结果，不是命令或快照队列
	// rejected evidence 只用于打断当前候选的连续恢复证明，不会进入 Arbiter
	struct RecoveryEvidence
	{
		RecoveryCandidateKey candidate;
		RejectReason result;
		// 只允许当前健康代次内到达的样本参与恢复，防止恢复前证据跨越故障边界
		std::uint64_t health_epoch;

		[[nodiscard]] bool accepted() const noexcept;
	};

	// ControlGatewayNode 在单次 output tick 开始时组装的一致输入批次
	struct StateMachineInput
	{
		GatewayHealthSnapshot health_snapshot;
		ArbitrationDecision arbitration;
		// 保留消息真实到达顺序；空 vector 表示本 tick 没有新输入，不能增加恢复计数
		std::vector<RecoveryEvidence> evidences;
		// 健康聚合结果每次跨越 healthy/unhealthy 边界时递增，并写入同期 evidence
		std::uint64_t health_epoch;
	};

	// 单次数据面决策；非 ACTIVE 时 selected 仅保留 canonical HOLD 的来源追踪元数据
	struct StateMachineDecision
	{
		DataState state;
		StateReason reason;
		std::optional<SourceSnapshot> selected;
		// 仅 RECOVERING 有意义，其他状态由 commit_decision() 强制清零
		std::uint16_t recovery_valid_count;
		// 仅状态或 selected source 变化时递增，周期性重复发布不递增
		std::uint64_t transition_sequence;
	};

	// 纯 C++ 数据面决策类，不查询 ROS Graph、不发布 Topic、也不管理 Lifecycle 资源
	// 内部包含跨 tick 状态且不保证线程安全，只能由串行化的 output tick owner 调用
	class GatewayStateMachine final
	{
	public:
		explicit GatewayStateMachine(control_link_contract::GatewayContractPtr contract);
		// 固定执行 ERROR 锁定、健康门、ACTIVE 边沿和 recovery 四级路由
		[[nodiscard]] StateMachineDecision evaluate(const StateMachineInput &input);

	private:
		// 状态、reason、selected source、恢复上下文和 transition sequence 的唯一提交入口
		[[nodiscard]] StateMachineDecision commit_decision(
			DataState next_state,
			StateReason transition_reason,
			const std::optional<SourceSnapshot> &selected);

		// 先按固定健康优先级覆盖状态，再处理无合格来源；有返回值即终止本 tick 后续路由
		[[nodiscard]] std::optional<StateMachineDecision> evaluate_safety_gate(
			const StateMachineInput &input);

		[[nodiscard]] StateMachineDecision evaluate_active(
			const ArbitrationDecision &arbitration);

		[[nodiscard]] StateMachineDecision evaluate_recovery(
			const StateMachineInput &input);

		control_link_contract::GatewayContractPtr contract_;
		DataState state_{DataState::kStandby};
		StateReason reason_{StateReason::kNone};
		std::optional<RecoveryCandidateKey> recovery_candidate_;
		std::uint16_t recovery_valid_count_{0};
		std::optional<std::uint64_t> recovery_health_epoch_;
		std::optional<std::string> last_selected_source_id_;
		std::uint64_t transition_sequence_{0};
	};
} // namespace control_link_gateway
