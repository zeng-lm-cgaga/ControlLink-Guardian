#pragma once

#include <atomic>
#include <chrono>
#include <cstdint>
#include <map>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <vector>

#include "control_link_contract/contract_bundle.hpp"
#include "control_link_contract/qos_factory.hpp"
#include "control_link_gateway/critical_endpoint_monitor.hpp"
#include "control_link_gateway/decision_engine.hpp"
#include "control_link_gateway/decision_trace_recorder.hpp"
#include "control_link_gateway/graph_monitor.hpp"
#include "control_link_gateway/model.hpp"
#include "control_link_gateway/vehicle_state_runtime.hpp"
#include "control_link_interfaces/msg/control_command.hpp"
#include "control_link_interfaces/msg/gateway_state.hpp"
#include "control_link_interfaces/msg/source_status.hpp"
#include "control_link_interfaces/msg/vehicle_state.hpp"

#include "diagnostic_msgs/msg/diagnostic_array.hpp"

#include "rclcpp/node_options.hpp"
#include "rclcpp/clock.hpp"
#include "rclcpp/message_info.hpp"
#include "rclcpp/qos.hpp"
#include "rclcpp/subscription.hpp"
#include "rclcpp/timer.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "rclcpp/callback_group.hpp"
#include "rclcpp_lifecycle/lifecycle_publisher.hpp"

namespace control_link_gateway
{
	// Lifecycle 只管理配置与 ROS 资源，运行期控制状态由 GatewayStateMachine 独立管理
	class ControlGatewayNode final : public rclcpp_lifecycle::LifecycleNode
	{
	public:
		using CallbackReturn =
			rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

		explicit ControlGatewayNode(
			const rclcpp::NodeOptions &options = rclcpp::NodeOptions());

		CallbackReturn on_configure(
			const rclcpp_lifecycle::State &previous_state) override;

		// 只有 Graph、required source、VehicleState bootstrap 和运行时依赖均健康时才激活 Publisher
		CallbackReturn on_activate(
			const rclcpp_lifecycle::State &previous_state) override;

		// 先阻断普通 Subscription 写入，再逆序停用数据面 Publisher
		CallbackReturn on_deactivate(
			const rclcpp_lifecycle::State &previous_state) override;

		CallbackReturn on_error(
			const rclcpp_lifecycle::State &previous_state) override;

		CallbackReturn on_cleanup(
			const rclcpp_lifecycle::State &previous_state) override;

	private:
		using ControlCommand = control_link_interfaces::msg::ControlCommand;
		using SourceSubscription = rclcpp::Subscription<ControlCommand>;
		using VehicleState = control_link_interfaces::msg::VehicleState;
		using VehicleStateSubscription = rclcpp::Subscription<VehicleState>;

		// health timer 只负责 ROS Graph 采样、stable window 和健康输入聚合
		void poll_graph() noexcept;
		// Lifecycle INACTIVE 期间也持续采样 ROS time，保证 activate gate 不依赖数据面 timer
		void poll_ros_clock() noexcept;
		// 在 health callback group 内采样 ROS time，并发布只读诊断快照
		void publish_diagnostics() noexcept;
		// 唯一的数据面决策与发布入口，使用 steady timer 驱动
		void run_output_tick() noexcept;
		// subscription 创建时绑定 expected_source_id，消息自身不能选择要写入的 SourceRuntimeSlot
		void handle_source_command(
			const std::string &expected_source_id,
			const ControlCommand &command,
			const rclcpp::MessageInfo &message_info) noexcept;
		// VehicleState 在 INACTIVE 期间也要接收，用于下一步的 activate gate
		void handle_vehicle_state(
			const VehicleState &state,
			const rclcpp::MessageInfo &message_info) noexcept;
		// 调用方必须已经持有 runtime_mutex_
		void refresh_vehicle_state_health_locked(
			std::chrono::steady_clock::time_point now);
		// 调用方必须已经持有 runtime_mutex_，ROS time 和 steady time 不得混算
		void update_ros_clock_health_locked(
			std::chrono::steady_clock::time_point now_steady,
			std::int64_t now_ros_ns);
		// 所有 event sequence 与相对 steady offset 都只能在 runtime_mutex_ 内生成
		[[nodiscard]] std::int64_t decision_steady_offset_locked(
			std::chrono::steady_clock::time_point now) const;
		[[nodiscard]] std::optional<DecisionResult> submit_decision_event_locked(
			DecisionEventPayload payload);
		void submit_health_snapshot_locked(
			std::chrono::steady_clock::time_point observed_at);
		void request_lifecycle_error() noexcept;

		// 所有组件共享同一份不可变 Contract 快照，cleanup 按依赖的逆序释放
		control_link_contract::ContractBundlePtr contract_bundle_;
		std::unique_ptr<control_link_contract::QosFactory> qos_factory_;
		// DecisionEngine 是 live 与 offline replay 共用的唯一确定性决策 owner
		std::unique_ptr<DecisionEngine> decision_engine_;
		std::unique_ptr<DecisionTraceRecorder> decision_trace_recorder_;
		std::optional<std::chrono::steady_clock::time_point> decision_steady_origin_;
		bool trace_recording_failed_{false};
		// CallbackGroup 在节点构造时创建并保持到节点析构，避免 Executor spin 期间重建 wait set 拓扑
		rclcpp::CallbackGroup::SharedPtr data_plane_group_;
		rclcpp::CallbackGroup::SharedPtr health_group_;
		// health_group_ 与 data_plane_group_ 会并行提交 DecisionEvent，统一持锁排序
		std::mutex runtime_mutex_;
		// 防止 output tick 发布与 Lifecycle Publisher activate/deactivate/cleanup 并发交错
		std::mutex publisher_mutex_;
		std::map<std::string, SourceEndpointStabilityTracker>
			source_endpoint_trackers_;
		std::map<std::string, CriticalEndpointStabilityTracker>
			critical_endpoint_trackers_;
		std::optional<rclcpp::QoS> source_input_qos_;
		std::optional<rclcpp::QoS> canonical_output_qos_;
		std::optional<rclcpp::QoS> vehicle_state_qos_;
		std::string rmw_implementation_;
		rclcpp::TimerBase::SharedPtr graph_timer_;
		rclcpp::TimerBase::SharedPtr clock_timer_;
		rclcpp::TimerBase::SharedPtr diagnostics_timer_;
		rclcpp::TimerBase::SharedPtr output_timer_;
		std::map<std::string, SourceSubscription::SharedPtr>
			source_subscriptions_;
		VehicleStateSubscription::SharedPtr vehicle_state_subscription_;
		// 普通 ROS2 Subscription 不受 Lifecycle 自动启停，只有 on_activate gate 成功后才允许写入 Slot
		bool data_plane_enabled_{false};
		// 配置提交前拒绝 VehicleState callback，避免局部构造期间误改成员健康状态
		bool health_plane_configured_{false};
		GatewayHealthSnapshot health_snapshot_;
		std::unique_ptr<VehicleStateValidator> vehicle_state_validator_;
		VehicleStateRuntime vehicle_state_runtime_;
		std::optional<PublisherGenerationKey>
			vehicle_state_publisher_generation_;
		// last_ros_time_ns_ 是最近一次采样值，last_ros_time_progress_at_ 属于 steady clock
		std::optional<std::int64_t> last_ros_time_ns_;
		std::optional<std::chrono::steady_clock::time_point>
			last_ros_time_progress_at_;
		// Clock jump callback 不能获取 runtime_mutex_，只把事件无锁转交给 health timer
		std::atomic<std::uint64_t> pending_ros_clock_backward_jumps_{0};
		rclcpp::JumpHandler::SharedPtr ros_clock_jump_handler_;
		bool ros_clock_backward_jump_{false};
		std::uint64_t ros_clock_backward_jump_count_{0};
		std::optional<std::chrono::steady_clock::time_point>
			last_output_tick_at_;
		bool lifecycle_error_requested_{false};
		std::optional<DecisionResult> last_decision_;

		rclcpp_lifecycle::LifecyclePublisher<
			control_link_interfaces::msg::ControlCommand>::SharedPtr
			canonical_output_publisher_;

		rclcpp_lifecycle::LifecyclePublisher<
			control_link_interfaces::msg::GatewayState>::SharedPtr
			gateway_state_publisher_;

		rclcpp_lifecycle::LifecyclePublisher<
			control_link_interfaces::msg::SourceStatus>::SharedPtr
			source_status_publisher_;

		rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr
			diagnostics_publisher_;
	};
} // namespace control_link_gateway
