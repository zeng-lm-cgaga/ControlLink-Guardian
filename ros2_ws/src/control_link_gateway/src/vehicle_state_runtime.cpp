#include "control_link_gateway/vehicle_state_runtime.hpp"

#include <stdexcept>
#include <utility>

namespace control_link_gateway
{
	void commit_vehicle_state_validation_result(
		VehicleStateRuntime &runtime,
		const VehicleStateValidationResult &result)
	{
		const bool accepted_reason =
			result.reason == VehicleStateRejectReason::kNone;
		const bool has_snapshot = result.snapshot.has_value();
		if (accepted_reason != has_snapshot)
		{
			throw std::logic_error(
				"VehicleStateValidationResult reason and snapshot are inconsistent");
		}

		if (!accepted_reason)
		{
			// 非法消息立即使内容健康失效，但不能刷新或清除 last-valid 接收时间
			runtime.latest_message_valid = false;
			runtime.last_reject_reason = result.reason;
			runtime.rejected_count += 1;
			return;
		}

		// 先完成可能抛异常的复制，避免 Runtime 进入半提交状态
		auto next_snapshot = result.snapshot.value();
		runtime.latest_valid_snapshot = std::move(next_snapshot);
		runtime.latest_message_valid = true;
		runtime.accepted_count += 1;
	}

	VehicleStateHealthAssessment assess_vehicle_state_health(
		const VehicleStateRuntime &runtime,
		std::chrono::steady_clock::time_point now,
		std::chrono::milliseconds timeout)
	{
		if (timeout <= std::chrono::milliseconds::zero())
		{
			throw std::invalid_argument(
				"VehicleState health timeout must be greater than zero");
		}

		if (!runtime.latest_valid_snapshot.has_value())
		{
			if (runtime.latest_message_valid)
			{
				throw std::logic_error(
					"VehicleState runtime marks the latest message valid without a snapshot");
			}

			return VehicleStateHealthAssessment{
				false,
				false,
				false,
				false};
		}

		const auto &snapshot = runtime.latest_valid_snapshot.value();
		if (now < snapshot.received_at)
		{
			throw std::logic_error(
				"VehicleState health assessment time precedes the last-valid receive time");
		}

		const bool fresh = now - snapshot.received_at <= timeout;
		const bool reports_safe_stop =
			snapshot.state ==
			control_link_interfaces::msg::VehicleState::SAFE_STOP;
		const bool reports_fault =
			snapshot.state == control_link_interfaces::msg::VehicleState::FAULT ||
			snapshot.fault_code !=
			control_link_interfaces::msg::VehicleState::FAULT_NONE;

		return VehicleStateHealthAssessment{
			runtime.latest_message_valid,
			fresh,
			reports_safe_stop,
			reports_fault};
	}
} // namespace control_link_gateway
