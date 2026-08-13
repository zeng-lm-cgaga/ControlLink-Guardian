#pragma once

#include <chrono>
#include <cstdint>
#include <optional>

#include "control_link_gateway/vehicle_state_validator.hpp"

namespace control_link_gateway
{
	struct VehicleStateRuntime
	{
		// 表示最近到达的消息是否合法，不等于“历史上是否收到过合法消息”
		bool latest_message_valid{false};

		VehicleStateRejectReason last_reject_reason{
			VehicleStateRejectReason::kNone};

		// 非法消息不能覆盖它，也不能修改其中的 received_at
		std::optional<VehicleStateSnapshot> latest_valid_snapshot;

		std::uint64_t accepted_count{0};
		std::uint64_t rejected_count{0};
	};

	struct VehicleStateHealthAssessment
	{
		bool valid;
		bool fresh;
		bool reports_safe_stop;
		bool reports_fault;
	};

	void commit_vehicle_state_validation_result(
		VehicleStateRuntime &runtime,
		const VehicleStateValidationResult &result);

	[[nodiscard]] VehicleStateHealthAssessment
	assess_vehicle_state_health(
		const VehicleStateRuntime &runtime,
		std::chrono::steady_clock::time_point now,
		std::chrono::milliseconds timeout);
} // namespace control_link_gateway
