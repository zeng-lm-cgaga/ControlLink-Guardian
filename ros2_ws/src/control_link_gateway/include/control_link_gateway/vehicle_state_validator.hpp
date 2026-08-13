#pragma once

#include <cstdint>
#include <chrono>
#include <optional>

#include "control_link_gateway/model.hpp"
#include "control_link_contract/model.hpp"
#include "control_link_interfaces/msg/vehicle_state.hpp"

namespace control_link_gateway
{
	enum class VehicleStateRejectReason : std::uint8_t
	{
		kNone,
		kPublisherGenerationUnstable,
		kUnknownState,
		kNonFinite,
		kZeroStamp,
		kInvalidStamp,
		kClockInvalid,
		kFutureStamp,
		kStale,
	};

	struct VehicleStateValidationContext
	{
		PublisherGenerationKey publisher_generation;
		std::int64_t now_ros_ns;
		bool ros_clock_valid;
		std::chrono::steady_clock::time_point received_at;
	};

	struct VehicleStateSnapshot
	{
		PublisherGenerationKey publisher_generation;
		std::int64_t observed_at_ns;
		std::uint8_t state;
		std::uint16_t fault_code;
		double linear_velocity_mps;
		double angular_velocity_radps;
		std::uint8_t rolling_counter;
		std::chrono::steady_clock::time_point received_at;
	};

	struct VehicleStateValidationResult
	{
		VehicleStateRejectReason reason;
		std::optional<VehicleStateSnapshot> snapshot;

		[[nodiscard]] bool accepted() const noexcept;
	};

	class VehicleStateValidator final
	{
	public:
		explicit VehicleStateValidator(
			control_link_contract::GatewayContractPtr contract);

		[[nodiscard]] VehicleStateValidationResult validate(
			const control_link_interfaces::msg::VehicleState &state,
			const VehicleStateValidationContext &context) const;

	private:
		control_link_contract::GatewayContractPtr contract_;
	};
} // namespace control_link_gateway
