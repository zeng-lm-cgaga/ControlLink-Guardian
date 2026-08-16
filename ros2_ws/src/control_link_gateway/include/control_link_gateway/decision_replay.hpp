#pragma once

#include <cstdint>
#include <optional>
#include <stdexcept>
#include <string>

#include "control_link_contract/contract_bundle.hpp"
#include "control_link_gateway/decision_trace.hpp"

namespace control_link_gateway
{
	enum class DecisionReplayErrorCode : std::uint8_t
	{
		kInvalidArgument,
		kInvalidTrace,
		kConfigIdentityMismatch,
		kInvalidEvent,
	};

	class DecisionReplayError final : public std::runtime_error
	{
	public:
		DecisionReplayError(
			DecisionReplayErrorCode code,
			std::uint64_t event_sequence,
			std::string message);

		[[nodiscard]] DecisionReplayErrorCode code() const noexcept;
		[[nodiscard]] std::uint64_t event_sequence() const noexcept;

	private:
		DecisionReplayErrorCode code_;
		std::uint64_t event_sequence_;
	};

	struct DecisionReplayReport
	{
		bool valid{false};
		bool matched{false};
		std::uint32_t completed_repetitions{0U};
		std::uint32_t requested_repetitions{0U};
		std::uint64_t event_count{0U};
		std::uint64_t result_count{0U};
		std::optional<std::uint64_t> first_difference_event_sequence;
		std::string first_difference_field;
		std::string message;
	};

	[[nodiscard]] DecisionReplayReport replay_decision_trace(
		control_link_contract::ContractBundlePtr contract_bundle,
		const DecisionTraceDocument &trace,
		std::uint32_t repetitions);

	[[nodiscard]] std::string serialize_decision_replay_report(
		const DecisionReplayReport &report);
}  // namespace control_link_gateway
