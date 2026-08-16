#include "control_link_gateway/decision_replay.hpp"

#include <limits>
#include <memory>
#include <stdexcept>
#include <utility>
#include <variant>

#include <nlohmann/json.hpp>

#include "control_link_gateway/decision_engine.hpp"

namespace control_link_gateway
{
	namespace
	{
		void validate_trace_identity(
			const control_link_contract::ContractBundle &bundle,
			const DecisionTraceDocument &trace)
		{
			if (!trace.footer.trace_valid || trace.footer.trace_overflow)
			{
				throw DecisionReplayError(
					DecisionReplayErrorCode::kInvalidTrace,
					trace.footer.last_event_sequence,
					"Decision Trace footer marks the trace INVALID: " +
					trace.footer.error_message);
			}
			const auto &actual = bundle.identity;
			const auto &header = trace.header;
			if (header.profile_id != actual.decision_config.profile_id ||
				header.contract_id != actual.contract.contract_id ||
				header.contract_version != actual.contract.contract_version ||
				header.contract_hash != actual.contract.contract_hash ||
				header.decision_config_hash != actual.decision_config.decision_config_hash)
			{
				throw DecisionReplayError(
					DecisionReplayErrorCode::kConfigIdentityMismatch,
					0U,
					"Decision Trace config identity does not match the loaded ContractBundle");
			}
			if (trace.frames.empty())
			{
				throw DecisionReplayError(
					DecisionReplayErrorCode::kInvalidTrace,
					0U,
					"Decision Trace contains no events");
			}
		}
	}  // namespace

	DecisionReplayError::DecisionReplayError(
		DecisionReplayErrorCode code,
		std::uint64_t event_sequence,
		std::string message)
		: std::runtime_error(
			event_sequence == 0U ? std::move(message) :
				"Decision Replay event " + std::to_string(event_sequence) + ": " + message),
		  code_(code),
		  event_sequence_(event_sequence)
	{
	}

	DecisionReplayErrorCode DecisionReplayError::code() const noexcept
	{
		return code_;
	}

	std::uint64_t DecisionReplayError::event_sequence() const noexcept
	{
		return event_sequence_;
	}

	DecisionReplayReport replay_decision_trace(
		control_link_contract::ContractBundlePtr contract_bundle,
		const DecisionTraceDocument &trace,
		std::uint32_t repetitions)
	{
		if (!contract_bundle)
		{
			throw DecisionReplayError(
				DecisionReplayErrorCode::kInvalidArgument,
				0U,
				"Decision Replay requires a non-null ContractBundle");
		}
		if (repetitions == 0U || repetitions > 10'000U)
		{
			throw DecisionReplayError(
				DecisionReplayErrorCode::kInvalidArgument,
				0U,
				"Decision Replay repetitions must be in [1, 10000]");
		}
		validate_trace_identity(*contract_bundle, trace);

		DecisionReplayReport report;
		report.valid = true;
		report.requested_repetitions = repetitions;
		report.event_count = trace.frames.size();
		report.result_count = trace.footer.result_count;

		for (std::uint32_t repetition = 0U; repetition < repetitions; ++repetition)
		{
			DecisionEngine engine{contract_bundle};
			for (const auto &frame : trace.frames)
			{
				std::optional<DecisionResult> actual_result;
				try
				{
					actual_result = engine.apply_event(frame.event);
				}
				catch (const std::exception &exception)
				{
					throw DecisionReplayError(
						DecisionReplayErrorCode::kInvalidEvent,
						frame.event.event_sequence,
						exception.what());
				}

				if (actual_result.has_value() != frame.expected_result.has_value())
				{
					report.first_difference_event_sequence = frame.event.event_sequence;
					report.first_difference_field = "result_presence";
					report.message = "output result presence differs from the trace";
					return report;
				}
				if (actual_result.has_value() && !decision_results_equal(
						frame.expected_result.value(), actual_result.value()))
				{
					report.first_difference_event_sequence = frame.event.event_sequence;
					report.first_difference_field = describe_first_decision_difference(
						frame.expected_result.value(), actual_result.value());
					report.message = "Decision Result diverged from the recorded result";
					return report;
				}
			}
			report.completed_repetitions += 1U;
		}

		report.matched = true;
		report.message = "all Decision Results matched";
		return report;
	}

	std::string serialize_decision_replay_report(const DecisionReplayReport &report)
	{
		const nlohmann::json result{
			{"valid", report.valid},
			{"matched", report.matched},
			{"completed_repetitions", report.completed_repetitions},
			{"requested_repetitions", report.requested_repetitions},
			{"event_count", report.event_count},
			{"result_count", report.result_count},
			{"first_difference_event_sequence",
				report.first_difference_event_sequence.has_value() ?
					nlohmann::json(report.first_difference_event_sequence.value()) :
					nlohmann::json(nullptr)},
			{"first_difference_field", report.first_difference_field},
			{"message", report.message}};
		return result.dump();
	}
}  // namespace control_link_gateway
