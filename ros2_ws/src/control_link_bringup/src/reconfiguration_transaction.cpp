#include "control_link_bringup/reconfiguration_transaction.hpp"

#include <chrono>
#include <exception>
#include <functional>
#include <stdexcept>
#include <utility>

#include <nlohmann/json.hpp>

namespace control_link_bringup
{
	namespace
	{
		using SteadyClock = std::chrono::steady_clock;

		void run_step(
			ReconfigurationResult &result,
			ReconfigurationStep step,
			const std::function<void()> &operation)
		{
			const auto started_at = SteadyClock::now();
			ReconfigurationStepRecord record;
			record.step = step;
			try
			{
				operation();
				record.success = true;
				record.detail = "completed";
			}
			catch (const std::exception &exception)
			{
				record.detail = exception.what();
				record.duration_ns = static_cast<std::uint64_t>(
					std::chrono::duration_cast<std::chrono::nanoseconds>(
						SteadyClock::now() - started_at).count());
				result.steps.push_back(std::move(record));
				throw;
			}
			catch (...)
			{
				record.detail = "unknown exception";
				record.duration_ns = static_cast<std::uint64_t>(
					std::chrono::duration_cast<std::chrono::nanoseconds>(
						SteadyClock::now() - started_at).count());
				result.steps.push_back(std::move(record));
				throw;
			}
			record.duration_ns = static_cast<std::uint64_t>(
				std::chrono::duration_cast<std::chrono::nanoseconds>(
					SteadyClock::now() - started_at).count());
			result.steps.push_back(std::move(record));
		}

		void require_hash(
			const std::string &actual,
			const std::string &expected,
			const char *context)
		{
			if (actual != expected)
			{
				throw std::runtime_error(
					std::string{context} + " identity mismatch: expected=" + expected +
					", actual=" + actual);
			}
		}
	}  // namespace

	const char *reconfiguration_status_name(ReconfigurationStatus status) noexcept
	{
		switch (status)
		{
		case ReconfigurationStatus::kNoOp:
			return "NO_OP";
		case ReconfigurationStatus::kRejected:
			return "REJECTED";
		case ReconfigurationStatus::kCommitted:
			return "COMMITTED";
		case ReconfigurationStatus::kRolledBack:
			return "ROLLED_BACK";
		case ReconfigurationStatus::kRollbackFailed:
			return "ROLLBACK_FAILED";
		}
		return "ROLLBACK_FAILED";
	}

	const char *reconfiguration_step_name(ReconfigurationStep step) noexcept
	{
		switch (step)
		{
		case ReconfigurationStep::kPrecheck:
			return "PRECHECK";
		case ReconfigurationStep::kDeactivateCurrent:
			return "DEACTIVATE_CURRENT";
		case ReconfigurationStep::kCleanupCurrent:
			return "CLEANUP_CURRENT";
		case ReconfigurationStep::kSetCandidateConfig:
			return "SET_CANDIDATE_CONFIG";
		case ReconfigurationStep::kConfigureCandidate:
			return "CONFIGURE_CANDIDATE";
		case ReconfigurationStep::kVerifyCandidate:
			return "VERIFY_CANDIDATE";
		case ReconfigurationStep::kActivateCandidate:
			return "ACTIVATE_CANDIDATE";
		case ReconfigurationStep::kCommitCandidate:
			return "COMMIT_CANDIDATE";
		case ReconfigurationStep::kCleanupCandidate:
			return "CLEANUP_CANDIDATE";
		case ReconfigurationStep::kRestorePreviousConfig:
			return "RESTORE_PREVIOUS_CONFIG";
		case ReconfigurationStep::kConfigurePrevious:
			return "CONFIGURE_PREVIOUS";
		case ReconfigurationStep::kVerifyPrevious:
			return "VERIFY_PREVIOUS";
		case ReconfigurationStep::kActivatePrevious:
			return "ACTIVATE_PREVIOUS";
		}
		return "UNKNOWN";
	}

	ReconfigurationResult execute_reconfiguration(
		const control_link_contract::ReconfigurationPlan &plan,
		std::string transaction_id,
		ReconfigurationBackend &backend)
	{
		ReconfigurationResult result;
		result.transaction_id = std::move(transaction_id);
		result.old_decision_config_hash =
			plan.diff.current_identity.decision_config.decision_config_hash;
		result.requested_decision_config_hash =
			plan.diff.candidate_identity.decision_config.decision_config_hash;
		result.change_class = control_link_contract::config_change_class_name(
			plan.diff.classification);
		result.changed_components = plan.diff.changed_components;

		if (result.transaction_id.empty())
		{
			result.message = "transaction_id must not be empty";
			return result;
		}

		try
		{
			run_step(result, ReconfigurationStep::kPrecheck, [&]
			{
				if (!plan.current || !plan.candidate)
				{
					throw std::invalid_argument(
						"reconfiguration plan does not own both immutable bundles");
				}
				if (!plan.diff.compatible())
				{
					throw std::invalid_argument(
						"candidate is incompatible: " + plan.diff.reason);
				}
				if (plan.diff.requires_profile_restart())
				{
					throw std::invalid_argument(
						"candidate requires coordinated Profile/process restart");
				}
				if (!plan.diff.no_op() && !plan.diff.requires_gateway_rebuild())
				{
					throw std::logic_error(
						"reconfiguration plan has an unsupported classification");
				}
				require_hash(
					backend.current_decision_config_hash(),
					result.old_decision_config_hash,
					"runtime current");
			});
		}
		catch (const std::exception &exception)
		{
			result.failed_step = reconfiguration_step_name(
				ReconfigurationStep::kPrecheck);
			result.message = exception.what();
			return result;
		}

		if (plan.diff.no_op())
		{
			result.status = ReconfigurationStatus::kNoOp;
			result.final_active_decision_config_hash = result.old_decision_config_hash;
			result.message = "candidate is an idempotent semantic no-op";
			return result;
		}

		ReconfigurationStep failed_step = ReconfigurationStep::kDeactivateCurrent;
		try
		{
			failed_step = ReconfigurationStep::kDeactivateCurrent;
			run_step(result, failed_step, [&]
			{
				backend.deactivate_current();
			});
			failed_step = ReconfigurationStep::kCleanupCurrent;
			run_step(result, failed_step, [&]
			{
				backend.cleanup_current();
			});
			failed_step = ReconfigurationStep::kSetCandidateConfig;
			run_step(result, failed_step, [&]
			{
				backend.set_candidate_config();
			});
			failed_step = ReconfigurationStep::kConfigureCandidate;
			run_step(result, failed_step, [&]
			{
				backend.configure_candidate();
			});
			failed_step = ReconfigurationStep::kVerifyCandidate;
			run_step(result, failed_step, [&]
			{
				backend.verify_candidate();
				require_hash(
					backend.current_decision_config_hash(),
					result.requested_decision_config_hash,
					"candidate verify");
			});
			failed_step = ReconfigurationStep::kActivateCandidate;
			run_step(result, failed_step, [&]
			{
				backend.activate_candidate();
				require_hash(
					backend.current_decision_config_hash(),
					result.requested_decision_config_hash,
					"candidate activate");
			});
			failed_step = ReconfigurationStep::kCommitCandidate;
			run_step(result, failed_step, [&]
			{
				backend.commit_candidate();
			});

			result.status = ReconfigurationStatus::kCommitted;
			result.final_active_decision_config_hash =
				result.requested_decision_config_hash;
			result.message = "candidate configuration committed";
			return result;
		}
		catch (const std::exception &exception)
		{
			result.failed_step = reconfiguration_step_name(failed_step);
			result.message = exception.what();
		}
		catch (...)
		{
			result.failed_step = reconfiguration_step_name(failed_step);
			result.message = "candidate transaction failed with an unknown exception";
		}

		result.rollback_attempted = true;
		ReconfigurationStep rollback_step = ReconfigurationStep::kCleanupCandidate;
		try
		{
			rollback_step = ReconfigurationStep::kCleanupCandidate;
			run_step(result, rollback_step, [&]
			{
				backend.cleanup_candidate();
			});
			rollback_step = ReconfigurationStep::kRestorePreviousConfig;
			run_step(result, rollback_step, [&]
			{
				backend.restore_previous_config();
			});
			rollback_step = ReconfigurationStep::kConfigurePrevious;
			run_step(result, rollback_step, [&]
			{
				backend.configure_previous();
			});
			rollback_step = ReconfigurationStep::kVerifyPrevious;
			run_step(result, rollback_step, [&]
			{
				backend.verify_previous();
				require_hash(
					backend.current_decision_config_hash(),
					result.old_decision_config_hash,
					"rollback verify");
			});
			rollback_step = ReconfigurationStep::kActivatePrevious;
			run_step(result, rollback_step, [&]
			{
				backend.activate_previous();
				require_hash(
					backend.current_decision_config_hash(),
					result.old_decision_config_hash,
					"rollback activate");
			});
			result.status = ReconfigurationStatus::kRolledBack;
			result.rollback_succeeded = true;
			result.final_active_decision_config_hash = result.old_decision_config_hash;
			result.message = "candidate failed and previous configuration was restored";
			return result;
		}
		catch (const std::exception &rollback_error)
		{
			result.status = ReconfigurationStatus::kRollbackFailed;
			result.rollback_failed_step = reconfiguration_step_name(rollback_step);
			result.message += "; rollback failed: ";
			result.message += rollback_error.what();
		}
		catch (...)
		{
			result.status = ReconfigurationStatus::kRollbackFailed;
			result.rollback_failed_step = reconfiguration_step_name(rollback_step);
			result.message += "; rollback failed with an unknown exception";
		}
		try
		{
			result.final_active_decision_config_hash =
				backend.current_decision_config_hash();
		}
		catch (...)
		{
			result.final_active_decision_config_hash.clear();
		}
		return result;
	}

	std::string serialize_reconfiguration_result(
		const ReconfigurationResult &result)
	{
		nlohmann::json steps = nlohmann::json::array();
		for (const auto &record : result.steps)
		{
			steps.push_back({
				{"step", reconfiguration_step_name(record.step)},
				{"duration_ns", record.duration_ns},
				{"success", record.success},
				{"detail", record.detail}});
		}
		const nlohmann::json document{
			{"schema_version", result.schema_version},
			{"transaction_id", result.transaction_id},
			{"status", reconfiguration_status_name(result.status)},
			{"old_decision_config_hash", result.old_decision_config_hash},
			{"requested_decision_config_hash", result.requested_decision_config_hash},
			{"final_active_decision_config_hash", result.final_active_decision_config_hash},
			{"change_class", result.change_class},
			{"changed_components", result.changed_components},
			{"failed_step", result.failed_step},
			{"rollback_failed_step", result.rollback_failed_step},
			{"rollback_attempted", result.rollback_attempted},
			{"rollback_succeeded", result.rollback_succeeded},
			{"steps", std::move(steps)},
			{"message", result.message}};
		return document.dump(2);
	}
}  // namespace control_link_bringup
