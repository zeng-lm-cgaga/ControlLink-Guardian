#pragma once

#include <cstdint>
#include <string>
#include <vector>

#include "control_link_contract/reconfiguration.hpp"

namespace control_link_bringup
{
	enum class ReconfigurationStatus : std::uint8_t
	{
		kNoOp,
		kRejected,
		kCommitted,
		kRolledBack,
		kRollbackFailed,
	};

	enum class ReconfigurationStep : std::uint8_t
	{
		kPrecheck,
		kDeactivateCurrent,
		kCleanupCurrent,
		kSetCandidateConfig,
		kConfigureCandidate,
		kVerifyCandidate,
		kActivateCandidate,
		kCommitCandidate,
		kCleanupCandidate,
		kRestorePreviousConfig,
		kConfigurePrevious,
		kVerifyPrevious,
		kActivatePrevious,
	};

	struct ReconfigurationStepRecord
	{
		ReconfigurationStep step{ReconfigurationStep::kPrecheck};
		std::uint64_t duration_ns{0U};
		bool success{false};
		std::string detail;
	};

	struct ReconfigurationResult
	{
		std::uint32_t schema_version{1U};
		std::string transaction_id;
		ReconfigurationStatus status{ReconfigurationStatus::kRejected};
		std::string old_decision_config_hash;
		std::string requested_decision_config_hash;
		std::string final_active_decision_config_hash;
		std::string change_class;
		std::vector<std::string> changed_components;
		std::string failed_step;
		std::string rollback_failed_step;
		bool rollback_attempted{false};
		bool rollback_succeeded{false};
		std::vector<ReconfigurationStepRecord> steps;
		std::string message;
	};

	// Backend 封装 ROS2 Lifecycle、参数、Graph 和持久化 I/O，事务顺序只在本类入口维护
	class ReconfigurationBackend
	{
	public:
		virtual ~ReconfigurationBackend() = default;

		[[nodiscard]] virtual std::string current_decision_config_hash() = 0;
		virtual void deactivate_current() = 0;
		virtual void cleanup_current() = 0;
		virtual void set_candidate_config() = 0;
		virtual void configure_candidate() = 0;
		virtual void verify_candidate() = 0;
		virtual void activate_candidate() = 0;
		virtual void commit_candidate() = 0;
		virtual void cleanup_candidate() = 0;
		virtual void restore_previous_config() = 0;
		virtual void configure_previous() = 0;
		virtual void verify_previous() = 0;
		virtual void activate_previous() = 0;
	};

	[[nodiscard]] const char *reconfiguration_status_name(
		ReconfigurationStatus status) noexcept;

	[[nodiscard]] const char *reconfiguration_step_name(
		ReconfigurationStep step) noexcept;

	[[nodiscard]] ReconfigurationResult execute_reconfiguration(
		const control_link_contract::ReconfigurationPlan &plan,
		std::string transaction_id,
		ReconfigurationBackend &backend);

	[[nodiscard]] std::string serialize_reconfiguration_result(
		const ReconfigurationResult &result);
}  // namespace control_link_bringup
