#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include <gtest/gtest.h>

#include "control_link_bringup/reconfiguration_transaction.hpp"

namespace control_link_bringup
{
	namespace
	{
		using control_link_contract::ConfigChangeClass;
		using control_link_contract::ContractBundle;
		using control_link_contract::ReconfigurationPlan;

		enum class FailurePoint
		{
			kNone,
			kConfigureCandidate,
			kVerifyCandidate,
			kActivateCandidate,
			kConfigurePrevious,
		};

		class FakeBackend final : public ReconfigurationBackend
		{
		public:
			explicit FakeBackend(
				FailurePoint candidate_failure = FailurePoint::kNone,
				FailurePoint rollback_failure = FailurePoint::kNone)
				: candidate_failure_(candidate_failure),
				  rollback_failure_(rollback_failure)
			{
			}

			std::string current_decision_config_hash() override
			{
				return current_hash_;
			}

			void deactivate_current() override
			{
				record("deactivate_current");
				active_ = false;
			}

			void cleanup_current() override
			{
				record("cleanup_current");
				configured_ = false;
				current_hash_.clear();
			}

			void set_candidate_config() override
			{
				record("set_candidate_config");
				candidate_selected_ = true;
			}

			void configure_candidate() override
			{
				record("configure_candidate");
				fail_if(
					candidate_failure_,
					FailurePoint::kConfigureCandidate,
					"candidate configure failure");
				configured_ = true;
				current_hash_ = kCandidateHash;
			}

			void verify_candidate() override
			{
				record("verify_candidate");
				fail_if(
					candidate_failure_,
					FailurePoint::kVerifyCandidate,
					"candidate Graph failure");
			}

			void activate_candidate() override
			{
				record("activate_candidate");
				fail_if(
					candidate_failure_,
					FailurePoint::kActivateCandidate,
					"candidate activation failure");
				active_ = true;
			}

			void commit_candidate() override
			{
				record("commit_candidate");
				committed_ = true;
			}

			void cleanup_candidate() override
			{
				record("cleanup_candidate");
				active_ = false;
				configured_ = false;
				current_hash_.clear();
			}

			void restore_previous_config() override
			{
				record("restore_previous_config");
				candidate_selected_ = false;
			}

			void configure_previous() override
			{
				record("configure_previous");
				fail_if(
					rollback_failure_,
					FailurePoint::kConfigurePrevious,
					"rollback configure failure");
				configured_ = true;
				current_hash_ = kCurrentHash;
			}

			void verify_previous() override
			{
				record("verify_previous");
			}

			void activate_previous() override
			{
				record("activate_previous");
				active_ = true;
			}

			const std::vector<std::string> &calls() const noexcept
			{
				return calls_;
			}

			bool active() const noexcept
			{
				return active_;
			}

			bool committed() const noexcept
			{
				return committed_;
			}

			static constexpr const char *kCurrentHash = "current_hash";
			static constexpr const char *kCandidateHash = "candidate_hash";

		private:
			void record(std::string call)
			{
				calls_.push_back(std::move(call));
			}

			void fail_if(
				FailurePoint actual,
				FailurePoint expected,
				const char *message) const
			{
				if (actual == expected)
				{
					throw std::runtime_error(message);
				}
			}

			FailurePoint candidate_failure_;
			FailurePoint rollback_failure_;
			std::vector<std::string> calls_;
			std::string current_hash_{kCurrentHash};
			bool configured_{true};
			bool active_{true};
			bool candidate_selected_{false};
			bool committed_{false};
		};

		ReconfigurationPlan make_plan(ConfigChangeClass classification)
		{
			auto current = std::make_shared<const ContractBundle>();
			auto candidate = std::make_shared<const ContractBundle>();
			control_link_contract::ConfigDiff diff;
			diff.classification = classification;
			diff.current_identity.decision_config.decision_config_hash =
				FakeBackend::kCurrentHash;
			diff.candidate_identity.decision_config.decision_config_hash =
				FakeBackend::kCandidateHash;
			diff.reason = "test plan";
			return ReconfigurationPlan{
				std::move(current),
				std::move(candidate),
				std::move(diff)};
		}

		TEST(ReconfigurationTransaction, CommitsCandidateInStrictOrder)
		{
			FakeBackend backend;
			const auto result = execute_reconfiguration(
				make_plan(ConfigChangeClass::kEndpointRebuildRequired),
				"tx-success",
				backend);

			EXPECT_EQ(result.status, ReconfigurationStatus::kCommitted);
			EXPECT_FALSE(result.rollback_attempted);
			EXPECT_TRUE(backend.active());
			EXPECT_TRUE(backend.committed());
			EXPECT_EQ(result.final_active_decision_config_hash, "candidate_hash");
			EXPECT_EQ(
				backend.calls(),
				(std::vector<std::string>{
					"deactivate_current",
					"cleanup_current",
					"set_candidate_config",
					"configure_candidate",
					"verify_candidate",
					"activate_candidate",
					"commit_candidate"}));
		}

		TEST(ReconfigurationTransaction, RollsBackThreeCandidateFailureBoundaries)
		{
			for (const auto failure : {
				FailurePoint::kConfigureCandidate,
				FailurePoint::kVerifyCandidate,
				FailurePoint::kActivateCandidate})
			{
				FakeBackend backend{failure};
				const auto result = execute_reconfiguration(
					make_plan(ConfigChangeClass::kEndpointRebuildRequired),
					"tx-rollback",
					backend);

				EXPECT_EQ(result.status, ReconfigurationStatus::kRolledBack);
				EXPECT_TRUE(result.rollback_attempted);
				EXPECT_TRUE(result.rollback_succeeded);
				EXPECT_TRUE(backend.active());
				EXPECT_EQ(result.final_active_decision_config_hash, "current_hash");
				EXPECT_EQ(backend.calls().back(), "activate_previous");
			}
		}

		TEST(ReconfigurationTransaction, LeavesCandidateNonActiveWhenRollbackFails)
		{
			FakeBackend backend{
				FailurePoint::kVerifyCandidate,
				FailurePoint::kConfigurePrevious};
			const auto result = execute_reconfiguration(
				make_plan(ConfigChangeClass::kEndpointRebuildRequired),
				"tx-rollback-failed",
				backend);

			EXPECT_EQ(result.status, ReconfigurationStatus::kRollbackFailed);
			EXPECT_TRUE(result.rollback_attempted);
			EXPECT_FALSE(result.rollback_succeeded);
			EXPECT_EQ(result.rollback_failed_step, "CONFIGURE_PREVIOUS");
			EXPECT_FALSE(backend.active());
			EXPECT_FALSE(backend.committed());
		}

		TEST(ReconfigurationTransaction, NoOpAndRestartRequiredNeverQuiesceGateway)
		{
			FakeBackend no_op_backend;
			const auto no_op = execute_reconfiguration(
				make_plan(ConfigChangeClass::kIdentityOnly),
				"tx-no-op",
				no_op_backend);
			EXPECT_EQ(no_op.status, ReconfigurationStatus::kNoOp);
			EXPECT_TRUE(no_op_backend.calls().empty());

			FakeBackend restart_backend;
			const auto restart = execute_reconfiguration(
				make_plan(ConfigChangeClass::kProfileRestartRequired),
				"tx-restart",
				restart_backend);
			EXPECT_EQ(restart.status, ReconfigurationStatus::kRejected);
			EXPECT_TRUE(restart_backend.calls().empty());
		}
	}  // namespace
}  // namespace control_link_bringup
