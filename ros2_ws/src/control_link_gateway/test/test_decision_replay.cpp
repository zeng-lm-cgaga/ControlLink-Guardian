#include <algorithm>
#include <filesystem>
#include <string>
#include <utility>
#include <variant>

#include <gtest/gtest.h>

#include "control_link_contract/contract_bundle.hpp"
#include "control_link_gateway/decision_replay.hpp"

namespace control_link_gateway
{
	namespace
	{
		constexpr std::int64_t kNanosecondsPerMillisecond = 1'000'000LL;

		control_link_contract::ContractBundlePtr load_robot_bundle()
		{
			return control_link_contract::load_contract_bundle(
				std::filesystem::path{CONTROL_LINK_TEST_ROBOT_PROFILE_PATH},
				std::filesystem::path{CONTROL_LINK_TEST_CONFIG_ROOT});
		}

		DecisionTraceDocument load_fixture()
		{
			return read_decision_trace_file(
				std::filesystem::path{CONTROL_LINK_TEST_DECISION_TRACE_PATH});
		}

		DecisionTraceFrame &first_output_tick(DecisionTraceDocument &trace)
		{
			const auto found = std::find_if(
				trace.frames.begin(),
				trace.frames.end(),
				[](const auto &frame)
				{
					return std::holds_alternative<DecisionOutputTickEvent>(
						frame.event.payload);
				});
			if (found == trace.frames.end())
			{
				throw std::logic_error("test trace has no output tick");
			}
			return *found;
		}

		TEST(DecisionReplay, ReplaysTheFixedFixtureOneHundredTimes)
		{
			const auto bundle = load_robot_bundle();
			const auto trace = load_fixture();
			const auto report = replay_decision_trace(bundle, trace, 100U);

			EXPECT_TRUE(report.valid);
			EXPECT_TRUE(report.matched);
			EXPECT_EQ(report.completed_repetitions, 100U);
			EXPECT_EQ(report.requested_repetitions, 100U);
			EXPECT_EQ(report.event_count, 17U);
			EXPECT_EQ(report.result_count, 6U);
			EXPECT_FALSE(report.first_difference_event_sequence.has_value());
		}

		TEST(DecisionReplay, LocatesTheFirstDifferenceAfterLogicalTimeChanges)
		{
			const auto bundle = load_robot_bundle();
			auto trace = load_fixture();
			auto &tick = first_output_tick(trace);
			auto &payload = std::get<DecisionOutputTickEvent>(tick.event.payload);
			payload.now_ros_ns += 200 * kNanosecondsPerMillisecond;

			const auto report = replay_decision_trace(bundle, trace, 100U);
			EXPECT_TRUE(report.valid);
			EXPECT_FALSE(report.matched);
			ASSERT_TRUE(report.first_difference_event_sequence.has_value());
			EXPECT_EQ(
				report.first_difference_event_sequence.value(),
				tick.event.event_sequence);
			EXPECT_NE(report.first_difference_field, "none");
			EXPECT_EQ(report.completed_repetitions, 0U);
		}

		TEST(DecisionReplay, RejectsHealthRevisionAndEventOrderCorruption)
		{
			const auto bundle = load_robot_bundle();
			auto stale_revision = load_fixture();
			auto &tick = first_output_tick(stale_revision);
			std::get<DecisionOutputTickEvent>(tick.event.payload).health_revision += 1U;
			try
			{
				(void)replay_decision_trace(bundle, stale_revision, 1U);
				FAIL() << "expected health revision failure";
			}
			catch (const DecisionReplayError &error)
			{
				EXPECT_EQ(error.code(), DecisionReplayErrorCode::kInvalidEvent);
				EXPECT_EQ(error.event_sequence(), tick.event.event_sequence);
			}

			auto wrong_order = load_fixture();
			std::swap(wrong_order.frames.at(2U), wrong_order.frames.at(3U));
			EXPECT_THROW(
				(void)replay_decision_trace(bundle, wrong_order, 1U),
				DecisionReplayError);
		}

		TEST(DecisionReplay, RejectsConfigIdentityInvalidFooterAndBadRepeatCount)
		{
			const auto bundle = load_robot_bundle();
			auto hash_mismatch = load_fixture();
			hash_mismatch.header.decision_config_hash.front() =
				hash_mismatch.header.decision_config_hash.front() == '0' ? '1' : '0';
			try
			{
				(void)replay_decision_trace(bundle, hash_mismatch, 1U);
				FAIL() << "expected config identity mismatch";
			}
			catch (const DecisionReplayError &error)
			{
				EXPECT_EQ(error.code(), DecisionReplayErrorCode::kConfigIdentityMismatch);
			}

			auto overflow = load_fixture();
			overflow.footer.trace_valid = false;
			overflow.footer.trace_overflow = true;
			overflow.footer.error_message = "queue overflow";
			EXPECT_THROW(
				(void)replay_decision_trace(bundle, overflow, 1U),
				DecisionReplayError);
			EXPECT_THROW(
				(void)replay_decision_trace(bundle, load_fixture(), 0U),
				DecisionReplayError);
		}

		TEST(DecisionReplay, KeepsRejectedSampleOutOfTheLatestValidSnapshot)
		{
			const auto trace = load_fixture();
			const auto last_result = std::find_if(
				trace.frames.rbegin(),
				trace.frames.rend(),
				[](const auto &frame)
				{
					return frame.expected_result.has_value();
				});
			ASSERT_NE(last_result, trace.frames.rend());
			const auto nav2 = std::find_if(
				last_result->expected_result->sources.begin(),
				last_result->expected_result->sources.end(),
				[](const auto &source)
				{
					return source.source_id == "nav2";
				});
			ASSERT_NE(nav2, last_result->expected_result->sources.end());
			EXPECT_EQ(nav2->accepted_count, 5U);
			EXPECT_EQ(nav2->rejected_count, 1U);
			EXPECT_EQ(nav2->last_reject_reason, RejectReason::kSequenceNotIncreasing);
			ASSERT_TRUE(nav2->last_accepted_sequence.has_value());
			EXPECT_EQ(nav2->last_accepted_sequence.value(), 5U);
			EXPECT_EQ(last_result->expected_result->canonical_command.source_sequence, 5U);
		}
	}  // namespace
}  // namespace control_link_gateway
