#include <chrono>

#include <gtest/gtest.h>

#include "control_link_adapters/graph_admission_policy.hpp"

namespace
{

using control_link_adapters::GraphAdmissionProbeResult;
using control_link_adapters::graph_admission_probe_failure_result;
using control_link_adapters::graph_admission_quiet_window_complete;
using namespace std::chrono_literals;

TEST(GraphAdmissionPolicyTest, RepeatedInitialProbeExceptionsEventuallyTimeout)
{
	const auto start = std::chrono::steady_clock::time_point{};
	const auto deadline = start + 100ms;

	EXPECT_EQ(
		graph_admission_probe_failure_result(start + 50ms, deadline, true, true, false),
		GraphAdmissionProbeResult::kWaiting);
	EXPECT_EQ(
		graph_admission_probe_failure_result(start + 100ms, deadline, true, true, false),
		GraphAdmissionProbeResult::kTimedOut);
}

TEST(GraphAdmissionPolicyTest, FinalProbeExceptionUsesSameDeadline)
{
	const auto start = std::chrono::steady_clock::time_point{};
	const auto deadline = start + 100ms;

	EXPECT_EQ(
		graph_admission_probe_failure_result(start + 99ms, deadline, true, true, false),
		GraphAdmissionProbeResult::kWaiting);
	EXPECT_EQ(
		graph_admission_probe_failure_result(start + 101ms, deadline, true, true, false),
		GraphAdmissionProbeResult::kTimedOut);
}

TEST(GraphAdmissionPolicyTest, StopAndSupersededAdmissionNeverReclaimsNewEpoch)
{
	const auto now = std::chrono::steady_clock::time_point{};
	const auto deadline = now + 100ms;

	EXPECT_EQ(
		graph_admission_probe_failure_result(now, deadline, true, true, true),
		GraphAdmissionProbeResult::kStopped);
	EXPECT_EQ(
		graph_admission_probe_failure_result(now + 200ms, deadline, true, false, false),
		GraphAdmissionProbeResult::kSuperseded);
	EXPECT_EQ(
		graph_admission_probe_failure_result(now + 200ms, deadline, false, true, false),
		GraphAdmissionProbeResult::kSuperseded);
}

TEST(GraphAdmissionPolicyTest, QuietWindowAllowsNormalRecoveryBeforeDeadline)
{
	const auto start = std::chrono::steady_clock::time_point{};
	const auto deadline = start + 1s;

	EXPECT_FALSE(graph_admission_quiet_window_complete(
		true, start + 100ms, start + 200ms, 150ms, deadline));
	EXPECT_TRUE(graph_admission_quiet_window_complete(
		true, start + 100ms, start + 250ms, 150ms, deadline));
	EXPECT_FALSE(graph_admission_quiet_window_complete(
		true, start + 100ms, deadline, 150ms, deadline));
}

}  // namespace
