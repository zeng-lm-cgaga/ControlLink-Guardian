#include <gtest/gtest.h>

#include <chrono>
#include <cstdint>
#include <deque>
#include <stdexcept>
#include <utility>
#include <vector>

#include "control_link_adapters/edge_command_wait_policy.hpp"
#include "edge_runtime/common/result.hpp"

namespace
{

using control_link_adapters::is_command_wait_without_new_sample;
using edge_runtime::ErrorCode;
using namespace std::chrono_literals;
using SteadyClock = std::chrono::steady_clock;

struct ObservedCommand
{
	std::uint64_t sequence;
	std::int64_t source_stamp_ns;
	std::uint64_t session;
};

// 夹具仅控制 Consumer 的外部等待结果和 steady 时间，不复制生产调度分支。
struct ControlledConsumer
{
	struct Response
	{
		std::chrono::milliseconds elapsed;
		edge_runtime::Result<ObservedCommand> result;
	};
	SteadyClock::time_point &clock;
	std::deque<Response> responses;
	std::vector<SteadyClock::time_point> wait_started;
	std::vector<std::chrono::nanoseconds> wait_timeouts;

	edge_runtime::Result<ObservedCommand> wait_latest(std::chrono::nanoseconds timeout)
	{
		if (responses.empty()) throw std::logic_error("unexpected consumer wait");
		wait_started.push_back(clock);
		wait_timeouts.push_back(timeout);
		auto response = std::move(responses.front());
		responses.pop_front();
		clock += response.elapsed;
		return std::move(response.result);
	}
};

class CommandWorkerWaitTest : public ::testing::Test
{
protected:
	SteadyClock::time_point clock{};
	SteadyClock::time_point next_reconnect{};
	std::chrono::milliseconds backoff{100};
	ControlledConsumer consumer{clock, {}, {}, {}};
	std::vector<ObservedCommand> forwarded;
	SteadyClock::time_point last_forward{};
	std::uint64_t error_calls{0};
	bool recovered{false};
	std::chrono::milliseconds sample_processing_elapsed{0};
	std::chrono::milliseconds recovery_processing_elapsed{0};

	void poll()
	{
		control_link_adapters::poll_edge_command_consumer(
			consumer, clock, next_reconnect, backoff,
			[this] { return clock; },
			[this](const ObservedCommand &sample)
			{
				forwarded.push_back(sample);
				clock += sample_processing_elapsed;
				last_forward = clock;
			},
			[this](const edge_runtime::Error &)
			{
				++error_calls;
				clock += recovery_processing_elapsed;
				return recovered;
			});
	}
};

TEST(EdgeCommandWaitPolicyTest, DataStaleDoesNotStartRecoveryBackoff)
{
	EXPECT_TRUE(is_command_wait_without_new_sample(ErrorCode::kNoNewSample));
	EXPECT_TRUE(is_command_wait_without_new_sample(ErrorCode::kTimeout));
	EXPECT_TRUE(is_command_wait_without_new_sample(ErrorCode::kDataStale));
}

TEST(EdgeCommandWaitPolicyTest, TransportFailuresRemainRecoveryInputs)
{
	EXPECT_FALSE(is_command_wait_without_new_sample(ErrorCode::kProducerOffline));
	EXPECT_FALSE(is_command_wait_without_new_sample(ErrorCode::kRecoveryBlocked));
	EXPECT_FALSE(is_command_wait_without_new_sample(ErrorCode::kStaleHandle));
	EXPECT_FALSE(is_command_wait_without_new_sample(ErrorCode::kTransportFailed));
	EXPECT_FALSE(is_command_wait_without_new_sample(ErrorCode::kPayloadCorrupt));
}

TEST_F(CommandWorkerWaitTest, FiftyMillisecondEmptyWaitReadsNextSampleWithoutRecoveryDelay)
{
	consumer.responses.push_back({0ms, ObservedCommand{1U, 12345000, 47U}});
	poll();
	backoff = 800ms;
	consumer.responses.push_back({50ms, {ErrorCode::kDataStale, "controlled_wait"}});
	poll();
	ASSERT_EQ(forwarded.size(), 1U);
	EXPECT_EQ(forwarded.front().source_stamp_ns, 12345000);
	EXPECT_EQ(last_forward, SteadyClock::time_point{});
	EXPECT_EQ(next_reconnect, SteadyClock::time_point{});
	EXPECT_EQ(backoff, 800ms);
	EXPECT_EQ(error_calls, 0U);

	consumer.responses.push_back({1ms, ObservedCommand{2U, 12346000, 47U}});
	poll();
	ASSERT_EQ(forwarded.size(), 2U);
	ASSERT_EQ(consumer.wait_started.size(), 3U);
	EXPECT_EQ(consumer.wait_started.back(), SteadyClock::time_point{} + 50ms);
	EXPECT_EQ(clock, SteadyClock::time_point{} + 51ms);
	EXPECT_EQ(forwarded.back().sequence, 2U);
	EXPECT_EQ(forwarded.back().source_stamp_ns, 12346000);
	EXPECT_EQ(forwarded.back().session, forwarded.front().session);
	EXPECT_EQ(error_calls, 0U);
	for (const auto timeout : consumer.wait_timeouts) EXPECT_EQ(timeout, 50ms);
	EXPECT_EQ(backoff, 100ms);
}

TEST_F(CommandWorkerWaitTest, HeartbeatOnlyEmptyWaitsDoNotReplayOrRefreshCommand)
{
	consumer.responses.push_back({0ms, ObservedCommand{9U, 7000000, 42U}});
	poll();
	for (unsigned int index = 0U; index < 6U; ++index)
	{
		consumer.responses.push_back({50ms, {ErrorCode::kDataStale, "heartbeat_alive"}});
		poll();
	}
	EXPECT_EQ(clock, SteadyClock::time_point{} + 300ms);
	ASSERT_EQ(forwarded.size(), 1U);
	EXPECT_EQ(forwarded.front().sequence, 9U);
	EXPECT_EQ(forwarded.front().source_stamp_ns, 7000000);
	EXPECT_EQ(forwarded.front().session, 42U);
	EXPECT_EQ(last_forward, SteadyClock::time_point{});
	EXPECT_EQ(error_calls, 0U);
	EXPECT_EQ(consumer.wait_started.size(), 7U);
}

TEST_F(CommandWorkerWaitTest, OfflineAndRecoveryBlockedKeepBoundedBackoff)
{
	consumer.responses.push_back({0ms, {ErrorCode::kProducerOffline, "offline"}});
	poll();
	EXPECT_EQ(next_reconnect, SteadyClock::time_point{} + 100ms);
	EXPECT_EQ(backoff, 200ms);
	consumer.responses.push_back({0ms, {ErrorCode::kRecoveryBlocked, "blocked"}});
	clock += 99ms;
	poll();
	EXPECT_EQ(consumer.wait_started.size(), 1U);
	clock += 1ms;
	poll();
	EXPECT_EQ(next_reconnect, SteadyClock::time_point{} + 300ms);
	EXPECT_EQ(backoff, 400ms);
	EXPECT_EQ(error_calls, 2U);
	EXPECT_TRUE(forwarded.empty());
}

TEST_F(CommandWorkerWaitTest, SuccessfulReconnectAllowsNextWaitWithoutExtraDelay)
{
	recovered = true;
	backoff = 800ms;
	consumer.responses.push_back({0ms, {ErrorCode::kStaleHandle, "reconnect"}});
	poll();
	EXPECT_EQ(next_reconnect, clock);
	EXPECT_EQ(backoff, 100ms);
	consumer.responses.push_back({1ms, ObservedCommand{1U, 9000000, 88U}});
	poll();
	ASSERT_EQ(forwarded.size(), 1U);
	EXPECT_EQ(clock, SteadyClock::time_point{} + 1ms);
	EXPECT_EQ(error_calls, 1U);
}

TEST_F(CommandWorkerWaitTest, FailedRecoveryDeadlineUsesTimeBeforeRecoveryCallback)
{
	recovery_processing_elapsed = 40ms;
	consumer.responses.push_back({10ms, {ErrorCode::kProducerOffline, "offline"}});
	poll();
	EXPECT_EQ(clock, SteadyClock::time_point{} + 50ms);
	EXPECT_EQ(next_reconnect, SteadyClock::time_point{} + 110ms);
	EXPECT_EQ(backoff, 200ms);
}

TEST_F(CommandWorkerWaitTest, SuccessfulSampleDeadlineUsesTimeAfterSampleCallback)
{
	sample_processing_elapsed = 7ms;
	consumer.responses.push_back({1ms, ObservedCommand{1U, 9000000, 88U}});
	poll();
	EXPECT_EQ(clock, SteadyClock::time_point{} + 8ms);
	EXPECT_EQ(next_reconnect, clock);
	EXPECT_EQ(last_forward, clock);
}

}  // namespace
