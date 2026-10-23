#include "control_link_edge_protocol/client.hpp"
#include "control_link_edge_protocol/edge_payload.hpp"

#include <atomic>
#include <filesystem>
#include <gtest/gtest.h>
#include <sys/eventfd.h>
#include <thread>
#include <unistd.h>

#if CONTROL_LINK_EDGE_HAS_NOTIFICATION
// 仅此 GTest 线程的单次提交注入 write(EIO)，不修改 SDK 或实际运行库。
thread_local bool fail_notification_write = false;
extern "C" ssize_t __real_write(int descriptor, const void *data, size_t size);
extern "C" ssize_t __wrap_write(int descriptor, const void *data, size_t size)
{
	if (fail_notification_write && size == sizeof(std::uint64_t))
	{
		errno = EIO;
		return -1;
	}
	return __real_write(descriptor, data, size);
}
#endif

namespace
{
using namespace std::chrono_literals;
using namespace control_link_edge_client;
using namespace control_link_edge_protocol;
using Producer = edge_runtime::Producer<CommandFrame>;
using Consumer = edge_runtime::Consumer<CommandFrame>;

CommandFrame command(std::uint64_t sequence)
{
	CommandFrame frame;
	frame.session_token.bytes[0] = std::byte{1};
	frame.command_sequence = sequence;
	frame.source_stamp_ns = system_time_now_ns();
	frame.linear_velocity_mps = 0.2;
	return frame;
}

class ChannelWaitTest : public testing::TestWithParam<WaitMode>
{
protected:
	void SetUp() override
	{
		static std::atomic<unsigned> next{0};
		options = channel_options("cl_wait_" + std::to_string(::getpid()) + "_" + std::to_string(next++));
		auto made = Producer::create(options, edge_runtime::command_schema());
		ASSERT_TRUE(made) << made.error().context;
		producer = std::make_unique<Producer>(std::move(made.value()));
		ASSERT_TRUE(enable_notifications(*producer, GetParam()));
		auto opened = Consumer::open(options, edge_runtime::command_schema());
		ASSERT_TRUE(opened);
		consumer = std::make_unique<Consumer>(std::move(opened.value()));
		reader = std::make_unique<ChannelWait<CommandFrame>>(GetParam());
	}
	void TearDown() override
	{
		reader.reset();
		consumer.reset();
		if (producer) (void)producer->remove_if_owner();
		producer.reset();
	}
	edge_runtime::ChannelOptions options;
	std::unique_ptr<Producer> producer;
	std::unique_ptr<Consumer> consumer;
	std::unique_ptr<ChannelWait<CommandFrame>> reader;
};

TEST_P(ChannelWaitTest, InitialSampleAndMergedLatest)
{
	ASSERT_TRUE(producer->publish(command(1)));
	ASSERT_TRUE(reader->ensure(*consumer, 20ms));
	auto first = reader->read(*consumer, 10ms);
	ASSERT_TRUE(first);
	EXPECT_EQ(first.value().value.command_sequence, 1U);
	for (std::uint64_t sequence = 2; sequence <= 100; ++sequence)
		ASSERT_TRUE(producer->publish(command(sequence)));
	auto latest = reader->read(*consumer, 20ms);
	ASSERT_TRUE(latest);
	EXPECT_EQ(latest.value().value.command_sequence, 100U);
	EXPECT_GT(latest.value().missed_samples, 0U);
}

TEST_P(ChannelWaitTest, EmptyReadKeepsOwnerAndRegistration)
{
	ASSERT_TRUE(reader->ensure(*consumer, 20ms));
	const auto registrations = reader->registrations();
	const int descriptor = reader->native_handle();
	for (unsigned count = 0; count < 12; ++count)
	{
		auto empty = reader->read(*consumer, 0ns);
		ASSERT_FALSE(empty);
		EXPECT_TRUE(empty.error().code == edge_runtime::ErrorCode::kNoNewSample ||
			empty.error().code == edge_runtime::ErrorCode::kDataStale);
		EXPECT_EQ(reader->native_handle(), descriptor);
		EXPECT_EQ(reader->registrations(), registrations);
	}
}

TEST_P(ChannelWaitTest, ProducerReplacementGetsFreshConsumerRegistration)
{
	ASSERT_TRUE(reader->ensure(*consumer, 20ms));
	ASSERT_TRUE(producer->publish(command(1)));
	auto original = reader->read(*consumer, 20ms);
	ASSERT_TRUE(original);
	ASSERT_TRUE(producer->remove_if_owner());
	producer.reset();
	reader->reset();
	EXPECT_EQ(reader->native_handle(), -1);
	consumer.reset();
	auto made = Producer::create(options, edge_runtime::command_schema());
	ASSERT_TRUE(made);
	producer = std::make_unique<Producer>(std::move(made.value()));
	ASSERT_TRUE(enable_notifications(*producer, GetParam()));
	auto opened = Consumer::open(options, edge_runtime::command_schema());
	ASSERT_TRUE(opened);
	consumer = std::make_unique<Consumer>(std::move(opened.value()));
	ASSERT_TRUE(reader->ensure(*consumer, 20ms));
	ASSERT_TRUE(producer->publish(command(2)));
	auto replacement = reader->read(*consumer, 20ms);
	ASSERT_TRUE(replacement);
	EXPECT_NE(original.value().instance_nonce, replacement.value().instance_nonce);
	EXPECT_EQ(replacement.value().value.command_sequence, 2U);
	if (GetParam() == WaitMode::kEventFd) { EXPECT_EQ(reader->registrations(), 2U); }
}

TEST_P(ChannelWaitTest, RegistrationRacingWithPublishDoesNotLoseLatest)
{
	std::atomic<bool> success{true};
	std::thread publisher([this, &success] {
		for (std::uint64_t sequence = 1; sequence <= 100; ++sequence)
			if (!producer->publish(command(sequence))) success = false;
	});
	auto registered = reader->ensure(*consumer, 20ms);
	publisher.join();
	ASSERT_TRUE(registered);
	ASSERT_TRUE(success.load());
	auto latest = reader->read(*consumer, 20ms);
	ASSERT_TRUE(latest);
	EXPECT_EQ(latest.value().value.command_sequence, 100U);
}

#if CONTROL_LINK_EDGE_HAS_NOTIFICATION
INSTANTIATE_TEST_SUITE_P(Modes, ChannelWaitTest, testing::Values(WaitMode::kFutex, WaitMode::kEventFd));

TEST(ChannelWaitLifecycle, CommittedNotificationFailureStaysLatchedAcrossNewFeedback)
{
	ClientOptions options;
	options.command_channel = "cl_notify_fail_" + std::to_string(::getpid());
	options.feedback_channel = options.command_channel + "_feedback";
	options.wait_mode = WaitMode::kEventFd;
	auto made = edge_runtime::Producer<FeedbackFrame>::create(channel_options(options.feedback_channel), edge_runtime::feedback_schema());
	ASSERT_TRUE(made);
	auto producer = std::move(made.value());
	ASSERT_TRUE(enable_notifications(producer, options.wait_mode));
	auto opened = Client::open(options);
	ASSERT_TRUE(opened);
	auto client = std::move(opened.value());
	auto command_opened = Consumer::open(channel_options(options.command_channel), edge_runtime::command_schema());
	ASSERT_TRUE(command_opened);
	auto consumer = std::move(command_opened.value());
	ChannelWait<CommandFrame> reader(WaitMode::kEventFd);
	ASSERT_TRUE(reader.ensure(consumer, 20ms));
	FeedbackFrame feedback;
	ASSERT_TRUE(generate_session_token(&feedback.session_token));
	feedback.feedback_sequence = 1;
	feedback.bridge_publish_boot_ns = boottime_now_ns();
	feedback.transport_state = TransportState::kWaitingForCommand;
	feedback.validity_flags = kSessionReady;
	ASSERT_TRUE(producer.publish(feedback));
	ASSERT_TRUE(client.wait_feedback(40ms));
	CommandTarget target;
	target.session = client.current_session().id;
	target.command_sequence = 1;
	target.source_stamp_ns = system_time_now_ns();
	fail_notification_write = true;
	auto committed = client.submit(target);
	fail_notification_write = false;
	ASSERT_TRUE(committed);
	EXPECT_EQ(client.status().command_notification_errno, EIO);
	auto last = reader.read(consumer, 0ns);
	ASSERT_TRUE(last);
	EXPECT_EQ(last.value().value.command_sequence, 1U);
	feedback.feedback_sequence = 2;
	feedback.bridge_publish_boot_ns = boottime_now_ns();
	ASSERT_TRUE(producer.publish(feedback));
	ASSERT_TRUE(client.wait_feedback(40ms));
	EXPECT_FALSE(client.current_session().ready);
	EXPECT_EQ(client.status().state, ClientState::kTransportLost);
	target.session = client.current_session().id;
	target.command_sequence = 2;
	auto rejected = client.submit(target);
	ASSERT_FALSE(rejected);
	EXPECT_EQ(rejected.error().code, edge_runtime::ErrorCode::kTransportFailed);
	EXPECT_FALSE(consumer.try_read_latest());
	reader.reset();
	EXPECT_TRUE(client.close());
	EXPECT_TRUE(producer.remove_if_owner());
}

TEST(ChannelWaitLifecycle, OfflineDrainStillChecksLastCommittedSample)
{
	auto options = channel_options("cl_final_" + std::to_string(::getpid()));
	auto made = Producer::create(options, edge_runtime::command_schema());
	ASSERT_TRUE(made);
	auto producer = std::move(made.value());
	ASSERT_TRUE(enable_notifications(producer, WaitMode::kEventFd));
	auto opened = Consumer::open(options, edge_runtime::command_schema());
	ASSERT_TRUE(opened);
	auto consumer = std::move(opened.value());
	ChannelWait<CommandFrame> reader(WaitMode::kEventFd);
	ASSERT_TRUE(reader.ensure(consumer, 20ms));
	ASSERT_TRUE(producer.publish(command(42)));
	ASSERT_TRUE(producer.disable_event_notifications());
	auto last = reader.read(consumer, 20ms);
	ASSERT_TRUE(last);
	EXPECT_EQ(last.value().value.command_sequence, 42U);
	auto closed = reader.read(consumer, 0ns);
	EXPECT_FALSE(closed);
	if (!closed) { EXPECT_TRUE(closed.error().code == edge_runtime::ErrorCode::kProducerOffline ||
		closed.error().code == edge_runtime::ErrorCode::kStaleHandle); }
	reader.reset();
	ASSERT_TRUE(producer.remove_if_owner());
}
#else
INSTANTIATE_TEST_SUITE_P(Modes, ChannelWaitTest, testing::Values(WaitMode::kFutex));
#endif

TEST(ChannelWaitLifecycle, StopFdInterruptsAbsoluteWait)
{
	const int stop = ::eventfd(0, EFD_NONBLOCK | EFD_CLOEXEC);
	ASSERT_GE(stop, 0);
	std::thread owner([stop] {
		std::this_thread::sleep_for(10ms);
		const std::uint64_t wake = 1;
		const auto count = ::write(stop, &wake, sizeof(wake));
		EXPECT_EQ(count, static_cast<ssize_t>(sizeof(wake)));
	});
	const auto start = std::chrono::steady_clock::now();
	auto result = wait_edge_fds(-1, stop, start + 2s);
	owner.join();
	::close(stop);
	ASSERT_TRUE(result);
	EXPECT_TRUE(result.value());
	EXPECT_LT(std::chrono::steady_clock::now() - start, 1s);
}

TEST(ChannelWaitLifecycle, InvalidModeIsNotSilentlyDowngraded)
{
	WaitMode mode = WaitMode::kFutex;
	EXPECT_FALSE(parse_wait_mode("driver", mode));
	EXPECT_FALSE(wait_mode_supported(static_cast<WaitMode>(19)));
}

TEST(ChannelWaitLifecycle, DefaultFeedbackFreshnessIsStill500msAndCloseIsIdempotent)
{
	ClientOptions options;
	options.command_channel = "cl_fresh_cmd_" + std::to_string(::getpid());
	options.feedback_channel = "cl_fresh_fb_" + std::to_string(::getpid());
#if CONTROL_LINK_EDGE_HAS_NOTIFICATION
	options.wait_mode = WaitMode::kEventFd;
#endif
	EXPECT_EQ(options.feedback_freshness, 500ms);
	using FeedbackProducer = edge_runtime::Producer<FeedbackFrame>;
	auto made = FeedbackProducer::create(channel_options(options.feedback_channel), edge_runtime::feedback_schema());
	ASSERT_TRUE(made);
	auto producer = std::move(made.value());
	ASSERT_TRUE(enable_notifications(producer, options.wait_mode));
	auto opened = Client::open(options);
	ASSERT_TRUE(opened);
	auto client = std::move(opened.value());
	FeedbackFrame feedback;
	ASSERT_TRUE(generate_session_token(&feedback.session_token));
	feedback.feedback_sequence = 1;
	feedback.bridge_publish_boot_ns = boottime_now_ns();
	feedback.transport_state = TransportState::kWaitingForCommand;
	feedback.validity_flags = kSessionReady;
	ASSERT_TRUE(producer.publish(feedback));
	ASSERT_TRUE(client.wait_feedback(40ms));
	for (unsigned count = 0; count < 12; ++count)
	{
		ASSERT_TRUE(producer.heartbeat());
		ASSERT_TRUE(client.heartbeat());
		std::this_thread::sleep_for(50ms);
	}
	CommandTarget target;
	target.session = client.current_session().id;
	target.command_sequence = 1;
	target.source_stamp_ns = system_time_now_ns();
	auto stale = client.submit(target);
	ASSERT_FALSE(stale);
	EXPECT_EQ(stale.error().code, edge_runtime::ErrorCode::kDataStale);
	feedback.feedback_sequence = 2;
	feedback.bridge_publish_boot_ns = boottime_now_ns();
	ASSERT_TRUE(producer.publish(feedback));
	ASSERT_TRUE(client.wait_feedback(40ms));
	target.source_stamp_ns = system_time_now_ns();
	EXPECT_TRUE(client.submit(target));
	EXPECT_TRUE(client.close());
	EXPECT_TRUE(client.close());
	EXPECT_FALSE(client.heartbeat());
	EXPECT_TRUE(producer.remove_if_owner());
}
}
