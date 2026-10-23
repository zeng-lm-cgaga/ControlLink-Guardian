#include "control_link_edge_protocol/client.hpp"
#include "control_link_edge_protocol/edge_payload.hpp"

#include <atomic>
#include <fcntl.h>
#include <gtest/gtest.h>
#include <optional>
#include <poll.h>
#include <sys/socket.h>
#include <sys/wait.h>
#include <thread>
#include <unistd.h>

#ifdef CONTROL_LOCK_PEER
#include "edge_runtime/channel/channel_layout.hpp"
#endif

thread_local bool fail_sync_after_remove = false;
thread_local bool removed_in_test = false;
thread_local unsigned remove_count = 0;
extern "C" int __real_fdatasync(int descriptor);
extern "C" int __real_shm_unlink(const char *name);
extern "C" int __wrap_fdatasync(int descriptor)
{
	if (fail_sync_after_remove && removed_in_test) { errno = EIO; return -1; }
	return __real_fdatasync(descriptor);
}
extern "C" int __wrap_shm_unlink(const char *name)
{
	const int result = __real_shm_unlink(name);
	if (result == 0) { removed_in_test = true; ++remove_count; }
	return result;
}

namespace
{
using namespace std::chrono_literals;
using namespace control_link_edge_client;
using namespace control_link_edge_protocol;
using FeedbackProducer = edge_runtime::Producer<FeedbackFrame>;

#ifdef CONTROL_LOCK_PEER
class HeldLock final
{
public:
	explicit HeldLock(const std::string &name)
	{
		int ready[2]{-1, -1}, release[2]{-1, -1};
		if (::socketpair(AF_UNIX, SOCK_STREAM | SOCK_CLOEXEC, 0, ready) != 0) return;
		if (::socketpair(AF_UNIX, SOCK_STREAM | SOCK_CLOEXEC, 0, release) != 0)
		{ ::close(ready[0]); ::close(ready[1]); return; }
		const auto path = edge_runtime::detail::channel_lock_path(name);
		const auto ready_text = std::to_string(ready[1]);
		const auto release_text = std::to_string(release[0]);
		child_ = ::fork();
		if (child_ == 0)
		{
			::fcntl(ready[1], F_SETFD, 0);
			::fcntl(release[0], F_SETFD, 0);
			::execl(CONTROL_LOCK_PEER, CONTROL_LOCK_PEER, path.c_str(), ready_text.c_str(), release_text.c_str(), nullptr);
			::_exit(127);
		}
		::close(ready[1]); ::close(release[0]);
		release_ = release[1];
		pollfd descriptor{ready[0], POLLIN, 0};
		char response = 0;
		ready_ = ::poll(&descriptor, 1, 2000) > 0 && ::read(ready[0], &response, 1) == 1 && response == 'r';
		::close(ready[0]);
	}
	HeldLock(const HeldLock &) = delete;
	HeldLock &operator=(const HeldLock &) = delete;
	~HeldLock() { release(); }
	bool ready() const { return ready_; }
	void release()
	{
		if (release_ >= 0) { const char command = 'q'; (void)::send(release_, &command, 1, MSG_NOSIGNAL); ::close(release_); release_ = -1; }
		if (child_ <= 0) return;
		int status = 0;
		pid_t result = 0;
		const auto deadline = std::chrono::steady_clock::now() + 2s;
		while ((result = ::waitpid(child_, &status, WNOHANG)) == 0 && std::chrono::steady_clock::now() < deadline)
			std::this_thread::sleep_for(1ms);
		if (result == 0) { ::kill(child_, SIGKILL); ::waitpid(child_, &status, 0); ADD_FAILURE() << "owned official helper exceeded cleanup deadline"; }
		EXPECT_TRUE(WIFEXITED(status));
		if (WIFEXITED(status)) { EXPECT_EQ(WEXITSTATUS(status), 0); }
		child_ = -1;
	}
private:
	pid_t child_{-1};
	int release_{-1};
	bool ready_{false};
};
#endif

class ControlContextTest : public testing::TestWithParam<WaitMode>
{
protected:
	void SetUp() override
	{
		static std::atomic<unsigned> next{0};
		const auto name = "cl_context_" + std::to_string(::getpid()) + "_" + std::to_string(next++);
		options.command_channel = name + "_command";
		options.feedback_channel = name + "_feedback";
		options.wait_mode = GetParam();
		options.reconnect_period = 1ms;
		auto prepared = Client::prepare(options);
		ASSERT_TRUE(prepared);
		client.emplace(std::move(prepared.value()));
	}
	void TearDown() override
	{
		fail_sync_after_remove = false;
		if (client)
		{
			auto closed = client->close(control_budget(2s));
			EXPECT_TRUE(closed);
			if (closed) { EXPECT_EQ(closed.value(), CloseStatus::kClosed); }
			client.reset();
		}
		if (feedback) { EXPECT_TRUE(feedback->remove_if_owner()); feedback.reset(); }
	}
	void make_feedback_producer()
	{
		auto made = FeedbackProducer::create(channel_options(options.feedback_channel), edge_runtime::feedback_schema());
		ASSERT_TRUE(made) << made.error().context;
		feedback.emplace(std::move(made.value()));
		ASSERT_TRUE(enable_notifications(*feedback, GetParam()));
	}
	void publish(std::uint64_t sequence, unsigned token = 1)
	{
		FeedbackFrame frame;
		frame.session_token.bytes[0] = static_cast<std::byte>(token);
		frame.feedback_sequence = sequence;
		frame.bridge_publish_boot_ns = boottime_now_ns();
		frame.validity_flags = kSessionReady;
		frame.transport_state = TransportState::kWaitingForCommand;
		ASSERT_TRUE(feedback->publish(frame));
	}
	CommandTarget target(std::uint64_t sequence)
	{
		CommandTarget value;
		value.session = client->current_session().id;
		value.command_sequence = sequence;
		value.source_stamp_ns = system_time_now_ns();
		value.linear_velocity_mps = 0.2;
		return value;
	}
	ClientOptions options;
	std::optional<Client> client;
	std::optional<FeedbackProducer> feedback;
};

TEST_P(ControlContextTest, InitialAndMergedLatestThenIdempotentClose)
{
	make_feedback_producer();
	publish(1);
	ASSERT_TRUE(client->connect(control_budget(500ms)));
	ASSERT_TRUE(client->wait_feedback(100ms));
	for (unsigned sequence = 2; sequence <= 20; ++sequence) publish(sequence);
	auto latest = client->wait_feedback(100ms);
	ASSERT_TRUE(latest);
	EXPECT_EQ(latest.value().frame.feedback_sequence, 20U);
	EXPECT_TRUE(client->submit(target(1)));
	const auto registrations = client->status().notification_registrations;
	for (unsigned count = 0; count < 5; ++count) EXPECT_FALSE(client->try_feedback());
	EXPECT_EQ(client->status().notification_registrations, registrations);
	EXPECT_EQ(client->close(control_budget(500ms)).value(), CloseStatus::kClosed);
	EXPECT_EQ(client->close(ControlContext(0)).value(), CloseStatus::kClosed);
	EXPECT_FALSE(client->connect(control_budget(10ms)));
	EXPECT_FALSE(client->submit(target(2)));
}

TEST_P(ControlContextTest, CancelledAndExpiredBeforeCommit)
{
	edge_runtime::CancellationSource cancellation;
	ASSERT_TRUE(cancellation.request_cancel());
	auto result = client->connect(control_budget(100ms, cancellation.token()));
	ASSERT_FALSE(result);
	EXPECT_EQ(result.error().code, edge_runtime::ErrorCode::kCancelled);
	EXPECT_FALSE(client->status().last_control_committed);
	result = client->connect(ControlContext(0));
	ASSERT_FALSE(result);
	EXPECT_EQ(result.error().code, edge_runtime::ErrorCode::kTimeout);
}

TEST_P(ControlContextTest, OneDeadlineIncludesMissingFeedbackAndFreshClose)
{
	const auto before = std::chrono::steady_clock::now();
	auto result = client->connect(control_budget(40ms));
	const auto elapsed = std::chrono::steady_clock::now() - before;
	RecordProperty("connect_elapsed_monotonic_ns", std::to_string(std::chrono::duration_cast<std::chrono::nanoseconds>(elapsed).count()));
	ASSERT_FALSE(result);
	EXPECT_EQ(result.error().code, edge_runtime::ErrorCode::kTimeout);
	EXPECT_GE(elapsed, 30ms);
	EXPECT_LT(elapsed, 500ms);
	auto pending = client->close(ControlContext(0));
	EXPECT_TRUE(!pending || pending.value() == CloseStatus::kPending);
	EXPECT_TRUE(client->status().cleanup_pending);
	EXPECT_EQ(client->status().state, ClientState::kClosing);
	EXPECT_EQ(client->close(control_budget(500ms)).value(), CloseStatus::kClosed);
}

TEST_P(ControlContextTest, CommittedRemovalRetainsReceiptUntilFreshFinalize)
{
	make_feedback_producer();
	ASSERT_TRUE(client->connect(control_budget(500ms)));
	removed_in_test = false;
	remove_count = 0;
	fail_sync_after_remove = true;
	auto result = client->close(control_budget(100ms));
	EXPECT_FALSE(result);
	EXPECT_TRUE(client->status().cleanup_pending);
	EXPECT_TRUE(client->status().last_control_committed);
	EXPECT_EQ(remove_count, 1U);
	fail_sync_after_remove = false;
	EXPECT_EQ(client->close(control_budget(500ms)).value(), CloseStatus::kClosed);
	EXPECT_EQ(remove_count, 1U);
}

TEST_P(ControlContextTest, HeartbeatDoesNotRenewDefaultFeedbackFreshness)
{
	EXPECT_EQ(options.feedback_freshness, 500ms);
	make_feedback_producer();
	ASSERT_TRUE(client->connect(control_budget(500ms)));
	publish(1);
	ASSERT_TRUE(client->wait_feedback(100ms));
	for (unsigned count = 0; count < 6; ++count)
	{ std::this_thread::sleep_for(100ms); ASSERT_TRUE(feedback->heartbeat()); }
	EXPECT_FALSE(client->submit(target(1)));
	EXPECT_FALSE(client->status().session.feedback_fresh);
	publish(2);
	ASSERT_TRUE(client->wait_feedback(100ms));
	EXPECT_TRUE(client->submit(target(1)));
}

TEST_P(ControlContextTest, ReplacementRetiresOldNotificationAndRejectsOldSession)
{
	make_feedback_producer();
	ASSERT_TRUE(client->connect(control_budget(500ms)));
	publish(1);
	ASSERT_TRUE(client->wait_feedback(100ms));
	const auto old = target(1);
	ASSERT_TRUE(feedback->remove_if_owner());
	feedback.reset();
	make_feedback_producer();
	bool recovered = false;
	for (unsigned count = 0; count < 20; ++count)
	{
		publish(count + 1, 2);
		auto sample = client->wait_feedback(50ms);
		if (sample && sample.value().frame.session_token.bytes[0] == std::byte{2}) { recovered = true; break; }
		std::this_thread::sleep_for(5ms);
	}
	ASSERT_TRUE(recovered);
	EXPECT_FALSE(client->submit(old));
	EXPECT_TRUE(client->submit(target(1)));
	if (GetParam() == WaitMode::kEventFd) { EXPECT_GE(client->status().notification_registrations, 2U); }
}

TEST_P(ControlContextTest, OfficialLockContentionDeadlineAndCancellation)
{
#ifdef CONTROL_LOCK_PEER
	make_feedback_producer();
	HeldLock held(options.command_channel);
	ASSERT_TRUE(held.ready());
	const auto before = std::chrono::steady_clock::now();
	auto result = client->connect(control_budget(30ms));
	RecordProperty("lock_deadline_elapsed_monotonic_ns", std::to_string(std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::steady_clock::now() - before).count()));
	EXPECT_FALSE(result);
	if (!result) { EXPECT_EQ(result.error().code, edge_runtime::ErrorCode::kTimeout); }
	EXPECT_LT(std::chrono::steady_clock::now() - before, 500ms);
	edge_runtime::CancellationSource cancellation;
	std::thread canceller([&] { std::this_thread::sleep_for(20ms); (void)cancellation.request_cancel(); });
	result = client->connect(control_budget(2s, cancellation.token()));
	canceller.join();
	EXPECT_FALSE(result);
	if (!result) { EXPECT_EQ(result.error().code, edge_runtime::ErrorCode::kCancelled); }
	held.release();
	EXPECT_TRUE(client->connect(control_budget(500ms)));
#else
	GTEST_SKIP() << "matching read-only official SDK fixture source not configured";
#endif
}

TEST_P(ControlContextTest, OfficialFeedbackLockSharesConnectDeadline)
{
#ifdef CONTROL_LOCK_PEER
	make_feedback_producer();
	HeldLock held(options.feedback_channel);
	ASSERT_TRUE(held.ready());
	const auto before = std::chrono::steady_clock::now();
	auto result = client->connect(control_budget(40ms));
	EXPECT_FALSE(result);
	if (!result) { EXPECT_EQ(result.error().code, edge_runtime::ErrorCode::kTimeout); }
	EXPECT_LT(std::chrono::steady_clock::now() - before, 500ms);
	held.release();
	EXPECT_TRUE(client->connect(control_budget(500ms)));
#else
	GTEST_SKIP() << "matching read-only official SDK fixture source not configured";
#endif
}

TEST_P(ControlContextTest, OfficialRemovalLockKeepsClosingOwnerAndCancellation)
{
#ifdef CONTROL_LOCK_PEER
	make_feedback_producer();
	ASSERT_TRUE(client->connect(control_budget(500ms)));
	HeldLock held(options.command_channel);
	ASSERT_TRUE(held.ready());
	auto result = client->close(control_budget(30ms));
	EXPECT_TRUE(!result || result.value() == CloseStatus::kPending);
	EXPECT_TRUE(client->status().cleanup_pending);
	EXPECT_FALSE(client->heartbeat());
	edge_runtime::CancellationSource cancellation;
	std::thread canceller([&] { std::this_thread::sleep_for(20ms); (void)cancellation.request_cancel(); });
	result = client->close(control_budget(2s, cancellation.token()));
	canceller.join();
	EXPECT_TRUE(!result || result.value() == CloseStatus::kPending);
	held.release();
	EXPECT_EQ(client->close(control_budget(500ms)).value(), CloseStatus::kClosed);
#else
	GTEST_SKIP() << "matching read-only official SDK fixture source not configured";
#endif
}

INSTANTIATE_TEST_SUITE_P(Modes, ControlContextTest, testing::Values(WaitMode::kFutex, WaitMode::kEventFd));
}
