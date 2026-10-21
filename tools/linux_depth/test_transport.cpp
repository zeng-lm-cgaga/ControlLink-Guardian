#include "control_link_adapters/socketcan_transport.hpp"
#include <gtest/gtest.h>
#include <algorithm>
#include <atomic>
#include <cerrno>
#include <chrono>
#include <cstring>
#include <mutex>
#include <thread>
#include <vector>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <unistd.h>

// 链接器只在测试可执行文件中注入 ENOBUFS；生产实现和 watchdog 原样编译。
namespace { std::atomic<bool> block_can_write{false}; std::atomic<int> interrupt_can_write{0}; }
extern "C" ssize_t __real_write(int, const void *, size_t);
extern "C" ssize_t __wrap_write(int fd, const void *buffer, size_t count)
{
	if (count == sizeof(can_frame) && static_cast<const can_frame *>(buffer)->can_id == 0x180)
	{
		if (interrupt_can_write.load() > 0) { interrupt_can_write.fetch_sub(1); errno = EINTR; return -1; }
		if (block_can_write.load()) { errno = ENOBUFS; return -1; }
	}
	return __real_write(fd, buffer, count);
}

namespace
{
	using namespace std::chrono_literals;
	using namespace control_link_adapters;
	SocketCanTransportConfig config() { return {"vcan0", 0x280, 5ms, 20ms}; }
	CanFrame frame(std::uint8_t value) { return {0x180, 8, false, false, {value,0,0,0,0,0,0,0}}; }

	TEST(LinuxDepth, BackpressureReevaluatesExpiredCommand)
	{
		SocketCanTransport transport(config());
		std::atomic<int> sent{-1};
		const auto started = std::chrono::steady_clock::now();
		block_can_write = true;
		transport.start([](const auto &, auto) {}, [](const auto &) {},
			[&](auto now) { return frame(now - started < 35ms ? 1 : 0); },
			[&](const CanFrame &value) { sent = value.data[0]; });
		std::this_thread::sleep_for(85ms);
		const auto blocked = transport.stats().transmit_would_block;
		block_can_write = false;
		for (int i = 0; i < 100 && sent < 0; ++i) { std::this_thread::sleep_for(1ms); }
		transport.stop();
		EXPECT_GT(blocked, 0U);
		EXPECT_EQ(sent.load(), 0) << "expired ACTIVE retry must be regenerated as HOLD";
	}

	TEST(LinuxDepth, PeriodDoesNotAccumulateCallbackDuration)
	{
		SocketCanTransport transport(config());
		std::mutex mutex; std::vector<std::chrono::steady_clock::time_point> starts;
		transport.start([](const auto &, auto) {}, [](const auto &) {}, [&](auto now) {
			{ std::lock_guard<std::mutex> lock(mutex); starts.push_back(now); }
			std::this_thread::sleep_for(7ms); return frame(0);
		});
		std::this_thread::sleep_for(310ms); transport.stop();
		ASSERT_GE(starts.size(), 8U);
		std::vector<double> intervals;
		for (std::size_t i = 1; i < starts.size(); ++i)
			intervals.push_back(std::chrono::duration<double, std::milli>(starts[i]-starts[i-1]).count());
		std::sort(intervals.begin(), intervals.end());
		std::cout << "period_median_ms=" << intervals[intervals.size()/2] << '\n';
		EXPECT_LT(intervals[intervals.size()/2], 24.0);
	}

	TEST(LinuxDepth, ContinuousReceiveAllowsTransmitAndStop)
	{
		const int fd = ::socket(PF_CAN, SOCK_RAW | SOCK_NONBLOCK | SOCK_CLOEXEC, CAN_RAW);
		ASSERT_GE(fd, 0);
		sockaddr_can address{}; address.can_family = AF_CAN; address.can_ifindex = if_nametoindex("vcan0");
		ASSERT_EQ(::bind(fd, reinterpret_cast<sockaddr *>(&address), sizeof(address)), 0);
		SocketCanTransport transport(config());
		std::atomic<bool> flood{true}, stopped{false};
		std::atomic<int> transmissions{0};
		transport.start([](const auto &, auto) { std::this_thread::sleep_for(2ms); }, [](const auto &) {},
			[](auto) { return frame(0); }, [&](const auto &) { ++transmissions; });
		std::thread producer([&] {
			can_frame state{}; state.can_id = 0x280; state.can_dlc = 8;
			while (flood) { (void)__real_write(fd, &state, sizeof(state)); }
		});
		std::this_thread::sleep_for(140ms);
		const auto tx_during_flood = transmissions.load();
		const auto stop_begin = std::chrono::steady_clock::now();
		std::thread stopper([&] { transport.stop(); stopped = true; });
		for (int i = 0; i < 150 && !stopped; ++i) { std::this_thread::sleep_for(1ms); }
		const bool bounded_stop = stopped;
		const auto stop_ms = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now()-stop_begin).count();
		flood = false; producer.join(); stopper.join(); ::close(fd);
		std::cout << "stop_ms=" << stop_ms << '\n';
		EXPECT_GE(tx_during_flood, 3);
		EXPECT_TRUE(bounded_stop);
		EXPECT_LT(stop_ms, 100.0);
#ifndef CONTROL_LINK_BASELINE_TRANSPORT
		std::cout << "rx_budget_yields=" << transport.stats().receive_budget_yields << '\n';
		EXPECT_GT(transport.stats().receive_budget_yields, 0U);
#endif
	}

	TEST(LinuxDepth, InterruptedWriteIsBoundedAndRegenerated)
	{
		SocketCanTransport transport(config());
		std::atomic<int> generations{0}, successes{0};
		interrupt_can_write = 10000;
		transport.start([](const auto &, auto) {}, [](const auto &) {},
			[&](auto) { ++generations; return frame(0); }, [&](const auto &) { ++successes; });
		std::this_thread::sleep_for(70ms);
		const int interruptions_left = interrupt_can_write.exchange(0);
		std::this_thread::sleep_for(40ms); transport.stop();
		EXPECT_GT(interruptions_left, 9900);
		EXPECT_GE(generations.load(), 3);
		EXPECT_GE(successes.load(), 1);
	}
#ifndef CONTROL_LINK_BASELINE_TRANSPORT
	TEST(LinuxDepth, ReceiveTraceCarriesRawShapeNotAssumedCounter)
	{
		SocketCanTransport transport(config());
		std::atomic<int> trace_id{0}, trace_dlc{0};
		transport.start([](const auto &, auto) {}, [](const auto &) {}, {}, {}, [&](const auto &event) {
			if (event.phase == SocketCanTracePhase::kReceive) { trace_id = event.value; trace_dlc = event.result; }
		});
		const int descriptor = ::socket(PF_CAN, SOCK_RAW | SOCK_CLOEXEC, CAN_RAW);
		ASSERT_GE(descriptor, 0);
		sockaddr_can address{}; address.can_family = AF_CAN; address.can_ifindex = if_nametoindex("vcan0");
		ASSERT_EQ(::bind(descriptor, reinterpret_cast<sockaddr *>(&address), sizeof(address)), 0);
		can_frame state{}; state.can_id = 0x280; state.can_dlc = 8; state.data[5] = 3; state.data[6] = 0xda;
		ASSERT_EQ(__real_write(descriptor, &state, sizeof(state)), static_cast<ssize_t>(sizeof(state)));
		for (int index = 0; index < 100 && trace_id == 0; ++index) { std::this_thread::sleep_for(1ms); }
		transport.stop(); ::close(descriptor);
		EXPECT_EQ(trace_id.load(), 0x280); EXPECT_EQ(trace_dlc.load(), 8);
	}
	TEST(LinuxDepth, TraceOrdersStopJoinAndClose)
	{
		std::mutex mutex;
		std::vector<SocketCanTrace> events;
		SocketCanTransport transport(config());
		transport.start([](const auto &, auto) {}, [](const auto &) {}, [](auto) { return frame(0); }, {},
			[&](const auto &event) { std::lock_guard<std::mutex> lock(mutex); events.push_back(event); });
		std::this_thread::sleep_for(35ms);
		std::vector<std::thread> owners;
		for (int index = 0; index < 8; ++index) { owners.emplace_back([&] { transport.stop(); }); }
		for (auto &owner : owners) { owner.join(); }
		std::vector<SocketCanTrace> stop;
		for (const auto &event : events)
			if (event.phase >= SocketCanTracePhase::kStop) { stop.push_back(event); }
		ASSERT_EQ(stop.size(), 4U);
		for (std::size_t index = 1; index < stop.size(); ++index) {
			EXPECT_LE(stop[index - 1].begin_ns, stop[index].begin_ns);
			EXPECT_LT(stop[index - 1].phase, stop[index].phase);
		}
	}
	TEST(LinuxDepth, SkipsMissedPeriodsWithoutBurstCatchup)
	{
		SocketCanTransport transport(config());
		std::vector<std::chrono::steady_clock::time_point> starts;
		transport.start([](const auto &, auto) {}, [](const auto &) {}, [&](auto now) {
			starts.push_back(now); std::this_thread::sleep_for(55ms); return frame(0);
		});
		std::this_thread::sleep_for(230ms); transport.stop();
		ASSERT_GE(starts.size(), 3U);
		EXPECT_GE(transport.stats().missed_transmit_periods, 6U);
		for (std::size_t index = 1; index < starts.size(); ++index) {
			EXPECT_GE(starts[index] - starts[index - 1], 55ms);
		}
	}
	TEST(LinuxDepth, RejectsZeroReceiveBudget)
	{
		auto invalid = config(); invalid.receive_frame_budget = 0;
		EXPECT_THROW(SocketCanTransport{invalid}, std::invalid_argument);
		invalid = config(); invalid.receive_time_budget = 0us;
		EXPECT_THROW(SocketCanTransport{invalid}, std::invalid_argument);
	}
	TEST(LinuxDepth, InterfaceDownEndsWorkerAndOwnerCloses)
	{
		SocketCanTransport transport(config());
		std::atomic<bool> fatal{false};
		transport.start([](const auto &, auto) {}, [&](const auto &error) { if (error.fatal) fatal = true; },
			[](auto) { return frame(0); });
		std::this_thread::sleep_for(30ms);
		const auto down = std::system("ip link set vcan0 down");
		for (int index = 0; index < 200 && !fatal; ++index) { std::this_thread::sleep_for(1ms); }
		transport.stop();
		const auto up = std::system("ip link set vcan0 up");
		EXPECT_EQ(down, 0); EXPECT_EQ(up, 0);
		EXPECT_TRUE(fatal); EXPECT_FALSE(transport.running());
	}
#endif
}
