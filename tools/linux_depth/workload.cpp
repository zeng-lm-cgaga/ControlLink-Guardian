#include "control_link_timing/command_event.hpp"
#include <chrono>
#include <condition_variable>
#include <iostream>
#include <mutex>
#include <thread>
#include <time.h>
#include <unistd.h>

// 仅作已知原因对照：主动 sleep 和另一个线程持有用户 mutex，不是生产故障开关。
int main(int argc, char **argv)
{
	if (argc != 2) { std::cerr << "usage: workload ABSOLUTE.csv\n"; return 2; }
	using namespace control_link_timing;
	using namespace std::chrono_literals;
	auto recorder = open_optional(argv[1]);
	std::mutex sync, held_mutex;
	std::condition_variable condition;
	bool requested = false, held = false, finished = false;
	Identity identity; identity.set_source("controlled_workload");
	std::thread holder([&] {
		for (;;)
		{
			std::unique_lock<std::mutex> lock(sync);
			condition.wait(lock, [&] { return requested || finished; });
			if (finished) { break; }
			requested = false;
			const auto held_identity = identity;
			std::unique_lock<std::mutex> hold(held_mutex);
			held = true; const auto begin = monotonic_ns();
			condition.notify_one(); lock.unlock();
			std::this_thread::sleep_for(8ms);
			hold.unlock();
			observe(recorder, Phase::WorkHold, held_identity, begin);
		}
	});
	std::cout << "pid=" << ::getpid() << " worker_tid=" << thread_id() << '\n' << std::flush;
	auto deadline = monotonic_ns() + 1000000000ULL;
	for (std::uint64_t sequence = 1; sequence <= 100; ++sequence)
	{
		identity.sequence = sequence;
		const auto cpu_begin = monotonic_ns();
		while (monotonic_ns() - cpu_begin < 2000000ULL) { }
		observe(recorder, Phase::WorkCpu, identity, cpu_begin);
		const auto sleep_begin = monotonic_ns();
		std::this_thread::sleep_for(5ms);
		observe(recorder, Phase::WorkSleep, identity, sleep_begin);
		{
			std::unique_lock<std::mutex> lock(sync);
			held = false; requested = true; condition.notify_one();
			condition.wait(lock, [&] { return held; });
		}
		const auto lock_begin = monotonic_ns();
		{ std::unique_lock<std::mutex> lock(held_mutex); }
		observe(recorder, Phase::WorkLock, identity, lock_begin);
		deadline += 40000000ULL;
		timespec target{static_cast<time_t>(deadline / 1000000000ULL), static_cast<long>(deadline % 1000000000ULL)};
		(void)::clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &target, nullptr);
	}
	{ std::lock_guard<std::mutex> lock(sync); finished = true; condition.notify_one(); }
	holder.join(); recorder->close();
	return recorder->stats().io_errors ? 1 : 0;
}
