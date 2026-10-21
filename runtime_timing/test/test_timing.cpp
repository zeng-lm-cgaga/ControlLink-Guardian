#include "control_link_timing/recorder.hpp"
#include <filesystem>
#include <fstream>
#include <iostream>
#include <stdexcept>
#include <vector>
#include <cerrno>
#include <unistd.h>

namespace { std::atomic<int> fail_sync{0}, fail_close{0}; }
extern "C" int __real_fsync(int);
extern "C" int __real_close(int);
extern "C" int __wrap_fsync(int descriptor)
{
	if (fail_sync.load() > 0 && fail_sync.fetch_sub(1) == 1) { errno = EIO; return -1; }
	return __real_fsync(descriptor);
}
extern "C" int __wrap_close(int descriptor)
{
	const auto result = __real_close(descriptor);
	if (fail_close.load() > 0 && fail_close.fetch_sub(1) == 1) { errno = EIO; return -1; }
	return result;
}

void require(bool value, const char *reason) { if (!value) { throw std::runtime_error(reason); } }
int main()
{
	using namespace control_link_timing;
	const auto mono_before = monotonic_ns();
	const auto steady = std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::steady_clock::now().time_since_epoch()).count();
	const auto mono_after = monotonic_ns();
	require(steady >= static_cast<std::int64_t>(mono_before) && steady <= static_cast<std::int64_t>(mono_after),
		"Linux std::chrono steady clock must share timerfd MONOTONIC epoch");
	const auto path = "/tmp/control-link-timing-test-" + std::to_string(::getpid()) + ".csv";
	Recorder recorder(path);
	bool duplicate_failed = false;
	try { Recorder duplicate(path); } catch (const std::system_error &) { duplicate_failed = true; }
	require(duplicate_failed, "exclusive evidence creation");
	std::vector<std::thread> producers;
	for (int p = 0; p < 4; ++p)
	{
		producers.emplace_back([&recorder, p] {
			for (int i = 0; i < 15000; ++i)
			{
				Event event; event.identity.set_source("source,with,commas");
				event.identity.sequence = p * 15000 + i;
				event.begin_ns = monotonic_ns(); event.end_ns = monotonic_ns();
				recorder.record(event);
			}
		});
	}
	for (auto &producer : producers) { producer.join(); }
	Event invalid; invalid.identity.set_source(std::string(100, 'x'));
	require(!recorder.record(invalid), "oversize identity must be counted, not truncated");
	recorder.close(); recorder.close();
	const auto stats = recorder.stats();
	require(stats.written == stats.queued && stats.io_errors == 0, "drain and I/O");
	require(stats.queued + stats.full + stats.contended == 60000, "exact accounting");
	require(stats.invalid_identity == 1 && stats.queued > 0, "invalid identity accounting");
	require(!recorder.record(Event{}), "record after close");
	std::ifstream input(path); std::string line; std::size_t rows = 0; bool footer = false;
	while (std::getline(input, line)) { if (line.rfind("source_receive,", 0) == 0) { ++rows; } if (line.rfind("# drain", 0) == 0) { footer = true; } }
	require(rows == stats.written && footer, "complete immutable CSV");
	require(std::filesystem::exists(path + ".status.json"), "finalized companion status");
	for (const auto &failure : {std::string("sync"), std::string("close"), std::string("status_close")})
	{
		const auto failed_path = path + "." + failure;
		Recorder failed(failed_path);
		failed.record(Event{});
		if (failure == "sync") { fail_sync = 1; }
		else { fail_close = failure == "close" ? 1 : 2; }
		failed.close();
		require(failed.stats().io_errors != 0, "finalization errors must remain visible");
		if (failure == "status_close")
		{
			require(!std::filesystem::exists(failed_path + ".status.json"), "no complete status after its own close failed");
		}
		else
		{
			std::ifstream status(failed_path + ".status.json");
			std::getline(status, line);
			require(line.find("\"io_errors\":1") != std::string::npos, "data close/sync error in final status");
		}
		std::filesystem::remove(failed_path);
		std::filesystem::remove(failed_path + ".status.json");
		std::filesystem::remove(failed_path + ".status.partial");
	}
	std::cout << "PASS concurrent_attempts=60000 written=" << stats.written
		<< " full=" << stats.full << " contended=" << stats.contended << '\n';
	std::filesystem::remove(path);
	std::filesystem::remove(path + ".status.json");
}
