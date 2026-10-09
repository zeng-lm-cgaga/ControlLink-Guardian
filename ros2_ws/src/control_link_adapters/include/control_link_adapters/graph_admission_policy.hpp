#pragma once

#include <chrono>
#include <cstdint>

namespace control_link_adapters
{

enum class GraphAdmissionProbeResult
{
	kWaiting,
	kReady,
	kTimedOut,
	kStopped,
	kSuperseded,
};

// Graph 查询异常也必须服从同一个 deadline；epoch 失效时不得清理新一轮准入。
inline GraphAdmissionProbeResult graph_admission_probe_failure_result(
	std::chrono::steady_clock::time_point now,
	std::chrono::steady_clock::time_point deadline,
	bool admission_pending,
	bool epoch_matches,
	bool stopping) noexcept
{
	if (stopping) return GraphAdmissionProbeResult::kStopped;
	if (!admission_pending || !epoch_matches)
	{
		return GraphAdmissionProbeResult::kSuperseded;
	}
	if (now >= deadline) return GraphAdmissionProbeResult::kTimedOut;
	return GraphAdmissionProbeResult::kWaiting;
}

inline bool graph_admission_quiet_window_complete(
	bool expected_publisher_set,
	std::chrono::steady_clock::time_point quiet_since,
	std::chrono::steady_clock::time_point now,
	std::chrono::milliseconds quiet_window,
	std::chrono::steady_clock::time_point deadline) noexcept
{
	if (!expected_publisher_set ||
		quiet_since == std::chrono::steady_clock::time_point::min() ||
		now < quiet_since || now >= deadline)
	{
		return false;
	}
	return now - quiet_since >= quiet_window;
}

}  // namespace control_link_adapters
