#pragma once

#include <algorithm>
#include <chrono>

#include "edge_runtime/common/error.hpp"

namespace control_link_adapters
{

// wait_latest 的无新样本结果只表示本次等待没有新数据，不能触发会话或连接恢复。
inline bool is_command_wait_without_new_sample(edge_runtime::ErrorCode code) noexcept
{
	return code == edge_runtime::ErrorCode::kNoNewSample ||
		code == edge_runtime::ErrorCode::kTimeout ||
		code == edge_runtime::ErrorCode::kDataStale;
}

// 单 handle owner 的一次生产等待/恢复调度；不处理业务时效或修改样本内容。
// 注入等待端与 steady now 只用于可复现测试，实际 worker 仍使用 Consumer 和 steady_clock。
template<typename Consumer, typename Now, typename OnSample, typename OnError>
void poll_edge_command_consumer(
	Consumer &consumer,
	std::chrono::steady_clock::time_point before_wait,
	std::chrono::steady_clock::time_point &next_reconnect,
	std::chrono::milliseconds &reconnect_backoff,
	Now now,
	OnSample on_sample,
	OnError on_error)
{
	if (before_wait < next_reconnect) return;
	auto sample = consumer.wait_latest(std::chrono::milliseconds{50});
	if (sample)
	{
		on_sample(sample.value());
		reconnect_backoff = std::chrono::milliseconds{100};
		next_reconnect = now();
	}
	else if (!is_command_wait_without_new_sample(sample.error().code) &&
		now() >= next_reconnect)
	{
		const auto attempt_at = now();
		if (on_error(sample.error()))
		{
			reconnect_backoff = std::chrono::milliseconds{100};
			next_reconnect = attempt_at;
		}
		else
		{
			next_reconnect = attempt_at + reconnect_backoff;
			reconnect_backoff = std::min(
				reconnect_backoff * 2, std::chrono::milliseconds{1000});
		}
	}
}

}  // namespace control_link_adapters
