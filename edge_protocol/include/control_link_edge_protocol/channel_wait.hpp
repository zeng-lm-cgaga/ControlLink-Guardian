#pragma once

#include <algorithm>
#include <cerrno>
#include <chrono>
#include <optional>
#include <poll.h>
#include <string>

#include "edge_runtime/channel/consumer.hpp"
#include "edge_runtime/channel/producer.hpp"

namespace control_link_edge_client
{
// 这里只选择等待方式；业务 freshness、token 和 sequence 仍由原 Client/Bridge 判断。
enum class WaitMode { kFutex, kEventFd };

inline bool parse_wait_mode(const std::string &text, WaitMode &mode) noexcept
{
	if (text == "futex") { mode = WaitMode::kFutex; return true; }
	if (text == "eventfd") { mode = WaitMode::kEventFd; return true; }
	return false;
}

inline bool wait_mode_supported(WaitMode mode) noexcept
{
#if CONTROL_LINK_EDGE_HAS_NOTIFICATION
	return mode == WaitMode::kFutex || mode == WaitMode::kEventFd;
#else
	return mode == WaitMode::kFutex;
#endif
}

inline edge_runtime::ChannelOptions channel_options(const std::string &name)
{
	edge_runtime::ChannelOptions options;
	options.name = name;
#if !CONTROL_LINK_EDGE_DEFAULT_STALE_ONLY
	options.stale_timeout = std::chrono::milliseconds{500};
#endif
	options.reconnect_timeout = std::chrono::milliseconds{500};
	return options;
}

template<typename Producer>
edge_runtime::Result<void> enable_notifications(Producer &producer, WaitMode mode)
{
#if CONTROL_LINK_EDGE_HAS_NOTIFICATION
	if (mode == WaitMode::kEventFd) return producer.enable_event_notifications();
#else
	(void)producer;
#endif
	if (mode != WaitMode::kFutex)
		return edge_runtime::make_error(edge_runtime::ErrorCode::kUnsupportedPlatform,
			"enable_notifications", "eventfd requires notification-capable SDK headers and library");
	return edge_runtime::Result<void>::ok();
}

template<typename Producer>
int notification_error(const Producer &producer, WaitMode mode) noexcept
{
#if CONTROL_LINK_EDGE_HAS_NOTIFICATION
	return mode == WaitMode::kEventFd ? producer.event_notification_error() : 0;
#else
	(void)producer; (void)mode;
	return 0;
#endif
}

// native_handle 是 SDK 聚合 epoll FD；poll 不持有它，也不跨 Consumer 代次缓存。
// 返回 true 表示本地 stop 唤醒。EINTR 重算同一个 steady 绝对期限。
inline edge_runtime::Result<bool> wait_edge_fds(
	int notification_fd, int stop_fd, std::chrono::steady_clock::time_point deadline)
{
	while (true)
	{
		const auto remaining = deadline - std::chrono::steady_clock::now();
		if (remaining <= decltype(remaining)::zero()) return false;
		const auto ns = std::chrono::duration_cast<std::chrono::nanoseconds>(remaining).count();
		const timespec timeout{ns / 1'000'000'000, ns % 1'000'000'000};
		pollfd descriptors[] = {{notification_fd, POLLIN, 0}, {stop_fd, POLLIN, 0}};
		const int count = ::ppoll(descriptors, 2, &timeout, nullptr);
		if (count < 0 && errno == EINTR) continue;
		if (count < 0) return edge_runtime::make_errno_error(errno, "wait_edge_fds");
		if (descriptors[1].revents) return true;
		if (descriptors[0].revents & (POLLERR | POLLNVAL))
			return edge_runtime::make_error(edge_runtime::ErrorCode::kTransportFailed, "wait_edge_fds", "invalid notification FD");
		return false;
	}
}

// Consumer 的单 owner 必须先 reset 本对象、再关闭/替换 Consumer；不创建后台线程。
template<typename Value>
class ChannelWait final
{
public:
	explicit ChannelWait(WaitMode mode = WaitMode::kFutex) : mode_(mode) {}
	void reset() noexcept
	{
#if CONTROL_LINK_EDGE_HAS_NOTIFICATION
		notification_.reset();
		terminal_.reset();
#endif
	}
	int native_handle() const noexcept
	{
#if CONTROL_LINK_EDGE_HAS_NOTIFICATION
		return notification_ ? notification_->native_handle() : -1;
#else
		return -1;
#endif
	}
	std::uint64_t registrations() const noexcept { return registrations_; }
	std::uint64_t registration_failures() const noexcept { return registration_failures_; }
	edge_runtime::Result<void> ensure(
		edge_runtime::Consumer<Value> &consumer, std::chrono::milliseconds budget)
	{
		if (mode_ == WaitMode::kFutex) return edge_runtime::Result<void>::ok();
#if CONTROL_LINK_EDGE_HAS_NOTIFICATION
		if (notification_ || terminal_) return edge_runtime::Result<void>::ok();
		// SDK 的 0ms 实际表示 20ms，故不向它传 0，且不使用默认 1000ms。
		if (budget.count() <= 0) return edge_runtime::make_error(edge_runtime::ErrorCode::kTimeout, "notification_register", "no remaining registration budget");
		++registrations_;
		auto result = consumer.event_notification(std::min(budget, std::chrono::milliseconds{20}));
		if (!result) { ++registration_failures_; return result.error(); }
		notification_.emplace(std::move(result.value()));
		return edge_runtime::Result<void>::ok();
#else
		(void)consumer; (void)budget;
		return edge_runtime::make_error(edge_runtime::ErrorCode::kUnsupportedPlatform, "notification_register", "eventfd unsupported by selected SDK");
#endif
	}
	edge_runtime::Result<edge_runtime::Sample<Value>> read(
		edge_runtime::Consumer<Value> &consumer, std::chrono::nanoseconds timeout)
	{
		if (mode_ == WaitMode::kFutex)
			return timeout.count() > 0 ? consumer.wait_latest(timeout) : consumer.try_read_latest();
#if CONTROL_LINK_EDGE_HAS_NOTIFICATION
		if (terminal_)
		{
			const auto error = *terminal_;
			terminal_.reset();
			return error;
		}
		const auto deadline = std::chrono::steady_clock::now() + std::max(timeout, std::chrono::nanoseconds::zero());
		if (!notification_ && timeout.count() > 0)
		{
			auto registered = ensure(consumer, std::chrono::duration_cast<std::chrono::milliseconds>(deadline - std::chrono::steady_clock::now()));
			if (!registered)
			{
				auto last = consumer.try_read_latest();
				if (last) { terminal_ = registered.error(); return last; }
				return registered.error();
			}
		}
		auto result = read_notified(consumer);
		if (result || timeout.count() <= 0 || !notification_ ||
			(result.error().code != edge_runtime::ErrorCode::kNoNewSample &&
			 result.error().code != edge_runtime::ErrorCode::kDataStale)) return result;
		auto waited = wait_edge_fds(native_handle(), -1, deadline);
		if (!waited) return waited.error();
		return read_notified(consumer);
#else
		return edge_runtime::make_error(edge_runtime::ErrorCode::kUnsupportedPlatform, "notification_read", "eventfd unsupported by selected SDK");
#endif
	}
private:
#if CONTROL_LINK_EDGE_HAS_NOTIFICATION
	edge_runtime::Result<edge_runtime::Sample<Value>> read_notified(edge_runtime::Consumer<Value> &consumer)
	{
		std::optional<edge_runtime::Error> error;
		if (notification_)
		{
			auto drained = notification_->drain();
			if (!drained) { error = drained.error(); notification_.reset(); }
		}
		// 即使 offline/stale 也先取最后已提交样本；原业务校验仍不可跳过。
		auto sample = consumer.try_read_latest();
		if (sample && error) terminal_ = error;
		if (!sample && error) return *error;
		return sample;
	}
	std::optional<edge_runtime::EventNotification> notification_;
	std::optional<edge_runtime::Error> terminal_;
#endif
	WaitMode mode_;
	std::uint64_t registrations_{0};
	std::uint64_t registration_failures_{0};
};
}
