#pragma once

#ifndef CONTROL_LINK_EDGE_HAS_CONTROL_CONTEXT
#define CONTROL_LINK_EDGE_HAS_CONTROL_CONTEXT 0
#endif

#if CONTROL_LINK_EDGE_HAS_CONTROL_CONTEXT
#include <chrono>
#include <algorithm>
#include <optional>
#include <self_ipcs/channel/control_context.hpp>

namespace control_link_edge_client
{
using ControlContext = edge_runtime::ControlOperationContext;
using CloseStatus = edge_runtime::ControlCloseStatus;

// 每个串行owner保留最后一次receipt；失败事务不能在返回错误时被析构丢弃。
class ControlOperation final
{
public:
	edge_runtime::Result<void> settle(const ControlContext &context) noexcept
	{
		if (receipt_ && receipt_->has_pending_cleanup()) return receipt_->finalize(context);
		return edge_runtime::Result<void>::ok();
	}
	edge_runtime::Result<void> begin(const ControlContext &context)
	{
		auto settled = settle(context);
		if (!settled) return settled;
		auto checked = context.check();
		if (!checked) return checked;
		receipt_.emplace();
		return edge_runtime::Result<void>::ok();
	}
	edge_runtime::ControlReceipt &receipt() noexcept { return *receipt_; }
	bool pending() const noexcept { return receipt_ && receipt_->has_pending_cleanup(); }
	bool committed() const noexcept { return receipt_ && receipt_->committed; }
	edge_runtime::ControlCleanupOwner cleanup_owner() const noexcept
	{
		return receipt_ ? receipt_->cleanup_owner : edge_runtime::ControlCleanupOwner::kNone;
	}
private:
	std::optional<edge_runtime::ControlReceipt> receipt_;
};

inline ControlContext control_budget(std::chrono::nanoseconds duration,
	edge_runtime::CancellationToken cancellation = {})
{
	auto context = ControlContext::after(std::max(duration, std::chrono::nanoseconds::zero()), cancellation);
	return context ? context.value() : ControlContext(0, std::move(cancellation));
}
}
#endif
