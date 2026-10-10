#pragma once

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

}  // namespace control_link_adapters
