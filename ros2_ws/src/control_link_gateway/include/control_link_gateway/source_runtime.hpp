#pragma once

#include <map>
#include <string>

#include "control_link_gateway/model.hpp"
#include "control_link_gateway/command_validator.hpp"

namespace control_link_gateway
{
	// accepted 结果原子式提交 latest-valid snapshot、sequence 与计数，rejected 结果只记录拒绝信息
	void commit_validation_result(
		SourceRuntimeSlot &slot,
		const CommandValidationResult &result);

	// 为 output tick 复制一致的有效单槽集合，freshness 与 lease 仍由 SourceArbiter 判断
	[[nodiscard]] std::map<std::string, SourceSnapshot>
	collect_latest_valid_snapshots(
		const std::map<std::string, SourceRuntimeSlot> &slots);
} // namespace control_link_gateway
