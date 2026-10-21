#pragma once
#include "control_link_timing/recorder.hpp"
#include <algorithm>

namespace control_link_timing
{
	// 模板仅读取现有消息字段，不依赖 ROS，也不新增或修复业务字段。
	template<class Command>
	Identity command_identity(const Command &command, const std::uint8_t *gid = nullptr) noexcept
	{
		Identity identity;
		identity.set_source(command.source_id);
		identity.sequence = command.source_sequence;
		identity.source_stamp_ns = static_cast<std::int64_t>(command.source_stamp.sec) * 1000000000LL + command.source_stamp.nanosec;
		if (gid != nullptr) { std::copy_n(gid, identity.publisher_gid.size(), identity.publisher_gid.begin()); }
		return identity;
	}

	inline void observe(const std::shared_ptr<Recorder> &recorder, Phase phase,
		const Identity &identity, std::uint64_t begin, std::int64_t result = 0,
		std::int64_t value = 0, std::uint64_t frame_id = 0) noexcept
	{
		if (!recorder) { return; }
		Event event;
		event.phase = phase; event.identity = identity;
		event.begin_ns = begin; event.end_ns = monotonic_ns();
		event.result = result; event.value = value; event.frame_id = frame_id;
		recorder->record(event);
	}
}
