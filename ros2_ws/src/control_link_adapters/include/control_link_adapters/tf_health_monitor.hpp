#pragma once

#include <chrono>
#include <cstdint>
#include <string>

#include "tf2_ros/buffer_interface.hpp"

namespace control_link_adapters
{
	enum class TfHealthState : std::uint8_t
	{
		kHealthy,
		kUnavailable,
		kInvalidTime,
		kStale,
	};

	// Ros2ControlAdapter 消费的不可变 TF 健康快照，不包含状态机或发布行为
	struct TfHealthSnapshot
	{
		TfHealthState state;
		std::int64_t transform_stamp_ns;
		std::int64_t age_ns;

		[[nodiscard]] bool healthy() const noexcept
		{
			return state == TfHealthState::kHealthy;
		}
	};

	// 只查询 target <- source 的最新 TF 并按 Robot Profile 阈值判断时效
	class TfHealthMonitor final
	{
	public:
		// buffer 的生命周期必须覆盖本对象，两个 timeout 均读取 Robot Profile
		TfHealthMonitor(
			tf2_ros::BufferInterface &buffer,
			std::string target_frame,
			std::string source_frame,
			std::uint64_t lookup_timeout_ms,
			std::uint64_t max_age_ms);

		// now_ros_ns 与 TF header stamp 使用同一 ROS clock，查询失败返回结构化状态
		[[nodiscard]] TfHealthSnapshot assess(std::int64_t now_ros_ns) const;

	private:
		tf2_ros::BufferInterface &buffer_;
		std::string target_frame_;
		std::string source_frame_;
		tf2::Duration lookup_timeout_;
		std::chrono::nanoseconds max_age_;
	};
} // namespace control_link_adapters
