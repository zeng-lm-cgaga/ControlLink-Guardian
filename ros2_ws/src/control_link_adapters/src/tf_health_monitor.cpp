#include "control_link_adapters/tf_health_monitor.hpp"

#include <limits>
#include <optional>
#include <stdexcept>
#include <utility>

#include "tf2/exceptions.hpp"
#include "tf2/time.hpp"

namespace control_link_adapters
{
	namespace
	{
		constexpr std::uint64_t kNanosecondsPerMillisecond = 1'000'000ULL;
		constexpr std::int64_t kNanosecondsPerSecond = 1'000'000'000LL;

		std::chrono::nanoseconds checked_nanoseconds(
			std::uint64_t milliseconds,
			const char *field_name)
		{
			const auto maximum = static_cast<std::uint64_t>(
				std::numeric_limits<std::chrono::nanoseconds::rep>::max());
			if (milliseconds == 0U ||
				milliseconds > maximum / kNanosecondsPerMillisecond)
			{
				throw std::invalid_argument(
					std::string(field_name) +
					" must fit a positive nanoseconds duration");
			}

			return std::chrono::nanoseconds{
				static_cast<std::chrono::nanoseconds::rep>(
					milliseconds * kNanosecondsPerMillisecond)};
		}

		std::optional<std::int64_t> stamp_nanoseconds(
			const builtin_interfaces::msg::Time &stamp) noexcept
		{
			if (stamp.sec < 0 ||
				stamp.nanosec >= static_cast<std::uint32_t>(kNanosecondsPerSecond))
			{
				return std::nullopt;
			}

			return static_cast<std::int64_t>(stamp.sec) * kNanosecondsPerSecond +
				static_cast<std::int64_t>(stamp.nanosec);
		}
	} // namespace

	TfHealthMonitor::TfHealthMonitor(
		tf2_ros::BufferInterface &buffer,
		std::string target_frame,
		std::string source_frame,
		std::uint64_t lookup_timeout_ms,
		std::uint64_t max_age_ms)
		: buffer_(buffer),
		  target_frame_(std::move(target_frame)),
		  source_frame_(std::move(source_frame)),
		  lookup_timeout_(checked_nanoseconds(
			  lookup_timeout_ms,
			  "tf_lookup_timeout_ms")),
		  max_age_(checked_nanoseconds(max_age_ms, "tf_max_age_ms"))
	{
		if (target_frame_.empty() || source_frame_.empty())
		{
			throw std::invalid_argument(
				"TfHealthMonitor requires non-empty target and source frames");
		}
		if (target_frame_ == source_frame_)
		{
			throw std::invalid_argument(
				"TfHealthMonitor target and source frames must differ");
		}
	}

	TfHealthSnapshot TfHealthMonitor::assess(std::int64_t now_ros_ns) const
	{
		if (now_ros_ns <= 0)
		{
			return TfHealthSnapshot{
				TfHealthState::kInvalidTime,
				0,
				0};
		}

		geometry_msgs::msg::TransformStamped transform;
		try
		{
			// TimePointZero 请求最新可用变换，等待时间只由 Robot Profile 控制
			transform = buffer_.lookupTransform(
				target_frame_,
				source_frame_,
				tf2::TimePointZero,
				lookup_timeout_);
		}
		catch (const tf2::TransformException &)
		{
			return TfHealthSnapshot{
				TfHealthState::kUnavailable,
				0,
				0};
		}

		const auto transform_stamp_ns = stamp_nanoseconds(transform.header.stamp);
		if (!transform_stamp_ns.has_value() ||
			transform_stamp_ns.value() <= 0 ||
			transform_stamp_ns.value() > now_ros_ns)
		{
			return TfHealthSnapshot{
				TfHealthState::kInvalidTime,
				transform_stamp_ns.value_or(0),
				0};
		}

		const auto age_ns = now_ros_ns - transform_stamp_ns.value();
		if (age_ns > max_age_.count())
		{
			return TfHealthSnapshot{
				TfHealthState::kStale,
				transform_stamp_ns.value(),
				age_ns};
		}

		return TfHealthSnapshot{
			TfHealthState::kHealthy,
			transform_stamp_ns.value(),
			age_ns};
	}
} // namespace control_link_adapters
