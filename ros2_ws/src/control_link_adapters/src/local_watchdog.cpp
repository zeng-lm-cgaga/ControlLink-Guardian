#include "control_link_adapters/local_watchdog.hpp"

#include <limits>
#include <stdexcept>

namespace control_link_adapters
{
	LocalWatchdog::LocalWatchdog(std::uint64_t timeout_ms)
	{
		using Milliseconds = std::chrono::milliseconds;

		const auto max_timeout_ms = static_cast<std::uint64_t>(
			std::numeric_limits<Milliseconds::rep>::max());
		if (timeout_ms == 0U || timeout_ms > max_timeout_ms)
		{
			throw std::invalid_argument(
				"LocalWatchdog timeout_ms must fit the positive milliseconds range");
		}
		timeout_ = Milliseconds{
			static_cast<Milliseconds::rep>(timeout_ms)};
	}

	void LocalWatchdog::reset() noexcept
	{
		last_valid_received_at_.reset();
	}

	void LocalWatchdog::observe_valid_command(std::chrono::steady_clock::time_point received_at)
	{
		if (last_valid_received_at_.has_value() && received_at < last_valid_received_at_.value())
		{
			throw std::logic_error("LocalWatchdog receive time moved backward");
		}
		last_valid_received_at_ = received_at;
	}

	LocalWatchdogState LocalWatchdog::evaluate(
		std::chrono::steady_clock::time_point now) const
	{
		if (!last_valid_received_at_.has_value())
			return LocalWatchdogState::kWaitingForFirstCommand;

		if (now < last_valid_received_at_.value())
		{
			throw std::logic_error(
				"LocalWatchdog evaluation time precedes last valid receive time");
		}

		if (now - last_valid_received_at_.value() > timeout_)
		{
			return LocalWatchdogState::kTimedOut;
		}

		return LocalWatchdogState::kHealthy;
	}
} // namespace control_link_adapters
