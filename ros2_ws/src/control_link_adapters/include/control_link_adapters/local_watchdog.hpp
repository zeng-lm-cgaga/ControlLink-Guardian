#pragma once

#include <chrono>
#include <cstdint>
#include <optional>

namespace control_link_adapters
{
	enum class LocalWatchdogState : std::uint8_t
	{
		// 当前 gateway generation 尚未提供过合法 canonical command
		kWaitingForFirstCommand,
		// 最近合法命令仍位于 profile 配置的 steady-clock timeout 内
		kHealthy,
		// 曾收到合法命令，但此后接收间隔已经超过 timeout
		kTimedOut,
	};

	// 执行 adapter 共用的本地接收 watchdog，只判断 gateway canonical output 是否沉默
	// 类本身不保证线程安全，adapter 必须串行调用或在外部同步 callback 与 output tick
	class LocalWatchdog final
	{
	public:
		// timeout_ms 必须能够表示为正的 std::chrono::milliseconds
		explicit LocalWatchdog(std::uint64_t timeout_ms);

		// adapter 重启或确认 gateway publisher generation 改变时回到 bootstrap wait
		void reset() noexcept;

		// 只能由 CanonicalInputGuard 已接受的命令调用，合法 HOLD 同样刷新存活时间
		void observe_valid_command(
			std::chrono::steady_clock::time_point received_at);

		// elapsed 等于 timeout 时仍健康，只有严格超过 timeout 才返回 kTimedOut
		[[nodiscard]] LocalWatchdogState evaluate(
			std::chrono::steady_clock::time_point now) const;

	private:
		std::chrono::milliseconds timeout_;
		std::optional<std::chrono::steady_clock::time_point>
			last_valid_received_at_;
	};
} // namespace control_link_adapters
