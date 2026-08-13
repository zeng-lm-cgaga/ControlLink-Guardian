#pragma once

#include <atomic>
#include <chrono>
#include <cstdint>
#include <functional>
#include <mutex>
#include <optional>
#include <string>
#include <thread>

#include "control_link_adapters/can_codec.hpp"

namespace control_link_adapters
{
	struct SocketCanTransportConfig final
	{
		std::string interface;
		std::uint32_t receive_can_id;
		std::chrono::milliseconds poll_timeout;
		std::chrono::milliseconds transmit_period;
	};

	enum class SocketCanIoErrorKind : std::uint8_t
	{
		kPoll,
		kCanRead,
		kCanWrite,
		kShortCanRead,
		kShortCanWrite,
		kEventRead,
		kReceiveHandler,
		kTransmitHandler,
		kTransmitSuccessHandler,
	};

	struct SocketCanIoError final
	{
		SocketCanIoErrorKind kind;
		// 仅保存 Linux 系统调用的 errno，纯 C++ handler 异常使用 0
		int error_number;
		bool fatal;
		// handler 异常的具体文本，用于区分 Linux I/O 失败与业务回调不变量失败
		std::string detail{};
	};

	struct SocketCanTransportStats final
	{
		std::uint64_t transmitted_frames;
		std::uint64_t received_frames;
		std::uint64_t transmit_would_block;
		std::uint64_t poll_errors;
		std::uint64_t read_errors;
		std::uint64_t write_errors;
		std::uint64_t wake_errors;
		std::uint64_t receive_handler_errors;
	};

	// SocketCAN fd、eventfd 与 I/O thread 的唯一 owner
	// ROS callback 只覆盖单槽 mailbox，Linux CAN I/O 只在 worker thread 中执行
	class SocketCanTransport final
	{
	public:
		using ReceiveHandler = std::function<void(
			const CanFrame &,
			std::chrono::steady_clock::time_point)>;
		using ErrorHandler = std::function<void(const SocketCanIoError &)>;
		using TransmitHandler = std::function<std::optional<CanFrame>(
			std::chrono::steady_clock::time_point)>;
		using TransmitSuccessHandler = std::function<void(const CanFrame &)>;

		explicit SocketCanTransport(SocketCanTransportConfig config);
		~SocketCanTransport();

		SocketCanTransport(const SocketCanTransport &) = delete;
		SocketCanTransport &operator=(const SocketCanTransport &) = delete;
		SocketCanTransport(SocketCanTransport &&) = delete;
		SocketCanTransport &operator=(SocketCanTransport &&) = delete;

		// handler 在 I/O thread 中执行，只能更新线程安全快照，不能直接发布 ROS Topic
		// handler 不得调用本对象的 start/notify/request_immediate_transmit/stop 生命周期接口
		// transmit_handler 在每个固定 TX 周期生成一帧，返回空表示本周期无可发数据
		void start(
			ReceiveHandler receive_handler,
			ErrorHandler error_handler,
			TransmitHandler transmit_handler = {},
			TransmitSuccessHandler transmit_success_handler = {});

		// 只唤醒 poll，普通 command 仍在下一个固定 TX 周期由 handler 生成
		void notify() noexcept;
		// 合并多个立即请求，worker 只在实际发送前生成一帧，避免 rolling counter 空耗
		void request_immediate_transmit() noexcept;

		// 固定顺序为 request stop、eventfd wake、join、close eventfd、close CAN fd
		void stop() noexcept;

		[[nodiscard]] bool running() const noexcept;
		[[nodiscard]] SocketCanTransportStats stats() const noexcept;

	private:
		enum class TransmitResult : std::uint8_t
		{
			kSent,
			kRetry,
			kFatal,
		};

		void run() noexcept;
		[[nodiscard]] bool drain_eventfd() noexcept;
		[[nodiscard]] bool receive_available_frames() noexcept;
		[[nodiscard]] TransmitResult transmit_frame(
			const CanFrame &frame) noexcept;
		void report_error(SocketCanIoError error) noexcept;
		void wake_worker_locked() noexcept;
		void close_descriptors_locked() noexcept;

		SocketCanTransportConfig config_;
		int can_fd_{-1};
		int event_fd_{-1};
		std::thread worker_;
		ReceiveHandler receive_handler_;
		ErrorHandler error_handler_;
		TransmitHandler transmit_handler_;
		TransmitSuccessHandler transmit_success_handler_;

		mutable std::mutex lifecycle_mutex_;
		std::mutex stop_mutex_;
		std::atomic<bool> stop_requested_{false};
		std::atomic<bool> running_{false};
		bool started_{false};
		bool stopped_{false};

		std::atomic<bool> immediate_transmit_requested_{false};

		std::atomic<std::uint64_t> transmitted_frames_{0U};
		std::atomic<std::uint64_t> received_frames_{0U};
		std::atomic<std::uint64_t> transmit_would_block_{0U};
		std::atomic<std::uint64_t> poll_errors_{0U};
		std::atomic<std::uint64_t> read_errors_{0U};
		std::atomic<std::uint64_t> write_errors_{0U};
		std::atomic<std::uint64_t> wake_errors_{0U};
		std::atomic<std::uint64_t> receive_handler_errors_{0U};
	};
} // namespace control_link_adapters
