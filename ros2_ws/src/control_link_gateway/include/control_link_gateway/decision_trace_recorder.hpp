#pragma once

#include <condition_variable>
#include <cstddef>
#include <cstdint>
#include <deque>
#include <filesystem>
#include <mutex>
#include <string>
#include <string_view>
#include <thread>

#include "control_link_gateway/decision_trace.hpp"

namespace control_link_gateway
{
	struct DecisionTraceRecorderStatus
	{
		bool accepting{false};
		bool stopped{false};
		bool trace_valid{false};
		bool trace_overflow{false};
		bool writer_failed{false};
		std::uint64_t accepted_event_count{0U};
		std::uint64_t accepted_result_count{0U};
		std::string error_message;
	};

	// callback 只调用 try_enqueue()，文件的 open/write/close 全部由 writer thread 独占
	class DecisionTraceRecorder final
	{
	public:
		DecisionTraceRecorder(
			std::filesystem::path output_path,
			DecisionTraceHeader header,
			std::size_t queue_capacity);

		~DecisionTraceRecorder();

		DecisionTraceRecorder(const DecisionTraceRecorder &) = delete;
		DecisionTraceRecorder &operator=(const DecisionTraceRecorder &) = delete;
		DecisionTraceRecorder(DecisionTraceRecorder &&) = delete;
		DecisionTraceRecorder &operator=(DecisionTraceRecorder &&) = delete;

		// 返回 false 表示当前 frame 没有进入 trace，调用方不得因此改变数据面结果
		[[nodiscard]] bool try_enqueue(DecisionTraceFrame frame) noexcept;
		// live 决策无法形成完整 frame 时立即封闭 trace，已入队记录仍由 stop() drain
		void invalidate(std::string_view message) noexcept;

		// 停止接受、唤醒、drain、join，返回最终 trace 状态
		[[nodiscard]] DecisionTraceRecorderStatus stop();

		[[nodiscard]] DecisionTraceRecorderStatus status() const;

	private:
		void writer_loop() noexcept;
		void mark_invalid_locked(
			std::string_view message,
			bool overflow,
			bool writer_failed) noexcept;
		[[nodiscard]] DecisionTraceRecorderStatus status_locked() const;

		std::filesystem::path output_path_;
		DecisionTraceHeader header_;
		std::size_t queue_capacity_;
		mutable std::mutex mutex_;
		std::condition_variable condition_;
		std::deque<DecisionTraceFrame> queue_;
		std::thread writer_thread_;
		bool accepting_{true};
		bool stop_requested_{false};
		bool stopped_{false};
		bool trace_valid_{true};
		bool trace_overflow_{false};
		bool writer_failed_{false};
		std::uint64_t accepted_event_count_{0U};
		std::uint64_t accepted_result_count_{0U};
		std::uint64_t last_accepted_event_sequence_{0U};
		std::string error_message_;
	};
}  // namespace control_link_gateway
