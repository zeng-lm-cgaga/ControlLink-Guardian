#include "control_link_gateway/decision_trace_recorder.hpp"

#include <fstream>
#include <limits>
#include <stdexcept>
#include <utility>
#include <variant>

namespace control_link_gateway
{
	namespace
	{
		bool frame_shape_is_valid(const DecisionTraceFrame &frame) noexcept
		{
			const bool output_tick = std::holds_alternative<DecisionOutputTickEvent>(
				frame.event.payload);
			return output_tick == frame.expected_result.has_value() &&
				(!frame.expected_result.has_value() ||
					frame.expected_result->event_sequence == frame.event.event_sequence);
		}
	}  // namespace

	DecisionTraceRecorder::DecisionTraceRecorder(
		std::filesystem::path output_path,
		DecisionTraceHeader header,
		std::size_t queue_capacity)
		: output_path_(std::move(output_path)),
		  header_(std::move(header)),
		  queue_capacity_(queue_capacity)
	{
		if (output_path_.empty())
		{
			throw std::invalid_argument("DecisionTraceRecorder output path must not be empty");
		}
		if (queue_capacity_ == 0U)
		{
			throw std::invalid_argument("DecisionTraceRecorder queue capacity must be positive");
		}
		writer_thread_ = std::thread(&DecisionTraceRecorder::writer_loop, this);
	}

	DecisionTraceRecorder::~DecisionTraceRecorder()
	{
		try
		{
			(void)stop();
		}
		catch (...)
		{
			// 析构路径不能传播异常，显式 stop() 的调用方仍可取得完整错误
		}
	}

	bool DecisionTraceRecorder::try_enqueue(DecisionTraceFrame frame) noexcept
	{
		try
		{
			std::lock_guard lock(mutex_);
			if (!accepting_)
			{
				return false;
			}
			if (last_accepted_event_sequence_ == std::numeric_limits<std::uint64_t>::max() ||
				frame.event.event_sequence != last_accepted_event_sequence_ + 1U ||
				!frame_shape_is_valid(frame))
			{
				mark_invalid_locked(
					"Decision Trace recorder received a non-contiguous or malformed frame",
					false,
					false);
				return false;
			}
			if (queue_.size() >= queue_capacity_)
			{
				mark_invalid_locked(
					"Decision Trace recorder queue overflow",
					true,
					false);
				condition_.notify_one();
				return false;
			}

			last_accepted_event_sequence_ = frame.event.event_sequence;
			accepted_event_count_ += 1U;
			if (frame.expected_result.has_value())
			{
				accepted_result_count_ += 1U;
			}
			queue_.push_back(std::move(frame));
			condition_.notify_one();
			return true;
		}
		catch (const std::exception &exception)
		{
			(void)exception;
			std::lock_guard lock(mutex_);
			mark_invalid_locked(
				"Decision Trace recorder enqueue failed",
				false,
				false);
			return false;
		}
		catch (...)
		{
			std::lock_guard lock(mutex_);
			mark_invalid_locked(
				"Decision Trace recorder enqueue failed with an unknown exception",
				false,
				false);
			return false;
		}
	}

	void DecisionTraceRecorder::invalidate(std::string_view message) noexcept
	{
		std::lock_guard lock(mutex_);
		mark_invalid_locked(message, false, false);
	}

	DecisionTraceRecorderStatus DecisionTraceRecorder::stop()
	{
		{
			std::lock_guard lock(mutex_);
			if (stopped_)
			{
				return status_locked();
			}
			accepting_ = false;
			stop_requested_ = true;
		}
		condition_.notify_one();
		if (writer_thread_.joinable())
		{
			writer_thread_.join();
		}
		std::lock_guard lock(mutex_);
		stopped_ = true;
		return status_locked();
	}

	DecisionTraceRecorderStatus DecisionTraceRecorder::status() const
	{
		std::lock_guard lock(mutex_);
		return status_locked();
	}

	void DecisionTraceRecorder::writer_loop() noexcept
	{
		try
		{
			std::ofstream output{output_path_, std::ios::binary | std::ios::trunc};
			if (!output)
			{
				throw std::runtime_error(
					"failed to open Decision Trace: " + output_path_.string());
			}
			output << serialize_decision_trace_record(header_) << '\n';
			if (!output)
			{
				throw std::runtime_error("failed to write Decision Trace header");
			}

			std::uint64_t written_events = 0U;
			std::uint64_t written_results = 0U;
			std::uint64_t last_written_sequence = 0U;
			for (;;)
			{
				DecisionTraceFrame frame;
				{
					std::unique_lock lock(mutex_);
					condition_.wait(lock, [this]
					{
						return stop_requested_ || !queue_.empty();
					});
					if (queue_.empty())
					{
						if (stop_requested_)
						{
							break;
						}
						continue;
					}
					frame = std::move(queue_.front());
					queue_.pop_front();
				}

				output << serialize_decision_trace_record(frame.event) << '\n';
				if (frame.expected_result.has_value())
				{
					output << serialize_decision_trace_record(frame.expected_result.value()) << '\n';
					written_results += 1U;
				}
				if (!output)
				{
					throw std::runtime_error("failed while writing Decision Trace frame");
				}
				last_written_sequence = frame.event.event_sequence;
				written_events += 1U;
			}

			DecisionTraceFooter footer;
			{
				std::lock_guard lock(mutex_);
				if (written_events != accepted_event_count_ ||
					written_results != accepted_result_count_)
				{
					mark_invalid_locked(
						"Decision Trace writer count mismatch",
						false,
						true);
				}
				footer.last_event_sequence = last_written_sequence;
				footer.event_count = written_events;
				footer.result_count = written_results;
				footer.trace_valid = trace_valid_;
				footer.trace_overflow = trace_overflow_;
				footer.error_message = error_message_;
			}
			output << serialize_decision_trace_record(footer) << '\n';
			output.close();
			if (!output)
			{
				throw std::runtime_error("failed to close Decision Trace");
			}
		}
		catch (const std::exception &exception)
		{
			std::lock_guard lock(mutex_);
			mark_invalid_locked(
				exception.what(),
				false,
				true);
		}
		catch (...)
		{
			std::lock_guard lock(mutex_);
			mark_invalid_locked(
				"Decision Trace writer failed with an unknown exception",
				false,
				true);
		}
	}

	void DecisionTraceRecorder::mark_invalid_locked(
		std::string_view message,
		bool overflow,
		bool writer_failed) noexcept
	{
		trace_valid_ = false;
		accepting_ = false;
		trace_overflow_ = trace_overflow_ || overflow;
		writer_failed_ = writer_failed_ || writer_failed;
		if (error_message_.empty())
		{
			try
			{
				error_message_.assign(message.data(), message.size());
			}
			catch (...)
			{
				// trace_valid_ 已经可靠落位，内存不足时允许诊断文本为空
			}
		}
	}

	DecisionTraceRecorderStatus DecisionTraceRecorder::status_locked() const
	{
		return DecisionTraceRecorderStatus{
			accepting_,
			stopped_,
			trace_valid_,
			trace_overflow_,
			writer_failed_,
			accepted_event_count_,
			accepted_result_count_,
			error_message_};
	}
}  // namespace control_link_gateway
