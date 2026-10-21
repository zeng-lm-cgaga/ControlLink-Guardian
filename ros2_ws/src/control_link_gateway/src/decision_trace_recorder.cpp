#include "control_link_gateway/decision_trace_recorder.hpp"

#include <fstream>
#include <limits>
#include <stdexcept>
#include <utility>
#include <variant>
#include <memory>
#include <nlohmann/json.hpp>
#include "control_link_timing/recorder.hpp"
#include <unistd.h>

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
		std::size_t queue_capacity,
		FaultCapturePolicy capture_policy)
		: output_path_(std::move(output_path)),
		  header_(std::move(header)),
		  queue_capacity_(queue_capacity), capture_policy_(capture_policy)
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
				++dropped_events_;
				return false;
			}
			if (last_accepted_event_sequence_ == std::numeric_limits<std::uint64_t>::max() ||
				frame.event.event_sequence != last_accepted_event_sequence_ + 1U ||
				!frame_shape_is_valid(frame))
			{
				++dropped_events_;
				mark_invalid_locked(
					"Decision Trace recorder received a non-contiguous or malformed frame",
					false,
					false);
				return false;
			}
			if (queue_.size() >= queue_capacity_)
			{
				++dropped_events_;
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
			queue_.push_back(QueuedFrame{std::move(frame), control_link_timing::monotonic_ns(), control_link_timing::thread_id()});
			condition_.notify_one();
			return true;
		}
		catch (const std::exception &exception)
		{
			(void)exception;
			std::lock_guard lock(mutex_);
			++dropped_events_;
			mark_invalid_locked(
				"Decision Trace recorder enqueue failed",
				false,
				false);
			return false;
		}
		catch (...)
		{
			std::lock_guard lock(mutex_);
			++dropped_events_;
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
		if (capture_policy_.enabled)
		{
			capture_loop();
			return;
		}
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
					frame = std::move(queue_.front().frame);
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

	void DecisionTraceRecorder::capture_loop() noexcept
	{
		std::unique_ptr<FaultCaptureWriter> writer;
		try
		{
			writer = std::make_unique<FaultCaptureWriter>(output_path_, serialize_decision_trace_record(header_), capture_policy_);
			std::pair<int, int> previous_fault{-1, -1};
			for (;;)
			{
				QueuedFrame item;
				{
					std::unique_lock lock(mutex_);
					condition_.wait(lock, [this] { return stop_requested_ || !queue_.empty(); });
					if (queue_.empty() && stop_requested_) break;
					item = std::move(queue_.front()); queue_.pop_front();
				}
				auto &frame = item.frame;
				nlohmann::json row{{"record_type", "observation"}, {"monotonic_ns", item.monotonic_ns}, {"pid", ::getpid()},
					{"tid", item.tid}, {"stage", "decision_event"}, {"session", nullptr}, {"source", nullptr}, {"gid", nullptr},
					{"sequence", nullptr}, {"source_stamp_ns", nullptr}, {"fault_state", nullptr}, {"fault_reason", nullptr},
					{"event", nlohmann::json::parse(serialize_decision_trace_record(frame.event))}};
				if (const auto *sample = std::get_if<DecisionSourceSampleEvent>(&frame.event.payload))
				{
					row["stage"] = "source_receive"; row["source"] = sample->command.source_id;
					row["sequence"] = sample->command.source_sequence;
					row["source_stamp_ns"] = static_cast<std::int64_t>(sample->command.source_stamp.sec) * 1000000000LL + sample->command.source_stamp.nanosec;
					std::string gid;
					constexpr char digits[] = "0123456789abcdef";
					for (const auto byte : sample->publisher_generation.publisher_gid) { gid += digits[byte >> 4]; gid += digits[byte & 15]; }
					row["gid"] = gid;
				}
				bool trigger = false;
				if (frame.expected_result)
				{
					const auto &result = *frame.expected_result;
					row["stage"] = "canonical_decision";
					row["result"] = nlohmann::json::parse(serialize_decision_trace_record(result));
					row["source"] = result.canonical_command.source_id; row["sequence"] = result.canonical_command.source_sequence;
					row["source_stamp_ns"] = static_cast<std::int64_t>(result.canonical_command.source_stamp.sec) * 1000000000LL + result.canonical_command.source_stamp.nanosec;
					row["fault_state"] = static_cast<int>(result.state); row["fault_reason"] = static_cast<int>(result.reason);
					const auto key = std::make_pair(static_cast<int>(result.state), static_cast<int>(result.reason));
					trigger = key != previous_fault && (result.state == DataState::kDegraded || result.state == DataState::kSafeStop || result.state == DataState::kError);
					previous_fault = key;
				}
				writer->append(row.dump() + '\n', item.monotonic_ns, trigger);
				std::lock_guard lock(mutex_); capture_status_ = writer->status();
			}
			writer->finish();
			std::lock_guard lock(mutex_); capture_status_ = writer->status();
		}
		catch (const std::exception &failure)
		{
			std::lock_guard lock(mutex_);
			if (writer) capture_status_ = writer->status();
			mark_invalid_locked(failure.what(), false, true);
		}
		catch (...)
		{
			std::lock_guard lock(mutex_); mark_invalid_locked("fault capture writer failed", false, true);
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
			trace_valid_ && !capture_policy_.enabled,
			trace_overflow_,
			writer_failed_,
			accepted_event_count_,
			accepted_result_count_,
			error_message_, capture_policy_.enabled, dropped_events_, capture_status_};
	}
}  // namespace control_link_gateway
