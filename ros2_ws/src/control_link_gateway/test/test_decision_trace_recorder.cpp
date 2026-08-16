#include <cerrno>
#include <cstring>
#include <filesystem>
#include <fcntl.h>
#include <stdexcept>
#include <string>
#include <system_error>
#include <unistd.h>

#include <gtest/gtest.h>
#include <sys/stat.h>

#include "control_link_gateway/decision_trace_recorder.hpp"

namespace control_link_gateway
{
	namespace
	{
		class TemporaryPath final
		{
		public:
			explicit TemporaryPath(std::string suffix)
				: path_(std::filesystem::temp_directory_path() /
					("control_link_trace_" + std::to_string(::getpid()) + std::move(suffix)))
			{
				std::error_code error;
				std::filesystem::remove(path_, error);
			}

			~TemporaryPath()
			{
				std::error_code error;
				std::filesystem::remove(path_, error);
			}

			[[nodiscard]] const std::filesystem::path &path() const noexcept
			{
				return path_;
			}

		private:
			std::filesystem::path path_;
		};

		DecisionTraceDocument fixture()
		{
			return read_decision_trace_file(CONTROL_LINK_TEST_DECISION_TRACE_PATH);
		}

		TEST(DecisionTraceRecorder, WriterThreadDrainsFramesAndClosesWithValidFooter)
		{
			const auto expected = fixture();
			TemporaryPath output{"_valid.jsonl"};
			DecisionTraceRecorder recorder{output.path(), expected.header, 32U};
			for (const auto &frame : expected.frames)
			{
				ASSERT_TRUE(recorder.try_enqueue(frame));
			}

			const auto status = recorder.stop();
			EXPECT_TRUE(status.stopped);
			EXPECT_TRUE(status.trace_valid);
			EXPECT_FALSE(status.trace_overflow);
			EXPECT_FALSE(status.writer_failed);
			EXPECT_EQ(status.accepted_event_count, expected.frames.size());
			EXPECT_EQ(status.accepted_result_count, expected.footer.result_count);

			const auto actual = read_decision_trace_file(output.path());
			ASSERT_EQ(actual.frames.size(), expected.frames.size());
			EXPECT_EQ(actual.footer.event_count, expected.footer.event_count);
			EXPECT_EQ(actual.footer.result_count, expected.footer.result_count);
			for (std::size_t index = 0U; index < actual.frames.size(); ++index)
			{
				EXPECT_EQ(
					serialize_decision_trace_record(actual.frames[index].event),
					serialize_decision_trace_record(expected.frames[index].event));
				EXPECT_EQ(
					actual.frames[index].expected_result.has_value(),
					expected.frames[index].expected_result.has_value());
				if (actual.frames[index].expected_result.has_value())
				{
					EXPECT_TRUE(decision_results_equal(
						expected.frames[index].expected_result.value(),
						actual.frames[index].expected_result.value()));
				}
			}
		}

		TEST(DecisionTraceRecorder, QueueOverflowStopsAcceptanceAndMarksTraceInvalid)
		{
			const auto expected = fixture();
			TemporaryPath fifo{"_overflow.fifo"};
			ASSERT_EQ(::mkfifo(fifo.path().c_str(), 0600), 0) << std::strerror(errno);
			DecisionTraceRecorder recorder{fifo.path(), expected.header, 1U};
			ASSERT_TRUE(recorder.try_enqueue(expected.frames.at(0U)));
			EXPECT_FALSE(recorder.try_enqueue(expected.frames.at(1U)));

			const int reader = ::open(fifo.path().c_str(), O_RDONLY | O_NONBLOCK);
			ASSERT_GE(reader, 0) << std::strerror(errno);
			const auto status = recorder.stop();
			::close(reader);

			EXPECT_TRUE(status.stopped);
			EXPECT_FALSE(status.trace_valid);
			EXPECT_TRUE(status.trace_overflow);
			EXPECT_FALSE(status.writer_failed);
			EXPECT_EQ(status.accepted_event_count, 1U);
			EXPECT_NE(status.error_message.find("overflow"), std::string::npos);
		}

		TEST(DecisionTraceRecorder, WriterFailureNeverProducesAValidStatus)
		{
			const auto expected = fixture();
			DecisionTraceRecorder recorder{"/dev/full", expected.header, 4U};
			(void)recorder.try_enqueue(expected.frames.at(0U));
			const auto status = recorder.stop();

			EXPECT_TRUE(status.stopped);
			EXPECT_FALSE(status.trace_valid);
			EXPECT_TRUE(status.writer_failed);
			EXPECT_FALSE(status.accepting);
			EXPECT_FALSE(status.error_message.empty());
		}

		TEST(DecisionTraceRecorder, MalformedSequenceIsRejectedBeforeFileIo)
		{
			auto expected = fixture();
			expected.frames.front().event.event_sequence = 2U;
			TemporaryPath output{"_sequence.jsonl"};
			DecisionTraceRecorder recorder{output.path(), expected.header, 4U};
			EXPECT_FALSE(recorder.try_enqueue(expected.frames.front()));
			const auto status = recorder.stop();

			EXPECT_FALSE(status.trace_valid);
			EXPECT_FALSE(status.trace_overflow);
			EXPECT_FALSE(status.writer_failed);
			EXPECT_EQ(status.accepted_event_count, 0U);
		}

		TEST(DecisionTraceRecorder, ExplicitInvalidationStopsAcceptanceAndWritesInvalidFooter)
		{
			const auto expected = fixture();
			TemporaryPath output{"_explicit_invalid.jsonl"};
			DecisionTraceRecorder recorder{output.path(), expected.header, 4U};
			ASSERT_TRUE(recorder.try_enqueue(expected.frames.front()));
			recorder.invalidate("live DecisionEngine event failed");
			EXPECT_FALSE(recorder.try_enqueue(expected.frames.at(1U)));

			const auto status = recorder.stop();
			EXPECT_FALSE(status.trace_valid);
			EXPECT_FALSE(status.trace_overflow);
			EXPECT_FALSE(status.writer_failed);
			EXPECT_FALSE(status.accepting);
			EXPECT_EQ(status.error_message, "live DecisionEngine event failed");

			const auto trace = read_decision_trace_file(output.path());
			EXPECT_FALSE(trace.footer.trace_valid);
			EXPECT_EQ(trace.footer.event_count, 1U);
			EXPECT_EQ(trace.footer.error_message, status.error_message);
		}
	}  // namespace
}  // namespace control_link_gateway
