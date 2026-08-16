#pragma once

#include <cstddef>
#include <cstdint>
#include <filesystem>
#include <iosfwd>
#include <optional>
#include <stdexcept>
#include <string>
#include <variant>
#include <vector>

#include "control_link_gateway/decision_event.hpp"

namespace control_link_gateway
{
	enum class DecisionTraceErrorCode : std::uint8_t
	{
		kIo,
		kInvalidJson,
		kUnknownField,
		kMissingField,
		kWrongType,
		kInvalidValue,
		kUnsupportedSchema,
		kSequence,
		kRecordOrder,
		kTruncated,
	};

	class DecisionTraceError final : public std::runtime_error
	{
	public:
		DecisionTraceError(
			DecisionTraceErrorCode code,
			std::size_t line,
			std::string message);

		[[nodiscard]] DecisionTraceErrorCode code() const noexcept;
		[[nodiscard]] std::size_t line() const noexcept;

	private:
		DecisionTraceErrorCode code_;
		std::size_t line_;
	};

	struct DecisionTraceFrame
	{
		DecisionEvent event;
		std::optional<DecisionResult> expected_result;
	};

	struct DecisionTraceDocument
	{
		DecisionTraceHeader header;
		std::vector<DecisionTraceFrame> frames;
		DecisionTraceFooter footer;
	};

	using DecisionTraceRecord = std::variant<
		DecisionTraceHeader,
		DecisionEvent,
		DecisionResult,
		DecisionTraceFooter>;

	[[nodiscard]] std::string serialize_decision_trace_record(
		const DecisionTraceRecord &record);

	[[nodiscard]] DecisionTraceRecord parse_decision_trace_record(
		const std::string &line,
		std::size_t line_number);

	[[nodiscard]] DecisionTraceDocument read_decision_trace(std::istream &input);

	[[nodiscard]] DecisionTraceDocument read_decision_trace_file(
		const std::filesystem::path &path);

	// 比较使用浮点 bit pattern，不使用 epsilon 掩盖 live/replay 差异
	[[nodiscard]] bool decision_results_equal(
		const DecisionResult &left,
		const DecisionResult &right) noexcept;

	[[nodiscard]] std::string describe_first_decision_difference(
		const DecisionResult &expected,
		const DecisionResult &actual);
}  // namespace control_link_gateway
