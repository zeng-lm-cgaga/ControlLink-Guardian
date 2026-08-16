#include <cstddef>
#include <cstdint>
#include <cmath>
#include <limits>
#include <sstream>
#include <string>

#include <gtest/gtest.h>

#include "control_link_gateway/decision_trace.hpp"

namespace control_link_gateway
{
	namespace
	{
		PublisherGenerationKey generation(std::uint8_t seed)
		{
			PublisherGenerationKey result{"rmw_fastrtps_cpp", {}};
			for (std::size_t index = 0U; index < result.publisher_gid.size(); ++index)
			{
				result.publisher_gid[index] = static_cast<std::uint8_t>(seed + index);
			}
			return result;
		}

		GatewayHealthSnapshot healthy_gateway()
		{
			GatewayHealthSnapshot health;
			health.critical_endpoints_healthy = true;
			health.critical_qos_compatible = true;
			health.ros_clock_healthy = true;
			health.vehicle_state_valid = true;
			health.vehicle_state_fresh = true;
			health.output_tick_healthy = true;
			health.internal_invariants_healthy = true;
			return health;
		}

		control_link_interfaces::msg::ControlCommand command(
			std::string source_id,
			std::uint64_t sequence)
		{
			control_link_interfaces::msg::ControlCommand result;
			result.source_stamp.sec = 12;
			result.source_stamp.nanosec = 345U;
			result.source_id = std::move(source_id);
			result.source_sequence = sequence;
			result.mode = control_link_interfaces::msg::ControlCommand::MODE_NORMAL;
			result.linear_velocity_mps = -0.0;
			result.angular_velocity_radps = std::numeric_limits<double>::quiet_NaN();
			return result;
		}

		DecisionTraceDocument document()
		{
			const auto nav2_generation = generation(1U);
			DecisionTraceDocument result;
			result.header = DecisionTraceHeader{
				kDecisionTraceSchemaVersion,
				"0123456789abcdef0123456789abcdef01234567",
				true,
				"RelWithDebInfo",
				"robot",
				"control_link_gateway_v1",
				1U,
				std::string(64U, 'a'),
				std::string(64U, 'b'),
				"rmw_fastrtps_cpp",
				"humble",
				"relative_ns_zero"};

			result.frames.push_back(DecisionTraceFrame{
				DecisionEvent{
					1U,
					DecisionLifecycleEvent{
						DecisionLifecycleTransition::kConfigure,
						DecisionLifecycleResult::kSuccess}},
				std::nullopt});
			result.frames.push_back(DecisionTraceFrame{
				DecisionEvent{
					2U,
					DecisionHealthSnapshotEvent{
						1U,
						10'000'000LL,
						healthy_gateway(),
						{
							DecisionSourceEndpoint{
								"nav2",
								DecisionSourceEndpointState::kUsable,
								nav2_generation},
							DecisionSourceEndpoint{
								"teleop",
								DecisionSourceEndpointState::kMissing,
								std::nullopt}}}},
				std::nullopt});
			result.frames.push_back(DecisionTraceFrame{
				DecisionEvent{
					3U,
					DecisionLifecycleEvent{
						DecisionLifecycleTransition::kActivate,
						DecisionLifecycleResult::kSuccess}},
				std::nullopt});
			result.frames.push_back(DecisionTraceFrame{
				DecisionEvent{
					4U,
					DecisionSourceSampleEvent{
						"nav2",
						nav2_generation,
						command("nav2", 7U),
						12'000'000'345LL,
						20'000'000LL}},
				std::nullopt});

			DecisionResult tick_result;
			tick_result.event_sequence = 5U;
			tick_result.state = DataState::kRecovering;
			tick_result.reason = StateReason::kFirstValidCommand;
			tick_result.recovery_candidate = RecoveryCandidateKey{
				"nav2", nav2_generation};
			tick_result.recovery_valid_count = 1U;
			tick_result.transition_sequence = 1U;
			tick_result.sources = {
				DecisionSourceStatus{
					"nav2", 1U, 0U, RejectReason::kNone, 7U, true, true, 0},
				DecisionSourceStatus{
					"teleop", 0U, 0U, RejectReason::kNone, std::nullopt, false, false, 0}};
			tick_result.canonical_command = command("nav2", 7U);
			tick_result.canonical_command.mode =
				control_link_interfaces::msg::ControlCommand::MODE_HOLD;
			tick_result.canonical_command.linear_velocity_mps = 0.0;
			tick_result.canonical_command.angular_velocity_radps = 0.0;
			result.frames.push_back(DecisionTraceFrame{
				DecisionEvent{
					5U,
					DecisionOutputTickEvent{
						40'000'000LL,
						12'020'000'345LL,
						20'000'000LL,
						0LL,
						1U}},
				tick_result});

			result.footer = DecisionTraceFooter{
				5U,
				5U,
				1U,
				true,
				false,
				""};
			return result;
		}

		DecisionTraceDocument parse_text(const std::string &text)
		{
			std::istringstream input{text};
			return read_decision_trace(input);
		}

		std::string serialize_document(const DecisionTraceDocument &value)
		{
			std::ostringstream output;
			output << serialize_decision_trace_record(value.header) << '\n';
			for (const auto &frame : value.frames)
			{
				output << serialize_decision_trace_record(frame.event) << '\n';
				if (frame.expected_result.has_value())
				{
					output << serialize_decision_trace_record(
						frame.expected_result.value()) << '\n';
				}
			}
			output << serialize_decision_trace_record(value.footer) << '\n';
			return output.str();
		}

		void expect_trace_error(
			const std::string &text,
			DecisionTraceErrorCode expected_code)
		{
			try
			{
				(void)parse_text(text);
				FAIL() << "expected DecisionTraceError";
			}
			catch (const DecisionTraceError &error)
			{
				EXPECT_EQ(error.code(), expected_code) << error.what();
			}
		}

		TEST(DecisionTraceSchema, RoundTripsAllRecordKindsAndFloatingPointBits)
		{
			const auto original = document();
			const auto encoded = serialize_document(original);
			const auto decoded = parse_text(encoded);

			EXPECT_EQ(decoded.header.trace_schema_version, kDecisionTraceSchemaVersion);
			EXPECT_EQ(decoded.header.contract_hash, original.header.contract_hash);
			ASSERT_EQ(decoded.frames.size(), original.frames.size());
			ASSERT_TRUE(decoded.frames.back().expected_result.has_value());
			ASSERT_TRUE(original.frames.back().expected_result.has_value());
			EXPECT_TRUE(decision_results_equal(
				original.frames.back().expected_result.value(),
				decoded.frames.back().expected_result.value()));

			const auto &sample = std::get<DecisionSourceSampleEvent>(
				decoded.frames.at(3U).event.payload);
			EXPECT_TRUE(std::signbit(sample.command.linear_velocity_mps));
			EXPECT_TRUE(std::isnan(sample.command.angular_velocity_radps));
			EXPECT_EQ(decoded.footer.event_count, 5U);
			EXPECT_EQ(decoded.footer.result_count, 1U);
		}

		TEST(DecisionTraceSchema, RejectsUnknownFieldsUnsupportedSchemaAndBadHashes)
		{
			auto encoded = serialize_document(document());
			const auto first_newline = encoded.find('\n');
			ASSERT_NE(first_newline, std::string::npos);

			auto unknown = encoded;
			const auto record_end = unknown.find('}', 0U);
			ASSERT_NE(record_end, std::string::npos);
			unknown.insert(record_end, ",\"unexpected\":1");
			expect_trace_error(unknown, DecisionTraceErrorCode::kUnknownField);

			auto schema = encoded;
			const auto version = schema.find("\"trace_schema_version\":1");
			ASSERT_NE(version, std::string::npos);
			schema.replace(version, std::string("\"trace_schema_version\":1").size(),
				"\"trace_schema_version\":2");
			expect_trace_error(schema, DecisionTraceErrorCode::kUnsupportedSchema);

			auto bad_hash = encoded;
			const auto hash = bad_hash.find(std::string(64U, 'a'));
			ASSERT_NE(hash, std::string::npos);
			bad_hash[hash] = 'A';
			expect_trace_error(bad_hash, DecisionTraceErrorCode::kInvalidValue);
		}

		TEST(DecisionTraceSchema, RejectsMissingResultsSequenceGapsAndTruncation)
		{
			const auto encoded = serialize_document(document());
			const auto footer_start = encoded.rfind("{\"error_message\"");
			ASSERT_NE(footer_start, std::string::npos);
			expect_trace_error(
				encoded.substr(0U, footer_start),
				DecisionTraceErrorCode::kTruncated);

			auto missing_result = encoded;
			const auto result_start = missing_result.find("{\"canonical_command\"");
			ASSERT_NE(result_start, std::string::npos);
			const auto result_end = missing_result.find('\n', result_start);
			ASSERT_NE(result_end, std::string::npos);
			missing_result.erase(result_start, result_end - result_start + 1U);
			expect_trace_error(missing_result, DecisionTraceErrorCode::kTruncated);

			auto gap = encoded;
			const auto sequence = gap.find("\"event_sequence\":4");
			ASSERT_NE(sequence, std::string::npos);
			gap.replace(sequence, std::string("\"event_sequence\":4").size(),
				"\"event_sequence\":6");
			expect_trace_error(gap, DecisionTraceErrorCode::kSequence);
		}

		TEST(DecisionTraceSchema, PreservesInvalidFooterWithoutTreatingItAsReplayable)
		{
			auto invalid = document();
			invalid.footer.trace_valid = false;
			invalid.footer.trace_overflow = true;
			invalid.footer.error_message = "trace queue overflow";
			const auto decoded = parse_text(serialize_document(invalid));
			EXPECT_FALSE(decoded.footer.trace_valid);
			EXPECT_TRUE(decoded.footer.trace_overflow);
			EXPECT_EQ(decoded.footer.error_message, "trace queue overflow");
		}

		TEST(DecisionTraceSchema, ReportsFirstResultDifferenceWithoutTolerance)
		{
			const auto original = document().frames.back().expected_result.value();
			auto changed = original;
			changed.canonical_command.linear_velocity_mps = -0.0;
			EXPECT_FALSE(decision_results_equal(original, changed));
			EXPECT_EQ(
				describe_first_decision_difference(original, changed),
				"canonical_command");
		}
	}  // namespace
}  // namespace control_link_gateway
