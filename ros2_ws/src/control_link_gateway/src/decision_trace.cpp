#include "control_link_gateway/decision_trace.hpp"

#include <algorithm>
#include <array>
#include <charconv>
#include <cstring>
#include <fstream>
#include <iomanip>
#include <limits>
#include <map>
#include <set>
#include <sstream>
#include <string_view>
#include <type_traits>
#include <utility>

#include <nlohmann/json.hpp>

namespace control_link_gateway
{
	namespace
	{
		using Json = nlohmann::json;

		[[noreturn]] void fail(
			DecisionTraceErrorCode code,
			std::size_t line,
			const std::string &message)
		{
			throw DecisionTraceError(code, line, message);
		}

		void require_exact_keys(
			const Json &value,
			std::initializer_list<std::string_view> expected,
			std::size_t line,
			std::string_view path)
		{
			if (!value.is_object())
			{
				fail(
					DecisionTraceErrorCode::kWrongType,
					line,
					std::string(path) + " must be an object");
			}

			std::set<std::string> expected_keys;
			for (const auto key : expected)
			{
				expected_keys.emplace(key);
				if (!value.contains(std::string(key)))
				{
					fail(
						DecisionTraceErrorCode::kMissingField,
						line,
						std::string(path) + " is missing field " + std::string(key));
				}
			}

			for (const auto &[key, unused] : value.items())
			{
				(void)unused;
				if (expected_keys.count(key) == 0U)
				{
					fail(
						DecisionTraceErrorCode::kUnknownField,
						line,
						std::string(path) + " contains unknown field " + key);
				}
			}
		}

		const Json &field(
			const Json &parent,
			std::string_view name,
			std::size_t line,
			std::string_view path)
		{
			const auto iterator = parent.find(std::string(name));
			if (iterator == parent.end())
			{
				fail(
					DecisionTraceErrorCode::kMissingField,
					line,
					std::string(path) + " is missing field " + std::string(name));
			}
			return *iterator;
		}

		std::string require_string(
			const Json &parent,
			std::string_view name,
			std::size_t line,
			std::string_view path,
			bool allow_empty = false)
		{
			const auto &value = field(parent, name, line, path);
			if (!value.is_string())
			{
				fail(
					DecisionTraceErrorCode::kWrongType,
					line,
					std::string(path) + "." + std::string(name) + " must be a string");
			}
			auto result = value.get<std::string>();
			if (!allow_empty && result.empty())
			{
				fail(
					DecisionTraceErrorCode::kInvalidValue,
					line,
					std::string(path) + "." + std::string(name) + " must not be empty");
			}
			return result;
		}

		bool require_bool(
			const Json &parent,
			std::string_view name,
			std::size_t line,
			std::string_view path)
		{
			const auto &value = field(parent, name, line, path);
			if (!value.is_boolean())
			{
				fail(
					DecisionTraceErrorCode::kWrongType,
					line,
					std::string(path) + "." + std::string(name) + " must be a boolean");
			}
			return value.get<bool>();
		}

		std::uint64_t require_unsigned(
			const Json &parent,
			std::string_view name,
			std::size_t line,
			std::string_view path)
		{
			const auto &value = field(parent, name, line, path);
			if (!value.is_number_unsigned())
			{
				fail(
					DecisionTraceErrorCode::kWrongType,
					line,
					std::string(path) + "." + std::string(name) + " must be an unsigned integer");
			}
			return value.get<std::uint64_t>();
		}

		std::int64_t require_signed(
			const Json &parent,
			std::string_view name,
			std::size_t line,
			std::string_view path)
		{
			const auto &value = field(parent, name, line, path);
			if (!value.is_number_integer())
			{
				fail(
					DecisionTraceErrorCode::kWrongType,
					line,
					std::string(path) + "." + std::string(name) + " must be an integer");
			}
			return value.get<std::int64_t>();
		}

		template<typename T>
		T checked_unsigned_cast(
			std::uint64_t value,
			std::size_t line,
			std::string_view path)
		{
			static_assert(std::is_unsigned<T>::value, "T must be unsigned");
			if (value > static_cast<std::uint64_t>(std::numeric_limits<T>::max()))
			{
				fail(
					DecisionTraceErrorCode::kInvalidValue,
					line,
					std::string(path) + " exceeds its integer range");
			}
			return static_cast<T>(value);
		}

		bool is_lower_hex(std::string_view value, std::size_t required_length)
		{
			return value.size() == required_length && std::all_of(
				value.begin(),
				value.end(),
				[](char character)
				{
					return (character >= '0' && character <= '9') ||
						(character >= 'a' && character <= 'f');
				});
		}

		void validate_hash(
			const std::string &value,
			std::size_t line,
			std::string_view path)
		{
			if (!is_lower_hex(value, 64U))
			{
				fail(
					DecisionTraceErrorCode::kInvalidValue,
					line,
					std::string(path) + " must be a lowercase SHA-256 hex string");
			}
		}

		std::string double_to_hex(double value)
		{
			std::uint64_t bits = 0U;
			static_assert(sizeof(bits) == sizeof(value), "double must be 64 bits");
			std::memcpy(&bits, &value, sizeof(bits));
			std::ostringstream stream;
			stream << std::hex << std::setfill('0') << std::setw(16) << bits;
			return stream.str();
		}

		double hex_to_double(
			const std::string &value,
			std::size_t line,
			std::string_view path)
		{
			if (!is_lower_hex(value, 16U))
			{
				fail(
					DecisionTraceErrorCode::kInvalidValue,
					line,
					std::string(path) + " must contain 16 lowercase hex digits");
			}
			std::uint64_t bits = 0U;
			const auto parsed = std::from_chars(
				value.data(), value.data() + value.size(), bits, 16);
			if (parsed.ec != std::errc{} || parsed.ptr != value.data() + value.size())
			{
				fail(
					DecisionTraceErrorCode::kInvalidValue,
					line,
					std::string(path) + " contains invalid floating point bits");
			}
			double result = 0.0;
			std::memcpy(&result, &bits, sizeof(result));
			return result;
		}

		std::string generation_gid_hex(const PublisherGenerationKey &generation)
		{
			constexpr char kHexDigits[] = "0123456789abcdef";
			std::string result;
			result.reserve(generation.publisher_gid.size() * 2U);
			for (const auto byte : generation.publisher_gid)
			{
				result.push_back(kHexDigits[(byte >> 4U) & 0x0FU]);
				result.push_back(kHexDigits[byte & 0x0FU]);
			}
			return result;
		}

		PublisherGenerationKey parse_generation(
			const Json &value,
			std::size_t line,
			std::string_view path)
		{
			require_exact_keys(
				value, {"rmw_implementation", "publisher_gid"}, line, path);
			PublisherGenerationKey result{
				require_string(value, "rmw_implementation", line, path), {}};
			const auto gid = require_string(value, "publisher_gid", line, path);
			if (!is_lower_hex(gid, result.publisher_gid.size() * 2U))
			{
				fail(
					DecisionTraceErrorCode::kInvalidValue,
					line,
					std::string(path) + ".publisher_gid has the wrong hex length or case");
			}
			for (std::size_t index = 0U; index < result.publisher_gid.size(); ++index)
			{
				unsigned int byte = 0U;
				const char *begin = gid.data() + index * 2U;
				const auto parsed = std::from_chars(begin, begin + 2U, byte, 16);
				if (parsed.ec != std::errc{} || parsed.ptr != begin + 2U)
				{
					fail(
						DecisionTraceErrorCode::kInvalidValue,
						line,
						std::string(path) + ".publisher_gid contains invalid hex");
				}
				result.publisher_gid[index] = static_cast<std::uint8_t>(byte);
			}
			return result;
		}

		Json generation_json(const PublisherGenerationKey &generation)
		{
			return Json{
				{"rmw_implementation", generation.rmw_implementation},
				{"publisher_gid", generation_gid_hex(generation)}};
		}

		Json command_json(const control_link_interfaces::msg::ControlCommand &command)
		{
			return Json{
				{"source_stamp", Json{
					{"sec", command.source_stamp.sec},
					{"nanosec", command.source_stamp.nanosec}}},
				{"source_id", command.source_id},
				{"source_sequence", command.source_sequence},
				{"mode", command.mode},
				{"linear_velocity_bits", double_to_hex(command.linear_velocity_mps)},
				{"angular_velocity_bits", double_to_hex(command.angular_velocity_radps)}};
		}

		control_link_interfaces::msg::ControlCommand parse_command(
			const Json &value,
			std::size_t line,
			std::string_view path)
		{
			require_exact_keys(
				value,
				{"source_stamp", "source_id", "source_sequence", "mode",
				 "linear_velocity_bits", "angular_velocity_bits"},
				line,
				path);
			const auto &stamp = field(value, "source_stamp", line, path);
			require_exact_keys(stamp, {"sec", "nanosec"}, line, "command.source_stamp");

			const auto sec = require_signed(stamp, "sec", line, "command.source_stamp");
			if (sec < std::numeric_limits<std::int32_t>::min() ||
				sec > std::numeric_limits<std::int32_t>::max())
			{
				fail(
					DecisionTraceErrorCode::kInvalidValue,
					line,
					std::string(path) + ".source_stamp.sec exceeds int32 range");
			}

			control_link_interfaces::msg::ControlCommand result;
			result.source_stamp.sec = static_cast<std::int32_t>(sec);
			result.source_stamp.nanosec = checked_unsigned_cast<std::uint32_t>(
				require_unsigned(stamp, "nanosec", line, "command.source_stamp"),
				line,
				"command.source_stamp.nanosec");
			result.source_id = require_string(value, "source_id", line, path, true);
			result.source_sequence = require_unsigned(value, "source_sequence", line, path);
			result.mode = checked_unsigned_cast<std::uint8_t>(
				require_unsigned(value, "mode", line, path), line, "command.mode");
			result.linear_velocity_mps = hex_to_double(
				require_string(value, "linear_velocity_bits", line, path),
				line,
				"command.linear_velocity_bits");
			result.angular_velocity_radps = hex_to_double(
				require_string(value, "angular_velocity_bits", line, path),
				line,
				"command.angular_velocity_bits");
			return result;
		}

		Json health_json(const GatewayHealthSnapshot &health)
		{
			return Json{
				{"critical_endpoints_healthy", health.critical_endpoints_healthy},
				{"critical_qos_compatible", health.critical_qos_compatible},
				{"ros_clock_healthy", health.ros_clock_healthy},
				{"vehicle_state_valid", health.vehicle_state_valid},
				{"vehicle_state_fresh", health.vehicle_state_fresh},
				{"vehicle_reports_safe_stop", health.vehicle_reports_safe_stop},
				{"vehicle_reports_fault", health.vehicle_reports_fault},
				{"output_tick_healthy", health.output_tick_healthy},
				{"internal_invariants_healthy", health.internal_invariants_healthy}};
		}

		GatewayHealthSnapshot parse_health(
			const Json &value,
			std::size_t line)
		{
			constexpr std::string_view kPath = "event.health";
			require_exact_keys(
				value,
				{"critical_endpoints_healthy", "critical_qos_compatible",
				 "ros_clock_healthy", "vehicle_state_valid", "vehicle_state_fresh",
				 "vehicle_reports_safe_stop", "vehicle_reports_fault",
				 "output_tick_healthy", "internal_invariants_healthy"},
				line,
				kPath);
			GatewayHealthSnapshot result;
			result.critical_endpoints_healthy = require_bool(
				value, "critical_endpoints_healthy", line, kPath);
			result.critical_qos_compatible = require_bool(
				value, "critical_qos_compatible", line, kPath);
			result.ros_clock_healthy = require_bool(
				value, "ros_clock_healthy", line, kPath);
			result.vehicle_state_valid = require_bool(
				value, "vehicle_state_valid", line, kPath);
			result.vehicle_state_fresh = require_bool(
				value, "vehicle_state_fresh", line, kPath);
			result.vehicle_reports_safe_stop = require_bool(
				value, "vehicle_reports_safe_stop", line, kPath);
			result.vehicle_reports_fault = require_bool(
				value, "vehicle_reports_fault", line, kPath);
			result.output_tick_healthy = require_bool(
				value, "output_tick_healthy", line, kPath);
			result.internal_invariants_healthy = require_bool(
				value, "internal_invariants_healthy", line, kPath);
			return result;
		}

		const char *lifecycle_transition_name(DecisionLifecycleTransition transition)
		{
			switch (transition)
			{
			case DecisionLifecycleTransition::kConfigure:
				return "configure";
			case DecisionLifecycleTransition::kActivate:
				return "activate";
			case DecisionLifecycleTransition::kDeactivate:
				return "deactivate";
			case DecisionLifecycleTransition::kCleanup:
				return "cleanup";
			case DecisionLifecycleTransition::kError:
				return "error";
			}
			throw std::invalid_argument("unsupported lifecycle transition in Decision Trace");
		}

		DecisionLifecycleTransition parse_lifecycle_transition(
			const std::string &value,
			std::size_t line)
		{
			if (value == "configure") return DecisionLifecycleTransition::kConfigure;
			if (value == "activate") return DecisionLifecycleTransition::kActivate;
			if (value == "deactivate") return DecisionLifecycleTransition::kDeactivate;
			if (value == "cleanup") return DecisionLifecycleTransition::kCleanup;
			if (value == "error") return DecisionLifecycleTransition::kError;
			fail(
				DecisionTraceErrorCode::kInvalidValue,
				line,
				"event.transition contains an unsupported lifecycle transition");
		}

		const char *lifecycle_result_name(DecisionLifecycleResult result)
		{
			switch (result)
			{
			case DecisionLifecycleResult::kSuccess:
				return "success";
			case DecisionLifecycleResult::kFailure:
				return "failure";
			case DecisionLifecycleResult::kError:
				return "error";
			}
			throw std::invalid_argument("unsupported lifecycle result in Decision Trace");
		}

		DecisionLifecycleResult parse_lifecycle_result(
			const std::string &value,
			std::size_t line)
		{
			if (value == "success") return DecisionLifecycleResult::kSuccess;
			if (value == "failure") return DecisionLifecycleResult::kFailure;
			if (value == "error") return DecisionLifecycleResult::kError;
			fail(
				DecisionTraceErrorCode::kInvalidValue,
				line,
				"event.result contains an unsupported lifecycle result");
		}

		const char *source_endpoint_state_name(DecisionSourceEndpointState state)
		{
			switch (state)
			{
			case DecisionSourceEndpointState::kMissing:
				return "missing";
			case DecisionSourceEndpointState::kAmbiguous:
				return "ambiguous";
			case DecisionSourceEndpointState::kUnexpectedDirection:
				return "unexpected_direction";
			case DecisionSourceEndpointState::kTypeMismatch:
				return "type_mismatch";
			case DecisionSourceEndpointState::kQosMismatch:
				return "qos_mismatch";
			case DecisionSourceEndpointState::kUsable:
				return "usable";
			}
			throw std::invalid_argument("unsupported source endpoint state in Decision Trace");
		}

		DecisionSourceEndpointState parse_source_endpoint_state(
			const std::string &value,
			std::size_t line)
		{
			if (value == "missing") return DecisionSourceEndpointState::kMissing;
			if (value == "ambiguous") return DecisionSourceEndpointState::kAmbiguous;
			if (value == "unexpected_direction") return DecisionSourceEndpointState::kUnexpectedDirection;
			if (value == "type_mismatch") return DecisionSourceEndpointState::kTypeMismatch;
			if (value == "qos_mismatch") return DecisionSourceEndpointState::kQosMismatch;
			if (value == "usable") return DecisionSourceEndpointState::kUsable;
			fail(
				DecisionTraceErrorCode::kInvalidValue,
				line,
				"event.source_endpoints contains an unsupported endpoint state");
		}

		bool is_known_data_state(DataState state) noexcept
		{
			switch (state)
			{
			case DataState::kStandby:
			case DataState::kActive:
			case DataState::kDegraded:
			case DataState::kSafeStop:
			case DataState::kRecovering:
			case DataState::kError:
				return true;
			}
			return false;
		}

		bool is_known_state_reason(StateReason reason) noexcept
		{
			switch (reason)
			{
			case StateReason::kNone:
			case StateReason::kFirstValidCommand:
			case StateReason::kSourceSwitch:
			case StateReason::kSourceFallback:
			case StateReason::kNoQualifiedSource:
			case StateReason::kCriticalEndpointUnhealthy:
			case StateReason::kCriticalQosMismatch:
			case StateReason::kVehicleStateTimeout:
			case StateReason::kVehicleSafeStop:
			case StateReason::kVehicleFault:
			case StateReason::kVehicleStateInvalid:
			case StateReason::kClockInvalid:
			case StateReason::kOutputTickOverrun:
			case StateReason::kRecoveryComplete:
			case StateReason::kInternalInvariant:
				return true;
			}
			return false;
		}

		bool is_known_reject_reason(RejectReason reason) noexcept
		{
			switch (reason)
			{
			case RejectReason::kNone:
			case RejectReason::kSourceIdMismatch:
			case RejectReason::kSourceEndpointAmbiguous:
			case RejectReason::kPublisherGenerationUnstable:
			case RejectReason::kUnknownMode:
			case RejectReason::kNonFinite:
			case RejectReason::kOutOfRange:
			case RejectReason::kZeroStamp:
			case RejectReason::kFutureStamp:
			case RejectReason::kStale:
			case RejectReason::kSequenceNotIncreasing:
			case RejectReason::kClockInvalid:
			case RejectReason::kHoldNonzero:
			case RejectReason::kInvalidStamp:
				return true;
			}
			return false;
		}

		Json source_endpoint_json(const DecisionSourceEndpoint &endpoint)
		{
			const bool usable = endpoint.state == DecisionSourceEndpointState::kUsable;
			if (usable != endpoint.publisher_generation.has_value())
			{
				throw std::invalid_argument(
					"usable source endpoint must have exactly one publisher generation");
			}
			return Json{
				{"source_id", endpoint.source_id},
				{"state", source_endpoint_state_name(endpoint.state)},
				{"publisher_generation", endpoint.publisher_generation.has_value() ?
					Json(generation_json(endpoint.publisher_generation.value())) : Json(nullptr)}};
		}

		DecisionSourceEndpoint parse_source_endpoint(
			const Json &value,
			std::size_t line)
		{
			require_exact_keys(
				value,
				{"source_id", "state", "publisher_generation"},
				line,
				"event.source_endpoint");
			DecisionSourceEndpoint result;
			result.source_id = require_string(
				value, "source_id", line, "event.source_endpoint");
			result.state = parse_source_endpoint_state(
				require_string(value, "state", line, "event.source_endpoint"), line);
			const auto &generation = field(
				value, "publisher_generation", line, "event.source_endpoint");
			if (!generation.is_null())
			{
				result.publisher_generation = parse_generation(
					generation, line, "event.source_endpoint.publisher_generation");
			}
			const bool usable = result.state == DecisionSourceEndpointState::kUsable;
			if (usable != result.publisher_generation.has_value())
			{
				fail(
					DecisionTraceErrorCode::kInvalidValue,
					line,
					"event.source_endpoint usable state and publisher_generation disagree");
			}
			return result;
		}

		Json recovery_candidate_json(const RecoveryCandidateKey &candidate)
		{
			return Json{
				{"source_id", candidate.source_id},
				{"publisher_generation", generation_json(candidate.publisher_generation)}};
		}

		RecoveryCandidateKey parse_recovery_candidate(
			const Json &value,
			std::size_t line)
		{
			require_exact_keys(
				value,
				{"source_id", "publisher_generation"},
				line,
				"result.recovery_candidate");
			return RecoveryCandidateKey{
				require_string(value, "source_id", line, "result.recovery_candidate"),
				parse_generation(
					field(value, "publisher_generation", line, "result.recovery_candidate"),
					line,
					"result.recovery_candidate.publisher_generation")};
		}

		Json source_status_json(const DecisionSourceStatus &status)
		{
			return Json{
				{"source_id", status.source_id},
				{"accepted_count", status.accepted_count},
				{"rejected_count", status.rejected_count},
				{"last_reject_reason", static_cast<std::uint16_t>(status.last_reject_reason)},
				{"last_accepted_sequence", status.last_accepted_sequence.has_value() ?
					Json(status.last_accepted_sequence.value()) : Json(nullptr)},
				{"command_valid", status.command_valid},
				{"lease_valid", status.lease_valid},
				{"command_age_ns", status.command_age_ns}};
		}

		DecisionSourceStatus parse_source_status(
			const Json &value,
			std::size_t line)
		{
			require_exact_keys(
				value,
				{"source_id", "accepted_count", "rejected_count", "last_reject_reason",
				 "last_accepted_sequence", "command_valid", "lease_valid", "command_age_ns"},
				line,
				"result.source");
			DecisionSourceStatus result;
			result.source_id = require_string(value, "source_id", line, "result.source");
			result.accepted_count = require_unsigned(
				value, "accepted_count", line, "result.source");
			result.rejected_count = require_unsigned(
				value, "rejected_count", line, "result.source");
			result.last_reject_reason = static_cast<RejectReason>(
				checked_unsigned_cast<std::uint16_t>(
					require_unsigned(value, "last_reject_reason", line, "result.source"),
					line,
					"result.source.last_reject_reason"));
			if (!is_known_reject_reason(result.last_reject_reason))
			{
				fail(
					DecisionTraceErrorCode::kInvalidValue,
					line,
					"result.source.last_reject_reason is unsupported");
			}
			const auto &sequence = field(
				value, "last_accepted_sequence", line, "result.source");
			if (!sequence.is_null())
			{
				if (!sequence.is_number_unsigned())
				{
					fail(
						DecisionTraceErrorCode::kWrongType,
						line,
						"result.source.last_accepted_sequence must be null or unsigned");
				}
				result.last_accepted_sequence = sequence.get<std::uint64_t>();
			}
			result.command_valid = require_bool(
				value, "command_valid", line, "result.source");
			result.lease_valid = require_bool(
				value, "lease_valid", line, "result.source");
			result.command_age_ns = require_signed(
				value, "command_age_ns", line, "result.source");
			if (result.command_age_ns < 0)
			{
				fail(
					DecisionTraceErrorCode::kInvalidValue,
					line,
					"result.source.command_age_ns must be non-negative");
			}
			return result;
		}

		Json header_json(const DecisionTraceHeader &header)
		{
			if (header.trace_schema_version != kDecisionTraceSchemaVersion)
			{
				throw std::invalid_argument("unsupported Decision Trace schema version");
			}
			if (header.steady_time_origin != "relative_ns_zero")
			{
				throw std::invalid_argument("Decision Trace steady origin must be relative_ns_zero");
			}
			validate_hash(header.contract_hash, 0U, "header.contract_hash");
			validate_hash(header.decision_config_hash, 0U, "header.decision_config_hash");
			return Json{
				{"record_type", "header"},
				{"trace_schema_version", header.trace_schema_version},
				{"git_commit", header.git_commit},
				{"git_dirty", header.git_dirty},
				{"build_type", header.build_type},
				{"profile_id", header.profile_id},
				{"contract_id", header.contract_id},
				{"contract_version", header.contract_version},
				{"contract_hash", header.contract_hash},
				{"decision_config_hash", header.decision_config_hash},
				{"rmw_implementation", header.rmw_implementation},
				{"ros_distro", header.ros_distro},
				{"steady_time_origin", header.steady_time_origin}};
		}

		DecisionTraceHeader parse_header(const Json &value, std::size_t line)
		{
			require_exact_keys(
				value,
				{"record_type", "trace_schema_version", "git_commit", "git_dirty",
				 "build_type", "profile_id", "contract_id", "contract_version",
				 "contract_hash", "decision_config_hash", "rmw_implementation",
				 "ros_distro", "steady_time_origin"},
				line,
				"header");
			DecisionTraceHeader result;
			result.trace_schema_version = checked_unsigned_cast<std::uint32_t>(
				require_unsigned(value, "trace_schema_version", line, "header"),
				line,
				"header.trace_schema_version");
			if (result.trace_schema_version != kDecisionTraceSchemaVersion)
			{
				fail(
					DecisionTraceErrorCode::kUnsupportedSchema,
					line,
					"unsupported Decision Trace schema version");
			}
			result.git_commit = require_string(value, "git_commit", line, "header");
			result.git_dirty = require_bool(value, "git_dirty", line, "header");
			result.build_type = require_string(value, "build_type", line, "header");
			result.profile_id = require_string(value, "profile_id", line, "header");
			result.contract_id = require_string(value, "contract_id", line, "header");
			result.contract_version = require_unsigned(
				value, "contract_version", line, "header");
			result.contract_hash = require_string(value, "contract_hash", line, "header");
			result.decision_config_hash = require_string(
				value, "decision_config_hash", line, "header");
			validate_hash(result.contract_hash, line, "header.contract_hash");
			validate_hash(
				result.decision_config_hash, line, "header.decision_config_hash");
			result.rmw_implementation = require_string(
				value, "rmw_implementation", line, "header");
			result.ros_distro = require_string(value, "ros_distro", line, "header");
			result.steady_time_origin = require_string(
				value, "steady_time_origin", line, "header");
			if (result.steady_time_origin != "relative_ns_zero")
			{
				fail(
					DecisionTraceErrorCode::kInvalidValue,
					line,
					"header.steady_time_origin must be relative_ns_zero");
			}
			return result;
		}

		Json event_json(const DecisionEvent &event)
		{
			if (event.event_sequence == 0U)
			{
				throw std::invalid_argument("Decision Event sequence must be positive");
			}
			Json result{{"record_type", "event"}, {"event_sequence", event.event_sequence}};
			std::visit(
				[&result](const auto &payload)
				{
					using Payload = std::decay_t<decltype(payload)>;
					if constexpr (std::is_same_v<Payload, DecisionLifecycleEvent>)
					{
						result["event_type"] = "lifecycle_transition";
						result["transition"] = lifecycle_transition_name(payload.transition);
						result["result"] = lifecycle_result_name(payload.result);
					}
					else if constexpr (std::is_same_v<Payload, DecisionSourceSampleEvent>)
					{
						if (payload.steady_receive_offset_ns < 0)
						{
							throw std::invalid_argument("source sample steady offset must be non-negative");
						}
						result["event_type"] = "source_sample";
						result["expected_source_id"] = payload.expected_source_id;
						result["publisher_generation"] = generation_json(payload.publisher_generation);
						result["command"] = command_json(payload.command);
						result["now_ros_ns"] = payload.now_ros_ns;
						result["steady_receive_offset_ns"] = payload.steady_receive_offset_ns;
					}
					else if constexpr (std::is_same_v<Payload, DecisionHealthSnapshotEvent>)
					{
						if (payload.health_revision == 0U || payload.steady_observed_offset_ns < 0)
						{
							throw std::invalid_argument("health event revision/steady offset is invalid");
						}
						result["event_type"] = "health_snapshot";
						result["health_revision"] = payload.health_revision;
						result["steady_observed_offset_ns"] = payload.steady_observed_offset_ns;
						result["health"] = health_json(payload.health);
						result["source_endpoints"] = Json::array();
						std::string previous;
						for (const auto &endpoint : payload.source_endpoints)
						{
							if (endpoint.source_id.empty() ||
								(!previous.empty() && endpoint.source_id <= previous))
							{
								throw std::invalid_argument("health source endpoints must be sorted and unique");
							}
							previous = endpoint.source_id;
							result["source_endpoints"].push_back(source_endpoint_json(endpoint));
						}
					}
					else
					{
						if (payload.steady_offset_ns < 0 || payload.tick_interval_ns < 0 ||
							payload.tick_lateness_ns < 0 || payload.health_revision == 0U)
						{
							throw std::invalid_argument("output tick timing/revision is invalid");
						}
						result["event_type"] = "output_tick";
						result["steady_offset_ns"] = payload.steady_offset_ns;
						result["now_ros_ns"] = payload.now_ros_ns;
						result["tick_interval_ns"] = payload.tick_interval_ns;
						result["tick_lateness_ns"] = payload.tick_lateness_ns;
						result["health_revision"] = payload.health_revision;
					}
				},
				event.payload);
			return result;
		}

		DecisionEvent parse_event(const Json &value, std::size_t line)
		{
			const auto event_sequence = require_unsigned(
				value, "event_sequence", line, "event");
			if (event_sequence == 0U)
			{
				fail(
					DecisionTraceErrorCode::kInvalidValue,
					line,
					"event.event_sequence must be positive");
			}
			const auto event_type = require_string(value, "event_type", line, "event");

			if (event_type == "lifecycle_transition")
			{
				require_exact_keys(
					value,
					{"record_type", "event_sequence", "event_type", "transition", "result"},
					line,
					"event");
				return DecisionEvent{
					event_sequence,
					DecisionLifecycleEvent{
						parse_lifecycle_transition(
							require_string(value, "transition", line, "event"), line),
						parse_lifecycle_result(
							require_string(value, "result", line, "event"), line)}};
			}

			if (event_type == "source_sample")
			{
				require_exact_keys(
					value,
					{"record_type", "event_sequence", "event_type", "expected_source_id",
					 "publisher_generation", "command", "now_ros_ns",
					 "steady_receive_offset_ns"},
					line,
					"event");
				const auto offset = require_signed(
					value, "steady_receive_offset_ns", line, "event");
				if (offset < 0)
				{
					fail(
						DecisionTraceErrorCode::kInvalidValue,
						line,
						"event.steady_receive_offset_ns must be non-negative");
				}
				return DecisionEvent{
					event_sequence,
					DecisionSourceSampleEvent{
						require_string(value, "expected_source_id", line, "event"),
						parse_generation(
							field(value, "publisher_generation", line, "event"),
							line,
							"event.publisher_generation"),
						parse_command(field(value, "command", line, "event"), line, "event.command"),
						require_signed(value, "now_ros_ns", line, "event"),
						offset}};
			}

			if (event_type == "health_snapshot")
			{
				require_exact_keys(
					value,
					{"record_type", "event_sequence", "event_type", "health_revision",
					 "steady_observed_offset_ns", "health", "source_endpoints"},
					line,
					"event");
				const auto revision = require_unsigned(
					value, "health_revision", line, "event");
				const auto offset = require_signed(
					value, "steady_observed_offset_ns", line, "event");
				if (revision == 0U || offset < 0)
				{
					fail(
						DecisionTraceErrorCode::kInvalidValue,
						line,
						"health event revision/steady offset is invalid");
				}
				const auto &endpoints = field(value, "source_endpoints", line, "event");
				if (!endpoints.is_array())
				{
					fail(
						DecisionTraceErrorCode::kWrongType,
						line,
						"event.source_endpoints must be an array");
				}
				std::vector<DecisionSourceEndpoint> parsed_endpoints;
				parsed_endpoints.reserve(endpoints.size());
				std::string previous;
				for (const auto &endpoint : endpoints)
				{
					auto parsed = parse_source_endpoint(endpoint, line);
					if (!previous.empty() && parsed.source_id <= previous)
					{
						fail(
							DecisionTraceErrorCode::kInvalidValue,
							line,
							"event.source_endpoints must be sorted and unique");
					}
					previous = parsed.source_id;
					parsed_endpoints.push_back(std::move(parsed));
				}
				return DecisionEvent{
					event_sequence,
					DecisionHealthSnapshotEvent{
						revision,
						offset,
						parse_health(field(value, "health", line, "event"), line),
						std::move(parsed_endpoints)}};
			}

			if (event_type == "output_tick")
			{
				require_exact_keys(
					value,
					{"record_type", "event_sequence", "event_type", "steady_offset_ns",
					 "now_ros_ns", "tick_interval_ns", "tick_lateness_ns", "health_revision"},
					line,
					"event");
				const auto offset = require_signed(value, "steady_offset_ns", line, "event");
				const auto interval = require_signed(value, "tick_interval_ns", line, "event");
				const auto lateness = require_signed(value, "tick_lateness_ns", line, "event");
				const auto revision = require_unsigned(value, "health_revision", line, "event");
				if (offset < 0 || interval < 0 || lateness < 0 || revision == 0U)
				{
					fail(
						DecisionTraceErrorCode::kInvalidValue,
						line,
						"output tick timing/revision is invalid");
				}
				return DecisionEvent{
					event_sequence,
					DecisionOutputTickEvent{
						offset,
						require_signed(value, "now_ros_ns", line, "event"),
						interval,
						lateness,
						revision}};
			}

			fail(
				DecisionTraceErrorCode::kInvalidValue,
				line,
				"event.event_type is unsupported");
		}

		Json result_json(const DecisionResult &result)
		{
			if (result.event_sequence == 0U || !is_known_data_state(result.state) ||
				!is_known_state_reason(result.reason))
			{
				throw std::invalid_argument("Decision Result contains an invalid sequence/state/reason");
			}
			Json sources = Json::array();
			std::string previous;
			for (const auto &source : result.sources)
			{
				if (source.source_id.empty() || (!previous.empty() && source.source_id <= previous))
				{
					throw std::invalid_argument("Decision Result sources must be sorted and unique");
				}
				previous = source.source_id;
				sources.push_back(source_status_json(source));
			}
			return Json{
				{"record_type", "result"},
				{"event_sequence", result.event_sequence},
				{"state", static_cast<std::uint8_t>(result.state)},
				{"reason", static_cast<std::uint16_t>(result.reason)},
				{"recovery_candidate", result.recovery_candidate.has_value() ?
					Json(recovery_candidate_json(result.recovery_candidate.value())) : Json(nullptr)},
				{"recovery_valid_count", result.recovery_valid_count},
				{"transition_sequence", result.transition_sequence},
				{"sources", std::move(sources)},
				{"canonical_command", command_json(result.canonical_command)},
				{"lifecycle_error_requested", result.lifecycle_error_requested}};
		}

		DecisionResult parse_result(const Json &value, std::size_t line)
		{
			require_exact_keys(
				value,
				{"record_type", "event_sequence", "state", "reason", "recovery_candidate",
				 "recovery_valid_count", "transition_sequence", "sources",
				 "canonical_command", "lifecycle_error_requested"},
				line,
				"result");
			DecisionResult result;
			result.event_sequence = require_unsigned(value, "event_sequence", line, "result");
			if (result.event_sequence == 0U)
			{
				fail(DecisionTraceErrorCode::kInvalidValue, line, "result sequence must be positive");
			}
			result.state = static_cast<DataState>(checked_unsigned_cast<std::uint8_t>(
				require_unsigned(value, "state", line, "result"), line, "result.state"));
			result.reason = static_cast<StateReason>(checked_unsigned_cast<std::uint16_t>(
				require_unsigned(value, "reason", line, "result"), line, "result.reason"));
			if (!is_known_data_state(result.state) || !is_known_state_reason(result.reason))
			{
				fail(
					DecisionTraceErrorCode::kInvalidValue,
					line,
					"result state or reason is unsupported");
			}
			const auto &candidate = field(value, "recovery_candidate", line, "result");
			if (!candidate.is_null())
			{
				result.recovery_candidate = parse_recovery_candidate(candidate, line);
			}
			result.recovery_valid_count = checked_unsigned_cast<std::uint16_t>(
				require_unsigned(value, "recovery_valid_count", line, "result"),
				line,
				"result.recovery_valid_count");
			result.transition_sequence = require_unsigned(
				value, "transition_sequence", line, "result");
			const auto &sources = field(value, "sources", line, "result");
			if (!sources.is_array())
			{
				fail(DecisionTraceErrorCode::kWrongType, line, "result.sources must be an array");
			}
			std::string previous;
			for (const auto &source : sources)
			{
				auto parsed = parse_source_status(source, line);
				if (!previous.empty() && parsed.source_id <= previous)
				{
					fail(
						DecisionTraceErrorCode::kInvalidValue,
						line,
						"result.sources must be sorted and unique");
				}
				previous = parsed.source_id;
				result.sources.push_back(std::move(parsed));
			}
			result.canonical_command = parse_command(
				field(value, "canonical_command", line, "result"),
				line,
				"result.canonical_command");
			result.lifecycle_error_requested = require_bool(
				value, "lifecycle_error_requested", line, "result");
			return result;
		}

		Json footer_json(const DecisionTraceFooter &footer)
		{
			if (footer.trace_overflow && footer.trace_valid)
			{
				throw std::invalid_argument("overflowed Decision Trace cannot be valid");
			}
			return Json{
				{"record_type", "footer"},
				{"last_event_sequence", footer.last_event_sequence},
				{"event_count", footer.event_count},
				{"result_count", footer.result_count},
				{"trace_valid", footer.trace_valid},
				{"trace_overflow", footer.trace_overflow},
				{"error_message", footer.error_message}};
		}

		DecisionTraceFooter parse_footer(const Json &value, std::size_t line)
		{
			require_exact_keys(
				value,
				{"record_type", "last_event_sequence", "event_count", "result_count",
				 "trace_valid", "trace_overflow", "error_message"},
				line,
				"footer");
			DecisionTraceFooter result;
			result.last_event_sequence = require_unsigned(
				value, "last_event_sequence", line, "footer");
			result.event_count = require_unsigned(value, "event_count", line, "footer");
			result.result_count = require_unsigned(value, "result_count", line, "footer");
			result.trace_valid = require_bool(value, "trace_valid", line, "footer");
			result.trace_overflow = require_bool(value, "trace_overflow", line, "footer");
			result.error_message = require_string(
				value, "error_message", line, "footer", true);
			if (result.trace_overflow && result.trace_valid)
			{
				fail(
					DecisionTraceErrorCode::kInvalidValue,
					line,
					"overflowed Decision Trace cannot be valid");
			}
			return result;
		}

		bool command_equal(
			const control_link_interfaces::msg::ControlCommand &left,
			const control_link_interfaces::msg::ControlCommand &right) noexcept
		{
			return left.source_stamp.sec == right.source_stamp.sec &&
				left.source_stamp.nanosec == right.source_stamp.nanosec &&
				left.source_id == right.source_id &&
				left.source_sequence == right.source_sequence &&
				left.mode == right.mode &&
				double_to_hex(left.linear_velocity_mps) == double_to_hex(right.linear_velocity_mps) &&
				double_to_hex(left.angular_velocity_radps) == double_to_hex(right.angular_velocity_radps);
		}

		bool source_status_equal(
			const DecisionSourceStatus &left,
			const DecisionSourceStatus &right) noexcept
		{
			return left.source_id == right.source_id &&
				left.accepted_count == right.accepted_count &&
				left.rejected_count == right.rejected_count &&
				left.last_reject_reason == right.last_reject_reason &&
				left.last_accepted_sequence == right.last_accepted_sequence &&
				left.command_valid == right.command_valid &&
				left.lease_valid == right.lease_valid &&
				left.command_age_ns == right.command_age_ns;
		}

		bool recovery_candidate_equal(
			const std::optional<RecoveryCandidateKey> &left,
			const std::optional<RecoveryCandidateKey> &right) noexcept
		{
			if (left.has_value() != right.has_value())
			{
				return false;
			}
			return !left.has_value() ||
				(left->source_id == right->source_id &&
				 left->publisher_generation == right->publisher_generation);
		}
	}  // namespace

	DecisionTraceError::DecisionTraceError(
		DecisionTraceErrorCode code,
		std::size_t line,
		std::string message)
		: std::runtime_error(
			line == 0U ? std::move(message) :
				"Decision Trace line " + std::to_string(line) + ": " + message),
		  code_(code),
		  line_(line)
	{
	}

	DecisionTraceErrorCode DecisionTraceError::code() const noexcept
	{
		return code_;
	}

	std::size_t DecisionTraceError::line() const noexcept
	{
		return line_;
	}

	std::string serialize_decision_trace_record(const DecisionTraceRecord &record)
	{
		return std::visit(
			[](const auto &typed_record)
			{
				using Record = std::decay_t<decltype(typed_record)>;
				if constexpr (std::is_same_v<Record, DecisionTraceHeader>)
				{
					return header_json(typed_record).dump();
				}
				else if constexpr (std::is_same_v<Record, DecisionEvent>)
				{
					return event_json(typed_record).dump();
				}
				else if constexpr (std::is_same_v<Record, DecisionResult>)
				{
					return result_json(typed_record).dump();
				}
				else
				{
					return footer_json(typed_record).dump();
				}
			},
			record);
	}

	DecisionTraceRecord parse_decision_trace_record(
		const std::string &line,
		std::size_t line_number)
	{
		if (line.empty())
		{
			fail(DecisionTraceErrorCode::kInvalidJson, line_number, "blank lines are not allowed");
		}

		try
		{
			const auto value = Json::parse(line);
			if (!value.is_object())
			{
				fail(
					DecisionTraceErrorCode::kWrongType,
					line_number,
					"trace record must be a JSON object");
			}
			const auto record_type = require_string(
				value, "record_type", line_number, "record");
			if (record_type == "header") return parse_header(value, line_number);
			if (record_type == "event") return parse_event(value, line_number);
			if (record_type == "result") return parse_result(value, line_number);
			if (record_type == "footer") return parse_footer(value, line_number);
			fail(
				DecisionTraceErrorCode::kInvalidValue,
				line_number,
				"record.record_type is unsupported");
		}
		catch (const DecisionTraceError &)
		{
			throw;
		}
		catch (const nlohmann::json::exception &exception)
		{
			fail(
				DecisionTraceErrorCode::kInvalidJson,
				line_number,
				std::string("invalid JSON: ") + exception.what());
		}
	}

	DecisionTraceDocument read_decision_trace(std::istream &input)
	{
		DecisionTraceDocument document;
		bool saw_header = false;
		bool saw_footer = false;
		bool waiting_for_result = false;
		std::uint64_t expected_sequence = 1U;
		std::uint64_t result_count = 0U;
		std::string line;
		std::size_t line_number = 0U;
		while (std::getline(input, line))
		{
			line_number += 1U;
			const auto record = parse_decision_trace_record(line, line_number);
			if (!saw_header)
			{
				if (!std::holds_alternative<DecisionTraceHeader>(record))
				{
					fail(
						DecisionTraceErrorCode::kRecordOrder,
						line_number,
						"first Decision Trace record must be the header");
				}
				document.header = std::get<DecisionTraceHeader>(record);
				saw_header = true;
				continue;
			}

			if (saw_footer)
			{
				fail(
					DecisionTraceErrorCode::kRecordOrder,
					line_number,
					"records after the Decision Trace footer are forbidden");
			}

			if (std::holds_alternative<DecisionTraceHeader>(record))
			{
				fail(
					DecisionTraceErrorCode::kRecordOrder,
					line_number,
					"Decision Trace contains a second header");
			}
			if (std::holds_alternative<DecisionEvent>(record))
			{
				if (waiting_for_result)
				{
					fail(
						DecisionTraceErrorCode::kTruncated,
						line_number,
						"output_tick is missing its Decision Result");
				}
				auto event = std::get<DecisionEvent>(record);
				if (event.event_sequence != expected_sequence)
				{
					fail(
						DecisionTraceErrorCode::kSequence,
						line_number,
						"event_sequence is missing, duplicated, or out of order");
				}
				waiting_for_result = std::holds_alternative<DecisionOutputTickEvent>(
					event.payload);
				document.frames.push_back(DecisionTraceFrame{std::move(event), std::nullopt});
				expected_sequence += 1U;
				continue;
			}
			if (std::holds_alternative<DecisionResult>(record))
			{
				if (!waiting_for_result || document.frames.empty())
				{
					fail(
						DecisionTraceErrorCode::kRecordOrder,
						line_number,
						"Decision Result must immediately follow one output_tick");
				}
				auto result = std::get<DecisionResult>(record);
				if (result.event_sequence != document.frames.back().event.event_sequence)
				{
					fail(
						DecisionTraceErrorCode::kSequence,
						line_number,
						"Decision Result sequence does not match its output_tick");
				}
				document.frames.back().expected_result = std::move(result);
				waiting_for_result = false;
				result_count += 1U;
				continue;
			}

			if (waiting_for_result)
			{
				fail(
					DecisionTraceErrorCode::kTruncated,
					line_number,
					"output_tick is missing its Decision Result before the footer");
			}
			document.footer = std::get<DecisionTraceFooter>(record);
			saw_footer = true;
			const auto expected_last = document.frames.empty() ? 0U :
				document.frames.back().event.event_sequence;
			if (document.footer.last_event_sequence != expected_last ||
				document.footer.event_count != document.frames.size() ||
				document.footer.result_count != result_count)
			{
				fail(
					DecisionTraceErrorCode::kSequence,
					line_number,
					"Decision Trace footer counts do not match parsed records");
			}
		}

		if (!input.eof() && input.fail())
		{
			fail(DecisionTraceErrorCode::kIo, line_number, "failed while reading Decision Trace");
		}
		if (!saw_header)
		{
			fail(DecisionTraceErrorCode::kTruncated, 0U, "Decision Trace is missing its header");
		}
		if (waiting_for_result)
		{
			fail(
				DecisionTraceErrorCode::kTruncated,
				line_number,
				"Decision Trace ended before its output result");
		}
		if (!saw_footer)
		{
			fail(
				DecisionTraceErrorCode::kTruncated,
				line_number,
				"Decision Trace ended without a footer");
		}
		return document;
	}

	DecisionTraceDocument read_decision_trace_file(const std::filesystem::path &path)
	{
		std::ifstream input{path, std::ios::binary};
		if (!input)
		{
			throw DecisionTraceError(
				DecisionTraceErrorCode::kIo,
				0U,
				"failed to open Decision Trace for reading: " + path.string());
		}
		return read_decision_trace(input);
	}

	bool decision_results_equal(
		const DecisionResult &left,
		const DecisionResult &right) noexcept
	{
		if (left.event_sequence != right.event_sequence ||
			left.state != right.state || left.reason != right.reason ||
			!recovery_candidate_equal(left.recovery_candidate, right.recovery_candidate) ||
			left.recovery_valid_count != right.recovery_valid_count ||
			left.transition_sequence != right.transition_sequence ||
			left.sources.size() != right.sources.size() ||
			!command_equal(left.canonical_command, right.canonical_command) ||
			left.lifecycle_error_requested != right.lifecycle_error_requested)
		{
			return false;
		}
		for (std::size_t index = 0U; index < left.sources.size(); ++index)
		{
			if (!source_status_equal(left.sources[index], right.sources[index]))
			{
				return false;
			}
		}
		return true;
	}

	std::string describe_first_decision_difference(
		const DecisionResult &expected,
		const DecisionResult &actual)
	{
		if (expected.event_sequence != actual.event_sequence) return "event_sequence";
		if (expected.state != actual.state) return "state";
		if (expected.reason != actual.reason) return "reason";
		if (!recovery_candidate_equal(expected.recovery_candidate, actual.recovery_candidate))
			return "recovery_candidate";
		if (expected.recovery_valid_count != actual.recovery_valid_count)
			return "recovery_valid_count";
		if (expected.transition_sequence != actual.transition_sequence)
			return "transition_sequence";
		if (expected.sources.size() != actual.sources.size()) return "sources.size";
		for (std::size_t index = 0U; index < expected.sources.size(); ++index)
		{
			if (!source_status_equal(expected.sources[index], actual.sources[index]))
			{
				return "sources[" + std::to_string(index) + "]";
			}
		}
		if (!command_equal(expected.canonical_command, actual.canonical_command))
			return "canonical_command";
		if (expected.lifecycle_error_requested != actual.lifecycle_error_requested)
			return "lifecycle_error_requested";
		return "none";
	}
}  // namespace control_link_gateway
