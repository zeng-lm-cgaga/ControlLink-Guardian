#include "control_link_contract/config_identity.hpp"

#include <algorithm>
#include <array>
#include <cstring>
#include <fstream>
#include <iomanip>
#include <iterator>
#include <limits>
#include <locale>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <string_view>
#include <utility>
#include <vector>

#include <openssl/evp.h>

namespace control_link_contract
{
	namespace
	{
		static_assert(
			std::numeric_limits<double>::is_iec559 && sizeof(double) == sizeof(std::uint64_t),
			"Config identity requires IEEE-754 binary64 double representation");

		class CanonicalWriter final
		{
		public:
			void string(std::string_view name, std::string_view value)
			{
				buffer_.append(name);
				buffer_.append("|string|");
				buffer_.append(std::to_string(value.size()));
				buffer_.push_back('|');
				buffer_.append(value);
				buffer_.push_back('\n');
			}

			void unsigned_integer(std::string_view name, std::uint64_t value)
			{
				buffer_.append(name);
				buffer_.append("|uint|");
				buffer_.append(std::to_string(value));
				buffer_.push_back('\n');
			}

			void boolean(std::string_view name, bool value)
			{
				buffer_.append(name);
				buffer_.append(value ? "|bool|1\n" : "|bool|0\n");
			}

			void floating_point(std::string_view name, double value)
			{
				// 正负零在当前配置语义中等价，统一为 +0 后再编码 IEEE-754 bit pattern
				if (value == 0.0)
				{
					value = 0.0;
				}
				std::uint64_t bits = 0U;
				std::memcpy(&bits, &value, sizeof(bits));

				std::ostringstream encoded;
				encoded.imbue(std::locale::classic());
				encoded << std::hex << std::setfill('0') << std::setw(16) << bits;
				string(name, encoded.str());
			}

			[[nodiscard]] std::string finish() &&
			{
				return std::move(buffer_);
			}

		private:
			std::string buffer_;
		};

		std::string reliability_name(ReliabilityPolicy policy)
		{
			switch (policy)
			{
				case ReliabilityPolicy::kReliable:
					return "reliable";
				case ReliabilityPolicy::kBestEffort:
					return "best_effort";
			}
			throw std::logic_error("unsupported ReliabilityPolicy in canonical serialization");
		}

		std::string durability_name(DurabilityPolicy policy)
		{
			switch (policy)
			{
				case DurabilityPolicy::kVolatile:
					return "volatile";
				case DurabilityPolicy::kTransientLocal:
					return "transient_local";
			}
			throw std::logic_error("unsupported DurabilityPolicy in canonical serialization");
		}

		std::string history_name(HistoryPolicy policy)
		{
			switch (policy)
			{
				case HistoryPolicy::kKeepLast:
					return "keep_last";
				case HistoryPolicy::kKeepAll:
					return "keep_all";
			}
			throw std::logic_error("unsupported HistoryPolicy in canonical serialization");
		}

		std::string liveliness_name(LivelinessPolicy policy)
		{
			switch (policy)
			{
				case LivelinessPolicy::kAutomatic:
					return "automatic";
				case LivelinessPolicy::kManualByTopic:
					return "manual_by_topic";
			}
			throw std::logic_error("unsupported LivelinessPolicy in canonical serialization");
		}

		std::string remote_direction_name(RemoteDirection direction)
		{
			switch (direction)
			{
				case RemoteDirection::kPublisher:
					return "publisher";
				case RemoteDirection::kSubscription:
					return "subscription";
			}
			throw std::logic_error("unsupported RemoteDirection in canonical serialization");
		}

		std::string runtime_loss_action_name(RuntimeLossAction action)
		{
			if (action == RuntimeLossAction::kSafeStop)
			{
				return "safe_stop";
			}
			throw std::logic_error("unsupported RuntimeLossAction in canonical serialization");
		}

		std::string runtime_qos_action_name(RuntimeQosMismatchAction action)
		{
			if (action == RuntimeQosMismatchAction::kDegraded)
			{
				return "degraded";
			}
			throw std::logic_error(
				"unsupported RuntimeQosMismatchAction in canonical serialization");
		}

		std::string clock_mode_name(ClockMode mode)
		{
			switch (mode)
			{
				case ClockMode::kSim:
					return "sim";
				case ClockMode::kSystem:
					return "system";
			}
			throw std::logic_error("unsupported ClockMode in canonical serialization");
		}

		std::string can_byte_order_name(CanByteOrder byte_order)
		{
			if (byte_order == CanByteOrder::kLittleEndian)
			{
				return "little_endian";
			}
			throw std::logic_error("unsupported CanByteOrder in canonical serialization");
		}

		std::string can_crc_algorithm_name(CanCrcAlgorithm algorithm)
		{
			if (algorithm == CanCrcAlgorithm::kCrc8SaeJ1850)
			{
				return "crc8_sae_j1850";
			}
			throw std::logic_error("unsupported CanCrcAlgorithm in canonical serialization");
		}

		template<typename T>
		void optional_unsigned(
			CanonicalWriter &writer,
			std::string_view name,
			const std::optional<T> &value)
		{
			writer.boolean(std::string{name} + ".present", value.has_value());
			if (value.has_value())
			{
				writer.unsigned_integer(std::string{name} + ".value", *value);
			}
		}

		void serialize_qos(CanonicalWriter &writer, const QosProfile &qos)
		{
			writer.string("qos.reliability", reliability_name(qos.reliability));
			writer.string("qos.durability", durability_name(qos.durability));
			writer.string("qos.history", history_name(qos.history));
			// KeepAll 不消费 depth，不能让一个失效字段制造语义哈希变化
			writer.boolean("qos.depth.present", qos.history == HistoryPolicy::kKeepLast);
			if (qos.history == HistoryPolicy::kKeepLast)
			{
				writer.unsigned_integer("qos.depth.value", qos.depth);
			}
			optional_unsigned(writer, "qos.deadline_ms", qos.deadline_ms);
			optional_unsigned(writer, "qos.lifespan_ms", qos.lifespan_ms);
			writer.boolean("qos.liveliness.present", qos.liveliness.has_value());
			if (qos.liveliness.has_value())
			{
				writer.string("qos.liveliness.value", liveliness_name(*qos.liveliness));
			}
			optional_unsigned(
				writer,
				"qos.liveliness_lease_duration_ms",
				qos.liveliness_lease_duration_ms);
		}

		void serialize_profile_common(CanonicalWriter &writer, const ProfileCommon &common)
		{
			writer.unsigned_integer("profile.schema_version", common.schema_version);
			writer.string("profile.clock_mode", clock_mode_name(common.clock_mode));
			writer.boolean("profile.use_sim_time", common.use_sim_time);

			auto enabled_sources = common.enabled_sources;
			std::sort(enabled_sources.begin(), enabled_sources.end());
			writer.unsigned_integer("profile.enabled_sources.count", enabled_sources.size());
			for (const auto &source_id : enabled_sources)
			{
				writer.string("profile.enabled_sources.id", source_id);
			}

			writer.unsigned_integer("profile.ingress.count", common.ingress.size());
			for (const auto &[source_id, binding] : common.ingress)
			{
				writer.string("profile.ingress.source_id", source_id);
				writer.string("profile.ingress.input_topic", binding.input_topic);
				writer.string("profile.ingress.input_type", binding.input_type);
				writer.string("profile.ingress.output_topic", binding.output_topic);
			}
		}

		void serialize_physical_signal(
			CanonicalWriter &writer,
			std::string_view prefix,
			const CanPhysicalSignalConfig &signal)
		{
			const std::string name{prefix};
			writer.floating_point(name + ".scale", signal.scale);
			writer.floating_point(name + ".offset", signal.offset);
			writer.floating_point(name + ".minimum", signal.minimum);
			writer.floating_point(name + ".maximum", signal.maximum);
		}

		void serialize_can_signal_map(CanonicalWriter &writer, const CanSignalMap &map)
		{
			writer.unsigned_integer("can.schema_version", map.schema_version);
			writer.string("can.protocol_id", map.protocol_id);
			writer.string("can.byte_order", can_byte_order_name(map.byte_order));
			writer.unsigned_integer("can.dlc", map.dlc);
			writer.string("can.crc.algorithm", can_crc_algorithm_name(map.crc.algorithm));
			writer.unsigned_integer("can.crc.polynomial", map.crc.polynomial);
			writer.unsigned_integer("can.crc.initial_value", map.crc.initial_value);
			writer.unsigned_integer("can.crc.final_xor", map.crc.final_xor);
			writer.boolean("can.crc.reflect_input", map.crc.reflect_input);
			writer.boolean("can.crc.reflect_output", map.crc.reflect_output);
			writer.boolean("can.crc.include_can_id_lsb_first", map.crc.include_can_id_lsb_first);
			writer.unsigned_integer(
				"can.crc.protected_payload_bytes.count",
				map.crc.protected_payload_bytes.size());
			for (const auto byte : map.crc.protected_payload_bytes)
			{
				writer.unsigned_integer("can.crc.protected_payload_bytes.value", byte);
			}
			writer.unsigned_integer("can.counter.bit_length", map.rolling_counter.bit_length);
			writer.unsigned_integer("can.counter.modulo", map.rolling_counter.modulo);
			writer.boolean(
				"can.counter.accept_any_first_value",
				map.rolling_counter.accept_any_first_value);
			writer.boolean("can.counter.duplicate_is_error", map.rolling_counter.duplicate_is_error);
			writer.boolean("can.counter.jump_is_error", map.rolling_counter.jump_is_error);
			writer.unsigned_integer("can.control.can_id", map.control_frame.can_id);
			serialize_physical_signal(
				writer, "can.control.target_speed_mps", map.control_frame.target_speed_mps);
			serialize_physical_signal(
				writer, "can.control.target_yaw_rate_radps", map.control_frame.target_yaw_rate_radps);
			writer.unsigned_integer("can.state.can_id", map.state_frame.can_id);
			serialize_physical_signal(
				writer, "can.state.measured_speed_mps", map.state_frame.measured_speed_mps);
			serialize_physical_signal(
				writer, "can.state.measured_yaw_rate_radps", map.state_frame.measured_yaw_rate_radps);
		}

		void serialize_effective_source_policy(
			CanonicalWriter &writer,
			const SourcePolicy &source_policy,
			const ProfileConfig &profile)
		{
			writer.unsigned_integer("source_policy.schema_version", source_policy.schema_version);
			const auto &common = std::visit(
				[](const auto &selected_profile) -> const ProfileCommon &
				{
					return selected_profile.common;
				},
				profile);
			auto enabled_sources = common.enabled_sources;
			std::sort(enabled_sources.begin(), enabled_sources.end());
			writer.unsigned_integer("source_policy.sources.count", enabled_sources.size());
			for (const auto &source_id : enabled_sources)
			{
				// Bundle 跨文件校验保证每个 enabled source 都有唯一 SourcePolicy 条目
				const auto &source = source_policy.sources.at(source_id);
				writer.string("source_policy.source.id", source_id);
				writer.string("source_policy.source.topic", source.topic);
				writer.string("source_policy.source.type", source.type);
				writer.unsigned_integer("source_policy.source.priority", source.priority);
				writer.unsigned_integer(
					"source_policy.source.lease_timeout_ms",
					source.lease_timeout_ms);
				writer.boolean(
					"source_policy.source.required_for_activation",
					source.required_for_activation);
			}
		}

		void serialize_profile_behavior(
			CanonicalWriter &writer,
			const ProfileConfig &profile)
		{
			if (const auto *robot = std::get_if<RobotProfile>(&profile))
			{
				writer.string("profile.id", "robot");
				serialize_profile_common(writer, robot->common);
				writer.floating_point(
					"robot.geometry.wheel_separation_m",
					robot->geometry.wheel_separation_m);
				writer.floating_point(
					"robot.geometry.wheel_radius_m",
					robot->geometry.wheel_radius_m);
				writer.string(
					"robot.adapter.canonical_input_topic",
					robot->adapter.canonical_input_topic);
				writer.string(
					"robot.adapter.controller_manager_fqn",
					robot->adapter.controller_manager_fqn);
				writer.string(
					"robot.adapter.controller_node_fqn",
					robot->adapter.controller_node_fqn);
				writer.string(
					"robot.adapter.hardware_component_name",
					robot->adapter.hardware_component_name);
				writer.string(
					"robot.adapter.controller_output_topic",
					robot->adapter.controller_output_topic);
				writer.string(
					"robot.adapter.controller_command_type",
					robot->adapter.controller_command_type);
				writer.string(
					"robot.adapter.odometry_topic",
					robot->adapter.odometry_topic);
				writer.unsigned_integer(
					"robot.adapter.local_watchdog_timeout_ms",
					robot->adapter.local_watchdog_timeout_ms);
				writer.string("robot.frames.map", robot->frames.map);
				writer.string("robot.frames.odom", robot->frames.odom);
				writer.string(
					"robot.frames.base_footprint",
					robot->frames.base_footprint);
				writer.string("robot.frames.base_link", robot->frames.base_link);
				writer.string("robot.frames.laser", robot->frames.laser);
				writer.unsigned_integer(
					"robot.health.vehicle_state_publish_period_ms",
					robot->health.vehicle_state_publish_period_ms);
				writer.unsigned_integer(
					"robot.health.tf_lookup_timeout_ms",
					robot->health.tf_lookup_timeout_ms);
				writer.unsigned_integer(
					"robot.health.tf_max_age_ms",
					robot->health.tf_max_age_ms);
				writer.unsigned_integer(
					"robot.health.controller_state_timeout_ms",
					robot->health.controller_state_timeout_ms);
				writer.unsigned_integer(
					"robot.health.odometry_timeout_ms",
					robot->health.odometry_timeout_ms);
				return;
			}

			const auto &adas = std::get<AdasProfile>(profile);
			writer.string("profile.id", "adas");
			serialize_profile_common(writer, adas.common);
			writer.string(
				"adas.adapter.canonical_input_topic",
				adas.adapter.canonical_input_topic);
			writer.string(
				"adas.adapter.vehicle_state_output_topic",
				adas.adapter.vehicle_state_output_topic);
			writer.string("adas.adapter.interface", adas.adapter.interface);
			writer.unsigned_integer(
				"adas.adapter.poll_timeout_ms",
				adas.adapter.poll_timeout_ms);
			writer.unsigned_integer(
				"adas.adapter.tx_period_ms",
				adas.adapter.tx_period_ms);
			writer.unsigned_integer(
				"adas.adapter.vehicle_state_publish_period_ms",
				adas.adapter.vehicle_state_publish_period_ms);
			writer.unsigned_integer(
				"adas.adapter.local_watchdog_timeout_ms",
				adas.adapter.local_watchdog_timeout_ms);
			writer.unsigned_integer(
				"adas.adapter.can_state_frame_timeout_ms",
				adas.adapter.can_state_frame_timeout_ms);
			writer.unsigned_integer(
				"adas.adapter.recovery_valid_frames",
				adas.adapter.recovery_valid_frames);
			writer.unsigned_integer(
				"adas.vehicle_simulator.state_period_ms",
				adas.vehicle_simulator.state_period_ms);
			writer.unsigned_integer(
				"adas.vehicle_simulator.local_watchdog_timeout_ms",
				adas.vehicle_simulator.local_watchdog_timeout_ms);
			writer.unsigned_integer(
				"adas.vehicle_simulator.first_order_time_constant_ms",
				adas.vehicle_simulator.first_order_time_constant_ms);
		}

		std::string sha256_hex(std::string_view input)
		{
			using ContextPtr = std::unique_ptr<EVP_MD_CTX, decltype(&EVP_MD_CTX_free)>;
			ContextPtr context{EVP_MD_CTX_new(), &EVP_MD_CTX_free};
			if (!context || EVP_DigestInit_ex(context.get(), EVP_sha256(), nullptr) != 1 ||
				EVP_DigestUpdate(context.get(), input.data(), input.size()) != 1)
			{
				throw std::runtime_error("OpenSSL failed to initialize SHA-256 config identity");
			}

			std::array<unsigned char, EVP_MAX_MD_SIZE> digest{};
			unsigned int digest_size = 0U;
			if (EVP_DigestFinal_ex(context.get(), digest.data(), &digest_size) != 1 ||
				digest_size != 32U)
			{
				throw std::runtime_error("OpenSSL failed to finalize SHA-256 config identity");
			}

			constexpr char kHexDigits[] = "0123456789abcdef";
			std::string result;
			result.reserve(digest_size * 2U);
			for (unsigned int index = 0U; index < digest_size; ++index)
			{
				const auto byte = digest[index];
				result.push_back(kHexDigits[(byte >> 4U) & 0x0FU]);
				result.push_back(kHexDigits[byte & 0x0FU]);
			}
			return result;
		}
	}  // namespace

	std::string canonical_serialize_gateway_contract(const GatewayContract &contract)
	{
		CanonicalWriter writer;
		writer.string("format", "control_link.gateway_contract.canonical.v1");
		writer.unsigned_integer("schema_version", contract.schema_version);

		const auto &gateway = contract.gateway;
		writer.string("gateway.node_fqn", gateway.node_fqn);
		writer.floating_point("gateway.output_rate_hz", gateway.output_rate_hz);
		writer.unsigned_integer("gateway.command_timeout_ms", gateway.command_timeout_ms);
		writer.unsigned_integer("gateway.source_switch_hold_ms", gateway.source_switch_hold_ms);
		writer.unsigned_integer("gateway.recovery_valid_samples", gateway.recovery_valid_samples);
		writer.unsigned_integer("gateway.graph_poll_ms", gateway.graph_poll_ms);
		writer.unsigned_integer("gateway.graph_stable_window_ms", gateway.graph_stable_window_ms);
		writer.unsigned_integer(
			"gateway.ros_clock_stall_timeout_ms",
			gateway.ros_clock_stall_timeout_ms);
		writer.unsigned_integer(
			"gateway.vehicle_state_topic_timeout_ms",
			gateway.vehicle_state_topic_timeout_ms);
		writer.unsigned_integer(
			"gateway.output_tick_late_threshold_ms",
			gateway.output_tick_late_threshold_ms);
		writer.unsigned_integer(
			"gateway.consecutive_late_ticks_to_safe_stop",
			gateway.consecutive_late_ticks_to_safe_stop);

		writer.floating_point(
			"limits.max_abs_linear_velocity_mps",
			contract.limits.max_abs_linear_velocity_mps);
		writer.floating_point(
			"limits.max_abs_angular_velocity_radps",
			contract.limits.max_abs_angular_velocity_radps);
		writer.boolean("limits.reject_non_finite", contract.limits.reject_non_finite);
		writer.boolean("limits.reject_zero_stamp", contract.limits.reject_zero_stamp);
		writer.unsigned_integer("limits.max_future_skew_ms", contract.limits.max_future_skew_ms);

		writer.unsigned_integer("qos_profiles.count", contract.qos_profiles.size());
		for (const auto &[name, qos] : contract.qos_profiles)
		{
			writer.string("qos_profiles.name", name);
			serialize_qos(writer, qos);
		}

		writer.string("input.topic_prefix", contract.input.topic_prefix);
		writer.string("input.type", contract.input.type);
		writer.string("input.qos_profile", contract.input.qos_profile);
		writer.string("output.topic", contract.output.topic);
		writer.string("output.type", contract.output.type);
		writer.string("output.qos_profile", contract.output.qos_profile);

		writer.unsigned_integer("state_topics.count", contract.state_topics.size());
		for (const auto &[name, topic] : contract.state_topics)
		{
			writer.string("state_topics.name", name);
			writer.string("state_topics.topic", topic.topic);
			writer.string("state_topics.type", topic.type);
			writer.boolean("state_topics.qos_profile.present", topic.qos_profile.has_value());
			if (topic.qos_profile.has_value())
			{
				writer.string("state_topics.qos_profile.value", *topic.qos_profile);
			}
		}

		std::vector<const CriticalEndpoint *> endpoints;
		endpoints.reserve(contract.critical_endpoints.size());
		for (const auto &endpoint : contract.critical_endpoints)
		{
			endpoints.push_back(&endpoint);
		}
		std::sort(
			endpoints.begin(), endpoints.end(),
			[](const CriticalEndpoint *left, const CriticalEndpoint *right)
			{
				return left->id < right->id;
			});
		writer.unsigned_integer("critical_endpoints.count", endpoints.size());
		for (const auto *endpoint : endpoints)
		{
			writer.string("critical_endpoint.id", endpoint->id);
			writer.string("critical_endpoint.topic", endpoint->topic);
			writer.string("critical_endpoint.type", endpoint->type);
			writer.string(
				"critical_endpoint.remote_direction",
				remote_direction_name(endpoint->remote_direction));
			writer.string("critical_endpoint.remote_node_fqn", endpoint->remote_node_fqn);
			writer.unsigned_integer("critical_endpoint.min_count", endpoint->min_count);
			optional_unsigned(writer, "critical_endpoint.max_count", endpoint->max_count);
			writer.boolean(
				"critical_endpoint.allow_additional_endpoints",
				endpoint->allow_additional_endpoints);
			writer.boolean(
				"critical_endpoint.required_for_activation",
				endpoint->required_for_activation);
			writer.boolean("critical_endpoint.exact_qos_required", endpoint->exact_qos_required);
			writer.string(
				"critical_endpoint.runtime_loss_action",
				runtime_loss_action_name(endpoint->runtime_loss_action));
			writer.string(
				"critical_endpoint.runtime_qos_mismatch_action",
				runtime_qos_action_name(endpoint->runtime_qos_mismatch_action));
		}

		return std::move(writer).finish();
	}

	std::string canonical_serialize_decision_config(
		const GatewayContract &contract,
		const SourcePolicy &source_policy,
		const ProfileConfig &profile,
		const CanSignalMap *can_signal_map)
	{
		CanonicalWriter writer;
		writer.string("format", "control_link.decision_config.canonical.v1");
		writer.string(
			"gateway_contract",
			canonical_serialize_gateway_contract(contract));
		serialize_effective_source_policy(writer, source_policy, profile);

		if (std::holds_alternative<RobotProfile>(profile))
		{
			if (can_signal_map != nullptr)
			{
				throw std::invalid_argument("Robot decision config must not include a CAN signal map");
			}
		}
		else
		{
			if (can_signal_map == nullptr)
			{
				throw std::invalid_argument("ADAS decision config requires a CAN signal map");
			}
		}
		serialize_profile_behavior(writer, profile);
		if (can_signal_map != nullptr)
		{
			serialize_can_signal_map(writer, *can_signal_map);
		}

		// 文件绝对路径、FastDDS transport、record topics、场景资源和 demo goal 由 run manifest 单独追踪
		return std::move(writer).finish();
	}

	std::string canonical_serialize_effective_source_policy(
		const SourcePolicy &source_policy,
		const ProfileConfig &profile)
	{
		CanonicalWriter writer;
		writer.string("format", "control_link.source_policy.canonical.v1");
		serialize_effective_source_policy(writer, source_policy, profile);
		return std::move(writer).finish();
	}

	std::string canonical_serialize_profile_behavior(const ProfileConfig &profile)
	{
		CanonicalWriter writer;
		writer.string("format", "control_link.profile_behavior.canonical.v1");
		serialize_profile_behavior(writer, profile);
		return std::move(writer).finish();
	}

	std::string canonical_serialize_can_signal_map_behavior(
		const CanSignalMap &can_signal_map)
	{
		CanonicalWriter writer;
		writer.string("format", "control_link.can_signal_map.canonical.v1");
		serialize_can_signal_map(writer, can_signal_map);
		return std::move(writer).finish();
	}

	std::string sha256_file_content(const std::filesystem::path &path)
	{
		std::ifstream input{path, std::ios::binary};
		if (!input)
		{
			throw std::runtime_error(
				"cannot open file for SHA-256: " + path.string());
		}
		const std::string content{
			std::istreambuf_iterator<char>{input},
			std::istreambuf_iterator<char>{}};
		if (input.bad())
		{
			throw std::runtime_error(
				"cannot read file for SHA-256: " + path.string());
		}
		return sha256_hex(content);
	}

	RuntimeConfigIdentity make_runtime_config_identity(
		const GatewayContract &contract,
		const SourcePolicy &source_policy,
		const ProfileConfig &profile,
		const CanSignalMap *can_signal_map)
	{
		const auto contract_semantics = canonical_serialize_gateway_contract(contract);
		const auto decision_semantics = canonical_serialize_decision_config(
			contract,
			source_policy,
			profile,
			can_signal_map);
		const std::string profile_id = std::holds_alternative<RobotProfile>(profile) ?
			"robot" : "adas";

		return RuntimeConfigIdentity{
			ContractIdentity{
				contract.contract_id,
				contract.contract_version,
				sha256_hex(contract_semantics)},
			DecisionConfigIdentity{
				profile_id,
				sha256_hex(decision_semantics)}};
	}
}  // namespace control_link_contract
