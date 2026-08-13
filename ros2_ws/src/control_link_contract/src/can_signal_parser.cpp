#include "control_link_contract/parser.hpp"

#include <cmath>
#include <cstdint>
#include <fstream>
#include <initializer_list>
#include <limits>
#include <set>
#include <sstream>
#include <string>
#include <utility>

#include <yaml-cpp/yaml.h>

namespace control_link_contract
{
	namespace
	{
		class CanSignalMapParser final
		{
		public:
			explicit CanSignalMapParser(std::string source_name)
				: source_name_(std::move(source_name))
			{
			}

			CanSignalMapPtr parse(std::string_view yaml_text) const
			{
				const auto root = load_document(yaml_text);
				check_keys(
					root,
					{"schema_version", "protocol_id", "description", "byte_order", "dlc",
					 "crc", "rolling_counter", "frames", "validation"},
					"root");

				const auto schema_version = require_uint(root, "schema_version", "schema_version");
				expect_uint(schema_version, 1U, "schema_version", "unsupported schema version");
				const auto protocol_id = require_string(root, "protocol_id", "protocol_id");
				expect_string(
					protocol_id, "control_link_demo_can_v1", "protocol_id",
					"unsupported CAN protocol");
				const auto description = require_string(root, "description", "description");
				if (description.empty())
				{
					fail(
						"description", "empty description", "non-empty string", "empty",
						"描述该演示协议的使用边界");
				}
				const auto byte_order = require_string(root, "byte_order", "byte_order");
				expect_string(
					byte_order, "little_endian", "byte_order", "unsupported byte order");
				const auto dlc_value = require_uint(root, "dlc", "dlc");
				expect_uint(dlc_value, 8U, "dlc", "unsupported frame length");

				const auto crc = parse_crc(require_field(root, "crc", "crc"));
				const auto rolling_counter = parse_rolling_counter(
					require_field(root, "rolling_counter", "rolling_counter"));
				const auto frames = require_field(root, "frames", "frames");
				check_keys(frames, {"control_command", "vehicle_state"}, "frames");
				const auto control_frame = parse_control_frame(
					require_field(frames, "control_command", "frames.control_command"));
				const auto state_frame = parse_state_frame(
					require_field(frames, "vehicle_state", "frames.vehicle_state"));
				if (control_frame.can_id == state_frame.can_id)
				{
					fail(
						"frames", "duplicate CAN identifier", "distinct control and state IDs",
						std::to_string(control_frame.can_id),
						"为双向 frame 配置不同的 11-bit CAN ID");
				}
				parse_validation(require_field(root, "validation", "validation"));

				CanSignalMap result{
					schema_version,
					protocol_id,
					description,
					CanByteOrder::kLittleEndian,
					static_cast<std::uint8_t>(dlc_value),
					crc,
					rolling_counter,
					control_frame,
					state_frame};
				return std::make_shared<const CanSignalMap>(std::move(result));
			}

		private:
			YAML::Node load_document(std::string_view yaml_text) const
			{
				try
				{
					return YAML::Load(std::string(yaml_text));
				}
				catch (const YAML::Exception &error)
				{
					fail(
						"root", "invalid YAML", "valid YAML document", error.what(),
						"检查 CAN signal map 的 YAML 语法");
				}
			}

			[[noreturn]] void fail(
				const std::string &path,
				const std::string &rule,
				const std::string &expected,
				const std::string &actual,
				const std::string &hint) const
			{
				throw ContractError(
					source_name_ + ":" + path + ": " + rule + "; expected=" + expected +
					"; actual=" + actual + "; hint=" + hint);
			}

			static std::string child_path(
				const std::string &parent,
				const std::string &key)
			{
				return parent == "root" ? key : parent + "." + key;
			}

			static std::string describe(const YAML::Node &node)
			{
				if (!node || node.IsNull())
					return "missing";
				if (node.IsMap())
					return "map";
				if (node.IsSequence())
					return "sequence";
				if (node.IsScalar())
					return "scalar(" + node.Scalar() + ")";
				return "unknown";
			}

			void require_map(const YAML::Node &node, const std::string &path) const
			{
				if (!node || !node.IsMap())
				{
					fail(
						path, "wrong YAML type", "map", describe(node),
						"使用 key: value 结构");
				}
			}

			void check_keys(
				const YAML::Node &node,
				std::initializer_list<const char *> allowed,
				const std::string &path) const
			{
				require_map(node, path);
				std::set<std::string> allowed_keys;
				for (const auto *key : allowed)
					allowed_keys.emplace(key);
				std::set<std::string> seen_keys;
				for (const auto &entry : node)
				{
					if (!entry.first.IsScalar())
					{
						fail(
							path, "non-scalar key", "string key", describe(entry.first),
							"使用字符串字段名");
					}
					const auto key = entry.first.Scalar();
					if (allowed_keys.count(key) == 0U)
					{
						fail(
							child_path(path, key), "unknown field", "declared v1 schema key",
							describe(entry.second), "删除字段或先升级协议 parser");
					}
					if (!seen_keys.emplace(key).second)
					{
						fail(
							child_path(path, key), "duplicate field", "unique field", key,
							"删除重复字段，确保协议只有一个配置值");
					}
				}
			}

			YAML::Node require_field(
				const YAML::Node &parent,
				const std::string &key,
				const std::string &path) const
			{
				const YAML::Node value = parent[key];
				if (!value || value.IsNull())
				{
					fail(
						path, "missing required field", "present", "missing",
						"补充 CAN v1 必填字段");
				}
				return value;
			}

			std::string require_string(
				const YAML::Node &parent,
				const std::string &key,
				const std::string &path) const
			{
				const auto node = require_field(parent, key, path);
				if (!node.IsScalar())
				{
					fail(path, "wrong YAML type", "string", describe(node), "填写字符串值");
				}
				try
				{
					return node.as<std::string>();
				}
				catch (const YAML::Exception &error)
				{
					fail(path, "invalid string", "string", error.what(), "检查字段类型");
				}
			}

			std::uint64_t require_uint(
				const YAML::Node &parent,
				const std::string &key,
				const std::string &path) const
			{
				const auto node = require_field(parent, key, path);
				if (!node.IsScalar())
				{
					fail(
						path, "wrong YAML type", "non-negative integer", describe(node),
						"填写整数值");
				}
				try
				{
					const auto value = node.as<long long>();
					if (value < 0)
					{
						fail(
							path, "negative integer", "non-negative integer",
							std::to_string(value), "使用大于或等于 0 的整数");
					}
					return static_cast<std::uint64_t>(value);
				}
				catch (const YAML::Exception &error)
				{
					fail(
						path, "invalid integer", "non-negative integer", error.what(),
						"检查字段类型和数值范围");
				}
			}

			double require_double(
				const YAML::Node &parent,
				const std::string &key,
				const std::string &path) const
			{
				const auto node = require_field(parent, key, path);
				if (!node.IsScalar())
				{
					fail(path, "wrong YAML type", "finite number", describe(node), "填写数值");
				}
				try
				{
					const auto value = node.as<double>();
					if (!std::isfinite(value))
					{
						fail(
							path, "non-finite number", "finite number", node.Scalar(),
							"使用有限物理量参数");
					}
					return value;
				}
				catch (const YAML::Exception &error)
				{
					fail(path, "invalid number", "finite number", error.what(), "检查字段类型");
				}
			}

			bool require_bool(
				const YAML::Node &parent,
				const std::string &key,
				const std::string &path) const
			{
				const auto node = require_field(parent, key, path);
				if (!node.IsScalar())
				{
					fail(path, "wrong YAML type", "boolean", describe(node), "填写 true 或 false");
				}
				try
				{
					return node.as<bool>();
				}
				catch (const YAML::Exception &error)
				{
					fail(path, "invalid boolean", "true|false", error.what(), "填写 YAML boolean");
				}
			}

			void expect_uint(
				std::uint64_t actual,
				std::uint64_t expected,
				const std::string &path,
				const std::string &rule) const
			{
				if (actual != expected)
				{
					fail(
						path, rule, std::to_string(expected), std::to_string(actual),
						"恢复 control_link_demo_can_v1 规定的字段布局");
				}
			}

			void expect_string(
				const std::string &actual,
				const std::string &expected,
				const std::string &path,
				const std::string &rule) const
			{
				if (actual != expected)
				{
					fail(
						path, rule, expected, actual,
						"使用当前 codec 支持的 v1 协议值");
				}
			}

			void expect_bool(
				bool actual,
				bool expected,
				const std::string &path,
				const std::string &rule) const
			{
				if (actual != expected)
				{
					fail(
						path, rule, expected ? "true" : "false", actual ? "true" : "false",
						"恢复 control_link_demo_can_v1 规定的校验策略");
				}
			}

			std::uint8_t uint8_value(
				std::uint64_t value,
				const std::string &path) const
			{
				if (value > std::numeric_limits<std::uint8_t>::max())
				{
					fail(
						path, "integer exceeds uint8", "0..255", std::to_string(value),
						"使用一个字节可表示的协议参数");
				}
				return static_cast<std::uint8_t>(value);
			}

			CanCrcConfig parse_crc(const YAML::Node &node) const
			{
				check_keys(
					node,
					{"algorithm", "polynomial", "initial_value", "final_xor", "reflect_input",
					 "reflect_output", "include_can_id_lsb_first", "protected_payload_bytes"},
					"crc");
				expect_string(
					require_string(node, "algorithm", "crc.algorithm"), "crc8_sae_j1850",
					"crc.algorithm", "unsupported CRC algorithm");
				const auto polynomial = require_uint(node, "polynomial", "crc.polynomial");
				const auto initial = require_uint(node, "initial_value", "crc.initial_value");
				const auto final_xor = require_uint(node, "final_xor", "crc.final_xor");
				expect_uint(polynomial, 0x1DU, "crc.polynomial", "wrong SAE-J1850 polynomial");
				expect_uint(initial, 0xFFU, "crc.initial_value", "wrong CRC initial value");
				expect_uint(final_xor, 0xFFU, "crc.final_xor", "wrong CRC final xor");
				const auto reflect_input = require_bool(node, "reflect_input", "crc.reflect_input");
				const auto reflect_output = require_bool(node, "reflect_output", "crc.reflect_output");
				const auto include_can_id = require_bool(
					node, "include_can_id_lsb_first", "crc.include_can_id_lsb_first");
				expect_bool(reflect_input, false, "crc.reflect_input", "reflected CRC not supported");
				expect_bool(reflect_output, false, "crc.reflect_output", "reflected CRC not supported");
				expect_bool(
					include_can_id, true, "crc.include_can_id_lsb_first",
					"CAN ID protection is required");

				const auto protected_bytes = require_field(
					node, "protected_payload_bytes", "crc.protected_payload_bytes");
				if (!protected_bytes.IsSequence() || protected_bytes.size() != 7U)
				{
					fail(
						"crc.protected_payload_bytes", "wrong protected byte list",
						"sequence [0,1,2,3,4,5,6]", describe(protected_bytes),
						"保护 payload byte 0..6，CRC byte 7 不参与自身计算");
				}
				std::vector<std::uint8_t> parsed_bytes;
				for (std::size_t index = 0U; index < protected_bytes.size(); ++index)
				{
					std::uint64_t value;
					try
					{
						value = protected_bytes[index].as<std::uint64_t>();
					}
					catch (const YAML::Exception &error)
					{
						fail(
							"crc.protected_payload_bytes[" + std::to_string(index) + "]",
							"invalid byte index", std::to_string(index), error.what(),
							"按顺序填写 0 到 6");
					}
					expect_uint(
						value, index,
						"crc.protected_payload_bytes[" + std::to_string(index) + "]",
						"protected byte order mismatch");
					parsed_bytes.push_back(static_cast<std::uint8_t>(value));
				}

				return CanCrcConfig{
					CanCrcAlgorithm::kCrc8SaeJ1850,
					uint8_value(polynomial, "crc.polynomial"),
					uint8_value(initial, "crc.initial_value"),
					uint8_value(final_xor, "crc.final_xor"),
					reflect_input,
					reflect_output,
					include_can_id,
					std::move(parsed_bytes)};
			}

			CanRollingCounterConfig parse_rolling_counter(const YAML::Node &node) const
			{
				check_keys(
					node,
					{"bit_length", "modulo", "accept_any_first_value", "duplicate_is_error",
					 "jump_is_error"},
					"rolling_counter");
				const auto bit_length = require_uint(
					node, "bit_length", "rolling_counter.bit_length");
				const auto modulo = require_uint(node, "modulo", "rolling_counter.modulo");
				expect_uint(bit_length, 4U, "rolling_counter.bit_length", "wrong counter width");
				expect_uint(modulo, 16U, "rolling_counter.modulo", "wrong counter modulo");
				const auto accept_first = require_bool(
					node, "accept_any_first_value", "rolling_counter.accept_any_first_value");
				const auto duplicate_error = require_bool(
					node, "duplicate_is_error", "rolling_counter.duplicate_is_error");
				const auto jump_error = require_bool(
					node, "jump_is_error", "rolling_counter.jump_is_error");
				expect_bool(
					accept_first, true, "rolling_counter.accept_any_first_value",
					"first counter must establish the baseline");
				expect_bool(
					duplicate_error, true, "rolling_counter.duplicate_is_error",
					"duplicate counter must be rejected");
				expect_bool(
					jump_error, true, "rolling_counter.jump_is_error",
					"counter jump must be rejected");
				return CanRollingCounterConfig{
					uint8_value(bit_length, "rolling_counter.bit_length"),
					uint8_value(modulo, "rolling_counter.modulo"),
					accept_first,
					duplicate_error,
					jump_error};
			}

			void validate_layout(
				const YAML::Node &node,
				const std::string &path,
				std::uint64_t expected_byte,
				std::uint64_t expected_bit,
				std::uint64_t expected_length,
				bool expected_signed) const
			{
				expect_uint(
					require_uint(node, "byte_offset", child_path(path, "byte_offset")),
					expected_byte, child_path(path, "byte_offset"), "wrong signal byte offset");
				expect_uint(
					require_uint(node, "bit_offset", child_path(path, "bit_offset")),
					expected_bit, child_path(path, "bit_offset"), "wrong signal bit offset");
				expect_uint(
					require_uint(node, "bit_length", child_path(path, "bit_length")),
					expected_length, child_path(path, "bit_length"), "wrong signal bit length");
				expect_bool(
					require_bool(node, "signed", child_path(path, "signed")),
					expected_signed, child_path(path, "signed"), "wrong signal signedness");
			}

			CanPhysicalSignalConfig parse_physical_signal(
				const YAML::Node &node,
				const std::string &path,
				std::uint64_t expected_byte) const
			{
				check_keys(
					node,
					{"byte_offset", "bit_offset", "bit_length", "signed", "scale", "offset",
					 "minimum", "maximum"},
					path);
				validate_layout(node, path, expected_byte, 0U, 16U, true);
				const auto scale = require_double(node, "scale", child_path(path, "scale"));
				const auto offset = require_double(node, "offset", child_path(path, "offset"));
				const auto minimum = require_double(node, "minimum", child_path(path, "minimum"));
				const auto maximum = require_double(node, "maximum", child_path(path, "maximum"));
				if (scale <= 0.0 || offset != 0.0 || minimum > maximum ||
					std::round(minimum / scale) < -32768.0 ||
					std::round(maximum / scale) > 32767.0)
				{
					fail(
						path, "invalid physical signal quantization",
						"positive scale, zero offset, ordered range fitting int16",
						"scale=" + std::to_string(scale) + ",offset=" + std::to_string(offset) +
						",minimum=" + std::to_string(minimum) +
						",maximum=" + std::to_string(maximum),
						"修正 scale/range，避免浮点转 int16 越界");
				}
				return CanPhysicalSignalConfig{scale, offset, minimum, maximum};
			}

			void parse_enum_signal(
				const YAML::Node &node,
				const std::string &path,
				std::uint64_t expected_byte,
				std::initializer_list<std::pair<const char *, std::uint64_t>> entries) const
			{
				check_keys(
					node, {"byte_offset", "bit_offset", "bit_length", "signed", "enum"}, path);
				validate_layout(node, path, expected_byte, 0U, 8U, false);
				const auto enum_node = require_field(node, "enum", child_path(path, "enum"));
				std::set<std::string> expected_keys;
				for (const auto &entry : entries)
					expected_keys.emplace(entry.first);
				require_map(enum_node, child_path(path, "enum"));
				if (enum_node.size() != entries.size())
				{
					fail(
						child_path(path, "enum"), "wrong enum entry count",
						std::to_string(entries.size()), std::to_string(enum_node.size()),
						"恢复 v1 mode 枚举集合");
				}
				for (const auto &entry : entries)
				{
					const auto key = std::string(entry.first);
					expect_uint(
						require_uint(enum_node, key, child_path(child_path(path, "enum"), key)),
						entry.second, child_path(child_path(path, "enum"), key),
						"wrong enum value");
				}
				for (const auto &entry : enum_node)
				{
					if (!entry.first.IsScalar() || expected_keys.count(entry.first.Scalar()) == 0U)
					{
						fail(
							child_path(path, "enum"), "unknown enum value", "v1 mode enum",
							describe(entry.first), "删除未实现的 mode");
					}
				}
			}

			void parse_unsigned_signal(
				const YAML::Node &node,
				const std::string &path,
				std::uint64_t expected_byte,
				std::uint64_t expected_bit,
				std::uint64_t expected_length) const
			{
				check_keys(node, {"byte_offset", "bit_offset", "bit_length", "signed"}, path);
				validate_layout(
					node, path, expected_byte, expected_bit, expected_length, false);
			}

			void parse_constant_signal(
				const YAML::Node &node,
				const std::string &path,
				std::uint64_t expected_byte,
				std::uint64_t expected_bit,
				std::uint64_t expected_length) const
			{
				check_keys(node, {"byte_offset", "bit_offset", "bit_length", "constant"}, path);
				expect_uint(
					require_uint(node, "byte_offset", child_path(path, "byte_offset")),
					expected_byte, child_path(path, "byte_offset"), "wrong constant byte offset");
				expect_uint(
					require_uint(node, "bit_offset", child_path(path, "bit_offset")),
					expected_bit, child_path(path, "bit_offset"), "wrong constant bit offset");
				expect_uint(
					require_uint(node, "bit_length", child_path(path, "bit_length")),
					expected_length, child_path(path, "bit_length"), "wrong constant bit length");
				expect_uint(
					require_uint(node, "constant", child_path(path, "constant")),
					0U, child_path(path, "constant"), "reserved bits must be zero");
			}

			void parse_crc_signal(
				const YAML::Node &node,
				const std::string &path) const
			{
				parse_unsigned_signal(node, path, 7U, 0U, 8U);
			}

			CanControlFrameConfig parse_control_frame(const YAML::Node &node) const
			{
				const std::string path = "frames.control_command";
				check_keys(node, {"can_id", "direction", "signals"}, path);
				const auto can_id = require_uint(node, "can_id", child_path(path, "can_id"));
				if (can_id > 0x7FFU)
				{
					fail(
						child_path(path, "can_id"), "extended CAN identifier", "0..0x7FF",
						std::to_string(can_id), "使用 11-bit standard CAN ID");
				}
				expect_string(
					require_string(node, "direction", child_path(path, "direction")),
					"gateway_to_vehicle", child_path(path, "direction"),
					"wrong control frame direction");
				const auto signals = require_field(node, "signals", child_path(path, "signals"));
				check_keys(
					signals,
					{"target_speed_mps", "target_yaw_rate_radps", "command_mode",
					 "rolling_counter", "reserved_counter_high_nibble", "command_valid",
					 "reserved_flags", "crc"},
					child_path(path, "signals"));
				const auto speed = parse_physical_signal(
					require_field(signals, "target_speed_mps", path + ".signals.target_speed_mps"),
					path + ".signals.target_speed_mps", 0U);
				const auto yaw = parse_physical_signal(
					require_field(
						signals, "target_yaw_rate_radps", path + ".signals.target_yaw_rate_radps"),
					path + ".signals.target_yaw_rate_radps", 2U);
				parse_enum_signal(
					require_field(signals, "command_mode", path + ".signals.command_mode"),
					path + ".signals.command_mode", 4U, {{"hold", 0U}, {"active", 1U}});
				parse_unsigned_signal(
					require_field(signals, "rolling_counter", path + ".signals.rolling_counter"),
					path + ".signals.rolling_counter", 5U, 0U, 4U);
				parse_constant_signal(
					require_field(
						signals, "reserved_counter_high_nibble",
						path + ".signals.reserved_counter_high_nibble"),
					path + ".signals.reserved_counter_high_nibble", 5U, 4U, 4U);
				parse_unsigned_signal(
					require_field(signals, "command_valid", path + ".signals.command_valid"),
					path + ".signals.command_valid", 6U, 0U, 1U);
				parse_constant_signal(
					require_field(signals, "reserved_flags", path + ".signals.reserved_flags"),
					path + ".signals.reserved_flags", 6U, 1U, 7U);
				parse_crc_signal(
					require_field(signals, "crc", path + ".signals.crc"),
					path + ".signals.crc");
				return CanControlFrameConfig{static_cast<std::uint32_t>(can_id), speed, yaw};
			}

			CanStateFrameConfig parse_state_frame(const YAML::Node &node) const
			{
				const std::string path = "frames.vehicle_state";
				check_keys(node, {"can_id", "direction", "signals"}, path);
				const auto can_id = require_uint(node, "can_id", child_path(path, "can_id"));
				if (can_id > 0x7FFU)
				{
					fail(
						child_path(path, "can_id"), "extended CAN identifier", "0..0x7FF",
						std::to_string(can_id), "使用 11-bit standard CAN ID");
				}
				expect_string(
					require_string(node, "direction", child_path(path, "direction")),
					"vehicle_to_gateway", child_path(path, "direction"),
					"wrong state frame direction");
				const auto signals = require_field(node, "signals", child_path(path, "signals"));
				check_keys(
					signals,
					{"measured_speed_mps", "measured_yaw_rate_radps", "vehicle_mode",
					 "fault_code", "state_counter", "echoed_control_counter", "crc"},
					child_path(path, "signals"));
				const auto speed = parse_physical_signal(
					require_field(signals, "measured_speed_mps", path + ".signals.measured_speed_mps"),
					path + ".signals.measured_speed_mps", 0U);
				const auto yaw = parse_physical_signal(
					require_field(
						signals, "measured_yaw_rate_radps", path + ".signals.measured_yaw_rate_radps"),
					path + ".signals.measured_yaw_rate_radps", 2U);
				parse_enum_signal(
					require_field(signals, "vehicle_mode", path + ".signals.vehicle_mode"),
					path + ".signals.vehicle_mode", 4U,
					{{"standby", 0U}, {"running", 1U}, {"safe_stop", 2U}, {"fault", 3U}});
				parse_unsigned_signal(
					require_field(signals, "fault_code", path + ".signals.fault_code"),
					path + ".signals.fault_code", 5U, 0U, 8U);
				parse_unsigned_signal(
					require_field(signals, "state_counter", path + ".signals.state_counter"),
					path + ".signals.state_counter", 6U, 0U, 4U);
				parse_unsigned_signal(
					require_field(
						signals, "echoed_control_counter",
						path + ".signals.echoed_control_counter"),
					path + ".signals.echoed_control_counter", 6U, 4U, 4U);
				parse_crc_signal(
					require_field(signals, "crc", path + ".signals.crc"),
					path + ".signals.crc");
				return CanStateFrameConfig{static_cast<std::uint32_t>(can_id), speed, yaw};
			}

			void parse_validation(const YAML::Node &node) const
			{
				const std::string path = "validation";
				check_keys(
					node,
					{"reject_wrong_dlc", "reject_extended_frame", "reject_remote_frame",
					 "reject_nonzero_reserved_bits"},
					path);
				for (const auto *key : {
					"reject_wrong_dlc", "reject_extended_frame", "reject_remote_frame",
					"reject_nonzero_reserved_bits"})
				{
					expect_bool(
						require_bool(node, key, child_path(path, key)), true,
						child_path(path, key), "required fail-closed validation disabled");
				}
			}

			std::string source_name_;
		};

		std::string read_can_signal_map_file(const std::filesystem::path &path)
		{
			std::ifstream input(path);
			if (!input)
			{
				throw ContractError(
					path.string() +
					":root: file open failed; expected=readable YAML file; actual=unreadable; "
					"hint=检查 CAN signal map 路径和权限");
			}
			std::ostringstream buffer;
			buffer << input.rdbuf();
			if (!input.good() && !input.eof())
			{
				throw ContractError(
					path.string() +
					":root: file read failed; expected=complete YAML file; actual=I/O error; "
					"hint=检查文件系统");
			}
			return buffer.str();
		}
	}

	CanSignalMapPtr parse_can_signal_map_text(
		std::string_view yaml_text,
		std::string source_name)
	{
		return CanSignalMapParser(std::move(source_name)).parse(yaml_text);
	}

	CanSignalMapPtr load_can_signal_map(const std::filesystem::path &path)
	{
		return parse_can_signal_map_text(read_can_signal_map_file(path), path.string());
	}
}  // namespace control_link_contract
