#include "control_link_contract/parser.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <fstream>
#include <initializer_list>
#include <limits>
#include <regex>
#include <set>
#include <sstream>
#include <system_error>
#include <utility>

#include <yaml-cpp/yaml.h>

namespace control_link_contract
{
	namespace
	{
		constexpr std::string_view kGatewayNodeFqn = "/control_link/gateway";
		constexpr std::string_view kVehicleAdapterNodeFqn = "/control_link/vehicle_adapter";
		constexpr std::string_view kCanonicalConsumerId = "canonical_output_consumer";
		constexpr std::string_view kVehicleStateProducerId = "vehicle_state_producer";

		// 这里只验证项目 Contract 接受的绝对 ROS 名称子集，不替代运行期 Graph/type 检查
		bool is_valid_topic_name(const std::string &value)
		{
			const std::regex pattern(
				R"(^/[A-Za-z_][A-Za-z0-9_]*(/[A-Za-z_][A-Za-z0-9_]*)*$)");
			return std::regex_match(value, pattern);
		}

		bool is_valid_node_fqn(const std::string &value)
		{
			const std::regex pattern(
				R"(^/[A-Za-z_][A-Za-z0-9_]*(/[A-Za-z_][A-Za-z0-9_]*)*$)");
			return std::regex_match(value, pattern);
		}

		bool is_valid_message_type(const std::string &value)
		{
			const std::regex pattern(
				R"(^[a-z][a-z0-9_]*/msg/[A-Za-z][A-Za-z0-9_]*$)");
			return std::regex_match(value, pattern);
		}

		bool is_valid_source_id(const std::string &value)
		{
			const std::regex pattern(R"(^[a-z][a-z0-9_]{0,31}$)");
			return std::regex_match(value, pattern);
		}

		bool is_valid_package_name(const std::string &value)
		{
			const std::regex pattern(R"(^[a-z][a-z0-9_-]*$)");
			return std::regex_match(value, pattern);
		}

		bool is_valid_frame_id(const std::string &value)
		{
			const std::regex pattern(R"(^[^/\s][^\s]*$)");
			return std::regex_match(value, pattern);
		}

		bool is_valid_network_interface_name(const std::string &value)
		{
			const std::regex pattern(R"(^[A-Za-z0-9][A-Za-z0-9_.-]{0,14}$)");
			return std::regex_match(value, pattern);
		}

		bool is_path_within(
			const std::filesystem::path &root,
			const std::filesystem::path &candidate)
		{
			auto root_part = root.begin();
			auto candidate_part = candidate.begin();

			for (; root_part != root.end(); ++root_part, ++candidate_part)
			{
				if (candidate_part == candidate.end() || *root_part != *candidate_part)
				{
					return false;
				}
			}
			return true;
		}

		std::filesystem::path canonical_regular_file_within_root(
			const std::filesystem::path &candidate,
			const std::filesystem::path &config_root,
			const std::string &source_name,
			const std::string &yaml_path,
			const std::string &subject)
		{
			const auto fail =
				[&source_name, &yaml_path](
					const std::string &rule,
					const std::string &expected,
					const std::string &actual,
					const std::string &hint)
				{
					throw ContractError(
						source_name + ":" + yaml_path + ": " + rule +
						"; expected=" + expected + "; actual=" + actual + "; hint=" + hint);
				};

			// 先 canonical 解析 .. 和符号链接，再按 path component 比较，不能使用字符串前缀判断
			std::error_code error;
			const auto canonical_root = std::filesystem::canonical(config_root, error);
			if (error)
			{
				fail(
					"invalid config root", "existing directory",
					config_root.string() + " (" + error.message() + ")",
					"检查 config_root 是否存在且可访问");
			}

			error.clear();
			const bool root_is_directory = std::filesystem::is_directory(canonical_root, error);
			if (error)
			{
				fail(
					"config root status failed", "readable directory", error.message(),
					"检查 config_root 权限和文件系统状态");
			}
			if (!root_is_directory)
			{
				fail(
					"config root is not a directory", "directory", canonical_root.string(),
					"传入配置目录而不是普通文件");
			}

			error.clear();
			const auto canonical_candidate = std::filesystem::canonical(candidate, error);
			if (error)
			{
				fail(
					"invalid " + subject, "existing file",
					candidate.string() + " (" + error.message() + ")",
					"检查路径、文件名和访问权限");
			}

			error.clear();
			const bool candidate_is_file =
				std::filesystem::is_regular_file(canonical_candidate, error);
			if (error)
			{
				fail(
					subject + " status failed", "readable regular file", error.message(),
					"检查目标文件权限和文件系统状态");
			}
			if (!candidate_is_file)
			{
				fail(
					subject + " is not a file", "regular file", canonical_candidate.string(),
					"引用具体配置文件而不是目录或其他文件类型");
			}

			if (!is_path_within(canonical_root, canonical_candidate))
			{
				fail(
					subject + " escapes config root",
					"file within " + canonical_root.string(), canonical_candidate.string(),
					"删除越界的 .. 或指向配置根外部的符号链接");
			}

			return canonical_candidate;
		}

		std::filesystem::path resolve_config_reference(
			const std::filesystem::path &source_path,
			const std::string &reference,
			const std::filesystem::path &config_root,
			const std::string &yaml_path)
		{
			const auto fail =
				[&source_path, &yaml_path](
					const std::string &rule,
					const std::string &expected,
					const std::string &actual,
					const std::string &hint)
				{
					throw ContractError(
						source_path.string() + ":" + yaml_path + ": " + rule +
						"; expected=" + expected + "; actual=" + actual + "; hint=" + hint);
				};

			const std::filesystem::path reference_path(reference);
			if (reference_path.empty())
			{
				fail(
					"empty config reference", "non-empty relative path", "empty",
					"填写相对当前 Profile 文件的配置路径");
			}
			if (reference_path.is_absolute())
			{
				fail(
					"absolute config reference", "relative path", reference_path.string(),
					"使用相对当前 Profile 文件的配置路径");
			}

			const auto unresolved_target = source_path.parent_path() / reference_path;
			return canonical_regular_file_within_root(
				unresolved_target, config_root, source_path.string(), yaml_path,
				"config reference");
		}

		// 公共读取层只负责 YAML 形状、标量类型、字段集合和统一错误格式
		// 具体 schema 与跨字段规则分别留给三个专用 parser
		class YamlNodeReader
		{
			protected:
			explicit YamlNodeReader(std::string source_name)
				: source_name_(std::move(source_name))
			{
			}

			YAML::Node load_document(std::string_view yaml_text) const
			{
				YAML::Node root;
				try
				{
					root = YAML::Load(std::string(yaml_text));
				}
				catch (const YAML::Exception &error)
				{
					fail(
						"root",
						"invalid YAML",
						"valid YAML document",
						error.what(),
						"检查 YAML 语法");
				}

				return root;
			}

			[[noreturn]] void fail(
				const std::string &path, const std::string &rule, const std::string &expected,
				const std::string &actual, const std::string &hint) const
			{
				throw ContractError(
					source_name_ + ":" + path + ": " + rule + "; expected=" + expected +
					"; actual=" + actual + "; hint=" + hint);
			}

			static std::string child_path(const std::string &parent, const std::string &key)
			{
				return parent == "root" ? key : parent + "." + key;
			}

			static std::string describe(const YAML::Node &node)
			{
				if (!node || node.IsNull())
				{
					return "missing";
				}
				if (node.IsMap())
				{
					return "map";
				}
				if (node.IsSequence())
				{
					return "sequence";
				}
				if (node.IsScalar())
				{
					return "scalar(" + node.Scalar() + ")";
				}
				return "unknown";
			}

			static std::string join_keys(const std::set<std::string> &keys)
			{
				std::ostringstream result;
				bool first = true;
				for (const auto &key : keys)
				{
					if (!first)
					{
						result << ',';
					}
					result << key;
					first = false;
				}
				return result.str();
			}

			void require_map(const YAML::Node &node, const std::string &path) const
			{
				if (!node || !node.IsMap())
				{
					fail(path, "wrong YAML type", "map", describe(node), "使用 key: value 结构");
				}
			}

			void require_sequence(const YAML::Node &node, const std::string &path) const
			{
				if (!node || !node.IsSequence())
				{
					fail(path, "wrong YAML type", "sequence", describe(node), "使用 YAML 列表");
				}
			}

			void check_keys(
				const YAML::Node &node, const std::initializer_list<const char *> allowed,
				const std::string &path) const
			{
				require_map(node, path);
				std::set<std::string> allowed_keys;
				for (const auto *key : allowed)
				{
					allowed_keys.emplace(key);
				}

				std::set<std::string> seen_keys;
				for (const auto &entry : node)
				{
					if (!entry.first.IsScalar())
					{
						fail(path, "non-scalar key", "string key", describe(entry.first), "使用字符串字段名");
					}
					const auto key = entry.first.Scalar();
					if (allowed_keys.count(key) == 0U)
					{
						fail(
							child_path(path, key), "unknown field", join_keys(allowed_keys), describe(entry.second),
							"删除字段或先更新权威 schema");
					}
					if (!seen_keys.emplace(key).second)
					{
						fail(
							child_path(path, key), "duplicate field", "unique field", key,
							"删除重复字段，确保配置只有一个明确值");
					}
				}
			}

			YAML::Node require_field(
				const YAML::Node &parent, const std::string &key, const std::string &path) const
			{
				const YAML::Node value = parent[key];
				if (!value || value.IsNull())
				{
					fail(path, "missing required field", "present", "missing", "补充必填字段");
				}
				return value;
			}

			std::string scalar_as_string(const YAML::Node &node, const std::string &path) const
			{
				if (!node || !node.IsScalar())
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

			std::string require_string(
				const YAML::Node &parent, const std::string &key, const std::string &path) const
			{
				return scalar_as_string(require_field(parent, key, path), path);
			}

			std::uint64_t scalar_as_uint(const YAML::Node &node, const std::string &path) const
			{
				if (!node || !node.IsScalar())
				{
					fail(path, "wrong YAML type", "non-negative integer", describe(node), "填写整数值");
				}
				try
				{
					const auto value = node.as<long long>();
					if (value < 0)
					{
						fail(
							path, "negative integer", "non-negative integer", std::to_string(value),
							"使用大于或等于 0 的整数");
					}
					return static_cast<std::uint64_t>(value);
				}
				catch (const YAML::Exception &error)
				{
					fail(path, "invalid integer", "non-negative integer", error.what(), "检查字段类型");
				}
			}

			std::uint64_t require_uint(
				const YAML::Node &parent, const std::string &key, const std::string &path) const
			{
				return scalar_as_uint(require_field(parent, key, path), path);
			}

			std::optional<std::uint64_t> optional_uint(
				const YAML::Node &parent, const std::string &key, const std::string &path) const
			{
				const YAML::Node value = parent[key];
				if (!value || value.IsNull())
				{
					return std::nullopt;
				}
				return scalar_as_uint(value, path);
			}

			double require_double(
				const YAML::Node &parent, const std::string &key, const std::string &path) const
			{
				const YAML::Node node = require_field(parent, key, path);
				if (!node.IsScalar())
				{
					fail(path, "wrong YAML type", "finite number", describe(node), "填写数值");
				}
				try
				{
					const auto value = node.as<double>();
					if (!std::isfinite(value))
					{
						fail(path, "non-finite number", "finite number", node.Scalar(), "使用有限数值");
					}
					return value;
				}
				catch (const YAML::Exception &error)
				{
					fail(path, "invalid number", "finite number", error.what(), "检查字段类型");
				}
			}

			bool require_bool(
				const YAML::Node &parent, const std::string &key, const std::string &path) const
			{
				const YAML::Node node = require_field(parent, key, path);
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

			private:
			std::string source_name_;
		};

		// GatewayContractParser 拥有单文件 schema 与同一 Contract 内的交叉引用校验
		class GatewayContractParser final : private YamlNodeReader
		{
			public:
			explicit GatewayContractParser(std::string source_name)
				: YamlNodeReader(std::move(source_name))
			{
			}

			GatewayContractPtr parse(const std::string_view yaml_text) const
			{
				const YAML::Node root = load_document(yaml_text);
				require_map(root, "root");
				check_keys(
					root,
					{"schema_version", "contract_id", "gateway", "limits", "qos_profiles", "input",
					 "output", "state_topics", "critical_endpoints"},
					"root");

				const auto schema_version = require_uint(root, "schema_version", "schema_version");
				if (schema_version != 1U)
				{
					fail(
						"schema_version", "unsupported schema version", "1",
						std::to_string(schema_version), "升级 parser 或改用受支持的 schema");
				}

				const auto contract_id = require_string(root, "contract_id", "contract_id");
				if (contract_id.empty())
				{
					fail("contract_id", "empty identifier", "non-empty string", "empty", "填写 Contract 标识");
				}

				GatewayContract contract{
					schema_version,
					contract_id,
					parse_gateway(require_field(root, "gateway", "gateway")),
					parse_limits(require_field(root, "limits", "limits")),
					parse_qos_profiles(require_field(root, "qos_profiles", "qos_profiles")),
					parse_input(require_field(root, "input", "input")),
					parse_output(require_field(root, "output", "output")),
					parse_state_topics(require_field(root, "state_topics", "state_topics")),
					parse_critical_endpoints(
						require_field(root, "critical_endpoints", "critical_endpoints")),
				};

				validate_cross_fields(contract);
				return std::make_shared<const GatewayContract>(std::move(contract));
			}

			private:
			GatewaySettings parse_gateway(const YAML::Node &node) const
			{
				check_keys(
					node,
					{"node_fqn", "output_rate_hz", "command_timeout_ms", "source_switch_hold_ms",
					 "recovery_valid_samples", "graph_poll_ms", "graph_stable_window_ms",
					 "ros_clock_stall_timeout_ms", "vehicle_state_topic_timeout_ms",
					 "output_tick_late_threshold_ms", "consecutive_late_ticks_to_safe_stop"},
					"gateway");

				GatewaySettings result{
					require_string(node, "node_fqn", "gateway.node_fqn"),
					require_double(node, "output_rate_hz", "gateway.output_rate_hz"),
					require_uint(node, "command_timeout_ms", "gateway.command_timeout_ms"),
					require_uint(node, "source_switch_hold_ms", "gateway.source_switch_hold_ms"),
					require_uint(node, "recovery_valid_samples", "gateway.recovery_valid_samples"),
					require_uint(node, "graph_poll_ms", "gateway.graph_poll_ms"),
					require_uint(node, "graph_stable_window_ms", "gateway.graph_stable_window_ms"),
					require_uint(
						node, "ros_clock_stall_timeout_ms", "gateway.ros_clock_stall_timeout_ms"),
					require_uint(
						node, "vehicle_state_topic_timeout_ms", "gateway.vehicle_state_topic_timeout_ms"),
					require_uint(
						node, "output_tick_late_threshold_ms", "gateway.output_tick_late_threshold_ms"),
					require_uint(
						node, "consecutive_late_ticks_to_safe_stop",
						"gateway.consecutive_late_ticks_to_safe_stop"),
				};

				if (!is_valid_node_fqn(result.node_fqn))
				{
					fail(
						"gateway.node_fqn", "invalid ROS node FQN", "absolute node FQN",
						result.node_fqn, "例如 /control_link/gateway");
				}
				if (result.node_fqn != kGatewayNodeFqn)
				{
					fail(
						"gateway.node_fqn", "unexpected Gateway node role",
						std::string(kGatewayNodeFqn), result.node_fqn,
						"v1 Gateway 使用固定 FQN，adapter 依赖该 role 进行运行时绑定");
				}
				if (result.output_rate_hz <= 0.0)
				{
					fail(
						"gateway.output_rate_hz", "non-positive rate", "> 0", std::to_string(result.output_rate_hz),
						"配置正数输出频率");
				}
				if (result.command_timeout_ms == 0U)
				{
					fail(
						"gateway.command_timeout_ms", "zero timeout", "> 0", "0",
						"timeout 必须覆盖至少一个输出周期");
				}
				if (result.recovery_valid_samples == 0U)
				{
					fail(
						"gateway.recovery_valid_samples", "invalid recovery count", ">= 1", "0",
						"至少要求一条有效恢复样本");
				}
				constexpr auto max_recovery_valid_samples =
					std::numeric_limits<std::uint16_t>::max();
				if (result.recovery_valid_samples > max_recovery_valid_samples)
				{
					fail(
						"gateway.recovery_valid_samples", "recovery count exceeds message range",
						"1.." + std::to_string(max_recovery_valid_samples),
						std::to_string(result.recovery_valid_samples),
						"恢复计数必须能由 GatewayState.recovery_valid_count 完整表示");
				}
				const auto max_milliseconds = static_cast<std::uint64_t>(
					std::numeric_limits<std::chrono::milliseconds::rep>::max());
				if (result.source_switch_hold_ms > max_milliseconds)
				{
					fail(
						"gateway.source_switch_hold_ms", "duration exceeds C++ steady-clock range",
						"0.." + std::to_string(max_milliseconds),
						std::to_string(result.source_switch_hold_ms),
						"切换保持时间必须能由 std::chrono::milliseconds 表示");
				}
				if (result.graph_poll_ms == 0U || result.graph_stable_window_ms == 0U ||
					result.ros_clock_stall_timeout_ms == 0U || result.vehicle_state_topic_timeout_ms == 0U)
				{
					fail(
						"gateway", "zero health timeout", "all health periods > 0", "contains zero",
						"Graph、clock 和 VehicleState 周期必须为正数");
				}
				if (result.consecutive_late_ticks_to_safe_stop == 0U)
				{
					fail(
						"gateway.consecutive_late_ticks_to_safe_stop", "invalid late tick count", ">= 1", "0",
						"至少要求一次连续超限判定");
				}

				const double output_period_ms = 1000.0 / result.output_rate_hz;
				if (static_cast<double>(result.command_timeout_ms) < output_period_ms)
				{
					fail(
						"gateway.command_timeout_ms", "timeout shorter than output period",
						">= " + std::to_string(output_period_ms), std::to_string(result.command_timeout_ms),
						"增大 timeout 或降低 output_rate_hz");
				}
				if (static_cast<double>(result.output_tick_late_threshold_ms) < output_period_ms)
				{
					fail(
						"gateway.output_tick_late_threshold_ms", "late threshold shorter than output period",
						">= " + std::to_string(output_period_ms),
						std::to_string(result.output_tick_late_threshold_ms), "阈值不能小于目标输出周期");
				}
				return result;
			}

			CommandLimits parse_limits(const YAML::Node &node) const
			{
				check_keys(
					node,
					{"max_abs_linear_velocity_mps", "max_abs_angular_velocity_radps", "reject_non_finite",
					 "reject_zero_stamp", "max_future_skew_ms"},
					"limits");

				CommandLimits result{
					require_double(
						node, "max_abs_linear_velocity_mps", "limits.max_abs_linear_velocity_mps"),
					require_double(
						node, "max_abs_angular_velocity_radps", "limits.max_abs_angular_velocity_radps"),
					require_bool(node, "reject_non_finite", "limits.reject_non_finite"),
					require_bool(node, "reject_zero_stamp", "limits.reject_zero_stamp"),
					require_uint(node, "max_future_skew_ms", "limits.max_future_skew_ms"),
				};

				if (result.max_abs_linear_velocity_mps <= 0.0)
				{
					fail(
						"limits.max_abs_linear_velocity_mps", "non-positive limit", "> 0",
						std::to_string(result.max_abs_linear_velocity_mps), "配置正数速度上限");
				}
				if (result.max_abs_angular_velocity_radps <= 0.0)
				{
					fail(
						"limits.max_abs_angular_velocity_radps", "non-positive limit", "> 0",
						std::to_string(result.max_abs_angular_velocity_radps), "配置正数角速度上限");
				}
				if (!result.reject_non_finite)
				{
					fail(
						"limits.reject_non_finite", "mandatory validation disabled", "true", "false",
						"v1 控制命令必须拒绝 NaN 和 Inf");
				}
				if (!result.reject_zero_stamp)
				{
					fail(
						"limits.reject_zero_stamp", "mandatory validation disabled", "true", "false",
						"v1 控制命令必须拒绝零 source stamp");
				}
				return result;
			}

			ReliabilityPolicy parse_reliability(
				const YAML::Node &node, const std::string &path) const
			{
				const auto value = scalar_as_string(node, path);
				if (value == "reliable")
				{
					return ReliabilityPolicy::kReliable;
				}
				if (value == "best_effort")
				{
					return ReliabilityPolicy::kBestEffort;
				}
				fail(path, "unknown QoS enum", "reliable|best_effort", value, "使用受支持的 reliability");
			}

			DurabilityPolicy parse_durability(
				const YAML::Node &node, const std::string &path) const
			{
				const auto value = scalar_as_string(node, path);
				if (value == "volatile")
				{
					return DurabilityPolicy::kVolatile;
				}
				if (value == "transient_local")
				{
					return DurabilityPolicy::kTransientLocal;
				}
				fail(path, "unknown QoS enum", "volatile|transient_local", value, "使用受支持的 durability");
			}

			HistoryPolicy parse_history(const YAML::Node &node, const std::string &path) const
			{
				const auto value = scalar_as_string(node, path);
				if (value == "keep_last")
				{
					return HistoryPolicy::kKeepLast;
				}
				if (value == "keep_all")
				{
					return HistoryPolicy::kKeepAll;
				}
				fail(path, "unknown QoS enum", "keep_last|keep_all", value, "使用受支持的 history");
			}

			LivelinessPolicy parse_liveliness(
				const YAML::Node &node, const std::string &path) const
			{
				const auto value = scalar_as_string(node, path);
				if (value == "automatic")
				{
					return LivelinessPolicy::kAutomatic;
				}
				if (value == "manual_by_topic")
				{
					return LivelinessPolicy::kManualByTopic;
				}
				fail(
					path, "unknown QoS enum", "automatic|manual_by_topic", value,
					"使用受支持的 liveliness");
			}

			void validate_qos_duration_range(
				const std::optional<std::uint64_t> &duration_ms,
				const std::string &path) const
			{
				if (!duration_ms.has_value())
				{
					return;
				}

				constexpr std::uint64_t kNanosecondsPerMillisecond = 1'000'000U;
				constexpr auto kMaxDurationMilliseconds =
					static_cast<std::uint64_t>(std::numeric_limits<std::int64_t>::max()) /
					kNanosecondsPerMillisecond;
				if (duration_ms.value() > kMaxDurationMilliseconds)
				{
					fail(
						path,
						"QoS duration exceeds ROS2 duration range",
						"1.." + std::to_string(kMaxDurationMilliseconds) + " ms",
						std::to_string(duration_ms.value()) + " ms",
						"缩短 QoS duration，确保转换为有符号纳秒时不会溢出");
				}
			}

			QosProfile parse_qos_profile(const YAML::Node &node, const std::string &path) const
			{
				check_keys(
					node,
					{"reliability", "durability", "history", "depth", "deadline_ms", "lifespan_ms",
					 "liveliness", "liveliness_lease_duration_ms"},
					path);

				const YAML::Node liveliness_node = node["liveliness"];
				const auto lease = optional_uint(
					node, "liveliness_lease_duration_ms", path + ".liveliness_lease_duration_ms");
				QosProfile result{
					parse_reliability(
						require_field(node, "reliability", path + ".reliability"), path + ".reliability"),
					parse_durability(
						require_field(node, "durability", path + ".durability"), path + ".durability"),
					parse_history(require_field(node, "history", path + ".history"), path + ".history"),
					require_uint(node, "depth", path + ".depth"),
					optional_uint(node, "deadline_ms", path + ".deadline_ms"),
					optional_uint(node, "lifespan_ms", path + ".lifespan_ms"),
					liveliness_node ? std::optional<LivelinessPolicy>(
						parse_liveliness(liveliness_node, path + ".liveliness"))
									: std::nullopt,
					lease,
				};

				if (result.history == HistoryPolicy::kKeepLast && result.depth == 0U)
				{
					fail(path + ".depth", "invalid keep_last depth", ">= 1", "0", "设置至少一个样本槽位");
				}
				if (result.deadline_ms.has_value() && *result.deadline_ms == 0U)
				{
					fail(path + ".deadline_ms", "zero duration", "> 0", "0", "配置正数 deadline");
				}
				if (result.lifespan_ms.has_value() && *result.lifespan_ms == 0U)
				{
					fail(path + ".lifespan_ms", "zero duration", "> 0", "0", "配置正数 lifespan");
				}
				if (result.liveliness.has_value() != result.liveliness_lease_duration_ms.has_value())
				{
					fail(
						path, "incomplete liveliness policy", "liveliness and lease both present or both absent",
						"partial policy", "同时配置或同时省略 liveliness 字段");
				}
				if (result.liveliness_lease_duration_ms.has_value() &&
					*result.liveliness_lease_duration_ms == 0U)
				{
					fail(
						path + ".liveliness_lease_duration_ms", "zero duration", "> 0", "0",
						"配置正数 liveliness lease");
				}

				validate_qos_duration_range(result.deadline_ms, path + ".deadline_ms");
				validate_qos_duration_range(result.lifespan_ms, path + ".lifespan_ms");
				validate_qos_duration_range(
					result.liveliness_lease_duration_ms,
					path + ".liveliness_lease_duration_ms");
				return result;
			}

			std::map<std::string, QosProfile> parse_qos_profiles(const YAML::Node &node) const
			{
				require_map(node, "qos_profiles");
				if (node.size() == 0U)
				{
					fail("qos_profiles", "empty profile map", "at least one profile", "empty", "定义 QoS profile");
				}

				std::map<std::string, QosProfile> profiles;
				for (const auto &entry : node)
				{
					const auto name = scalar_as_string(entry.first, "qos_profiles");
					if (name.empty())
					{
						fail("qos_profiles", "empty profile name", "non-empty string", "empty", "填写 profile 名称");
					}
					const auto [iterator, inserted] = profiles.emplace(
						name, parse_qos_profile(entry.second, "qos_profiles." + name));
					static_cast<void>(iterator);
					if (!inserted)
					{
						fail(
							"qos_profiles." + name, "duplicate profile", "unique name", name,
							"删除重复的 QoS profile");
					}
				}
				return profiles;
			}

			InputContract parse_input(const YAML::Node &node) const
			{
				check_keys(node, {"topic_prefix", "type", "qos_profile"}, "input");
				InputContract result{
					require_string(node, "topic_prefix", "input.topic_prefix"),
					require_string(node, "type", "input.type"),
					require_string(node, "qos_profile", "input.qos_profile"),
				};
				validate_topic_and_type(result.topic_prefix, result.type, "input.topic_prefix", "input.type");
				return result;
			}

			OutputContract parse_output(const YAML::Node &node) const
			{
				check_keys(node, {"topic", "type", "qos_profile"}, "output");
				OutputContract result{
					require_string(node, "topic", "output.topic"),
					require_string(node, "type", "output.type"),
					require_string(node, "qos_profile", "output.qos_profile"),
				};
				validate_topic_and_type(result.topic, result.type, "output.topic", "output.type");
				return result;
			}

			StateTopicContract parse_state_topic(
				const YAML::Node &node, const std::string &path) const
			{
				check_keys(node, {"topic", "type", "qos_profile"}, path);
				const YAML::Node qos_node = node["qos_profile"];
				StateTopicContract result{
					require_string(node, "topic", path + ".topic"),
					require_string(node, "type", path + ".type"),
					qos_node ? std::optional<std::string>(scalar_as_string(qos_node, path + ".qos_profile")) : std::nullopt,
				};
				validate_topic_and_type(result.topic, result.type, path + ".topic", path + ".type");
				return result;
			}

			std::map<std::string, StateTopicContract> parse_state_topics(const YAML::Node &node) const
			{
				check_keys(
					node, {"gateway_state", "source_status", "vehicle_state", "diagnostics"}, "state_topics");
				std::map<std::string, StateTopicContract> topics;
				for (const auto *name : {"gateway_state", "source_status", "vehicle_state", "diagnostics"})
				{
					const auto path = std::string("state_topics.") + name;
					topics.emplace(name, parse_state_topic(require_field(node, name, path), path));
				}
				return topics;
			}

			RemoteDirection parse_remote_direction(
				const YAML::Node &node, const std::string &path) const
			{
				const auto value = scalar_as_string(node, path);
				if (value == "publisher")
				{
					return RemoteDirection::kPublisher;
				}
				if (value == "subscription")
				{
					return RemoteDirection::kSubscription;
				}
				fail(path, "unknown direction", "publisher|subscription", value, "使用远端 endpoint 方向");
			}

			RuntimeLossAction parse_runtime_loss_action(
				const YAML::Node &node, const std::string &path) const
			{
				const auto value = scalar_as_string(node, path);
				if (value == "safe_stop")
				{
					return RuntimeLossAction::kSafeStop;
				}
				fail(path, "unknown runtime action", "safe_stop", value, "v1 只支持 safe_stop");
			}

			RuntimeQosMismatchAction parse_runtime_qos_action(
				const YAML::Node &node, const std::string &path) const
			{
				const auto value = scalar_as_string(node, path);
				if (value == "degraded")
				{
					return RuntimeQosMismatchAction::kDegraded;
				}
				fail(path, "unknown runtime action", "degraded", value, "v1 只支持 degraded");
			}

			CriticalEndpoint parse_critical_endpoint(
				const YAML::Node &node, const std::string &path) const
			{
				check_keys(
					node,
					{"id", "topic", "type", "remote_direction", "remote_node_fqn", "min_count", "max_count",
					 "allow_additional_endpoints", "required_for_activation", "exact_qos_required",
					 "runtime_loss_action", "runtime_qos_mismatch_action"},
					path);

				const auto max_count = optional_uint(node, "max_count", path + ".max_count");
				CriticalEndpoint result{
					require_string(node, "id", path + ".id"),
					require_string(node, "topic", path + ".topic"),
					require_string(node, "type", path + ".type"),
					parse_remote_direction(
						require_field(node, "remote_direction", path + ".remote_direction"),
						path + ".remote_direction"),
					require_string(node, "remote_node_fqn", path + ".remote_node_fqn"),
					require_uint(node, "min_count", path + ".min_count"),
					max_count,
					require_bool(node, "allow_additional_endpoints", path + ".allow_additional_endpoints"),
					require_bool(node, "required_for_activation", path + ".required_for_activation"),
					require_bool(node, "exact_qos_required", path + ".exact_qos_required"),
					parse_runtime_loss_action(
						require_field(node, "runtime_loss_action", path + ".runtime_loss_action"),
						path + ".runtime_loss_action"),
					parse_runtime_qos_action(
						require_field(
							node, "runtime_qos_mismatch_action", path + ".runtime_qos_mismatch_action"),
						path + ".runtime_qos_mismatch_action"),
				};

				if (result.id.empty())
				{
					fail(path + ".id", "empty endpoint id", "non-empty string", "empty", "填写 endpoint id");
				}
				validate_topic_and_type(result.topic, result.type, path + ".topic", path + ".type");
				if (!is_valid_node_fqn(result.remote_node_fqn))
				{
					fail(
						path + ".remote_node_fqn", "invalid ROS node FQN", "absolute node FQN",
						result.remote_node_fqn, "例如 /control_link/vehicle_adapter");
				}
				if (result.min_count == 0U)
				{
					fail(path + ".min_count", "invalid endpoint count", ">= 1", "0", "配置至少一个 endpoint");
				}
				if (result.max_count.has_value() && *result.max_count < result.min_count)
				{
					fail(
						path + ".max_count", "max_count smaller than min_count",
						">= " + std::to_string(result.min_count), std::to_string(*result.max_count),
						"修正 endpoint count 区间");
				}
				return result;
			}

			std::vector<CriticalEndpoint> parse_critical_endpoints(const YAML::Node &node) const
			{
				require_sequence(node, "critical_endpoints");
				if (node.size() == 0U)
				{
					fail(
						"critical_endpoints", "empty endpoint list", "at least one endpoint", "empty",
						"定义关键执行边界");
				}

				std::set<std::string> ids;
				std::vector<CriticalEndpoint> endpoints;
				endpoints.reserve(node.size());
				for (std::size_t index = 0; index < node.size(); ++index)
				{
					const auto path = "critical_endpoints[" + std::to_string(index) + "]";
					auto endpoint = parse_critical_endpoint(node[index], path);
					if (!ids.emplace(endpoint.id).second)
					{
						fail(path + ".id", "duplicate endpoint id", "unique id", endpoint.id, "删除重复 endpoint");
					}
					endpoints.push_back(std::move(endpoint));
				}
				return endpoints;
			}

			void validate_topic_and_type(
				const std::string &topic, const std::string &type, const std::string &topic_path,
				const std::string &type_path) const
			{
				if (!is_valid_topic_name(topic))
				{
					fail(topic_path, "invalid ROS topic", "absolute topic name", topic, "使用不含通配符的绝对 Topic");
				}
				if (!is_valid_message_type(type))
				{
					fail(
						type_path, "invalid ROS message type", "package/msg/Type", type,
						"填写完整 ROS message 类型");
				}
			}

			void validate_qos_reference(
				const std::map<std::string, QosProfile> &profiles, const std::string &reference,
				const std::string &path) const
			{
				if (profiles.count(reference) == 0U)
				{
					fail(path, "unknown QoS reference", "defined qos_profiles key", reference, "修正 profile 引用");
				}
			}

			static std::string remote_direction_name(RemoteDirection direction)
			{
				switch (direction)
				{
					case RemoteDirection::kPublisher:
						return "publisher";

					case RemoteDirection::kSubscription:
						return "subscription";
				}

				return "unsupported";
			}

			std::size_t require_critical_endpoint_index(
				const GatewayContract &contract,
				std::string_view endpoint_id) const
			{
				for (std::size_t index = 0; index < contract.critical_endpoints.size(); ++index)
				{
					if (contract.critical_endpoints[index].id == endpoint_id)
					{
						return index;
					}
				}

				fail(
					"critical_endpoints", "missing required v1 critical role",
					std::string(endpoint_id), "missing",
					"同时配置 canonical command consumer 与唯一 VehicleState producer");
			}

			void validate_v1_critical_role(
				const CriticalEndpoint &endpoint,
				const std::string &path,
				const std::string &expected_topic,
				const std::string &expected_type,
				RemoteDirection expected_direction,
				bool expected_allow_additional) const
			{
				if (endpoint.topic != expected_topic)
				{
					fail(
						path + ".topic", "critical role uses wrong Topic",
						expected_topic, endpoint.topic,
						"让固定执行 role 绑定对应的公共控制接口");
				}

				if (endpoint.type != expected_type)
				{
					fail(
						path + ".type", "critical role uses wrong message type",
						expected_type, endpoint.type,
						"让 fixed role 的 Topic 与消息类型同时匹配公共 Contract");
				}

				if (endpoint.remote_direction != expected_direction)
				{
					fail(
						path + ".remote_direction", "critical role uses wrong direction",
						remote_direction_name(expected_direction),
						remote_direction_name(endpoint.remote_direction),
						"方向始终描述 Gateway 对端的 publisher 或 subscription");
				}

				if (endpoint.remote_node_fqn != kVehicleAdapterNodeFqn)
				{
					fail(
						path + ".remote_node_fqn", "critical role uses wrong adapter FQN",
						std::string(kVehicleAdapterNodeFqn), endpoint.remote_node_fqn,
						"Robot 与 ADAS execution adapter 共用固定运行时 role");
				}

				if (endpoint.min_count != 1U ||
					!endpoint.max_count.has_value() || endpoint.max_count.value() != 1U)
				{
					const auto actual_max = endpoint.max_count.has_value()
						? std::to_string(endpoint.max_count.value())
						: std::string("unbounded");
					fail(
						path, "critical role count is not exactly one", "min=1,max=1",
						"min=" + std::to_string(endpoint.min_count) + ",max=" + actual_max,
						"v1 execution role 只能由一个匹配 endpoint 承担");
				}

				if (endpoint.allow_additional_endpoints != expected_allow_additional)
				{
					fail(
						path + ".allow_additional_endpoints",
						"critical role additional-endpoint policy mismatch",
						expected_allow_additional ? "true" : "false",
						endpoint.allow_additional_endpoints ? "true" : "false",
						"canonical output 允许观察订阅，VehicleState 禁止第二个 publisher");
				}

				if (!endpoint.required_for_activation)
				{
					fail(
						path + ".required_for_activation",
						"critical role is optional during activation", "true", "false",
						"两个 execution boundary role 都必须通过 activation gate");
				}

				if (!endpoint.exact_qos_required)
				{
					fail(
						path + ".exact_qos_required",
						"critical role disables exact QoS", "true", "false",
						"v1 critical role 同时要求 DDS compatibility 与 exact policy match");
				}
			}

			void validate_v1_critical_endpoints(const GatewayContract &contract) const
			{
				if (contract.critical_endpoints.size() != 2U)
				{
					fail(
						"critical_endpoints", "unexpected v1 critical role count", "2",
						std::to_string(contract.critical_endpoints.size()),
						"只配置 canonical_output_consumer 与 vehicle_state_producer");
				}

				const auto canonical_index = require_critical_endpoint_index(
					contract, kCanonicalConsumerId);
				validate_v1_critical_role(
					contract.critical_endpoints[canonical_index],
					"critical_endpoints[" + std::to_string(canonical_index) + "]",
					contract.output.topic,
					contract.output.type,
					RemoteDirection::kSubscription,
					true);

				const auto vehicle_state_index = require_critical_endpoint_index(
					contract, kVehicleStateProducerId);
				const auto &vehicle_state = contract.state_topics.at("vehicle_state");
				validate_v1_critical_role(
					contract.critical_endpoints[vehicle_state_index],
					"critical_endpoints[" + std::to_string(vehicle_state_index) + "]",
					vehicle_state.topic,
					vehicle_state.type,
					RemoteDirection::kPublisher,
					false);
			}

			void validate_cross_fields(const GatewayContract &contract) const
			{
				if (contract.limits.max_future_skew_ms > contract.gateway.command_timeout_ms)
				{
					fail(
						"limits.max_future_skew_ms", "future skew exceeds command timeout",
						"<= " + std::to_string(contract.gateway.command_timeout_ms),
						std::to_string(contract.limits.max_future_skew_ms), "缩小 future skew");
				}

				validate_qos_reference(
					contract.qos_profiles, contract.input.qos_profile, "input.qos_profile");
				validate_qos_reference(
					contract.qos_profiles, contract.output.qos_profile, "output.qos_profile");
				for (const auto &[name, topic] : contract.state_topics)
				{
					if (topic.qos_profile.has_value())
					{
						validate_qos_reference(
							contract.qos_profiles, *topic.qos_profile, "state_topics." + name + ".qos_profile");
					}
				}

				for (std::size_t index = 0; index < contract.critical_endpoints.size(); ++index)
				{
					const auto &endpoint = contract.critical_endpoints[index];
					bool matches_public_interface =
						endpoint.topic == contract.output.topic && endpoint.type == contract.output.type;
					for (const auto &[name, topic] : contract.state_topics)
					{
						static_cast<void>(name);
						matches_public_interface = matches_public_interface ||
												   (endpoint.topic == topic.topic && endpoint.type == topic.type);
					}
					if (!matches_public_interface)
					{
						const auto path = "critical_endpoints[" + std::to_string(index) + "]";
						fail(
							path + ".topic", "endpoint is outside public Contract", "configured output/state topic+type",
							endpoint.topic + " " + endpoint.type, "关键 endpoint 必须引用公共运行接口");
						}
					}

				validate_v1_critical_endpoints(contract);
			}

			std::string source_name_;
		};

		// SourcePolicyParser 只验证来源表本身，依赖 GatewayContract 的规则留给 ContractBundle
		class SourcePolicyParser final : private YamlNodeReader
		{
			public:
			explicit SourcePolicyParser(std::string source_name)
				: YamlNodeReader(std::move(source_name))
			{
			}

			SourcePolicyPtr parse(std::string_view yaml_text) const
			{
				const YAML::Node root = load_document(yaml_text);
				require_map(root, "root");
				check_keys(root, {"schema_version", "policy_id", "sources"}, "root");

				const auto schema_version = require_uint(root, "schema_version", "schema_version");
				if (schema_version != 1U)
				{
					fail(
						"schema_version", "unsupported schema version", "1",
						std::to_string(schema_version), "升级 parser 或改用受支持的 schema");
				}

				const auto policy_id = require_string(root, "policy_id", "policy_id");
				if (policy_id.empty())
				{
					fail("policy_id", "empty identifier", "non-empty string", "empty", "填写 Policy 标识");
				}

				SourcePolicy policy{
					schema_version,
					policy_id,
					parse_sources(require_field(root, "sources", "sources")),
				};
				return std::make_shared<const SourcePolicy>(std::move(policy));
			}

			private:
			std::pair<std::string, SourcePolicyEntry> parse_source_entry(
				const YAML::Node &node, const std::string &path) const
			{
				check_keys(
					node,
					{"id", "topic", "type", "priority", "lease_timeout_ms", "required_for_activation"},
					path);

				const auto id = require_string(node, "id", path + ".id");
				const auto topic = require_string(node, "topic", path + ".topic");
				const auto type = require_string(node, "type", path + ".type");
				const auto priority = require_uint(node, "priority", path + ".priority");
				const auto lease_timeout_ms =
					require_uint(node, "lease_timeout_ms", path + ".lease_timeout_ms");
				const auto required_for_activation = require_bool(
					node, "required_for_activation", path + ".required_for_activation");

				if (!is_valid_source_id(id))
				{
					fail(
						path + ".id", "invalid source id", "[a-z][a-z0-9_]{0,31}", id,
						"使用小写字母开头且不超过 32 个字符的 source id");
				}
				if (!is_valid_topic_name(topic))
				{
					fail(
						path + ".topic", "invalid ROS topic", "absolute topic name", topic,
						"使用不含通配符的绝对 Topic");
				}
				if (!is_valid_message_type(type))
				{
					fail(
						path + ".type", "invalid ROS message type", "package/msg/Type", type,
						"填写完整 ROS message 类型");
				}
				if (priority > std::numeric_limits<std::uint8_t>::max())
				{
					fail(
						path + ".priority", "priority out of range", "0..255",
						std::to_string(priority), "配置 uint8 范围内的优先级");
				}
				if (lease_timeout_ms == 0U || lease_timeout_ms > 2000U)
				{
					fail(
						path + ".lease_timeout_ms", "lease timeout out of range", "1..2000",
						std::to_string(lease_timeout_ms), "配置有效 lease，最小周期约束由组合校验处理");
				}

				return {
					id,
					SourcePolicyEntry{
						topic,
						type,
						static_cast<std::uint8_t>(priority),
						lease_timeout_ms,
						required_for_activation,
					},
				};
			}

			std::map<std::string, SourcePolicyEntry> parse_sources(const YAML::Node &node) const
			{
				require_sequence(node, "sources");
				if (node.size() == 0U)
				{
					fail("sources", "empty source list", "at least one source", "empty", "定义控制来源");
				}

				std::set<std::string> topics;
				std::map<std::string, SourcePolicyEntry> sources;
				for (std::size_t index = 0; index < node.size(); ++index)
				{
					const auto path = "sources[" + std::to_string(index) + "]";
					auto [id, entry] = parse_source_entry(node[index], path);

					if (sources.count(id) != 0U)
					{
						fail(path + ".id", "duplicate source id", "unique id", id, "删除重复 source");
					}
					if (!topics.emplace(entry.topic).second)
					{
						fail(
							path + ".topic", "duplicate source topic", "unique topic", entry.topic,
							"每个 Topic 只能绑定一个 source id");
					}

					sources.emplace(std::move(id), std::move(entry));
				}
				return sources;
			}
		};

		// ProfileParser 先按 profile_id 选择 Robot/ADAS schema，再组装公共与平台专属配置
		class ProfileParser final : private YamlNodeReader
		{
			public:
			ProfileParser(
				std::filesystem::path source_path,
				std::filesystem::path config_root)
				: YamlNodeReader(source_path.string()),
				source_path_(std::move(source_path)),
				config_root_(std::move(config_root))
			{
			}

			ProfileConfigPtr parse(std::string_view yaml_text) const
			{
				const auto root = load_document(yaml_text);
				require_map(root, "root");

				const auto profile_id = require_string(root, "profile_id", "profile_id");
				if (profile_id == "robot")
				{
					check_keys(
						root,
						{"schema_version", "profile_id", "contract", "source_policy",
						 "fastdds_profile", "enabled_sources", "clock_mode", "use_sim_time",
						 "geometry", "adapter", "ingress", "resources", "frames", "health",
						 "fixed_demo_goal", "record_topics"},
						"root");

					RobotProfile profile{
						parse_common(root),
						parse_robot_geometry(
							require_field(root, "geometry", "geometry"), "geometry"),
						parse_robot_adapter(
							require_field(root, "adapter", "adapter"), "adapter"),
						parse_robot_resources(
							require_field(root, "resources", "resources"), "resources"),
						parse_robot_frames(
							require_field(root, "frames", "frames"), "frames"),
						parse_robot_health(
							require_field(root, "health", "health"), "health"),
						parse_fixed_demo_goal(
							require_field(root, "fixed_demo_goal", "fixed_demo_goal"),
							"fixed_demo_goal")};

					validate_robot_cross_fields(profile);

					return std::make_shared<const ProfileConfig>(std::move(profile));
				}

				if (profile_id == "adas")
				{
					check_keys(
						root,
						{"schema_version", "profile_id", "contract", "source_policy",
						 "fastdds_profile", "enabled_sources", "clock_mode", "use_sim_time",
						 "adapter", "ingress", "vehicle_simulator", "record_topics", "replay"},
						"root");

					AdasProfile profile{
						parse_common(root),
						parse_adas_adapter(
							require_field(root, "adapter", "adapter"), "adapter"),
						parse_vehicle_simulator(
							require_field(root, "vehicle_simulator", "vehicle_simulator"),
							"vehicle_simulator"),
						parse_replay(require_field(root, "replay", "replay"), "replay")};

					validate_adas_cross_fields(profile);

					return std::make_shared<const ProfileConfig>(std::move(profile));
				}

				fail(
					"profile_id", "unsupported profile id", "robot|adas", profile_id,
					"使用受支持的 Robot 或 ADAS Profile schema");
			}

			private:
			std::uint64_t require_positive_uint(
				const YAML::Node &parent,
				const std::string &key,
				const std::string &path) const
			{
				const auto value = require_uint(parent, key, path);
				if (value == 0U)
				{
					fail(
						path, "zero value", "positive integer", std::to_string(value),
						"为该周期、超时或恢复计数配置大于 0 的整数");
				}
				return value;
			}

			std::filesystem::path parse_package_resource_path(
				const YAML::Node &parent,
				const std::string &key,
				const std::string &path) const
			{
				const auto reference = require_string(parent, key, path);
				const std::filesystem::path resource_path(reference);

				if (resource_path.empty())
				{
					fail(
						path, "empty package resource path", "non-empty relative path", "empty",
						"填写相对 resources.package 对应 package share 的资源路径");
				}
				if (resource_path.is_absolute())
				{
					fail(
						path, "absolute package resource path", "relative package resource path",
						resource_path.string(),
						"删除根目录前缀，由 bringup 从 package share 解析资源");
				}

				for (const auto &component : resource_path)
				{
					if (component == "..")
					{
						fail(
							path, "package resource path traversal",
							"relative path without ..", resource_path.string(),
							"删除 .. 分量，资源必须保留在 package share 边界内");
					}
				}

				return resource_path.lexically_normal();
			}

			std::string require_frame_id(
				const YAML::Node &parent,
				const std::string &key,
				const std::string &path) const
			{
				auto frame_id = require_string(parent, key, path);
				if (!is_valid_frame_id(frame_id))
				{
					fail(
						path, "invalid ROS frame id", "relative frame id without whitespace",
						frame_id,
						"使用 map、base_link 或 robot/base_link 形式的 frame id");
				}
				return frame_id;
			}

			ClockMode parse_clock_mode(
				const YAML::Node &node,
				const std::string &path) const
			{
				const auto clock_mode = scalar_as_string(node, path);
				if (clock_mode == "sim")
				{
					return ClockMode::kSim;
				}
				if (clock_mode == "system")
				{
					return ClockMode::kSystem;
				}
				fail(
					path, "unknown clock mode", "sim|system", clock_mode,
					"使用受支持的 Profile 时钟模式");
			}

			std::vector<std::string> parse_enabled_sources(
				const YAML::Node &node,
				const std::string &path) const
			{
				require_sequence(node, path);

				if (node.size() == 0U)
				{
					fail(
						path, "empty enabled source list", "at least one source", "empty",
						"至少启用一个当前 Profile 使用的控制来源");
				}

				std::vector<std::string> result;
				std::set<std::string> seen;
				result.reserve(node.size());

				for (std::size_t index = 0; index < node.size(); ++index)
				{
					std::string element_path = path + "[" + std::to_string(index) + "]";

					std::string source_id = scalar_as_string(node[index], element_path);
					if (!is_valid_source_id(source_id))
					{
						fail(
							element_path, "invalid source id", "[a-z][a-z0-9_]{0,31}", source_id,
							"使用小写字母开头且不超过 32 个字符的 source id");
					}

					if (!seen.emplace(source_id).second)
					{
						fail(
							element_path, "duplicate enabled source", "unique source id", source_id,
							"删除重复项，每个 source id 只能启用一次");
					}
					result.push_back(std::move(source_id));
				}
				return result;
			}

			std::vector<std::string> parse_record_topics(
				const YAML::Node &node,
				const std::string &path) const
			{
				require_sequence(node, path);

				std::vector<std::string> result;
				std::set<std::string> seen;
				result.reserve(node.size());

				for (std::size_t index = 0; index < node.size(); ++index)
				{
					std::string element_path = path + "[" + std::to_string(index) + "]";

					std::string topic = scalar_as_string(node[index], element_path);
					if (!is_valid_topic_name(topic))
					{
						fail(
							element_path, "invalid record topic", "absolute ROS topic name", topic,
							"使用以 / 开头且不含通配符的绝对 Topic");
					}

					if (!seen.emplace(topic).second)
					{
						fail(
							element_path, "duplicate record topic", "unique topic", topic,
							"删除重复项，每个 Topic 只记录一次");
					}
					result.push_back(std::move(topic));
				}
				return result;
			}

			IngressBinding parse_ingress_binding(
				const YAML::Node &node,
				const std::string &path) const
			{
				check_keys(
					node,
					{"input_topic", "input_type", "output_topic"},
					path);

				IngressBinding result{
					require_string(node, "input_topic", path + ".input_topic"),
					require_string(node, "input_type", path + ".input_type"),
					require_string(node, "output_topic", path + ".output_topic"),
				};

				if (!is_valid_topic_name(result.input_topic))
				{
					fail(
						path + ".input_topic", "invalid ingress input topic",
						"absolute ROS topic name", result.input_topic,
						"使用以 / 开头且不含通配符的上游 Topic");
				}

				if (!is_valid_message_type(result.input_type))
				{
					fail(
						path + ".input_type", "invalid ingress input type",
						"package/msg/Type", result.input_type,
						"Topic ingress 只能使用完整 ROS message 类型");
				}

				if (!is_valid_topic_name(result.output_topic))
				{
					fail(
						path + ".output_topic", "invalid ingress output topic",
						"absolute ROS topic name", result.output_topic,
						"使用 Guardian 标准输入边界内的绝对 Topic");
				}

				return result;
			}

			std::map<std::string, IngressBinding> parse_ingress(
				const YAML::Node &node,
				const std::string &path) const
			{
				require_map(node, path);
				if (node.size() == 0U)
				{
					fail(
						path, "empty ingress map", "at least one ingress binding", "empty",
						"为当前 Profile 至少配置一个输入来源映射");
				}

				std::set<std::string> input_topics;
				std::set<std::string> output_topics;
				std::map<std::string, IngressBinding> bindings;

				for (const auto &entry : node)
				{
					auto source_id = scalar_as_string(entry.first, path);
					const auto element_path = path + "." + source_id;

					if (!is_valid_source_id(source_id))
					{
						fail(
							element_path, "invalid ingress source id", "[a-z][a-z0-9_]{0,31}",
							source_id, "使用合法 source id 作为 ingress map 的 key");
					}
					if (bindings.count(source_id) != 0U)
					{
						fail(
							element_path, "duplicate ingress source id", "unique source id", source_id,
							"删除重复 binding，每个 source id 只能出现一次");
					}

					auto binding = parse_ingress_binding(entry.second, element_path);
					// 外部输入与 Guardian 内部输出必须处于互斥 Topic 边界，避免 adapter 接收自己的发布
					if (output_topics.count(binding.input_topic) != 0U)
					{
						fail(
							element_path + ".input_topic", "ingress input collides with output topic",
							"topic not used by any ingress output", binding.input_topic,
							"将外部输入 Topic 与 Guardian 内部 source Topic 分开");
					}
					if (!input_topics.emplace(binding.input_topic).second)
					{
						fail(
							element_path + ".input_topic", "duplicate ingress input topic",
							"unique input topic", binding.input_topic,
							"一个上游 Topic 不能映射为多个控制来源");
					}
					if (input_topics.count(binding.output_topic) != 0U)
					{
						fail(
							element_path + ".output_topic", "ingress output collides with input topic",
							"topic not used by any ingress input", binding.output_topic,
							"将 Guardian 内部 source Topic 与所有外部输入 Topic 分开");
					}
					if (!output_topics.emplace(binding.output_topic).second)
					{
						fail(
							element_path + ".output_topic", "duplicate ingress output topic",
							"unique output topic", binding.output_topic,
							"每个来源必须发布到独立的 Guardian 标准输入 Topic");
					}

					bindings.emplace(std::move(source_id), std::move(binding));
				}

				return bindings;
			}

			ProfileCommon parse_common(const YAML::Node &root) const
			{
				// root 还包含 Robot/ADAS 专属字段，完整 unknown-key 检查已在 parse() 的分支中完成
				require_map(root, "root");

				const uint64_t schema_version = require_uint(root, "schema_version", "schema_version");
				if (schema_version != 1U)
				{
					fail(
						"schema_version", "unsupported schema version", "1",
						std::to_string(schema_version),
						"升级 Profile parser 或改用受支持的 schema");
				}

				const auto contract_reference = require_string(root, "contract", "contract");
				const auto contract_path = resolve_config_reference(
					source_path_,
					contract_reference,
					config_root_,
					"contract");

				const auto source_policy_reference = require_string(root, "source_policy", "source_policy");
				const auto source_policy_path = resolve_config_reference(
					source_path_,
					source_policy_reference,
					config_root_,
					"source_policy");

				const auto fastdds_profile_reference = require_string(root, "fastdds_profile", "fastdds_profile");
				const auto fastdds_profile_path = resolve_config_reference(
					source_path_,
					fastdds_profile_reference,
					config_root_,
					"fastdds_profile");

				const auto enabled_sources = parse_enabled_sources(require_field(root, "enabled_sources", "enabled_sources"), "enabled_sources");

				const auto clock_mode = parse_clock_mode(require_field(root, "clock_mode", "clock_mode"), "clock_mode");

				const auto use_sim_time = require_bool(root, "use_sim_time", "use_sim_time");

				const auto ingress = parse_ingress(require_field(root, "ingress", "ingress"), "ingress");

				const auto record_topics = parse_record_topics(require_field(root, "record_topics", "record_topics"), "record_topics");

				ProfileCommon result{
					schema_version,
					contract_path,
					source_policy_path,
					fastdds_profile_path,
					enabled_sources,
					clock_mode,
					use_sim_time,
					ingress,
					record_topics};

				validate_common_cross_fields(result);
				return result;
			}

			RobotGeometry parse_robot_geometry(
				const YAML::Node &node,
				const std::string &path) const
			{
				check_keys(
					node,
					{"wheel_separation_m", "wheel_radius_m"},
					path);

				const double wheel_separation_m = require_double(
					node, "wheel_separation_m", child_path(path, "wheel_separation_m"));
				const double wheel_radius_m = require_double(
					node, "wheel_radius_m", child_path(path, "wheel_radius_m"));

				if (wheel_separation_m <= 0.0)
				{
					fail(
						child_path(path, "wheel_separation_m"),
						"non-positive wheel separation", "positive finite meters",
						std::to_string(wheel_separation_m),
						"使用大于 0 的机器人轮距");
				}

				if (wheel_radius_m <= 0.0)
				{
					fail(
						child_path(path, "wheel_radius_m"),
						"non-positive wheel radius", "positive finite meters",
						std::to_string(wheel_radius_m),
						"使用大于 0 的驱动轮半径");
				}

				RobotGeometry result{
					wheel_separation_m,
					wheel_radius_m};
				return result;
			}

			RobotAdapterConfig parse_robot_adapter(
				const YAML::Node &node,
				const std::string &path) const
			{
				check_keys(
					node,
					{"type", "config", "canonical_input_topic", "controller_node_fqn",
					 "controller_output_topic", "controller_command_type", "odometry_topic",
					 "local_watchdog_timeout_ms"},
					path);

				const auto type = require_string(node, "type", child_path(path, "type"));
				if (type != "ros2_control")
				{
					fail(
						child_path(path, "type"), "unsupported Robot adapter type",
						"ros2_control", type,
						"Robot Profile 只能选择 ros2_control adapter");
				}

				const auto config_reference = require_string(
					node, "config", child_path(path, "config"));
				const auto config_path = resolve_config_reference(
					source_path_, config_reference, config_root_, child_path(path, "config"));

				const auto canonical_input_topic = require_string(
					node,
					"canonical_input_topic",
					child_path(path, "canonical_input_topic"));
				const auto controller_node_fqn = require_string(
					node,
					"controller_node_fqn",
					child_path(path, "controller_node_fqn"));
				const auto controller_output_topic = require_string(
					node,
					"controller_output_topic",
					child_path(path, "controller_output_topic"));
				const auto odometry_topic = require_string(
					node,
					"odometry_topic",
					child_path(path, "odometry_topic"));
				if (!is_valid_topic_name(canonical_input_topic))
				{
					fail(
						child_path(path, "canonical_input_topic"), "invalid topic name",
						"absolute ROS topic name", canonical_input_topic,
						"使用以 / 开头且不含通配符的绝对 Topic");
				}
				if (!is_valid_node_fqn(controller_node_fqn))
				{
					fail(
						child_path(path, "controller_node_fqn"), "invalid ROS node FQN",
						"absolute node FQN", controller_node_fqn,
						"填写提供 controller command subscription 的 ros2_control node FQN");
				}
				if (!is_valid_topic_name(controller_output_topic))
				{
					fail(
						child_path(path, "controller_output_topic"), "invalid topic name",
						"absolute ROS topic name", controller_output_topic,
						"使用以 / 开头且不含通配符的绝对 Topic");
				}
				if (!is_valid_topic_name(odometry_topic))
				{
					fail(
						child_path(path, "odometry_topic"), "invalid topic name",
						"absolute ROS topic name", odometry_topic,
						"使用 diff_drive_controller 发布的绝对 odometry Topic");
				}

				const auto controller_command_type = require_string(
					node,
					"controller_command_type",
					child_path(path, "controller_command_type"));
				if (!is_valid_message_type(controller_command_type))
				{
					fail(
						child_path(path, "controller_command_type"), "invalid message type",
						"package/msg/Type", controller_command_type,
						"使用 ROS message 类型，不要填写 srv 或省略 msg");
				}

				const auto local_watchdog_timeout_ms = require_positive_uint(
					node,
					"local_watchdog_timeout_ms",
					child_path(path, "local_watchdog_timeout_ms"));

				RobotAdapterConfig result{
					config_path,
					canonical_input_topic,
					controller_node_fqn,
					controller_output_topic,
					controller_command_type,
					odometry_topic,
					local_watchdog_timeout_ms};

				return result;
			}

			RobotResources parse_robot_resources(
				const YAML::Node &node,
				const std::string &path) const
			{
				check_keys(
					node,
					{"package", "robot_description", "world", "map_yaml", "nav2_params"},
					path);

				auto package = require_string(node, "package", child_path(path, "package"));
				if (!is_valid_package_name(package))
				{
					fail(
						child_path(path, "package"), "invalid ROS package name",
						"lowercase package name", package,
						"使用小写字母开头且只包含小写字母、数字、下划线和连字符的 package 名称");
				}

				RobotResources result{
					std::move(package),
					parse_package_resource_path(
						node, "robot_description", child_path(path, "robot_description")),
					parse_package_resource_path(node, "world", child_path(path, "world")),
					parse_package_resource_path(node, "map_yaml", child_path(path, "map_yaml")),
					parse_package_resource_path(
						node, "nav2_params", child_path(path, "nav2_params"))};
				return result;
			}

			RobotFrames parse_robot_frames(
				const YAML::Node &node,
				const std::string &path) const
			{
				check_keys(node, {"map", "odom", "base_footprint", "base_link", "laser"}, path);

				RobotFrames result{
					require_frame_id(node, "map", child_path(path, "map")),
					require_frame_id(node, "odom", child_path(path, "odom")),
					require_frame_id(
						node, "base_footprint", child_path(path, "base_footprint")),
					require_frame_id(node, "base_link", child_path(path, "base_link")),
					require_frame_id(node, "laser", child_path(path, "laser"))};

				std::set<std::string> unique_frames;
				const auto add_frame =
					[this, &unique_frames, &path](
						const std::string &key,
						const std::string &frame_id)
					{
						if (!unique_frames.emplace(frame_id).second)
						{
							fail(
								child_path(path, key), "duplicate Robot frame id",
								"unique frame id", frame_id,
								"为 map、odom、base 和 laser 配置不同的 frame id");
						}
					};

				add_frame("map", result.map);
				add_frame("odom", result.odom);
				add_frame("base_footprint", result.base_footprint);
				add_frame("base_link", result.base_link);
				add_frame("laser", result.laser);

				return result;
			}

			RobotHealth parse_robot_health(
				const YAML::Node &node,
				const std::string &path) const
			{
				check_keys(
					node,
					{"vehicle_state_publish_period_ms", "tf_lookup_timeout_ms", "tf_max_age_ms",
					 "controller_state_timeout_ms", "odometry_timeout_ms"},
					path);

				RobotHealth result{
					require_positive_uint(
						node, "vehicle_state_publish_period_ms",
						child_path(path, "vehicle_state_publish_period_ms")),
					require_positive_uint(
						node, "tf_lookup_timeout_ms", child_path(path, "tf_lookup_timeout_ms")),
					require_positive_uint(
						node, "tf_max_age_ms", child_path(path, "tf_max_age_ms")),
					require_positive_uint(
						node, "controller_state_timeout_ms",
						child_path(path, "controller_state_timeout_ms")),
					require_positive_uint(
						node, "odometry_timeout_ms", child_path(path, "odometry_timeout_ms"))};
				return result;
			}

			FixedDemoGoal parse_fixed_demo_goal(
				const YAML::Node &node,
				const std::string &path) const
			{
				check_keys(node, {"frame_id", "x_m", "y_m", "yaw_rad"}, path);

				FixedDemoGoal result{
					require_frame_id(node, "frame_id", child_path(path, "frame_id")),
					require_double(node, "x_m", child_path(path, "x_m")),
					require_double(node, "y_m", child_path(path, "y_m")),
					require_double(node, "yaw_rad", child_path(path, "yaw_rad"))};
				return result;
			}

			AdasAdapterConfig parse_adas_adapter(
				const YAML::Node &node,
				const std::string &path) const
			{
				check_keys(
					node,
					{"type", "config", "canonical_input_topic", "vehicle_state_output_topic",
					 "interface", "poll_timeout_ms", "tx_period_ms",
					 "vehicle_state_publish_period_ms", "local_watchdog_timeout_ms",
					 "can_state_frame_timeout_ms", "recovery_valid_frames"},
					path);

				const auto type = require_string(node, "type", child_path(path, "type"));
				if (type != "socketcan")
				{
					fail(
						child_path(path, "type"), "unsupported ADAS adapter type", "socketcan",
						type, "ADAS Profile 只能选择 SocketCAN adapter");
				}

				const auto config_reference = require_string(
					node, "config", child_path(path, "config"));
				const auto config_path = resolve_config_reference(
					source_path_, config_reference, config_root_, child_path(path, "config"));

				const auto canonical_input_topic = require_string(
					node, "canonical_input_topic", child_path(path, "canonical_input_topic"));
				if (!is_valid_topic_name(canonical_input_topic))
				{
					fail(
						child_path(path, "canonical_input_topic"), "invalid topic name",
						"absolute ROS topic name", canonical_input_topic,
						"使用以 / 开头且不含通配符的 canonical input Topic");
				}

				const auto vehicle_state_output_topic = require_string(
					node, "vehicle_state_output_topic",
					child_path(path, "vehicle_state_output_topic"));
				if (!is_valid_topic_name(vehicle_state_output_topic))
				{
					fail(
						child_path(path, "vehicle_state_output_topic"), "invalid topic name",
						"absolute ROS topic name", vehicle_state_output_topic,
						"使用以 / 开头且不含通配符的 VehicleState Topic");
				}

				const auto interface = require_string(
					node, "interface", child_path(path, "interface"));
				if (!is_valid_network_interface_name(interface))
				{
					fail(
						child_path(path, "interface"), "invalid Linux network interface name",
						"1-15 characters from A-Za-z0-9_.-", interface,
						"填写当前 Linux 主机上的 SocketCAN 或 vcan interface 名称");
				}

				AdasAdapterConfig result{
					config_path,
					canonical_input_topic,
					vehicle_state_output_topic,
					interface,
					require_positive_uint(
						node, "poll_timeout_ms", child_path(path, "poll_timeout_ms")),
					require_positive_uint(
						node, "tx_period_ms", child_path(path, "tx_period_ms")),
					require_positive_uint(
						node, "vehicle_state_publish_period_ms",
						child_path(path, "vehicle_state_publish_period_ms")),
					require_positive_uint(
						node, "local_watchdog_timeout_ms",
						child_path(path, "local_watchdog_timeout_ms")),
					require_positive_uint(
						node, "can_state_frame_timeout_ms",
						child_path(path, "can_state_frame_timeout_ms")),
					require_positive_uint(
						node, "recovery_valid_frames",
						child_path(path, "recovery_valid_frames"))};
				return result;
			}

			VehicleSimulatorConfig parse_vehicle_simulator(
				const YAML::Node &node,
				const std::string &path) const
			{
				check_keys(
					node,
					{"state_period_ms", "local_watchdog_timeout_ms",
					 "first_order_time_constant_ms"},
					path);

				VehicleSimulatorConfig result{
					require_positive_uint(
						node, "state_period_ms", child_path(path, "state_period_ms")),
					require_positive_uint(
						node, "local_watchdog_timeout_ms",
						child_path(path, "local_watchdog_timeout_ms")),
					require_positive_uint(
						node, "first_order_time_constant_ms",
						child_path(path, "first_order_time_constant_ms"))};
				return result;
			}

			ReplayConfig parse_replay(
				const YAML::Node &node,
				const std::string &path) const
			{
				check_keys(node, {"input_namespace"}, path);

				const auto input_namespace = require_string(
					node, "input_namespace", child_path(path, "input_namespace"));
				if (!is_valid_topic_name(input_namespace))
				{
					fail(
						child_path(path, "input_namespace"), "invalid replay namespace",
						"absolute ROS namespace", input_namespace,
						"使用以 / 开头且不含通配符的 replay namespace");
				}

				ReplayConfig result{input_namespace};
				return result;
			}

			void validate_common_cross_fields(const ProfileCommon &common) const
			{
				switch (common.clock_mode)
				{
					case ClockMode::kSim:
						if (!common.use_sim_time)
						{
							fail(
								"use_sim_time", "clock configuration mismatch",
								"true when clock_mode=sim", "false",
								"仿真时钟模式必须启用 ROS 仿真时间");
						}
						break;

					case ClockMode::kSystem:
						if (common.use_sim_time)
						{
							fail(
								"use_sim_time", "clock configuration mismatch",
								"false when clock_mode=system", "true",
								"系统时钟模式不得启用 ROS 仿真时间");
						}
						break;

					default:
						fail(
							"clock_mode", "unsupported parsed clock mode", "sim|system",
							std::to_string(static_cast<int>(common.clock_mode)),
							"检查 Profile parser 的枚举转换和 model 构造路径");
				}

				for (std::size_t index = 0; index < common.enabled_sources.size(); ++index)
				{
					const auto &source = common.enabled_sources[index];
					if (common.ingress.count(source) == 0U)
					{
						fail(
							"enabled_sources[" + std::to_string(index) + "]",
							"enabled source has no ingress binding",
							"matching ingress key", source,
							"为每个启用来源配置同名 ingress 映射");
					}
				}

				for (const auto &entry : common.ingress)
				{
					if (std::find(
						common.enabled_sources.begin(), common.enabled_sources.end(),
						entry.first) == common.enabled_sources.end())
					{
						fail(
							"ingress." + entry.first, "ingress source is not enabled",
							"source id listed in enabled_sources", entry.first,
							"删除多余 ingress 或将该来源加入 enabled_sources");
					}
				}
			}

			void validate_robot_cross_fields(const RobotProfile &profile) const
			{
				if (profile.adapter.canonical_input_topic ==
						profile.adapter.controller_output_topic ||
					profile.adapter.canonical_input_topic ==
						profile.adapter.odometry_topic ||
					profile.adapter.controller_output_topic ==
						profile.adapter.odometry_topic)
				{
					fail(
						"adapter", "Robot adapter Topics overlap",
						"distinct canonical input, controller output and odometry Topics",
						profile.adapter.canonical_input_topic + "," +
						profile.adapter.controller_output_topic + "," +
						profile.adapter.odometry_topic,
						"为命令输入、控制器输出和执行反馈分别配置独立 Topic");
				}

				if (profile.fixed_demo_goal.frame_id != profile.frames.map)
				{
					fail(
						"fixed_demo_goal.frame_id", "demo goal frame does not match map frame",
						profile.frames.map, profile.fixed_demo_goal.frame_id,
						"将固定导航目标定义在当前 Robot Profile 的 map frame 中");
				}
			}

			void validate_adas_cross_fields(const AdasProfile &profile) const
			{
				if (profile.adapter.poll_timeout_ms > profile.adapter.tx_period_ms)
				{
					fail(
						"adapter.poll_timeout_ms", "poll timeout exceeds TX period",
						"less than or equal to " +
						std::to_string(profile.adapter.tx_period_ms) + " ms",
						std::to_string(profile.adapter.poll_timeout_ms) + " ms",
						"缩短 poll 阻塞时间，避免 I/O thread 睡过 CAN 发送周期");
				}

				if (profile.adapter.tx_period_ms >= profile.adapter.local_watchdog_timeout_ms)
				{
					fail(
						"adapter.tx_period_ms", "TX period reaches adapter watchdog timeout",
						"less than " +
						std::to_string(profile.adapter.local_watchdog_timeout_ms) + " ms",
						std::to_string(profile.adapter.tx_period_ms) + " ms",
						"为正常 CAN TX 保留至少一个 adapter watchdog 前的发送窗口");
				}

				if (profile.adapter.tx_period_ms >=
					profile.vehicle_simulator.local_watchdog_timeout_ms)
				{
					fail(
						"adapter.tx_period_ms", "TX period reaches simulator watchdog timeout",
						"less than " +
						std::to_string(
							profile.vehicle_simulator.local_watchdog_timeout_ms) +
						" ms",
						std::to_string(profile.adapter.tx_period_ms) + " ms",
						"让正常 control frame 到达频率快于 simulator watchdog");
				}

				if (profile.vehicle_simulator.state_period_ms >=
					profile.adapter.can_state_frame_timeout_ms)
				{
					fail(
						"vehicle_simulator.state_period_ms",
						"simulator state period reaches CAN state timeout",
						"less than " +
						std::to_string(profile.adapter.can_state_frame_timeout_ms) + " ms",
						std::to_string(profile.vehicle_simulator.state_period_ms) + " ms",
						"让正常 CAN state frame 到达频率快于 adapter timeout");
				}

				if (profile.adapter.vehicle_state_publish_period_ms >=
					profile.adapter.can_state_frame_timeout_ms)
				{
					fail(
						"adapter.vehicle_state_publish_period_ms",
						"VehicleState publish period reaches CAN state timeout",
						"less than " +
						std::to_string(profile.adapter.can_state_frame_timeout_ms) + " ms",
						std::to_string(profile.adapter.vehicle_state_publish_period_ms) + " ms",
						"提高 VehicleState 发布频率，及时向 Gateway 暴露 CAN timeout");
				}
			}

			std::filesystem::path source_path_;
			std::filesystem::path config_root_;
		};

		std::string read_text_file(const std::filesystem::path &path)
		{
			std::ifstream input(path);
			if (!input)
			{
				throw ContractError(
					path.string() +
					":root: file open failed; expected=readable YAML file; actual=unreadable; "
					"hint=检查路径和权限");
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

	} // namespace

	GatewayContractPtr parse_gateway_contract_text(
		const std::string_view yaml_text,
		std::string source_name)
	{
		return GatewayContractParser(std::move(source_name)).parse(yaml_text);
	}

	GatewayContractPtr load_gateway_contract(const std::filesystem::path &path)
	{
		const auto yaml_text = read_text_file(path);
		return parse_gateway_contract_text(yaml_text, path.string());
	}

	SourcePolicyPtr parse_source_policy_text(
		const std::string_view yaml_text,
		std::string source_name)
	{
		return SourcePolicyParser(std::move(source_name)).parse(yaml_text);
	}

	SourcePolicyPtr load_source_policy(const std::filesystem::path &path)
	{
		const auto yaml_text = read_text_file(path);
		return parse_source_policy_text(yaml_text, path.string());
	}

	ProfileConfigPtr parse_profile_text(
		const std::string_view yaml_text,
		const std::filesystem::path &source_path,
		const std::filesystem::path &config_root)
	{
		return ProfileParser(source_path, config_root).parse(yaml_text);
	}

	ProfileConfigPtr load_profile(
		const std::filesystem::path &path,
		const std::filesystem::path &config_root)
	{
		const auto canonical_path = canonical_regular_file_within_root(
			path, config_root, path.string(), "root", "profile file");
		const auto yaml_text = read_text_file(canonical_path);
		return parse_profile_text(yaml_text, canonical_path, config_root);
	}

} // namespace control_link_contract
