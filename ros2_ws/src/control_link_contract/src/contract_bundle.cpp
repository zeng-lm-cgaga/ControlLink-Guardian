#include "control_link_contract/contract_bundle.hpp"

#include <cstddef>
#include <cstdint>
#include <stdexcept>
#include <string>
#include <utility>
#include <variant>

#include "control_link_contract/parser.hpp"

namespace control_link_contract
{
	namespace
	{
		[[noreturn]] void fail_bundle(
			const std::filesystem::path &source_path,
			const std::string &yaml_path,
			const std::string &rule,
			const std::string &expected,
			const std::string &actual,
			const std::string &hint)
		{
			throw ContractError(
				source_path.string() + ":" + yaml_path + ": " + rule +
				"; expected=" + expected + "; actual=" + actual + "; hint=" + hint);
		}

		const ProfileCommon &profile_common(const ProfileConfig &config)
		{
			// 两种 Profile 都把 common 放在自身对象中，visit 返回的引用随 config 生命周期有效
			return std::visit(
				[](const auto &profile) -> const ProfileCommon &
				{
					return profile.common;
				},
				config);
		}

		std::string node_namespace(const std::string &node_fqn)
		{
			const auto separator = node_fqn.rfind('/');
			if (separator == 0U)
			{
				return "/";
			}
			if (separator == std::string::npos)
			{
				throw std::logic_error("validated Gateway node FQN has no namespace separator");
			}
			return node_fqn.substr(0U, separator);
		}

		void validate_sources(
			const ProfileCommon &common,
			const GatewayContract &contract,
			const SourcePolicy &source_policy,
			const std::filesystem::path &profile_path)
		{
			// 将 Profile 启用来源、SourcePolicy 和 GatewayContract input 前缀收敛成同一条内部边界
			const double minimum_lease_timeout_ms =
				2'000.0 / contract.gateway.output_rate_hz;
			for (std::size_t index = 0; index < common.enabled_sources.size(); ++index)
			{
				const auto &source = common.enabled_sources[index];
				const auto iterator = source_policy.sources.find(source);
				if (iterator == source_policy.sources.end())
				{
					fail_bundle(
						profile_path,
						"enabled_sources[" + std::to_string(index) + "]",
						"enabled source is absent from SourcePolicy",
						"source id defined in " + common.source_policy_path.string(),
						source,
							"在 source_policy.yaml 中定义该来源，或从当前 Profile 移除它");
				}

				if (static_cast<double>(iterator->second.lease_timeout_ms) <
					minimum_lease_timeout_ms)
				{
					fail_bundle(
						common.source_policy_path,
						"sources[" + source + "].lease_timeout_ms",
						"source lease is shorter than two Gateway output periods",
						">= " + std::to_string(minimum_lease_timeout_ms) + " ms",
						std::to_string(iterator->second.lease_timeout_ms) + " ms",
						"增大 lease_timeout_ms 或降低 Gateway output_rate_hz");
				}

				const auto expected_source_topic = contract.input.topic_prefix + "/" + source;
				if (iterator->second.topic != expected_source_topic)
				{
					fail_bundle(
						common.source_policy_path,
						"sources[" + source + "].topic",
						"source topic does not follow GatewayContract input prefix",
						expected_source_topic,
						iterator->second.topic,
						"使用 input.topic_prefix/source_id 作为内部控制 Topic");
				}

				const auto &ingress = common.ingress.at(source);
				if (ingress.output_topic != iterator->second.topic)
				{
					fail_bundle(
						profile_path,
						"ingress." + source + ".output_topic",
						"ingress output topic does not match SourcePolicy",
						iterator->second.topic,
						ingress.output_topic,
						"让 ingress adapter 输出到该 source 唯一绑定的内部 Topic");
				}

				if (iterator->second.type != contract.input.type)
				{
					fail_bundle(
						common.source_policy_path,
						"sources[" + source + "].type",
						"source message type does not match GatewayContract input type",
						contract.input.type,
						iterator->second.type,
						"所有内部 source Topic 必须发布统一的 ControlCommand 类型");
				}
			}
		}

		const StateTopicContract &vehicle_state_contract(const GatewayContract &contract)
		{
			// GatewayContractParser 已强制四个 state topic 存在，这里不复制同一存在性规则
			return contract.state_topics.at("vehicle_state");
		}

		void validate_adapter_watchdog(
			const GatewayContract &contract,
			std::uint64_t local_watchdog_timeout_ms,
			const std::filesystem::path &profile_path)
		{
			const double output_period_ms =
				1'000.0 / contract.gateway.output_rate_hz;
			if (static_cast<double>(local_watchdog_timeout_ms) <= output_period_ms)
			{
				fail_bundle(
					profile_path,
					"adapter.local_watchdog_timeout_ms",
					"adapter watchdog does not exceed Gateway output period",
					"> " + std::to_string(output_period_ms) + " ms",
					std::to_string(local_watchdog_timeout_ms) + " ms",
					"增大 adapter watchdog 或提高 Gateway output_rate_hz，为正常输出留出调度余量");
			}
		}

		void validate_profile_specific(
			const ProfileConfig &profile,
			const GatewayContract &contract,
			const StateTopicContract &vehicle_state,
			const std::filesystem::path &profile_path)
		{
			if (const auto *robot = std::get_if<RobotProfile>(&profile))
			{
				if (robot->adapter.canonical_input_topic != contract.output.topic)
				{
					fail_bundle(
						profile_path,
						"adapter.canonical_input_topic",
						"Robot adapter input does not match canonical output",
						contract.output.topic,
						robot->adapter.canonical_input_topic,
						"让 ros2_control adapter 只消费 Gateway canonical output");
				}

				validate_adapter_watchdog(
					contract,
					robot->adapter.local_watchdog_timeout_ms,
					profile_path);

				if (robot->health.vehicle_state_publish_period_ms >= contract.gateway.vehicle_state_topic_timeout_ms)
				{
					fail_bundle(
						profile_path,
						"health.vehicle_state_publish_period_ms",
						"Robot VehicleState period reaches Gateway topic timeout",
						"less than " +
							std::to_string(contract.gateway.vehicle_state_topic_timeout_ms) + " ms",
						std::to_string(robot->health.vehicle_state_publish_period_ms) + " ms",
						"提高 Robot VehicleState 发布频率，为调度抖动保留余量");
				}

				return;
			}

			const auto &adas = std::get<AdasProfile>(profile);
			const auto live_gateway_namespace = node_namespace(
				contract.gateway.node_fqn);
			if (adas.replay.input_namespace == live_gateway_namespace)
			{
				fail_bundle(
					profile_path,
					"replay.input_namespace",
					"replay namespace overlaps the live Gateway namespace",
					"namespace different from " + live_gateway_namespace,
					adas.replay.input_namespace,
					"使用独立 replay namespace，防止录包消息进入 live Topic 或形成回放自激循环");
			}
			if (adas.adapter.canonical_input_topic != contract.output.topic)
			{
				fail_bundle(
					profile_path,
					"adapter.canonical_input_topic",
					"ADAS adapter input does not match canonical output",
					contract.output.topic,
					adas.adapter.canonical_input_topic,
					"让 SocketCAN adapter 只消费 Gateway canonical output");
			}

			validate_adapter_watchdog(
				contract,
				adas.adapter.local_watchdog_timeout_ms,
				profile_path);

			if (adas.adapter.vehicle_state_output_topic != vehicle_state.topic)
			{
				fail_bundle(
					profile_path,
					"adapter.vehicle_state_output_topic",
					"ADAS VehicleState output does not match GatewayContract",
					vehicle_state.topic,
					adas.adapter.vehicle_state_output_topic,
					"让 SocketCAN adapter 发布到公共 VehicleState Topic");
			}

			if (adas.adapter.vehicle_state_publish_period_ms >=
				contract.gateway.vehicle_state_topic_timeout_ms)
			{
				fail_bundle(
					profile_path,
					"adapter.vehicle_state_publish_period_ms",
					"ADAS VehicleState period reaches Gateway topic timeout",
					"less than " +
						std::to_string(contract.gateway.vehicle_state_topic_timeout_ms) + " ms",
					std::to_string(adas.adapter.vehicle_state_publish_period_ms) + " ms",
					"提高 ADAS VehicleState 发布频率，为调度抖动保留余量");
			}
		}
	}  // namespace

	ContractBundlePtr load_contract_bundle(
		const std::filesystem::path &profile_path,
		const std::filesystem::path &config_root)
	{
		// 先加载 Profile 获取受 config_root 约束的引用，再加载并校验被引用的两个配置
		auto profile = load_profile(profile_path, config_root);
		const auto &common = profile_common(*profile);

		auto gateway_contract =
			load_gateway_contract(common.contract_path);
		auto source_policy =
			load_source_policy(common.source_policy_path);

		validate_sources(
			common,
			*gateway_contract,
			*source_policy,
			profile_path);

		const auto &vehicle_state =
			vehicle_state_contract(*gateway_contract);

		validate_profile_specific(
			*profile,
			*gateway_contract,
			vehicle_state,
			profile_path);

		ContractBundle bundle{
			std::move(profile),
			std::move(gateway_contract),
			std::move(source_policy)};

		return std::make_shared<const ContractBundle>(
			std::move(bundle));
	}
}  // namespace control_link_contract
