#pragma once

#include <cstdint>
#include <filesystem>
#include <string>

#include "control_link_contract/model.hpp"

namespace control_link_contract
{
	// Contract 身份把人工版本与程序计算的语义哈希分开，版本号不参与哈希计算
	struct ContractIdentity
	{
		std::string contract_id;
		std::uint64_t contract_version;
		std::string contract_hash;
	};

	// decision_config_hash 标识一次运行中会改变控制行为的已验证配置快照
	struct DecisionConfigIdentity
	{
		std::string profile_id;
		std::string decision_config_hash;
	};

	struct RuntimeConfigIdentity
	{
		ContractIdentity contract;
		DecisionConfigIdentity decision_config;
	};

	// canonical serialization 只接收已验证 model，不读取 YAML 文本或文件路径
	[[nodiscard]] std::string canonical_serialize_gateway_contract(
		const GatewayContract &contract);

	[[nodiscard]] std::string canonical_serialize_decision_config(
		const GatewayContract &contract,
		const SourcePolicy &source_policy,
		const ProfileConfig &profile,
		const CanSignalMap *can_signal_map);

	// X7 ConfigDiff 复用与 decision_config_hash 相同的字段边界，禁止维护第二套比较规则
	[[nodiscard]] std::string canonical_serialize_effective_source_policy(
		const SourcePolicy &source_policy,
		const ProfileConfig &profile);

	[[nodiscard]] std::string canonical_serialize_profile_behavior(
		const ProfileConfig &profile);

	[[nodiscard]] std::string canonical_serialize_can_signal_map_behavior(
		const CanSignalMap &can_signal_map);

	// FastDDS participant 配置不属于决策 hash，但事务必须按文件内容识别 transport 漂移
	[[nodiscard]] std::string sha256_file_content(
		const std::filesystem::path &path);

	[[nodiscard]] RuntimeConfigIdentity make_runtime_config_identity(
		const GatewayContract &contract,
		const SourcePolicy &source_policy,
		const ProfileConfig &profile,
		const CanSignalMap *can_signal_map);
}  // namespace control_link_contract
