#pragma once

#include <filesystem>
#include <stdexcept>
#include <string>
#include <string_view>

#include "control_link_contract/model.hpp"

namespace control_link_contract
{

	// 所有解析失败统一携带 source:path、rule、expected、actual 和 hint
	// 调用方只按异常类型处理失败，diagnostics 可以直接保留完整定位信息
	class ContractError final : public std::runtime_error
	{
		public:
		using std::runtime_error::runtime_error;
	};

	// 解析内存文本，source_name 仅用于生成可定位的结构化错误
	GatewayContractPtr parse_gateway_contract_text(
		std::string_view yaml_text,
		std::string source_name = "<memory>");

	// 从唯一配置源加载文件；读取或解析失败时抛出 ContractError
	GatewayContractPtr load_gateway_contract(const std::filesystem::path & path);

	// 解析内存中的 Source Policy YAML，source_name 用于生成可定位的结构化错误
	SourcePolicyPtr parse_source_policy_text(
		std::string_view yaml_text,
		std::string source_name = "<memory>");

	// 从唯一配置源加载 Source Policy 文件；读取或解析失败时抛出 ContractError
	SourcePolicyPtr load_source_policy(const std::filesystem::path & path);

	// 解析项目自有 vcan 演示协议，固定校验 byte layout、CRC、counter 和 frame flags 规则
	CanSignalMapPtr parse_can_signal_map_text(
		std::string_view yaml_text,
		std::string source_name = "<memory>");

	// 从唯一配置源加载 CAN signal map；读取或 schema 失败时抛出 ContractError
	CanSignalMapPtr load_can_signal_map(const std::filesystem::path & path);

	// 解析内存中的 Profile；source_path 提供相对配置引用基准，config_root 限制引用边界
	ProfileConfigPtr parse_profile_text(
		std::string_view yaml_text,
		const std::filesystem::path & source_path,
		const std::filesystem::path & config_root);

	// 规范化并读取 config_root 内的 Profile 文件，再进入同一内存解析链路
	ProfileConfigPtr load_profile(
		const std::filesystem::path & path,
		const std::filesystem::path & config_root);

}  // namespace control_link_contract
