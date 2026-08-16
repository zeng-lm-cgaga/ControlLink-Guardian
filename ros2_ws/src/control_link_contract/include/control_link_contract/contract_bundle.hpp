#pragma once

#include <filesystem>
#include <memory>

#include "control_link_contract/config_identity.hpp"
#include "control_link_contract/model.hpp"

namespace control_link_contract
{
	// Bundle 是 Profile、GatewayContract、SourcePolicy 的统一跨文件校验边界
	// 只有当前校验函数全部通过后才发布，缺失的跨文件规则不能由调用方静默补默认值
	// Profile、Contract、Policy 与可选 CAN map 共同形成一次启动期间不可变的配置快照
	// identity 只从这些已验证对象计算，不能与运行组件实际消费的配置发生分叉
	struct ContractBundle
	{
		ProfileConfigPtr profile;
		GatewayContractPtr gateway_contract;
		SourcePolicyPtr source_policy;
		CanSignalMapPtr can_signal_map;
		std::string fastdds_profile_hash;
		RuntimeConfigIdentity identity;
	};

	using ContractBundlePtr = std::shared_ptr<const ContractBundle>;

	// Profile 决定另外两个文件的引用，config_root 同时限制主文件和所有间接引用
	[[nodiscard]] ContractBundlePtr load_contract_bundle(
		const std::filesystem::path &profile_path,
		const std::filesystem::path &config_root);
}  // namespace control_link_contract
