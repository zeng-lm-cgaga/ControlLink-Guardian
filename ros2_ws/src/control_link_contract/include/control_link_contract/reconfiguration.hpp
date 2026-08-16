#pragma once

#include <cstdint>
#include <stdexcept>
#include <string>
#include <vector>

#include "control_link_contract/contract_bundle.hpp"

namespace control_link_contract
{
	enum class ConfigChangeClass : std::uint8_t
	{
		kIdentityOnly,
		kEndpointRebuildRequired,
		kProfileRestartRequired,
		kIncompatible,
	};

	enum class ReconfigurationPrecheckErrorCode : std::uint8_t
	{
		kIncompleteBundle,
		kMissingExpectedHash,
		kStaleCurrentHash,
	};

	class ReconfigurationPrecheckError final : public std::runtime_error
	{
	public:
		ReconfigurationPrecheckError(
			ReconfigurationPrecheckErrorCode code,
			std::string message);

		[[nodiscard]] ReconfigurationPrecheckErrorCode code() const noexcept;

	private:
		ReconfigurationPrecheckErrorCode code_;
	};

	struct ConfigDiff
	{
		ConfigChangeClass classification{ConfigChangeClass::kIdentityOnly};
		RuntimeConfigIdentity current_identity;
		RuntimeConfigIdentity candidate_identity;
		bool gateway_contract_changed{false};
		bool source_policy_changed{false};
		bool profile_behavior_changed{false};
		bool can_signal_map_changed{false};
		bool fastdds_profile_changed{false};
		bool contract_metadata_changed{false};
		std::vector<std::string> changed_components;
		std::string reason;

		[[nodiscard]] bool no_op() const noexcept;
		[[nodiscard]] bool requires_gateway_rebuild() const noexcept;
		[[nodiscard]] bool requires_profile_restart() const noexcept;
		[[nodiscard]] bool compatible() const noexcept;
	};

	struct ReconfigurationPlan
	{
		ContractBundlePtr current;
		ContractBundlePtr candidate;
		ConfigDiff diff;
	};

	[[nodiscard]] const char *config_change_class_name(
		ConfigChangeClass classification) noexcept;

	[[nodiscard]] ConfigDiff diff_contract_bundles(
		const ContractBundle &current,
		const ContractBundle &candidate);

	// expected_current_hash 是调用者读取 current 后取得的乐观并发令牌
	[[nodiscard]] ReconfigurationPlan build_reconfiguration_plan(
		ContractBundlePtr current,
		ContractBundlePtr candidate,
		const std::string &expected_current_hash);
}  // namespace control_link_contract
