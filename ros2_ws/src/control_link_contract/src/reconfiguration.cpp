#include "control_link_contract/reconfiguration.hpp"

#include <utility>

#include "control_link_contract/config_identity.hpp"

namespace control_link_contract
{
	namespace
	{
		void require_complete_bundle(const ContractBundle &bundle)
		{
			if (!bundle.profile || !bundle.gateway_contract || !bundle.source_policy)
			{
				throw ReconfigurationPrecheckError(
					ReconfigurationPrecheckErrorCode::kIncompleteBundle,
					"reconfiguration precheck requires a complete ContractBundle");
			}
			if (bundle.fastdds_profile_hash.size() != 64U)
			{
				throw ReconfigurationPrecheckError(
					ReconfigurationPrecheckErrorCode::kIncompleteBundle,
					"reconfiguration precheck requires a FastDDS content hash");
			}
			const bool adas = std::holds_alternative<AdasProfile>(*bundle.profile);
			if (adas != static_cast<bool>(bundle.can_signal_map))
			{
				throw ReconfigurationPrecheckError(
					ReconfigurationPrecheckErrorCode::kIncompleteBundle,
					"reconfiguration precheck received a Profile/CAN map mismatch");
			}
		}

		bool can_signal_map_changed(
			const ContractBundle &current,
			const ContractBundle &candidate)
		{
			if (static_cast<bool>(current.can_signal_map) !=
				static_cast<bool>(candidate.can_signal_map))
			{
				return true;
			}
			if (!current.can_signal_map)
			{
				return false;
			}
			return canonical_serialize_can_signal_map_behavior(*current.can_signal_map) !=
				canonical_serialize_can_signal_map_behavior(*candidate.can_signal_map);
		}

		void append_changed(
			std::vector<std::string> &components,
			bool changed,
			const char *name)
		{
			if (changed)
			{
				components.emplace_back(name);
			}
		}
	}  // namespace

	ReconfigurationPrecheckError::ReconfigurationPrecheckError(
		ReconfigurationPrecheckErrorCode code,
		std::string message)
		: std::runtime_error(std::move(message)),
		  code_(code)
	{
	}

	ReconfigurationPrecheckErrorCode ReconfigurationPrecheckError::code() const noexcept
	{
		return code_;
	}

	bool ConfigDiff::no_op() const noexcept
	{
		return classification == ConfigChangeClass::kIdentityOnly;
	}

	bool ConfigDiff::requires_gateway_rebuild() const noexcept
	{
		return classification == ConfigChangeClass::kEndpointRebuildRequired;
	}

	bool ConfigDiff::requires_profile_restart() const noexcept
	{
		return classification == ConfigChangeClass::kProfileRestartRequired;
	}

	bool ConfigDiff::compatible() const noexcept
	{
		return classification != ConfigChangeClass::kIncompatible;
	}

	const char *config_change_class_name(ConfigChangeClass classification) noexcept
	{
		switch (classification)
		{
		case ConfigChangeClass::kIdentityOnly:
			return "IDENTITY_ONLY";
		case ConfigChangeClass::kEndpointRebuildRequired:
			return "ENDPOINT_REBUILD_REQUIRED";
		case ConfigChangeClass::kProfileRestartRequired:
			return "PROFILE_RESTART_REQUIRED";
		case ConfigChangeClass::kIncompatible:
			return "INCOMPATIBLE";
		}
		return "INCOMPATIBLE";
	}

	ConfigDiff diff_contract_bundles(
		const ContractBundle &current,
		const ContractBundle &candidate)
	{
		require_complete_bundle(current);
		require_complete_bundle(candidate);

		ConfigDiff result;
		result.current_identity = current.identity;
		result.candidate_identity = candidate.identity;
		result.gateway_contract_changed =
			canonical_serialize_gateway_contract(*current.gateway_contract) !=
			canonical_serialize_gateway_contract(*candidate.gateway_contract);
		result.source_policy_changed =
			canonical_serialize_effective_source_policy(
				*current.source_policy, *current.profile) !=
			canonical_serialize_effective_source_policy(
				*candidate.source_policy, *candidate.profile);
		result.profile_behavior_changed =
			canonical_serialize_profile_behavior(*current.profile) !=
			canonical_serialize_profile_behavior(*candidate.profile);
		result.can_signal_map_changed = can_signal_map_changed(current, candidate);
		result.fastdds_profile_changed =
			current.fastdds_profile_hash != candidate.fastdds_profile_hash;
		result.contract_metadata_changed =
			current.identity.contract.contract_id != candidate.identity.contract.contract_id ||
			current.identity.contract.contract_version !=
				candidate.identity.contract.contract_version;

		append_changed(
			result.changed_components,
			result.gateway_contract_changed,
			"gateway_contract");
		append_changed(
			result.changed_components,
			result.source_policy_changed,
			"source_policy");
		append_changed(
			result.changed_components,
			result.profile_behavior_changed,
			"profile_behavior");
		append_changed(
			result.changed_components,
			result.can_signal_map_changed,
			"can_signal_map");
		append_changed(
			result.changed_components,
			result.fastdds_profile_changed,
			"fastdds_profile");
		append_changed(
			result.changed_components,
			result.contract_metadata_changed,
			"contract_metadata");

		if (current.identity.contract.contract_id !=
			candidate.identity.contract.contract_id)
		{
			result.classification = ConfigChangeClass::kIncompatible;
			result.reason = "candidate belongs to a different Contract family";
			return result;
		}
		if (current.identity.decision_config.profile_id !=
			candidate.identity.decision_config.profile_id)
		{
			result.classification = ConfigChangeClass::kIncompatible;
			result.reason = "candidate changes the running Profile kind";
			return result;
		}
		if (candidate.identity.contract.contract_version <
			current.identity.contract.contract_version)
		{
			result.classification = ConfigChangeClass::kIncompatible;
			result.reason = "candidate contract_version moves backwards";
			return result;
		}
		if (result.gateway_contract_changed &&
			candidate.identity.contract.contract_version <=
				current.identity.contract.contract_version)
		{
			result.classification = ConfigChangeClass::kIncompatible;
			result.reason =
				"GatewayContract behavior changed without increasing contract_version";
			return result;
		}

		const bool decision_hash_changed =
			current.identity.decision_config.decision_config_hash !=
			candidate.identity.decision_config.decision_config_hash;
		const bool classified_behavior_change =
			result.gateway_contract_changed || result.source_policy_changed ||
			result.profile_behavior_changed || result.can_signal_map_changed;
		if (decision_hash_changed != classified_behavior_change)
		{
			result.classification = ConfigChangeClass::kIncompatible;
			result.reason = "candidate contains an unclassified decision behavior change";
			return result;
		}

		if (result.profile_behavior_changed || result.can_signal_map_changed ||
			result.fastdds_profile_changed)
		{
			result.classification = ConfigChangeClass::kProfileRestartRequired;
			result.reason =
				"candidate changes Profile, adapter, CAN or participant-owned behavior";
			return result;
		}
		if (result.gateway_contract_changed || result.source_policy_changed)
		{
			result.classification = ConfigChangeClass::kEndpointRebuildRequired;
			result.reason = "candidate changes Gateway-owned runtime behavior";
			return result;
		}

		result.classification = ConfigChangeClass::kIdentityOnly;
		result.reason = "candidate does not change effective runtime behavior";
		return result;
	}

	ReconfigurationPlan build_reconfiguration_plan(
		ContractBundlePtr current,
		ContractBundlePtr candidate,
		const std::string &expected_current_hash)
	{
		if (!current || !candidate)
		{
			throw ReconfigurationPrecheckError(
				ReconfigurationPrecheckErrorCode::kIncompleteBundle,
				"reconfiguration plan requires non-null current and candidate bundles");
		}
		if (expected_current_hash.empty())
		{
			throw ReconfigurationPrecheckError(
				ReconfigurationPrecheckErrorCode::kMissingExpectedHash,
				"expected current decision_config_hash must not be empty");
		}
		if (current->identity.decision_config.decision_config_hash !=
			expected_current_hash)
		{
			throw ReconfigurationPrecheckError(
				ReconfigurationPrecheckErrorCode::kStaleCurrentHash,
				"expected current decision_config_hash is stale");
		}

		auto diff = diff_contract_bundles(*current, *candidate);
		return ReconfigurationPlan{
			std::move(current),
			std::move(candidate),
			std::move(diff)};
	}
}  // namespace control_link_contract
