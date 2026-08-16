#include <filesystem>
#include <memory>
#include <optional>
#include <string>
#include <utility>

#include <gtest/gtest.h>

#include "control_link_contract/config_identity.hpp"
#include "control_link_contract/contract_bundle.hpp"
#include "control_link_contract/reconfiguration.hpp"

namespace control_link_contract
{
	namespace
	{
		std::filesystem::path config_root()
		{
			return std::filesystem::path{CONTROL_LINK_TEST_CONFIG_ROOT};
		}

		std::filesystem::path robot_profile_path()
		{
			return std::filesystem::path{CONTROL_LINK_TEST_ROBOT_PROFILE_PATH};
		}

		std::filesystem::path adas_profile_path()
		{
			return std::filesystem::path{CONTROL_LINK_TEST_ADAS_PROFILE_PATH};
		}

		struct MutableBundle
		{
			GatewayContract gateway_contract;
			SourcePolicy source_policy;
			ProfileConfig profile;
			std::optional<CanSignalMap> can_signal_map;
			std::string fastdds_profile_hash;

			[[nodiscard]] ContractBundlePtr freeze() const
			{
				auto contract = std::make_shared<const GatewayContract>(gateway_contract);
				auto policy = std::make_shared<const SourcePolicy>(source_policy);
				auto frozen_profile = std::make_shared<const ProfileConfig>(profile);
				CanSignalMapPtr frozen_map;
				if (can_signal_map.has_value())
				{
					frozen_map = std::make_shared<const CanSignalMap>(can_signal_map.value());
				}
				return std::make_shared<const ContractBundle>(
					ContractBundle{
						frozen_profile,
						contract,
						policy,
						frozen_map,
						fastdds_profile_hash,
						make_runtime_config_identity(
							*contract,
							*policy,
							*frozen_profile,
							frozen_map.get())});
			}
		};

		MutableBundle mutable_copy(const ContractBundle &bundle)
		{
			return MutableBundle{
				*bundle.gateway_contract,
				*bundle.source_policy,
				*bundle.profile,
				bundle.can_signal_map ?
					std::optional<CanSignalMap>{*bundle.can_signal_map} :
					std::nullopt,
				bundle.fastdds_profile_hash};
		}

		TEST(ReconfigurationPrecheck, TreatsSameSemanticsAsIdempotentNoOp)
		{
			const auto current = load_contract_bundle(robot_profile_path(), config_root());
			auto same = mutable_copy(*current);
			const auto same_candidate = same.freeze();
			const auto plan = build_reconfiguration_plan(
				current,
				same_candidate,
				current->identity.decision_config.decision_config_hash);

			EXPECT_TRUE(plan.diff.no_op());
			EXPECT_TRUE(plan.diff.compatible());
			EXPECT_TRUE(plan.diff.changed_components.empty());
			EXPECT_STREQ(
				config_change_class_name(plan.diff.classification),
				"IDENTITY_ONLY");

			auto metadata_only = mutable_copy(*current);
			metadata_only.gateway_contract.contract_version += 1U;
			const auto metadata_diff = diff_contract_bundles(
				*current,
				*metadata_only.freeze());
			EXPECT_TRUE(metadata_diff.no_op());
			EXPECT_TRUE(metadata_diff.contract_metadata_changed);
			EXPECT_FALSE(metadata_diff.gateway_contract_changed);
		}

		TEST(ReconfigurationPrecheck, RejectsMissingAndStaleExpectedHash)
		{
			const auto current = load_contract_bundle(robot_profile_path(), config_root());
			const auto candidate = mutable_copy(*current).freeze();

			try
			{
				(void)build_reconfiguration_plan(current, candidate, "");
				FAIL() << "missing expected hash was accepted";
			}
			catch (const ReconfigurationPrecheckError &error)
			{
				EXPECT_EQ(
					error.code(),
					ReconfigurationPrecheckErrorCode::kMissingExpectedHash);
			}

			try
			{
				(void)build_reconfiguration_plan(
					current,
					candidate,
					std::string(64U, '0'));
				FAIL() << "stale expected hash was accepted";
			}
			catch (const ReconfigurationPrecheckError &error)
			{
				EXPECT_EQ(
					error.code(),
					ReconfigurationPrecheckErrorCode::kStaleCurrentHash);
			}
		}

		TEST(ReconfigurationPrecheck, ClassifiesGatewayAndPolicyChangesAsRebuild)
		{
			const auto current = load_contract_bundle(robot_profile_path(), config_root());

			auto contract_change = mutable_copy(*current);
			contract_change.gateway_contract.gateway.command_timeout_ms += 1U;
			contract_change.gateway_contract.contract_version += 1U;
			const auto contract_diff = diff_contract_bundles(
				*current,
				*contract_change.freeze());
			EXPECT_TRUE(contract_diff.requires_gateway_rebuild());
			EXPECT_TRUE(contract_diff.gateway_contract_changed);
			EXPECT_FALSE(contract_diff.profile_behavior_changed);

			auto policy_change = mutable_copy(*current);
			policy_change.source_policy.sources.at("teleop").priority -= 1U;
			const auto policy_diff = diff_contract_bundles(
				*current,
				*policy_change.freeze());
			EXPECT_TRUE(policy_diff.requires_gateway_rebuild());
			EXPECT_TRUE(policy_diff.source_policy_changed);
			EXPECT_FALSE(policy_diff.gateway_contract_changed);
		}

		TEST(ReconfigurationPrecheck, ClassifiesPlatformOwnedChangesAsRestart)
		{
			const auto robot = load_contract_bundle(robot_profile_path(), config_root());
			auto robot_change = mutable_copy(*robot);
			std::get<RobotProfile>(robot_change.profile).health.tf_max_age_ms += 1U;
			const auto robot_diff = diff_contract_bundles(
				*robot,
				*robot_change.freeze());
			EXPECT_TRUE(robot_diff.requires_profile_restart());
			EXPECT_TRUE(robot_diff.profile_behavior_changed);

			auto transport_change = mutable_copy(*robot);
			transport_change.fastdds_profile_hash = std::string(64U, 'a');
			const auto transport_diff = diff_contract_bundles(
				*robot,
				*transport_change.freeze());
			EXPECT_TRUE(transport_diff.requires_profile_restart());
			EXPECT_TRUE(transport_diff.fastdds_profile_changed);
			EXPECT_FALSE(transport_diff.profile_behavior_changed);

			auto relocated_same_transport = mutable_copy(*robot);
			std::get<RobotProfile>(relocated_same_transport.profile)
				.common.fastdds_profile_path = "/versioned/config/fastdds.xml";
			const auto relocated_diff = diff_contract_bundles(
				*robot,
				*relocated_same_transport.freeze());
			EXPECT_TRUE(relocated_diff.no_op());
			EXPECT_FALSE(relocated_diff.fastdds_profile_changed);

			const auto adas = load_contract_bundle(adas_profile_path(), config_root());
			auto map_change = mutable_copy(*adas);
			map_change.can_signal_map->control_frame.target_speed_mps.scale = 0.02;
			const auto map_diff = diff_contract_bundles(
				*adas,
				*map_change.freeze());
			EXPECT_TRUE(map_diff.requires_profile_restart());
			EXPECT_TRUE(map_diff.can_signal_map_changed);
		}

		TEST(ReconfigurationPrecheck, RejectsFamilyProfileAndVersionViolations)
		{
			const auto robot = load_contract_bundle(robot_profile_path(), config_root());

			auto family_change = mutable_copy(*robot);
			family_change.gateway_contract.contract_id = "different_contract_family";
			const auto family_diff = diff_contract_bundles(
				*robot,
				*family_change.freeze());
			EXPECT_FALSE(family_diff.compatible());

			const auto adas = load_contract_bundle(adas_profile_path(), config_root());
			const auto profile_diff = diff_contract_bundles(*robot, *adas);
			EXPECT_FALSE(profile_diff.compatible());
			EXPECT_NE(profile_diff.reason.find("Profile kind"), std::string::npos);

			auto missing_version_bump = mutable_copy(*robot);
			missing_version_bump.gateway_contract.gateway.command_timeout_ms += 1U;
			const auto behavior_diff = diff_contract_bundles(
				*robot,
				*missing_version_bump.freeze());
			EXPECT_FALSE(behavior_diff.compatible());
			EXPECT_NE(
				behavior_diff.reason.find("without increasing"),
				std::string::npos);

			auto version_regression = mutable_copy(*robot);
			version_regression.gateway_contract.contract_version = 0U;
			const auto version_diff = diff_contract_bundles(
				*robot,
				*version_regression.freeze());
			EXPECT_FALSE(version_diff.compatible());
			EXPECT_NE(version_diff.reason.find("backwards"), std::string::npos);
		}
	}  // namespace
}  // namespace control_link_contract
