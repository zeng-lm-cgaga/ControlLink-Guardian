#include <algorithm>
#include <cctype>
#include <cstddef>
#include <filesystem>
#include <fstream>
#include <sstream>
#include <stdexcept>
#include <string>
#include <variant>

#include <gtest/gtest.h>
#include <yaml-cpp/yaml.h>

#include "control_link_contract/config_identity.hpp"
#include "control_link_contract/contract_bundle.hpp"
#include "control_link_contract/parser.hpp"

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

		bool is_lowercase_sha256(const std::string &value)
		{
			return value.size() == 64U && std::all_of(
				value.begin(), value.end(),
				[](unsigned char character)
				{
					return std::isdigit(character) != 0 ||
						(character >= 'a' && character <= 'f');
				});
		}

		std::string read_text(const std::filesystem::path &path)
		{
			std::ifstream input{path};
			if (!input)
			{
				throw std::runtime_error("failed to read identity test fixture: " + path.string());
			}
			std::ostringstream buffer;
			buffer << input.rdbuf();
			return buffer.str();
		}

		TEST(ConfigIdentity, LoadsStableIdentityForBothProfiles)
		{
			const auto robot = load_contract_bundle(robot_profile_path(), config_root());
			const auto adas = load_contract_bundle(adas_profile_path(), config_root());

			ASSERT_NE(robot, nullptr);
			ASSERT_NE(adas, nullptr);
			EXPECT_EQ(robot->identity.contract.contract_id, "control_link_gateway_v1");
			EXPECT_EQ(robot->identity.contract.contract_version, 1U);
			EXPECT_EQ(robot->identity.decision_config.profile_id, "robot");
			EXPECT_EQ(adas->identity.decision_config.profile_id, "adas");
			EXPECT_TRUE(is_lowercase_sha256(robot->identity.contract.contract_hash));
			EXPECT_TRUE(is_lowercase_sha256(
				robot->identity.decision_config.decision_config_hash));
			EXPECT_TRUE(is_lowercase_sha256(robot->fastdds_profile_hash));
			EXPECT_EQ(robot->fastdds_profile_hash, adas->fastdds_profile_hash);
			for (std::size_t repetition = 0U; repetition < 100U; ++repetition)
			{
				const auto repeated = make_runtime_config_identity(
					*robot->gateway_contract,
					*robot->source_policy,
					*robot->profile,
					nullptr);
				EXPECT_EQ(
					repeated.contract.contract_hash,
					robot->identity.contract.contract_hash);
				EXPECT_EQ(
					repeated.decision_config.decision_config_hash,
					robot->identity.decision_config.decision_config_hash);
			}
			EXPECT_EQ(
				robot->identity.contract.contract_hash,
				adas->identity.contract.contract_hash);
			EXPECT_NE(
				robot->identity.decision_config.decision_config_hash,
				adas->identity.decision_config.decision_config_hash);
			EXPECT_EQ(robot->can_signal_map, nullptr);
			ASSERT_NE(adas->can_signal_map, nullptr);
			EXPECT_EQ(adas->can_signal_map->protocol_id, "control_link_demo_can_v1");
		}

		TEST(ConfigIdentity, IgnoresYamlCommentsWhitespaceAndMapPresentation)
		{
			const auto contract_path = config_root() / "common/gateway_contract.yaml";
			const auto original = load_gateway_contract(contract_path);
			const auto reformatted_text =
				std::string{"# presentation-only change\n\n"} +
				YAML::Dump(YAML::Load(read_text(contract_path))) +
				"\n# trailing comment\n";
			const auto reformatted = parse_gateway_contract_text(
				reformatted_text,
				"reformatted_contract.yaml");

			EXPECT_EQ(
				canonical_serialize_gateway_contract(*original),
				canonical_serialize_gateway_contract(*reformatted));
		}

		TEST(ConfigIdentity, IgnoresMetadataPathsAndSemanticallyUnorderedSequences)
		{
			const auto bundle = load_contract_bundle(robot_profile_path(), config_root());
			auto contract = *bundle->gateway_contract;
			auto source_policy = *bundle->source_policy;
			auto profile = *bundle->profile;
			auto &robot = std::get<RobotProfile>(profile);

			contract.contract_id = "renamed_contract_family";
			contract.contract_version = 42U;
			source_policy.policy_id = "renamed_source_policy_family";
			std::reverse(
				contract.critical_endpoints.begin(),
				contract.critical_endpoints.end());
			std::reverse(
				robot.common.enabled_sources.begin(),
				robot.common.enabled_sources.end());
			robot.common.contract_path = "/different/install/root/gateway_contract.yaml";
			robot.common.source_policy_path = "/different/install/root/source_policy.yaml";
			robot.common.fastdds_profile_path = "/different/install/root/fastdds.xml";
			robot.adapter.config_path = "/different/install/root/ros2_control.yaml";
			std::reverse(robot.common.record_topics.begin(), robot.common.record_topics.end());
			robot.fixed_demo_goal.x_m = 99.0;

			const auto identity = make_runtime_config_identity(
				contract,
				source_policy,
				profile,
				nullptr);

			EXPECT_EQ(
				identity.contract.contract_hash,
				bundle->identity.contract.contract_hash);
			EXPECT_EQ(
				identity.decision_config.decision_config_hash,
				bundle->identity.decision_config.decision_config_hash);
			EXPECT_EQ(identity.contract.contract_id, "renamed_contract_family");
			EXPECT_EQ(identity.contract.contract_version, 42U);
		}

		TEST(ConfigIdentity, IgnoresFieldsWithoutRuntimeBehavior)
		{
			const auto robot_bundle = load_contract_bundle(robot_profile_path(), config_root());
			auto source_policy = *robot_bundle->source_policy;
			source_policy.sources.at("planning").priority -= 1U;
			const auto unused_source_identity = make_runtime_config_identity(
				*robot_bundle->gateway_contract,
				source_policy,
				*robot_bundle->profile,
				nullptr);
			EXPECT_EQ(
				unused_source_identity.decision_config.decision_config_hash,
				robot_bundle->identity.decision_config.decision_config_hash);

			auto keep_all_contract = *robot_bundle->gateway_contract;
			auto &qos = keep_all_contract.qos_profiles.at("control_input");
			qos.history = HistoryPolicy::kKeepAll;
			qos.depth = 1U;
			const auto first = make_runtime_config_identity(
				keep_all_contract,
				*robot_bundle->source_policy,
				*robot_bundle->profile,
				nullptr);
			qos.depth = 999U;
			const auto second = make_runtime_config_identity(
				keep_all_contract,
				*robot_bundle->source_policy,
				*robot_bundle->profile,
				nullptr);
			EXPECT_EQ(first.contract.contract_hash, second.contract.contract_hash);
			EXPECT_EQ(
				first.decision_config.decision_config_hash,
				second.decision_config.decision_config_hash);

			const auto adas_bundle = load_contract_bundle(adas_profile_path(), config_root());
			auto descriptive_map = *adas_bundle->can_signal_map;
			descriptive_map.description = "presentation-only description";
			const auto descriptive_identity = make_runtime_config_identity(
				*adas_bundle->gateway_contract,
				*adas_bundle->source_policy,
				*adas_bundle->profile,
				&descriptive_map);
			EXPECT_EQ(
				descriptive_identity.decision_config.decision_config_hash,
				adas_bundle->identity.decision_config.decision_config_hash);
		}

		TEST(ConfigIdentity, ChangesHashesWhenBehaviorChanges)
		{
			const auto robot_bundle = load_contract_bundle(robot_profile_path(), config_root());
			auto changed_contract = *robot_bundle->gateway_contract;
			changed_contract.gateway.command_timeout_ms += 1U;
			const auto changed_contract_identity = make_runtime_config_identity(
				changed_contract,
				*robot_bundle->source_policy,
				*robot_bundle->profile,
				nullptr);
			EXPECT_NE(
				changed_contract_identity.contract.contract_hash,
				robot_bundle->identity.contract.contract_hash);
			EXPECT_NE(
				changed_contract_identity.decision_config.decision_config_hash,
				robot_bundle->identity.decision_config.decision_config_hash);

			auto changed_policy = *robot_bundle->source_policy;
			changed_policy.sources.at("teleop").priority -= 1U;
			const auto changed_policy_identity = make_runtime_config_identity(
				*robot_bundle->gateway_contract,
				changed_policy,
				*robot_bundle->profile,
				nullptr);
			EXPECT_EQ(
				changed_policy_identity.contract.contract_hash,
				robot_bundle->identity.contract.contract_hash);
			EXPECT_NE(
				changed_policy_identity.decision_config.decision_config_hash,
				robot_bundle->identity.decision_config.decision_config_hash);

			auto changed_robot_profile = *robot_bundle->profile;
			std::get<RobotProfile>(changed_robot_profile).health.tf_max_age_ms += 1U;
			const auto changed_profile_identity = make_runtime_config_identity(
				*robot_bundle->gateway_contract,
				*robot_bundle->source_policy,
				changed_robot_profile,
				nullptr);
			EXPECT_NE(
				changed_profile_identity.decision_config.decision_config_hash,
				robot_bundle->identity.decision_config.decision_config_hash);

			const auto adas_bundle = load_contract_bundle(adas_profile_path(), config_root());
			auto changed_map = *adas_bundle->can_signal_map;
			changed_map.control_frame.target_speed_mps.scale = 0.02;
			const auto changed_map_identity = make_runtime_config_identity(
				*adas_bundle->gateway_contract,
				*adas_bundle->source_policy,
				*adas_bundle->profile,
				&changed_map);
			EXPECT_NE(
				changed_map_identity.decision_config.decision_config_hash,
				adas_bundle->identity.decision_config.decision_config_hash);
		}

		TEST(ConfigIdentity, RejectsProfileAndCanMapMismatch)
		{
			const auto robot = load_contract_bundle(robot_profile_path(), config_root());
			const auto adas = load_contract_bundle(adas_profile_path(), config_root());

			EXPECT_THROW(
				canonical_serialize_decision_config(
					*robot->gateway_contract,
					*robot->source_policy,
					*robot->profile,
					adas->can_signal_map.get()),
				std::invalid_argument);
			EXPECT_THROW(
				canonical_serialize_decision_config(
					*adas->gateway_contract,
					*adas->source_policy,
					*adas->profile,
					nullptr),
				std::invalid_argument);
		}
	}  // namespace
}  // namespace control_link_contract
