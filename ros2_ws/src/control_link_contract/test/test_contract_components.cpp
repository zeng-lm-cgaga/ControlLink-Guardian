#include <algorithm>
#include <atomic>
#include <chrono>
#include <cstdint>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <functional>
#include <memory>
#include <stdexcept>
#include <string>
#include <thread>
#include <variant>
#include <vector>

#include <gtest/gtest.h>
#include <rclcpp/node.hpp>
#include <rclcpp/qos.hpp>
#include <rmw/qos_profiles.h>
#include <std_msgs/msg/string.hpp>
#include <yaml-cpp/yaml.h>

#include "control_link_contract/compatibility.hpp"
#include "control_link_contract/contract_bundle.hpp"
#include "control_link_contract/parser.hpp"
#include "control_link_contract/qos_factory.hpp"

namespace control_link_contract
{
	namespace
	{
		using namespace std::chrono_literals;

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

		GatewayContractPtr load_test_contract()
		{
			return load_gateway_contract(config_root() / "common/gateway_contract.yaml");
		}

		std::filesystem::path create_unique_temporary_directory()
		{
			static std::atomic<std::uint64_t> sequence{0U};
			const auto temporary_root = std::filesystem::temp_directory_path();
			for (std::uint64_t attempt = 0U; attempt < 100U; ++attempt)
			{
				const auto candidate = temporary_root /
					("control_link_contract_" +
					std::to_string(sequence.fetch_add(1U, std::memory_order_relaxed)));
				std::error_code error;
				if (std::filesystem::create_directory(candidate, error))
				{
					return candidate;
				}
				if (error && error != std::errc::file_exists)
				{
					throw std::filesystem::filesystem_error(
						"failed to create temporary Contract directory",
						candidate,
						error);
				}
			}

			throw std::runtime_error("could not allocate a unique temporary Contract directory");
		}

		class TemporaryConfigTree
		{
		public:
			TemporaryConfigTree()
			: root_(create_unique_temporary_directory())
			{
				std::filesystem::copy(
					config_root(),
					root_,
					std::filesystem::copy_options::recursive |
					std::filesystem::copy_options::overwrite_existing);
			}

			~TemporaryConfigTree()
			{
				std::error_code ignored;
				std::filesystem::remove_all(root_, ignored);
			}

			TemporaryConfigTree(const TemporaryConfigTree &) = delete;
			TemporaryConfigTree &operator=(const TemporaryConfigTree &) = delete;

			[[nodiscard]] const std::filesystem::path &root() const noexcept
			{
				return root_;
			}

			[[nodiscard]] std::filesystem::path path(const std::string &relative) const
			{
				return root_ / relative;
			}

		private:
			std::filesystem::path root_;
		};

		void write_yaml(const std::filesystem::path &path, const YAML::Node &root)
		{
			std::ofstream output{path, std::ios::out | std::ios::trunc};
			if (!output)
			{
				throw std::runtime_error("failed to open YAML fixture: " + path.string());
			}
			output << root;
			if (!output)
			{
				throw std::runtime_error("failed to write YAML fixture: " + path.string());
			}
		}

		YAML::Node source_entry(YAML::Node sources, const std::string &source_id)
		{
			for (auto entry : sources)
			{
				if (entry["id"] && entry["id"].as<std::string>() == source_id)
				{
					return entry;
				}
			}
			throw std::runtime_error("source fixture is missing: " + source_id);
		}

		void expect_bundle_rejection(
			const std::string &profile_relative_path,
			const std::string &file_relative_path,
			const std::function<void(YAML::Node &)> &mutate,
			const std::string &expected_error)
		{
			TemporaryConfigTree fixture;
			const auto target_path = fixture.path(file_relative_path);
			auto yaml = YAML::LoadFile(target_path.string());
			mutate(yaml);
			write_yaml(target_path, yaml);

			try
			{
				(void)load_contract_bundle(
					fixture.path(profile_relative_path),
					fixture.root());
				FAIL() << "expected ContractError containing: " << expected_error;
			}
			catch (const ContractError &error)
			{
				EXPECT_NE(std::string{error.what()}.find(expected_error), std::string::npos)
					<< error.what();
			}
		}

		std::vector<rclcpp::TopicEndpointInfo> wait_for_endpoints(
			const std::function<std::vector<rclcpp::TopicEndpointInfo>()> &query,
			const std::string &expected_node_name)
		{
			for (std::size_t attempt = 0U; attempt < 200U; ++attempt)
			{
				auto endpoints = query();
				for (const auto &endpoint : endpoints)
				{
					if (endpoint.node_name() == expected_node_name)
					{
						return endpoints;
					}
				}
				std::this_thread::sleep_for(10ms);
			}

			return {};
		}

		const rclcpp::TopicEndpointInfo &find_endpoint(
			const std::vector<rclcpp::TopicEndpointInfo> &endpoints,
			const std::string &node_name)
		{
			const auto iterator = std::find_if(
				endpoints.begin(),
				endpoints.end(),
				[&node_name](const rclcpp::TopicEndpointInfo &endpoint)
				{
					return endpoint.node_name() == node_name;
				});
			if (iterator == endpoints.end())
			{
				throw std::runtime_error("Graph endpoint is missing for node: " + node_name);
			}
			return *iterator;
		}

		class RosGraphCompatibilityTest : public testing::Test
		{
		protected:
			static void SetUpTestSuite()
			{
				ASSERT_EQ(
					setenv(
						"FASTRTPS_DEFAULT_PROFILES_FILE",
						CONTROL_LINK_TEST_FASTDDS_PROFILE_PATH,
						1),
					0);
				ASSERT_EQ(setenv("RMW_FASTRTPS_USE_QOS_FROM_XML", "0", 1), 0);
				if (!rclcpp::ok())
				{
					rclcpp::init(0, nullptr);
				}
			}

			static void TearDownTestSuite()
			{
				if (rclcpp::ok())
				{
					rclcpp::shutdown();
				}
			}
		};

		TEST(ContractComponents, LoadsSourcePolicyAndBothProfileVariants)
		{
			const auto policy = load_source_policy(config_root() / "common/source_policy.yaml");
			ASSERT_NE(policy, nullptr);
			EXPECT_EQ(policy->schema_version, 1U);
			ASSERT_EQ(policy->sources.size(), 3U);
			EXPECT_EQ(policy->sources.at("teleop").priority, 200U);
			EXPECT_EQ(policy->sources.at("planning").lease_timeout_ms, 120U);

			const auto robot = load_profile(robot_profile_path(), config_root());
			ASSERT_NE(robot, nullptr);
			ASSERT_TRUE(std::holds_alternative<RobotProfile>(*robot));
			const auto &robot_profile = std::get<RobotProfile>(*robot);
			EXPECT_EQ(robot_profile.common.clock_mode, ClockMode::kSim);
			EXPECT_TRUE(robot_profile.common.use_sim_time);
			EXPECT_EQ(robot_profile.common.enabled_sources.size(), 2U);

			const auto adas = load_profile(adas_profile_path(), config_root());
			ASSERT_NE(adas, nullptr);
			ASSERT_TRUE(std::holds_alternative<AdasProfile>(*adas));
			const auto &adas_profile = std::get<AdasProfile>(*adas);
			EXPECT_EQ(adas_profile.common.clock_mode, ClockMode::kSystem);
			EXPECT_FALSE(adas_profile.common.use_sim_time);
			EXPECT_EQ(adas_profile.adapter.interface, "vcan0");
		}

		TEST(ContractComponents, LoadsImmutableBundleForRobotAndAdas)
		{
			const auto robot_bundle = load_contract_bundle(robot_profile_path(), config_root());
			ASSERT_NE(robot_bundle, nullptr);
			ASSERT_NE(robot_bundle->profile, nullptr);
			ASSERT_NE(robot_bundle->gateway_contract, nullptr);
			ASSERT_NE(robot_bundle->source_policy, nullptr);
			EXPECT_TRUE(std::holds_alternative<RobotProfile>(*robot_bundle->profile));
			EXPECT_EQ(robot_bundle->gateway_contract->output.topic,
				"/control_link/output/control_cmd");

			const auto adas_bundle = load_contract_bundle(adas_profile_path(), config_root());
			ASSERT_NE(adas_bundle, nullptr);
			EXPECT_TRUE(std::holds_alternative<AdasProfile>(*adas_bundle->profile));
			EXPECT_EQ(
				std::get<AdasProfile>(*adas_bundle->profile).adapter.vehicle_state_output_topic,
				adas_bundle->gateway_contract->state_topics.at("vehicle_state").topic);
		}

		TEST(ContractComponents, RejectsCrossFileSourceAndIngressDrift)
		{
			expect_bundle_rejection(
				"adas/adas_profile.yaml",
				"common/source_policy.yaml",
				[](YAML::Node &root)
				{
					source_entry(root["sources"], "planning")["id"] = "renamed_planning";
				},
				"enabled source is absent from SourcePolicy");

			expect_bundle_rejection(
				"adas/adas_profile.yaml",
				"common/source_policy.yaml",
				[](YAML::Node &root)
				{
					source_entry(root["sources"], "planning")["lease_timeout_ms"] = 20U;
				},
				"source lease is shorter than two Gateway output periods");

			expect_bundle_rejection(
				"adas/adas_profile.yaml",
				"common/source_policy.yaml",
				[](YAML::Node &root)
				{
					source_entry(root["sources"], "planning")["topic"] =
						"/different/input/planning";
				},
				"source topic does not follow GatewayContract input prefix");

			expect_bundle_rejection(
				"adas/adas_profile.yaml",
				"adas/adas_profile.yaml",
				[](YAML::Node &root)
				{
					root["ingress"]["planning"]["output_topic"] =
						"/control_link/input/planning_drift";
				},
				"ingress output topic does not match SourcePolicy");

			expect_bundle_rejection(
				"adas/adas_profile.yaml",
				"common/source_policy.yaml",
				[](YAML::Node &root)
				{
					source_entry(root["sources"], "planning")["type"] =
						"geometry_msgs/msg/Twist";
				},
				"source message type does not match GatewayContract input type");
		}

		TEST(ContractComponents, RejectsCrossFileAdapterAndHealthDrift)
		{
			expect_bundle_rejection(
				"robot/robot_profile.yaml",
				"robot/robot_profile.yaml",
				[](YAML::Node &root)
				{
					root["adapter"]["canonical_input_topic"] = "/different/control_cmd";
				},
				"Robot adapter input does not match canonical output");

			expect_bundle_rejection(
				"robot/robot_profile.yaml",
				"robot/robot_profile.yaml",
				[](YAML::Node &root)
				{
					root["adapter"]["local_watchdog_timeout_ms"] = 20U;
				},
				"adapter watchdog does not exceed Gateway output period");

			expect_bundle_rejection(
				"robot/robot_profile.yaml",
				"robot/robot_profile.yaml",
				[](YAML::Node &root)
				{
					root["health"]["vehicle_state_publish_period_ms"] = 150U;
				},
				"Robot VehicleState period reaches Gateway topic timeout");

			expect_bundle_rejection(
				"adas/adas_profile.yaml",
				"adas/adas_profile.yaml",
				[](YAML::Node &root)
				{
					root["adapter"]["vehicle_state_output_topic"] =
						"/different/vehicle_state";
				},
				"ADAS VehicleState output does not match GatewayContract");

			expect_bundle_rejection(
				"adas/adas_profile.yaml",
				"adas/adas_profile.yaml",
				[](YAML::Node &root)
				{
					root["adapter"]["vehicle_state_publish_period_ms"] = 150U;
					root["adapter"]["can_state_frame_timeout_ms"] = 200U;
				},
				"ADAS VehicleState period reaches Gateway topic timeout");

			expect_bundle_rejection(
				"adas/adas_profile.yaml",
				"adas/adas_profile.yaml",
				[](YAML::Node &root)
				{
					root["replay"]["input_namespace"] = "/control_link";
				},
				"replay namespace overlaps the live Gateway namespace");
		}

		TEST(ContractComponents, QosFactoryMapsConfiguredPoliciesAndDurations)
		{
			const auto contract = load_test_contract();
			QosFactory factory{contract};
			const auto qos = factory.make("control_input");
			const auto &profile = contract->qos_profiles.at("control_input");

			EXPECT_EQ(qos.reliability(), rclcpp::ReliabilityPolicy::Reliable);
			EXPECT_EQ(qos.durability(), rclcpp::DurabilityPolicy::Volatile);
			EXPECT_EQ(qos.history(), rclcpp::HistoryPolicy::KeepLast);
			EXPECT_EQ(qos.depth(), profile.depth);
			ASSERT_TRUE(profile.deadline_ms.has_value());
			ASSERT_TRUE(profile.lifespan_ms.has_value());
			ASSERT_TRUE(profile.liveliness.has_value());
			ASSERT_TRUE(profile.liveliness_lease_duration_ms.has_value());
			EXPECT_EQ(
				qos.deadline().nanoseconds(),
				static_cast<std::int64_t>(*profile.deadline_ms * 1'000'000U));
			EXPECT_EQ(
				qos.lifespan().nanoseconds(),
				static_cast<std::int64_t>(*profile.lifespan_ms * 1'000'000U));
			EXPECT_EQ(qos.liveliness(), rclcpp::LivelinessPolicy::Automatic);
			EXPECT_EQ(
				qos.liveliness_lease_duration().nanoseconds(),
				static_cast<std::int64_t>(*profile.liveliness_lease_duration_ms * 1'000'000U));

			const auto runtime_state = factory.make("runtime_state");
			EXPECT_EQ(runtime_state.depth(), 10U);
			EXPECT_EQ(runtime_state.history(), rclcpp::HistoryPolicy::KeepLast);
		}

		TEST(ContractComponents, QosFactoryRejectsNullAndUnknownProfiles)
		{
			EXPECT_THROW(QosFactory{GatewayContractPtr{}}, std::invalid_argument);

			QosFactory factory{load_test_contract()};
			EXPECT_THROW(factory.make("does_not_exist"), std::out_of_range);
		}

		TEST(ContractComponents, CompatibilitySeparatesDdsAndExactPolicyMatch)
		{
			const auto contract = load_test_contract();
			QosFactory factory{contract};
			const auto qos = factory.make("control_input");
			const auto expected = contract->qos_profiles.at("control_input");

			const auto report = assess_endpoint_qos(
				RemoteDirection::kSubscription,
				qos,
				qos,
				expected);
			EXPECT_EQ(report.dds_compatibility, rclcpp::QoSCompatibility::Ok);
			EXPECT_TRUE(report.exact_match());
			EXPECT_TRUE(report.observed_policies_match());
			EXPECT_TRUE(report.exact_mismatches.empty());
			EXPECT_TRUE(report.unobservable_policies.empty());

			auto best_effort_remote = qos;
			best_effort_remote.best_effort();
			const auto mismatch = assess_endpoint_qos(
				RemoteDirection::kSubscription,
				qos,
				best_effort_remote,
				expected);
			EXPECT_FALSE(mismatch.exact_match());
			EXPECT_FALSE(mismatch.observed_policies_match());
			ASSERT_FALSE(mismatch.exact_mismatches.empty());
			EXPECT_EQ(mismatch.exact_mismatches.front().policy, "reliability");
		}

		TEST(ContractComponents, CompatibilityReportsRmwObservationGapSeparately)
		{
			const auto contract = load_test_contract();
			QosFactory factory{contract};
			const auto local = factory.make("control_input");
			const auto expected = contract->qos_profiles.at("control_input");
			rmw_qos_profile_t rmw_profile = local.get_rmw_qos_profile();
			rmw_profile.history = RMW_QOS_POLICY_HISTORY_UNKNOWN;
			rmw_profile.depth = 0U;
			// Humble 的 from_rmw() 会规范化 UNKNOWN，这里按 Graph endpoint 的直接构造路径保留观测缺口
			const rclcpp::QoS unobservable_remote{
				rclcpp::QoSInitialization{
					rmw_profile.history,
					rmw_profile.depth},
				rmw_profile};

			const auto report = assess_endpoint_qos(
				RemoteDirection::kSubscription,
				local,
				unobservable_remote,
				expected);
			EXPECT_TRUE(report.observed_policies_match());
			EXPECT_FALSE(report.exact_match());
			ASSERT_EQ(report.unobservable_policies.size(), 2U);
			EXPECT_EQ(report.unobservable_policies.at(0).policy, "history");
			EXPECT_EQ(report.unobservable_policies.at(1).policy, "depth");
		}

		TEST(ContractComponents, CompatibilityUsesRemoteDirectionForDdsArgumentOrder)
		{
			const auto contract = load_test_contract();
			QosFactory factory{contract};
			const auto qos = factory.make("control_input");
			const auto expected = contract->qos_profiles.at("control_input");

			const auto publisher_report = assess_endpoint_qos(
				RemoteDirection::kPublisher,
				qos,
				qos,
				expected);
			EXPECT_EQ(publisher_report.dds_compatibility, rclcpp::QoSCompatibility::Ok);
			EXPECT_TRUE(publisher_report.exact_match());
		}

		TEST_F(RosGraphCompatibilityTest, UsesDiscoveredEndpointQosAndRemoteDirection)
		{
			static std::atomic<std::uint64_t> graph_sequence{0U};
			const auto suffix = std::to_string(
				graph_sequence.fetch_add(1U, std::memory_order_relaxed));
			const std::string publisher_node_name = "graph_publisher_" + suffix;
			const std::string subscription_node_name = "graph_subscription_" + suffix;
			const std::string observer_node_name = "graph_observer_" + suffix;
			const std::string publisher_topic = "/contract_graph/publisher_" + suffix;
			const std::string subscription_topic = "/contract_graph/subscription_" + suffix;

			const auto contract = load_test_contract();
			QosFactory factory{contract};
			const auto reliable_qos = factory.make("control_input");
			auto best_effort_qos = reliable_qos;
			best_effort_qos.best_effort();
			auto best_effort_profile = contract->qos_profiles.at("control_input");
			best_effort_profile.reliability = ReliabilityPolicy::kBestEffort;

			const auto publisher_node = std::make_shared<rclcpp::Node>(publisher_node_name);
			const auto subscription_node =
				std::make_shared<rclcpp::Node>(subscription_node_name);
			const auto observer_node = std::make_shared<rclcpp::Node>(observer_node_name);
			const auto publisher = publisher_node->create_publisher<std_msgs::msg::String>(
				publisher_topic,
				best_effort_qos);
			const auto subscription = subscription_node->create_subscription<std_msgs::msg::String>(
				subscription_topic,
				best_effort_qos,
				[](std_msgs::msg::String::ConstSharedPtr) {});

			const auto publishers = wait_for_endpoints(
				[&observer_node, &publisher_topic]()
				{
					return observer_node->get_publishers_info_by_topic(publisher_topic);
				},
				publisher_node_name);
			ASSERT_FALSE(publishers.empty());
			const auto &publisher_endpoint = find_endpoint(publishers, publisher_node_name);

			const auto publisher_report = assess_endpoint_qos(
				RemoteDirection::kPublisher,
				reliable_qos,
				publisher_endpoint.qos_profile(),
				best_effort_profile);
			// best-effort publisher 无法满足 reliable subscription 的 requested QoS
			EXPECT_EQ(publisher_report.dds_compatibility, rclcpp::QoSCompatibility::Error);
			EXPECT_TRUE(publisher_report.observed_policies_match());
			EXPECT_EQ(
				publisher_report.exact_match(),
				publisher_report.unobservable_policies.empty());

			const auto publisher_exact_mismatch = assess_endpoint_qos(
				RemoteDirection::kPublisher,
				reliable_qos,
				publisher_endpoint.qos_profile(),
				contract->qos_profiles.at("control_input"));
			EXPECT_FALSE(publisher_exact_mismatch.observed_policies_match());
			ASSERT_FALSE(publisher_exact_mismatch.exact_mismatches.empty());
			EXPECT_EQ(publisher_exact_mismatch.exact_mismatches.front().policy, "reliability");

			const auto subscriptions = wait_for_endpoints(
				[&observer_node, &subscription_topic]()
				{
					return observer_node->get_subscriptions_info_by_topic(subscription_topic);
				},
				subscription_node_name);
			ASSERT_FALSE(subscriptions.empty());
			const auto &subscription_endpoint =
				find_endpoint(subscriptions, subscription_node_name);
			const auto subscription_report = assess_endpoint_qos(
				RemoteDirection::kSubscription,
				reliable_qos,
				subscription_endpoint.qos_profile(),
				best_effort_profile);
			// reliable publisher 可以满足 best-effort subscription，方向反转后结论不同
			EXPECT_EQ(subscription_report.dds_compatibility, rclcpp::QoSCompatibility::Ok);
			EXPECT_TRUE(subscription_report.observed_policies_match());

			(void)publisher;
			(void)subscription;
		}
	}
}  // namespace control_link_contract
