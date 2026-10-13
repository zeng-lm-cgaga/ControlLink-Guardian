#include <chrono>
#include <filesystem>
#include <limits>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

#include <gtest/gtest.h>

#include "control_link_adapters/control_command_ingress_adapter.hpp"
#include "control_link_contract/parser.hpp"
#include "control_link_contract/qos_factory.hpp"

namespace
{
	using Command = control_link_interfaces::msg::ControlCommand;
	using namespace std::chrono_literals;

	class ControlCommandIngressTest : public ::testing::Test
	{
	protected:
		static void SetUpTestSuite() { rclcpp::init(0, nullptr); }
		static void TearDownTestSuite() { rclcpp::shutdown(); }

		rclcpp::NodeOptions options(const std::string & source = "planning")
		{
			return rclcpp::NodeOptions{}.parameter_overrides({
				rclcpp::Parameter("profile_path", std::string{CONTROL_LINK_TEST_ADAS_PROFILE}),
				rclcpp::Parameter("config_root", std::string{CONTROL_LINK_TEST_CONFIG_ROOT}),
				rclcpp::Parameter("source_id", source)});
		}

		template<typename Predicate>
		bool spin_until(rclcpp::Executor & executor, Predicate predicate)
		{
			const auto deadline = std::chrono::steady_clock::now() + 5s;
			while (!predicate() && std::chrono::steady_clock::now() < deadline)
			{
				executor.spin_some();
				std::this_thread::sleep_for(5ms);
			}
			return predicate();
		}
	};

	TEST_F(ControlCommandIngressTest, RejectsDisabledSourceAndWrongInputType)
	{
		EXPECT_THROW(control_link_adapters::ControlCommandIngressAdapter{options("nav2")},
			std::invalid_argument);
		EXPECT_THROW(control_link_adapters::ControlCommandIngressAdapter{options("teleop")},
			std::invalid_argument);
	}

	TEST_F(ControlCommandIngressTest, BindsSourceButPreservesEveryBusinessFieldIncludingInvalidOnes)
	{
		auto ingress = std::make_shared<control_link_adapters::ControlCommandIngressAdapter>(options());
		auto peer = std::make_shared<rclcpp::Node>("control_command_ingress_test_peer");
		const auto contract = control_link_contract::load_gateway_contract(
			std::filesystem::path{CONTROL_LINK_TEST_CONFIG_ROOT} / "common/gateway_contract.yaml");
		const auto qos = control_link_contract::QosFactory{contract}.make(contract->input.qos_profile);
		auto publisher = peer->create_publisher<Command>("/planning/control_cmd", qos);
		std::vector<Command> received;
		auto subscription = peer->create_subscription<Command>("/control_link/input/planning", qos,
			[&received](const Command & command) { received.push_back(command); });
		rclcpp::executors::SingleThreadedExecutor executor;
		executor.add_node(ingress);
		executor.add_node(peer);
		ASSERT_TRUE(spin_until(executor, [&] {
			return publisher->get_subscription_count() == 1U && subscription->get_publisher_count() == 1U;
		}));
		Command input;
		input.source_stamp.sec = 1;
		input.source_stamp.nanosec = 123456789U;
		input.source_id = "upstream_label";
		input.source_sequence = 77U;
		input.mode = 255U;
		input.linear_velocity_mps = std::numeric_limits<double>::infinity();
		input.angular_velocity_radps = -2.0;
		publisher->publish(input);
		ASSERT_TRUE(spin_until(executor, [&] { return received.size() == 1U; }));
		EXPECT_EQ(received.front().source_id, "planning");
		EXPECT_EQ(received.front().source_stamp, input.source_stamp);
		EXPECT_EQ(received.front().source_sequence, input.source_sequence);
		EXPECT_EQ(received.front().mode, input.mode);
		EXPECT_EQ(received.front().linear_velocity_mps, input.linear_velocity_mps);
		EXPECT_EQ(received.front().angular_velocity_radps, input.angular_velocity_radps);
		publisher->publish(input);
		ASSERT_TRUE(spin_until(executor, [&] { return received.size() == 2U; }));
		EXPECT_EQ(received.back(), received.front());
	}
}
