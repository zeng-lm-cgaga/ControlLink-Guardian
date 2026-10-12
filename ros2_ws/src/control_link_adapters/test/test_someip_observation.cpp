#include "../src/someip_observation.hpp"

#include <filesystem>
#include <fstream>
#include <limits>
#include <string>
#include <unistd.h>

#include "control_link_contract/contract_bundle.hpp"
#include "gtest/gtest.h"
#include "yaml-cpp/yaml.h"

namespace control_link_adapters
{
namespace
{
namespace wire = control_link::network::v1;
using VehicleState = control_link_interfaces::msg::VehicleState;

VehicleState normal_vehicle()
{
	VehicleState message;
	message.observed_at.sec = 42;
	message.observed_at.nanosec = 123;
	message.state = VehicleState::RUNNING;
	message.linear_velocity_mps = 0.3;
	message.angular_velocity_radps = -0.1;
	message.rolling_counter = 8;
	return message;
}

TEST(SomeipObservation, UsesOnlyOriginalContractBudgets)
{
	const auto bundle = control_link_contract::load_contract_bundle(
		std::string{CONTROL_LINK_TEST_CONFIG_ROOT} + "/adas/adas_profile.yaml", CONTROL_LINK_TEST_CONFIG_ROOT);
	const auto budgets = someip_observation_budgets(*bundle->gateway_contract);
	const auto &contract = *bundle->gateway_contract;
	EXPECT_EQ(budgets.gateway_ms, *contract.qos_profiles.at("runtime_state").liveliness_lease_duration_ms);
	EXPECT_EQ(budgets.source_ms, budgets.gateway_ms);
	EXPECT_EQ(budgets.vehicle_ms, contract.gateway.vehicle_state_topic_timeout_ms);
}

TEST(SomeipObservation, LegalBundleWithoutLeaseFailsBeforeSomeipInitialization)
{
	const auto root = std::filesystem::temp_directory_path() /
		("control-link-someip-contract-" + std::to_string(::getpid()));
	ASSERT_FALSE(std::filesystem::exists(root));
	std::filesystem::copy(CONTROL_LINK_TEST_CONFIG_ROOT, root, std::filesystem::copy_options::recursive);
	const auto path = root / "common/gateway_contract.yaml";
	auto document = YAML::LoadFile(path.string());
	auto profile = document["qos_profiles"]["runtime_state"];
	profile.remove("liveliness");
	profile.remove("liveliness_lease_duration_ms");
	std::ofstream output(path);
	output << document;
	output.close();
	ASSERT_TRUE(output.good());
	const auto bundle = control_link_contract::load_contract_bundle(
		(root / "adas/adas_profile.yaml").string(), root.string());
	ASSERT_FALSE(bundle->gateway_contract->qos_profiles.at("runtime_state").liveliness_lease_duration_ms.has_value());
	try
	{
		static_cast<void>(someip_observation_budgets(*bundle->gateway_contract));
		FAIL() << "missing observation budget must not reach callbacks";
	}
	catch (const std::invalid_argument &error)
	{
		EXPECT_NE(std::string{error.what()}.find("gateway_state"), std::string::npos);
		EXPECT_NE(std::string{error.what()}.find("liveliness_lease_duration_ms"), std::string::npos);
	}
	std::filesystem::remove_all(root);
}

TEST(SomeipObservation, NormalAndFaultStateAreStructurallyValid)
{
	for (const auto state : {VehicleState::STANDBY, VehicleState::RUNNING, VehicleState::SAFE_STOP, VehicleState::FAULT})
	{
		auto message = normal_vehicle();
		message.state = state;
		message.fault_code = 101;
		wire::VehicleObservation target;
		set_someip_vehicle_observation(&target, message, 900);
		EXPECT_EQ(target.observation().validity(), wire::OBSERVATION_VALIDITY_VALID);
		EXPECT_EQ(target.observation().observed_system_ns(), 42'000'000'123LL);
		EXPECT_EQ(target.observation().received_boottime_ns(), 900U);
		EXPECT_EQ(target.state(), state);
		EXPECT_EQ(target.fault_code(), 101U);
		EXPECT_EQ(target.rolling_counter(), message.rolling_counter);
	}
}

TEST(SomeipObservation, UnknownStateInvalidPreservesRawObservationAndRecovers)
{
	auto message = normal_vehicle();
	message.state = 255;
	wire::VehicleObservation target;
	set_someip_vehicle_observation(&target, message, 900);
	EXPECT_EQ(target.observation().validity(), wire::OBSERVATION_VALIDITY_INVALID);
	EXPECT_EQ(target.state(), 255U);
	EXPECT_EQ(target.observation().observed_system_ns(), 42'000'000'123LL);
	message.state = VehicleState::RUNNING;
	message.observed_at.nanosec = 124;
	set_someip_vehicle_observation(&target, message, 901);
	EXPECT_EQ(target.observation().validity(), wire::OBSERVATION_VALIDITY_VALID);
	EXPECT_EQ(target.observation().observed_system_ns(), 42'000'000'124LL);
	EXPECT_EQ(target.observation().received_boottime_ns(), 901U);
}

TEST(SomeipObservation, NonfiniteInvalidPreservesRawValues)
{
	for (const auto value : {std::numeric_limits<double>::quiet_NaN(),
		std::numeric_limits<double>::infinity(), -std::numeric_limits<double>::infinity()})
	{
		for (bool linear : {false, true})
		{
			auto message = normal_vehicle();
			(linear ? message.linear_velocity_mps : message.angular_velocity_radps) = value;
			wire::VehicleObservation target;
			set_someip_vehicle_observation(&target, message, 900);
			EXPECT_EQ(target.observation().validity(), wire::OBSERVATION_VALIDITY_INVALID);
			const auto observed = linear ? target.linear_velocity_mps() : target.angular_velocity_radps();
			EXPECT_TRUE(std::isnan(value) ? std::isnan(observed) : observed == value);
			EXPECT_EQ(target.observation().received_boottime_ns(), 900U);
			set_someip_vehicle_observation(&target, normal_vehicle(), 901);
			EXPECT_EQ(target.observation().validity(), wire::OBSERVATION_VALIDITY_VALID);
		}
	}
}

TEST(SomeipObservation, InvalidTimestampOrMissingReceiveCannotBeValid)
{
	auto message = normal_vehicle();
	wire::VehicleObservation target;
	set_someip_vehicle_observation(&target, message, 0);
	EXPECT_EQ(target.observation().validity(), wire::OBSERVATION_VALIDITY_INVALID);
	message.observed_at.nanosec = 1'000'000'000U;
	set_someip_vehicle_observation(&target, message, 900);
	EXPECT_EQ(target.observation().validity(), wire::OBSERVATION_VALIDITY_INVALID);
	message.observed_at.sec = 0;
	message.observed_at.nanosec = 0;
	set_someip_vehicle_observation(&target, message, 900);
	EXPECT_EQ(target.observation().validity(), wire::OBSERVATION_VALIDITY_INVALID);
}
}
}
