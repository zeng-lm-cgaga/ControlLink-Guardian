#include <chrono>
#include <cstdint>
#include <limits>
#include <string>

#include <gtest/gtest.h>

#include "control_link_adapters/canonical_input_guard.hpp"
#include "control_link_adapters/local_watchdog.hpp"
#include "control_link_contract/parser.hpp"
#include "control_link_interfaces/msg/control_command.hpp"

namespace control_link_adapters
{
	namespace
	{
		using namespace std::chrono_literals;
		constexpr std::int64_t kNanosecondsPerSecond = 1'000'000'000LL;

		control_link_contract::GatewayContractPtr load_contract()
		{
			return control_link_contract::load_gateway_contract(
				CONTROL_LINK_TEST_CONTRACT_PATH);
		}

		CanonicalPublisherKey publisher_key(std::uint8_t seed)
		{
			CanonicalPublisherKey result{"rmw_fastrtps_cpp", {}};
			for (std::size_t index = 0U; index < result.publisher_gid.size(); ++index)
			{
				result.publisher_gid[index] = static_cast<std::uint8_t>(seed + index);
			}
			return result;
		}

		builtin_interfaces::msg::Time stamp_from_nanoseconds(std::int64_t nanoseconds)
		{
			builtin_interfaces::msg::Time stamp;
			stamp.sec = static_cast<std::int32_t>(nanoseconds / kNanosecondsPerSecond);
			stamp.nanosec = static_cast<std::uint32_t>(nanoseconds % kNanosecondsPerSecond);
			return stamp;
		}

		control_link_interfaces::msg::ControlCommand normal_command()
		{
			control_link_interfaces::msg::ControlCommand command;
			command.source_stamp = stamp_from_nanoseconds(9'950'000'000LL);
			command.source_id = "planning";
			command.source_sequence = 42U;
			command.mode = control_link_interfaces::msg::ControlCommand::MODE_NORMAL;
			command.linear_velocity_mps = 0.5;
			command.angular_velocity_radps = -0.25;
			return command;
		}

		CanonicalEndpointSnapshot confirmed_endpoint(
			const control_link_contract::GatewayContract &contract,
			const CanonicalPublisherKey &publisher)
		{
			return CanonicalEndpointSnapshot{
				CanonicalEndpointState::kConfirmed,
				contract.gateway.node_fqn,
				publisher};
		}

		void expect_canonical_reject(
			const CanonicalInputGuard &guard,
			const control_link_interfaces::msg::ControlCommand &command,
			const CanonicalPublisherKey &publisher,
			const CanonicalEndpointSnapshot &endpoint,
			CanonicalRejectReason expected,
			std::int64_t now_ros_ns = 10'000'000'000LL,
			bool clock_healthy = true)
		{
			const auto result = guard.validate(
				command,
				publisher,
				endpoint,
				now_ros_ns,
				clock_healthy);
			EXPECT_EQ(result.reason, expected);
			EXPECT_FALSE(result.accepted());
		}

		TEST(LocalWatchdogTest, UsesStrictTimeoutAndResetsOnPublisherGenerationChange)
		{
			EXPECT_THROW(LocalWatchdog{0U}, std::invalid_argument);
			LocalWatchdog watchdog{100U};
			const auto base = std::chrono::steady_clock::time_point{10s};

			EXPECT_EQ(
				watchdog.evaluate(base),
				LocalWatchdogState::kWaitingForFirstCommand);
			watchdog.observe_valid_command(base);
			EXPECT_EQ(watchdog.evaluate(base + 100ms), LocalWatchdogState::kHealthy);
			EXPECT_EQ(watchdog.evaluate(base + 101ms), LocalWatchdogState::kTimedOut);
			EXPECT_THROW((void)watchdog.evaluate(base - 1ns), std::logic_error);
			EXPECT_THROW(watchdog.observe_valid_command(base - 1ns), std::logic_error);

			watchdog.reset();
			EXPECT_EQ(
				watchdog.evaluate(base + 1s),
				LocalWatchdogState::kWaitingForFirstCommand);
		}

		TEST(CanonicalInputGuardTest, AcceptsNormalRepeatedSequenceAndBothHoldForms)
		{
			const auto contract = load_contract();
			CanonicalInputGuard guard{contract};
			const auto publisher = publisher_key(1U);
			const auto endpoint = confirmed_endpoint(*contract, publisher);
			const auto normal = normal_command();

			EXPECT_TRUE(guard.validate(
				normal, publisher, endpoint, 10'000'000'000LL, true).accepted());
			// canonical 以固定频率重复 latest sequence，adapter 不能要求每个 tick 递增
			EXPECT_TRUE(guard.validate(
				normal, publisher, endpoint, 10'000'000'000LL, true).accepted());

			auto sourced_hold = normal;
			sourced_hold.mode = control_link_interfaces::msg::ControlCommand::MODE_HOLD;
			sourced_hold.linear_velocity_mps = 0.0;
			sourced_hold.angular_velocity_radps = 0.0;
			// HOLD 的安全语义来自持续零输出，保留的来源 stamp 可以已经超过 command timeout
			EXPECT_TRUE(guard.validate(
				sourced_hold, publisher, endpoint, 20'000'000'000LL, true).accepted());

			control_link_interfaces::msg::ControlCommand empty_hold;
			empty_hold.mode = control_link_interfaces::msg::ControlCommand::MODE_HOLD;
			EXPECT_TRUE(guard.validate(
				empty_hold, publisher, endpoint, -1, false).accepted());
		}

		TEST(CanonicalInputGuardTest, RejectsEndpointAuthorityAndGidFailuresFirst)
		{
			const auto contract = load_contract();
			CanonicalInputGuard guard{contract};
			const auto publisher = publisher_key(2U);
			const auto command = normal_command();

			expect_canonical_reject(
				guard,
				command,
				publisher,
				CanonicalEndpointSnapshot{
					CanonicalEndpointState::kUnavailable, "", std::nullopt},
				CanonicalRejectReason::kEndpointUnavailable);
			expect_canonical_reject(
				guard,
				command,
				publisher,
				CanonicalEndpointSnapshot{
					CanonicalEndpointState::kUnstable, "", std::nullopt},
				CanonicalRejectReason::kEndpointUnstable);
			expect_canonical_reject(
				guard,
				command,
				publisher,
				CanonicalEndpointSnapshot{
					CanonicalEndpointState::kAmbiguous, "", std::nullopt},
				CanonicalRejectReason::kEndpointAmbiguous);

			auto endpoint = confirmed_endpoint(*contract, publisher);
			endpoint.node_fqn = "/other/gateway";
			expect_canonical_reject(
				guard, command, publisher, endpoint, CanonicalRejectReason::kPublisherMismatch);
			endpoint = confirmed_endpoint(*contract, publisher_key(3U));
			expect_canonical_reject(
				guard, command, publisher, endpoint, CanonicalRejectReason::kPublisherMismatch);

			endpoint = CanonicalEndpointSnapshot{
				CanonicalEndpointState::kConfirmed,
				contract->gateway.node_fqn,
				std::nullopt};
			EXPECT_THROW(
				(void)guard.validate(command, publisher, endpoint, 10'000'000'000LL, true),
				std::logic_error);
		}

		TEST(CanonicalInputGuardTest, RejectsContentMetadataAndRosTimeFailures)
		{
			const auto contract = load_contract();
			CanonicalInputGuard guard{contract};
			const auto publisher = publisher_key(4U);
			const auto endpoint = confirmed_endpoint(*contract, publisher);
			const auto base = normal_command();

			auto command = base;
			command.mode = 255U;
			expect_canonical_reject(
				guard, command, publisher, endpoint, CanonicalRejectReason::kUnknownMode);
			command = base;
			command.linear_velocity_mps = std::numeric_limits<double>::quiet_NaN();
			expect_canonical_reject(
				guard, command, publisher, endpoint, CanonicalRejectReason::kNonFinite);
			command = base;
			command.angular_velocity_radps = 1.51;
			expect_canonical_reject(
				guard, command, publisher, endpoint, CanonicalRejectReason::kOutOfRange);

			command = base;
			command.source_id.clear();
			expect_canonical_reject(
				guard, command, publisher, endpoint, CanonicalRejectReason::kInvalidMetadata);
			command = base;
			command.source_stamp.nanosec = 1'000'000'000U;
			expect_canonical_reject(
				guard, command, publisher, endpoint, CanonicalRejectReason::kInvalidStamp);
			command = base;
			expect_canonical_reject(
				guard,
				command,
				publisher,
				endpoint,
				CanonicalRejectReason::kClockInvalid,
				10'000'000'000LL,
				false);
			command.source_stamp = stamp_from_nanoseconds(10'020'000'001LL);
			expect_canonical_reject(
				guard, command, publisher, endpoint, CanonicalRejectReason::kFutureStamp);
			command.source_stamp = stamp_from_nanoseconds(9'899'999'999LL);
			expect_canonical_reject(
				guard, command, publisher, endpoint, CanonicalRejectReason::kStale);

			command = base;
			command.mode = control_link_interfaces::msg::ControlCommand::MODE_HOLD;
			command.linear_velocity_mps = 0.1;
			expect_canonical_reject(
				guard, command, publisher, endpoint, CanonicalRejectReason::kHoldNonzero);
			command = control_link_interfaces::msg::ControlCommand{};
			command.mode = control_link_interfaces::msg::ControlCommand::MODE_HOLD;
			command.source_sequence = 1U;
			expect_canonical_reject(
				guard, command, publisher, endpoint, CanonicalRejectReason::kInvalidMetadata);
		}

		TEST(CanonicalInputGuardTest, RejectsNullContract)
		{
			EXPECT_THROW(
				CanonicalInputGuard{control_link_contract::GatewayContractPtr{}},
				std::invalid_argument);
		}
	}
}  // namespace control_link_adapters
