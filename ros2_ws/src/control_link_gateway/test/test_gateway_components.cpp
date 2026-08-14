#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <limits>
#include <map>
#include <memory>
#include <optional>
#include <random>
#include <string>
#include <vector>

#include <gtest/gtest.h>

#include "control_link_contract/parser.hpp"
#include "control_link_gateway/command_validator.hpp"
#include "control_link_gateway/source_arbiter.hpp"
#include "control_link_gateway/source_binding.hpp"
#include "control_link_gateway/source_runtime.hpp"
#include "control_link_gateway/state_machine.hpp"
#include "control_link_gateway/vehicle_state_runtime.hpp"
#include "control_link_gateway/vehicle_state_validator.hpp"
#include "control_link_interfaces/msg/control_command.hpp"
#include "control_link_interfaces/msg/vehicle_state.hpp"

namespace control_link_gateway
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

		control_link_contract::SourcePolicyPtr load_policy()
		{
			return control_link_contract::load_source_policy(
				CONTROL_LINK_TEST_SOURCE_POLICY_PATH);
		}

		PublisherGenerationKey generation(std::uint8_t seed)
		{
			PublisherGenerationKey result{"rmw_fastrtps_cpp", {}};
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

		std::chrono::steady_clock::time_point steady_time(std::chrono::seconds value)
		{
			return std::chrono::steady_clock::time_point{
				std::chrono::duration_cast<std::chrono::steady_clock::duration>(value)};
		}

		control_link_interfaces::msg::ControlCommand valid_command(
			std::string source_id = "planning",
			std::uint64_t sequence = 1U,
			std::int64_t stamp_ns = 9'950'000'000LL)
		{
			control_link_interfaces::msg::ControlCommand command;
			command.source_stamp = stamp_from_nanoseconds(stamp_ns);
			command.source_id = std::move(source_id);
			command.source_sequence = sequence;
			command.mode = control_link_interfaces::msg::ControlCommand::MODE_NORMAL;
			command.linear_velocity_mps = 0.5;
			command.angular_velocity_radps = -0.25;
			return command;
		}

		CommandValidationContext validation_context(
			const PublisherGenerationKey &publisher_generation,
			std::optional<std::uint64_t> last_sequence = std::nullopt)
		{
			return CommandValidationContext{
				"planning",
				publisher_generation,
				last_sequence,
				150U,
				10'000'000'000LL,
				true,
				std::chrono::steady_clock::time_point{10s}};
		}

		void expect_command_reject(
			const CommandValidator &validator,
			const control_link_interfaces::msg::ControlCommand &command,
			const CommandValidationContext &context,
			RejectReason expected)
		{
			const auto result = validator.validate(command, context);
			EXPECT_EQ(result.reason, expected);
			EXPECT_FALSE(result.accepted());
			EXPECT_FALSE(result.snapshot.has_value());
		}

		SourceSnapshot snapshot(
			std::string source_id,
			std::uint8_t priority,
			PublisherGenerationKey publisher_generation,
			std::uint64_t sequence,
			std::int64_t source_stamp_ns,
			std::chrono::steady_clock::time_point received_at)
		{
			return SourceSnapshot{
				std::move(source_id),
				priority,
				std::move(publisher_generation),
				sequence,
				source_stamp_ns,
				CommandMode::kNormal,
				0.5,
				0.1,
				received_at};
		}

		GatewayHealthSnapshot healthy_gateway()
		{
			GatewayHealthSnapshot health;
			health.critical_endpoints_healthy = true;
			health.critical_qos_compatible = true;
			health.ros_clock_healthy = true;
			health.vehicle_state_valid = true;
			health.vehicle_state_fresh = true;
			health.vehicle_reports_safe_stop = false;
			health.vehicle_reports_fault = false;
			health.output_tick_healthy = true;
			health.internal_invariants_healthy = true;
			return health;
		}

		ArbitrationDecision arbitration(
			const SourceSnapshot &selected,
			ArbitrationEvent event)
		{
			return ArbitrationDecision{selected, event};
		}

		RecoveryEvidence evidence(
			const SourceSnapshot &selected,
			RejectReason result,
			std::uint64_t health_epoch)
		{
			return RecoveryEvidence{
				RecoveryCandidateKey{
					selected.source_id,
					selected.publisher_generation},
				result,
				health_epoch};
		}

		TEST(CommandValidatorTest, AcceptsBoundaryValuesAndCreatesImmutableSnapshot)
		{
			const auto contract = load_contract();
			CommandValidator validator{contract};
			const auto publisher_generation = generation(1U);
			auto context = validation_context(publisher_generation, 9U);
			auto command = valid_command("planning", 10U, 9'900'000'000LL);
			command.linear_velocity_mps = contract->limits.max_abs_linear_velocity_mps;
			command.angular_velocity_radps =
				-contract->limits.max_abs_angular_velocity_radps;

			const auto result = validator.validate(command, context);
			ASSERT_TRUE(result.accepted());
			ASSERT_TRUE(result.snapshot.has_value());
			EXPECT_EQ(result.reason, RejectReason::kNone);
			EXPECT_EQ(result.snapshot->source_id, "planning");
			EXPECT_EQ(result.snapshot->sequence, 10U);
			EXPECT_EQ(result.snapshot->publisher_generation, publisher_generation);
			EXPECT_EQ(result.snapshot->source_stamp_ns, 9'900'000'000LL);
			EXPECT_EQ(result.snapshot->received_at, context.received_at);

			command.source_stamp = stamp_from_nanoseconds(10'020'000'000LL);
			EXPECT_TRUE(validator.validate(command, context).accepted());
		}

		TEST(CommandValidatorTest, RejectsIdentityContentClockTimeAndSequenceFailures)
		{
			CommandValidator validator{load_contract()};
			const auto publisher_generation = generation(2U);
			const auto context = validation_context(publisher_generation, 9U);
			const auto base = valid_command("planning", 10U);

			auto command = base;
			command.source_id = "teleop";
			expect_command_reject(validator, command, context, RejectReason::kSourceIdMismatch);

			command = base;
			command.mode = 255U;
			expect_command_reject(validator, command, context, RejectReason::kUnknownMode);

			command = base;
			command.linear_velocity_mps = std::numeric_limits<double>::quiet_NaN();
			expect_command_reject(validator, command, context, RejectReason::kNonFinite);

			command = base;
			command.linear_velocity_mps = 1.01;
			expect_command_reject(validator, command, context, RejectReason::kOutOfRange);

			command = base;
			command.mode = control_link_interfaces::msg::ControlCommand::MODE_HOLD;
			command.linear_velocity_mps = 0.1;
			expect_command_reject(validator, command, context, RejectReason::kHoldNonzero);

			command = base;
			command.source_stamp.sec = 0;
			command.source_stamp.nanosec = 0U;
			expect_command_reject(validator, command, context, RejectReason::kZeroStamp);

			command = base;
			command.source_stamp.nanosec = 1'000'000'000U;
			expect_command_reject(validator, command, context, RejectReason::kInvalidStamp);

			auto invalid_clock_context = context;
			invalid_clock_context.ros_clock_valid = false;
			expect_command_reject(
				validator, base, invalid_clock_context, RejectReason::kClockInvalid);

			command = base;
			command.source_stamp = stamp_from_nanoseconds(10'020'000'001LL);
			expect_command_reject(validator, command, context, RejectReason::kFutureStamp);

			command = base;
			command.source_stamp = stamp_from_nanoseconds(9'899'999'999LL);
			expect_command_reject(validator, command, context, RejectReason::kStale);

			command = base;
			command.source_sequence = 9U;
			expect_command_reject(
				validator, command, context, RejectReason::kSequenceNotIncreasing);
		}

		TEST(SourceRuntimeTest, CommitsAcceptedResultsWithoutLettingRejectsPolluteTheSlot)
		{
			SourceRuntimeSlot slot;
			const auto first_generation = generation(3U);
			EXPECT_EQ(
				confirm_publisher_generation(slot, first_generation),
				PublisherGenerationUpdate::kFirstConfirmation);

			const auto first_snapshot = snapshot(
				"planning", 150U, first_generation, 1U, 10'000'000'000LL, steady_time(10s));
			commit_validation_result(
				slot,
				CommandValidationResult{RejectReason::kNone, first_snapshot});
			ASSERT_TRUE(slot.latest_valid_snapshot.has_value());
			EXPECT_EQ(slot.last_accepted_sequence, 1U);
			EXPECT_EQ(slot.accepted_count, 1U);

			commit_validation_result(
				slot,
				CommandValidationResult{RejectReason::kStale, std::nullopt});
			EXPECT_EQ(slot.rejected_count, 1U);
			EXPECT_EQ(slot.last_reject_reason, RejectReason::kStale);
			ASSERT_TRUE(slot.latest_valid_snapshot.has_value());
			EXPECT_EQ(slot.latest_valid_snapshot->sequence, 1U);

			invalidate_source_endpoint_snapshot(slot);
			EXPECT_FALSE(slot.latest_valid_snapshot.has_value());
			EXPECT_EQ(slot.last_accepted_sequence, 1U);
			EXPECT_TRUE(message_matches_confirmed_generation(slot, first_generation));

			const auto second_generation = generation(4U);
			EXPECT_EQ(
				confirm_publisher_generation(slot, second_generation),
				PublisherGenerationUpdate::kChanged);
			EXPECT_FALSE(slot.last_accepted_sequence.has_value());
			EXPECT_FALSE(message_matches_confirmed_generation(slot, first_generation));
			EXPECT_TRUE(message_matches_confirmed_generation(slot, second_generation));
		}

		TEST(SourceRuntimeTest, RejectsInconsistentCommitAndCollectionState)
		{
			SourceRuntimeSlot slot;
			EXPECT_THROW(
				commit_validation_result(
					slot,
					CommandValidationResult{RejectReason::kNone, std::nullopt}),
				std::logic_error);

			const auto publisher_generation = generation(5U);
			const auto valid_snapshot = snapshot(
				"planning", 150U, publisher_generation, 1U, 10'000'000'000LL, steady_time(10s));
			EXPECT_THROW(
				commit_validation_result(
					slot,
					CommandValidationResult{RejectReason::kNone, valid_snapshot}),
				std::logic_error);

			slot.confirmed_publisher_generation = publisher_generation;
			slot.last_accepted_sequence = 1U;
			slot.latest_valid_snapshot = valid_snapshot;
			std::map<std::string, SourceRuntimeSlot> slots{{"planning", slot}};
			const auto snapshots = collect_latest_valid_snapshots(slots);
			ASSERT_EQ(snapshots.size(), 1U);
			EXPECT_EQ(snapshots.at("planning").sequence, 1U);

			slots.begin()->second.last_accepted_sequence = 2U;
			EXPECT_THROW(collect_latest_valid_snapshots(slots), std::logic_error);
		}

		TEST(SourceArbiterTest, AppliesSwitchHoldThenFallsBackImmediatelyWhenActiveExpires)
		{
			const auto contract = load_contract();
			const auto policy = load_policy();
			SourceArbiter arbiter{contract, policy};
			const auto base = std::chrono::steady_clock::time_point{10s};
			std::map<std::string, SourceSnapshot> snapshots{
				{"planning", snapshot("planning", 150U, generation(6U), 1U,
					10'000'000'000LL, base)}};

			auto decision = arbiter.evaluate(
				ArbitrationInput{&snapshots, 10'000'000'000LL, base});
			ASSERT_TRUE(decision.selected.has_value());
			EXPECT_EQ(decision.selected->source_id, "planning");
			EXPECT_EQ(decision.event, ArbitrationEvent::kFirstSelection);

			snapshots.emplace(
				"teleop",
				snapshot("teleop", 200U, generation(7U), 1U,
					10'000'000'000LL, base + 10ms));
			decision = arbiter.evaluate(
				ArbitrationInput{&snapshots, 10'000'000'000LL, base + 10ms});
			EXPECT_EQ(decision.selected->source_id, "planning");
			EXPECT_EQ(decision.event, ArbitrationEvent::kNoChange);

			decision = arbiter.evaluate(
				ArbitrationInput{&snapshots, 10'000'000'000LL, base + 109ms});
			EXPECT_EQ(decision.selected->source_id, "planning");
			EXPECT_EQ(decision.event, ArbitrationEvent::kNoChange);

			decision = arbiter.evaluate(
				ArbitrationInput{&snapshots, 10'000'000'000LL, base + 110ms});
			EXPECT_EQ(decision.selected->source_id, "teleop");
			EXPECT_EQ(decision.event, ArbitrationEvent::kSwitch);

			snapshots.at("planning").received_at = base + 120ms;
			decision = arbiter.evaluate(
				ArbitrationInput{&snapshots, 10'000'000'000LL, base + 131ms});
			EXPECT_EQ(decision.selected->source_id, "planning");
			EXPECT_EQ(decision.event, ArbitrationEvent::kFallback);

			decision = arbiter.evaluate(
				ArbitrationInput{&snapshots, 10'000'000'000LL, base + 300ms});
			EXPECT_FALSE(decision.selected.has_value());
			EXPECT_EQ(decision.event, ArbitrationEvent::kNoQualifiedSource);
			decision = arbiter.evaluate(
				ArbitrationInput{&snapshots, 10'000'000'000LL, base + 301ms});
			EXPECT_FALSE(decision.selected.has_value());
			EXPECT_EQ(decision.event, ArbitrationEvent::kNoChange);
		}

		TEST(SourceArbiterTest, OrdersByPriorityAgeAndSourceIdAndChecksInvariants)
		{
			const auto contract = load_contract();
			auto mutable_policy = std::make_shared<control_link_contract::SourcePolicy>(
				*load_policy());
			mutable_policy->sources.at("planning").priority = 200U;
			const auto policy = std::const_pointer_cast<const control_link_contract::SourcePolicy>(
				mutable_policy);
			const auto base = std::chrono::steady_clock::time_point{20s};
			std::map<std::string, SourceSnapshot> snapshots{
				{"planning", snapshot("planning", 200U, generation(8U), 1U,
					20'000'000'000LL, base)},
				{"teleop", snapshot("teleop", 200U, generation(9U), 1U,
					20'000'000'000LL, base)}};

			SourceArbiter lexical_arbiter{contract, policy};
			const auto lexical = lexical_arbiter.evaluate(
				ArbitrationInput{&snapshots, 20'000'000'000LL, base});
			ASSERT_TRUE(lexical.selected.has_value());
			EXPECT_EQ(lexical.selected->source_id, "planning");

			snapshots.at("teleop").source_stamp_ns = 20'000'000'001LL;
			SourceArbiter age_arbiter{contract, policy};
			const auto by_age = age_arbiter.evaluate(
				ArbitrationInput{&snapshots, 20'000'000'000LL, base});
			ASSERT_TRUE(by_age.selected.has_value());
			EXPECT_EQ(by_age.selected->source_id, "teleop");

			snapshots.at("planning").priority = 199U;
			EXPECT_THROW(
				(void)assess_source_snapshot(
					"planning", snapshots.at("planning"), *contract, *policy,
					20'000'000'000LL, base),
				std::logic_error);
			EXPECT_THROW(
				age_arbiter.evaluate(ArbitrationInput{nullptr, 0, base}),
				std::invalid_argument);
		}

		TEST(SourceArbiterTest, ProducesTheSameDecisionAcrossRandomInsertionOrders)
		{
			const auto contract = load_contract();
			auto mutable_policy = std::make_shared<control_link_contract::SourcePolicy>(
				*load_policy());
			for (auto &[source_id, entry] : mutable_policy->sources)
			{
				(void)source_id;
				entry.priority = 100U;
			}
			const auto policy = std::const_pointer_cast<const control_link_contract::SourcePolicy>(
				mutable_policy);
			const auto base = std::chrono::steady_clock::time_point{25s};
			std::vector<SourceSnapshot> values{
				snapshot("nav2", 100U, generation(20U), 1U, 25'000'000'000LL, base),
				snapshot("planning", 100U, generation(21U), 1U, 25'000'000'000LL, base),
				snapshot("teleop", 100U, generation(22U), 1U, 25'000'000'000LL, base)};
			std::mt19937 random{0xC011U};

			for (std::size_t iteration = 0U; iteration < 100U; ++iteration)
			{
				std::shuffle(values.begin(), values.end(), random);
				std::map<std::string, SourceSnapshot> snapshots;
				for (const auto &value : values)
				{
					snapshots.emplace(value.source_id, value);
				}

				SourceArbiter arbiter{contract, policy};
				const auto decision = arbiter.evaluate(
					ArbitrationInput{&snapshots, 25'000'000'000LL, base});
				ASSERT_TRUE(decision.selected.has_value()) << "iteration=" << iteration;
				EXPECT_EQ(decision.selected->source_id, "nav2") << "iteration=" << iteration;
				EXPECT_EQ(decision.event, ArbitrationEvent::kFirstSelection);
			}
		}

		TEST(GatewayStateMachineTest, RecoversThenHandlesSwitchDegradedAndSafetyStates)
		{
			GatewayStateMachine machine{load_contract()};
			const auto planning = snapshot(
				"planning", 150U, generation(10U), 1U, 30'000'000'000LL, steady_time(30s));
			const auto teleop = snapshot(
				"teleop", 200U, generation(11U), 1U, 30'000'000'000LL, steady_time(30s));
			const auto health = healthy_gateway();

			std::vector<RecoveryEvidence> first_batch(4U, evidence(
				planning, RejectReason::kNone, 1U));
			auto decision = machine.evaluate(StateMachineInput{
				health,
				arbitration(planning, ArbitrationEvent::kFirstSelection),
				first_batch,
				1U});
			EXPECT_EQ(decision.state, DataState::kRecovering);
			EXPECT_EQ(decision.reason, StateReason::kFirstValidCommand);
			EXPECT_EQ(decision.recovery_valid_count, 4U);
			EXPECT_EQ(decision.transition_sequence, 1U);

			decision = machine.evaluate(StateMachineInput{
				health,
				arbitration(planning, ArbitrationEvent::kNoChange),
				{evidence(planning, RejectReason::kNone, 1U)},
				1U});
			EXPECT_EQ(decision.state, DataState::kActive);
			EXPECT_EQ(decision.reason, StateReason::kRecoveryComplete);
			EXPECT_EQ(decision.recovery_valid_count, 0U);

			decision = machine.evaluate(StateMachineInput{
				health,
				arbitration(teleop, ArbitrationEvent::kSwitch),
				{},
				1U});
			EXPECT_EQ(decision.state, DataState::kActive);
			EXPECT_EQ(decision.reason, StateReason::kSourceSwitch);
			EXPECT_EQ(decision.selected->source_id, "teleop");

			decision = machine.evaluate(StateMachineInput{
				health,
				arbitration(planning, ArbitrationEvent::kFallback),
				{},
				1U});
			EXPECT_EQ(decision.state, DataState::kActive);
			EXPECT_EQ(decision.reason, StateReason::kSourceFallback);
			EXPECT_EQ(decision.selected->source_id, "planning");

			auto degraded_health = health;
			degraded_health.critical_qos_compatible = false;
			decision = machine.evaluate(StateMachineInput{
				degraded_health,
				arbitration(planning, ArbitrationEvent::kNoChange),
				{},
				2U});
			EXPECT_EQ(decision.state, DataState::kDegraded);
			EXPECT_EQ(decision.reason, StateReason::kCriticalQosMismatch);

			std::vector<RecoveryEvidence> recovery_batch(5U, evidence(
				planning, RejectReason::kNone, 3U));
			decision = machine.evaluate(StateMachineInput{
				health,
				arbitration(planning, ArbitrationEvent::kNoChange),
				recovery_batch,
				3U});
			EXPECT_EQ(decision.state, DataState::kActive);
			EXPECT_EQ(decision.reason, StateReason::kRecoveryComplete);

			auto fault_health = health;
			fault_health.vehicle_reports_fault = true;
			decision = machine.evaluate(StateMachineInput{
				fault_health,
				arbitration(planning, ArbitrationEvent::kNoChange),
				{},
				4U});
			EXPECT_EQ(decision.state, DataState::kSafeStop);
			EXPECT_EQ(decision.reason, StateReason::kVehicleFault);

			auto invariant_failure = fault_health;
			invariant_failure.internal_invariants_healthy = false;
			decision = machine.evaluate(StateMachineInput{
				invariant_failure,
				arbitration(planning, ArbitrationEvent::kNoChange),
				{},
				5U});
			EXPECT_EQ(decision.state, DataState::kError);
			EXPECT_EQ(decision.reason, StateReason::kInternalInvariant);

			decision = machine.evaluate(StateMachineInput{
				health,
				arbitration(planning, ArbitrationEvent::kFallback),
				{},
				6U});
			EXPECT_EQ(decision.state, DataState::kError);
			EXPECT_EQ(decision.reason, StateReason::kInternalInvariant);
		}

		TEST(GatewayStateMachineTest, CountsOnlyOrderedEvidenceForCurrentCandidateAndEpoch)
		{
			GatewayStateMachine machine{load_contract()};
			const auto health = healthy_gateway();
			const auto planning = snapshot(
				"planning", 150U, generation(12U), 1U, 40'000'000'000LL, steady_time(40s));
			const auto teleop = snapshot(
				"teleop", 200U, generation(13U), 1U, 40'000'000'000LL, steady_time(40s));

			auto decision = machine.evaluate(StateMachineInput{
				health,
				arbitration(planning, ArbitrationEvent::kFirstSelection),
				{evidence(planning, RejectReason::kNone, 1U),
					evidence(planning, RejectReason::kNone, 1U)},
				1U});
			EXPECT_EQ(decision.recovery_valid_count, 2U);

			decision = machine.evaluate(StateMachineInput{
				health,
				arbitration(planning, ArbitrationEvent::kNoChange),
				{evidence(teleop, RejectReason::kNone, 1U),
					evidence(planning, RejectReason::kStale, 1U),
					evidence(planning, RejectReason::kNone, 1U)},
				1U});
			EXPECT_EQ(decision.state, DataState::kRecovering);
			EXPECT_EQ(decision.recovery_valid_count, 1U);

			decision = machine.evaluate(StateMachineInput{
				health,
				arbitration(planning, ArbitrationEvent::kNoChange),
				{evidence(planning, RejectReason::kNone, 1U),
					evidence(planning, RejectReason::kNone, 2U)},
				2U});
			EXPECT_EQ(decision.recovery_valid_count, 1U);

			decision = machine.evaluate(StateMachineInput{
				health,
				arbitration(teleop, ArbitrationEvent::kSwitch),
				{evidence(planning, RejectReason::kNone, 2U),
					evidence(teleop, RejectReason::kNone, 2U)},
				2U});
			EXPECT_EQ(decision.state, DataState::kRecovering);
			EXPECT_EQ(decision.recovery_valid_count, 1U);
			EXPECT_EQ(decision.selected->source_id, "teleop");
		}

		TEST(GatewayStateMachineTest, AppliesFailClosedHealthPriorityAndNoSourceRules)
		{
			const auto selected = snapshot(
				"planning", 150U, generation(14U), 1U, 50'000'000'000LL, steady_time(50s));
			GatewayStateMachine standby_machine{load_contract()};
			auto decision = standby_machine.evaluate(StateMachineInput{
				healthy_gateway(),
				ArbitrationDecision{std::nullopt, ArbitrationEvent::kNoChange},
				{},
				1U});
			EXPECT_EQ(decision.state, DataState::kStandby);

			GatewayStateMachine priority_machine{load_contract()};
			auto health = healthy_gateway();
			health.output_tick_healthy = false;
			health.critical_endpoints_healthy = false;
			health.critical_qos_compatible = false;
			decision = priority_machine.evaluate(StateMachineInput{
				health,
				arbitration(selected, ArbitrationEvent::kFirstSelection),
				{},
				1U});
			EXPECT_EQ(decision.state, DataState::kSafeStop);
			EXPECT_EQ(decision.reason, StateReason::kOutputTickOverrun);

			GatewayStateMachine clock_machine{load_contract()};
			health = healthy_gateway();
			health.ros_clock_healthy = false;
			decision = clock_machine.evaluate(StateMachineInput{
				health,
				arbitration(selected, ArbitrationEvent::kFirstSelection),
				{},
				1U});
			EXPECT_EQ(decision.state, DataState::kSafeStop);
			EXPECT_EQ(decision.reason, StateReason::kClockInvalid);

			GatewayStateMachine endpoint_machine{load_contract()};
			health = healthy_gateway();
			health.critical_endpoints_healthy = false;
			decision = endpoint_machine.evaluate(StateMachineInput{
				health,
				arbitration(selected, ArbitrationEvent::kFirstSelection),
				{},
				1U});
			EXPECT_EQ(decision.state, DataState::kSafeStop);
			EXPECT_EQ(decision.reason, StateReason::kCriticalEndpointUnhealthy);

			GatewayStateMachine no_source_machine{load_contract()};
			decision = no_source_machine.evaluate(StateMachineInput{
				healthy_gateway(),
				arbitration(selected, ArbitrationEvent::kFirstSelection),
				{},
				1U});
			ASSERT_EQ(decision.state, DataState::kRecovering);
			decision = no_source_machine.evaluate(StateMachineInput{
				healthy_gateway(),
				ArbitrationDecision{std::nullopt, ArbitrationEvent::kNoQualifiedSource},
				{},
				1U});
			EXPECT_EQ(decision.state, DataState::kSafeStop);
			EXPECT_EQ(decision.reason, StateReason::kNoQualifiedSource);
		}

		control_link_interfaces::msg::VehicleState valid_vehicle_state(
			std::int64_t observed_at_ns = 59'950'000'000LL)
		{
			control_link_interfaces::msg::VehicleState state;
			state.observed_at = stamp_from_nanoseconds(observed_at_ns);
			state.state = control_link_interfaces::msg::VehicleState::RUNNING;
			state.fault_code = control_link_interfaces::msg::VehicleState::FAULT_NONE;
			state.linear_velocity_mps = 0.4;
			state.angular_velocity_radps = 0.1;
			state.rolling_counter = 7U;
			return state;
		}

		VehicleStateValidationContext vehicle_context(
			const PublisherGenerationKey &publisher_generation)
		{
			return VehicleStateValidationContext{
				publisher_generation,
				60'000'000'000LL,
				true,
				std::chrono::steady_clock::time_point{60s}};
		}

		TEST(VehicleStateTest, RejectsBadFeedbackWithoutRefreshingLastValidReceiveTime)
		{
			VehicleStateValidator validator{load_contract()};
			VehicleStateRuntime runtime;
			const auto publisher_generation = generation(15U);
			const auto context = vehicle_context(publisher_generation);
			const auto accepted = validator.validate(valid_vehicle_state(), context);
			ASSERT_TRUE(accepted.accepted());
			commit_vehicle_state_validation_result(runtime, accepted);
			ASSERT_TRUE(runtime.latest_valid_snapshot.has_value());

			auto health = assess_vehicle_state_health(runtime, context.received_at + 150ms, 150ms);
			EXPECT_TRUE(health.valid);
			EXPECT_TRUE(health.fresh);

			auto invalid_state = valid_vehicle_state();
			invalid_state.state = 255U;
			const auto rejected = validator.validate(invalid_state, context);
			EXPECT_EQ(rejected.reason, VehicleStateRejectReason::kUnknownState);
			commit_vehicle_state_validation_result(runtime, rejected);
			EXPECT_FALSE(runtime.latest_message_valid);
			EXPECT_EQ(runtime.rejected_count, 1U);
			ASSERT_TRUE(runtime.latest_valid_snapshot.has_value());
			EXPECT_EQ(runtime.latest_valid_snapshot->received_at, context.received_at);

			health = assess_vehicle_state_health(runtime, context.received_at + 151ms, 150ms);
			EXPECT_FALSE(health.valid);
			EXPECT_FALSE(health.fresh);
		}

		TEST(VehicleStateTest, ValidatesContentTimeFaultAndRuntimeInvariants)
		{
			VehicleStateValidator validator{load_contract()};
			const auto publisher_generation = generation(16U);
			const auto context = vehicle_context(publisher_generation);
			auto state = valid_vehicle_state();

			state.linear_velocity_mps = std::numeric_limits<double>::infinity();
			EXPECT_EQ(
				validator.validate(state, context).reason,
				VehicleStateRejectReason::kNonFinite);
			state = valid_vehicle_state(60'020'000'001LL);
			EXPECT_EQ(
				validator.validate(state, context).reason,
				VehicleStateRejectReason::kFutureStamp);
			state = valid_vehicle_state(59'849'999'999LL);
			EXPECT_EQ(
				validator.validate(state, context).reason,
				VehicleStateRejectReason::kStale);

			VehicleStateRuntime fault_runtime;
			state = valid_vehicle_state();
			state.state = control_link_interfaces::msg::VehicleState::SAFE_STOP;
			state.fault_code = control_link_interfaces::msg::VehicleState::FAULT_ADAS_CAN_IO;
			commit_vehicle_state_validation_result(
				fault_runtime,
				validator.validate(state, context));
			const auto fault_health = assess_vehicle_state_health(
				fault_runtime, context.received_at, 150ms);
			EXPECT_TRUE(fault_health.reports_safe_stop);
			EXPECT_TRUE(fault_health.reports_fault);

			EXPECT_THROW(
				commit_vehicle_state_validation_result(
					fault_runtime,
					VehicleStateValidationResult{
						VehicleStateRejectReason::kNone,
						std::nullopt}),
				std::logic_error);
			EXPECT_THROW(
				(void)assess_vehicle_state_health(fault_runtime, context.received_at, 0ms),
				std::invalid_argument);
		}
	}
}  // namespace control_link_gateway
