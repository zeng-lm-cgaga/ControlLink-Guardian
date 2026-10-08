#include <algorithm>
#include <chrono>
#include <cstdint>
#include <limits>
#include <map>
#include <memory>
#include <optional>
#include <random>
#include <string>
#include <utility>
#include <vector>

#include <gtest/gtest.h>

#include "control_link_contract/contract_bundle.hpp"
#include "control_link_contract/parser.hpp"
#include "control_link_gateway/command_validator.hpp"
#include "control_link_gateway/decision_engine.hpp"
#include "control_link_gateway/source_arbiter.hpp"
#include "control_link_gateway/source_binding.hpp"
#include "control_link_gateway/source_runtime.hpp"
#include "control_link_gateway/state_machine.hpp"
#include "control_link_interfaces/msg/control_command.hpp"

namespace control_link_gateway
{
	namespace
	{
		using namespace std::chrono_literals;

		constexpr std::int64_t kNanosecondsPerSecond = 1'000'000'000LL;
		constexpr std::int64_t kNanosecondsPerMillisecond = 1'000'000LL;
		constexpr std::uint64_t kValidatorIterations = 2048U;
		constexpr std::uint64_t kArbiterIterations = 10'000U;
		constexpr std::uint64_t kStateMachineIterations = 2048U;

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

		control_link_contract::ContractBundlePtr load_adas_bundle()
		{
			return control_link_contract::load_contract_bundle(
				CONTROL_LINK_TEST_ADAS_PROFILE_PATH,
				CONTROL_LINK_TEST_CONFIG_ROOT);
		}

		PublisherGenerationKey generation_for(std::uint8_t seed)
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

		control_link_interfaces::msg::ControlCommand valid_command(
			std::string source_id,
			std::uint64_t sequence,
			std::int64_t stamp_ns)
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

		void expect_same_decision_result(
			const std::optional<DecisionResult> &left,
			const std::optional<DecisionResult> &right)
		{
			ASSERT_EQ(left.has_value(), right.has_value());
			if (!left.has_value())
			{
				return;
			}

			EXPECT_EQ(left->event_sequence, right->event_sequence);
			EXPECT_EQ(left->state, right->state);
			EXPECT_EQ(left->reason, right->reason);
			EXPECT_EQ(left->recovery_valid_count, right->recovery_valid_count);
			EXPECT_EQ(left->transition_sequence, right->transition_sequence);
			EXPECT_EQ(left->lifecycle_error_requested, right->lifecycle_error_requested);
			EXPECT_EQ(left->canonical_command.source_id, right->canonical_command.source_id);
			EXPECT_EQ(
				left->canonical_command.source_sequence,
				right->canonical_command.source_sequence);
			EXPECT_EQ(left->canonical_command.mode, right->canonical_command.mode);
			EXPECT_EQ(
				left->canonical_command.source_stamp,
				right->canonical_command.source_stamp);
			EXPECT_EQ(
				left->canonical_command.linear_velocity_mps,
				right->canonical_command.linear_velocity_mps);
			EXPECT_EQ(
				left->canonical_command.angular_velocity_radps,
				right->canonical_command.angular_velocity_radps);
			ASSERT_EQ(left->sources.size(), right->sources.size());
			for (std::size_t index = 0U; index < left->sources.size(); ++index)
			{
				EXPECT_EQ(left->sources[index].source_id, right->sources[index].source_id);
				EXPECT_EQ(left->sources[index].accepted_count, right->sources[index].accepted_count);
				EXPECT_EQ(left->sources[index].rejected_count, right->sources[index].rejected_count);
				EXPECT_EQ(
					left->sources[index].last_reject_reason,
					right->sources[index].last_reject_reason);
				EXPECT_EQ(
					left->sources[index].last_accepted_sequence,
					right->sources[index].last_accepted_sequence);
				EXPECT_EQ(left->sources[index].command_valid, right->sources[index].command_valid);
				EXPECT_EQ(left->sources[index].lease_valid, right->sources[index].lease_valid);
				EXPECT_EQ(left->sources[index].command_age_ns, right->sources[index].command_age_ns);
			}
		}

		TEST(PropertyInvariant, RejectedCommandsPreserveSlotAndGenerationChangesResetBaseline)
		{
			constexpr std::uint64_t seed = 0xC011'0009ULL;
			RecordProperty("property_seed", std::to_string(seed));
			RecordProperty("generator_version", "validator-v1");
			RecordProperty("iterations", std::to_string(kValidatorIterations));

			const auto contract = load_contract();
			CommandValidator validator{contract};
			for (std::uint64_t iteration = 0U; iteration < kValidatorIterations; ++iteration)
			{
				const auto generation_seed = static_cast<std::uint8_t>(iteration + 7U);
				const auto first_generation = generation_for(generation_seed);
				SourceRuntimeSlot slot;
				EXPECT_EQ(
					confirm_publisher_generation(slot, first_generation),
					PublisherGenerationUpdate::kFirstConfirmation);

				const auto initial_context = CommandValidationContext{
					"planning",
					first_generation,
					std::nullopt,
					150U,
					10'000'000'000LL,
					true,
					std::chrono::steady_clock::time_point{10s}};
				const auto initial_result = validator.validate(
					valid_command("planning", 1U, 9'950'000'000LL),
					initial_context);
				ASSERT_TRUE(initial_result.accepted()) << "iteration=" << iteration;
				commit_validation_result(slot, initial_result);

				const auto before_rejection = slot;
				auto invalid_command = valid_command(
					"planning", 1U, 9'950'000'000LL);
				switch (iteration % 6U)
				{
				case 0U:
					invalid_command.source_id = "teleop";
					break;
				case 1U:
					invalid_command.mode = 255U;
					break;
				case 2U:
					invalid_command.linear_velocity_mps =
						std::numeric_limits<double>::quiet_NaN();
					break;
				case 3U:
					invalid_command.source_stamp = stamp_from_nanoseconds(9'899'999'999LL);
					break;
				case 4U:
					invalid_command.source_stamp = stamp_from_nanoseconds(10'020'000'001LL);
					break;
				case 5U:
					break;
				}

				const auto rejected = validator.validate(
					invalid_command,
					CommandValidationContext{
						"planning",
						first_generation,
						before_rejection.last_accepted_sequence,
						150U,
						10'000'000'000LL,
						true,
						std::chrono::steady_clock::time_point{10s}});
				ASSERT_FALSE(rejected.accepted()) << "iteration=" << iteration;
				commit_validation_result(slot, rejected);

				ASSERT_TRUE(slot.latest_valid_snapshot.has_value()) << "iteration=" << iteration;
				ASSERT_TRUE(before_rejection.latest_valid_snapshot.has_value());
				EXPECT_EQ(
					slot.latest_valid_snapshot->publisher_generation,
					before_rejection.latest_valid_snapshot->publisher_generation);
				EXPECT_EQ(
					slot.latest_valid_snapshot->sequence,
					before_rejection.latest_valid_snapshot->sequence);
				EXPECT_EQ(
					slot.latest_valid_snapshot->source_stamp_ns,
					before_rejection.latest_valid_snapshot->source_stamp_ns);
				EXPECT_EQ(slot.last_accepted_sequence, before_rejection.last_accepted_sequence);
				EXPECT_EQ(slot.accepted_count, before_rejection.accepted_count);
				EXPECT_EQ(slot.rejected_count, before_rejection.rejected_count + 1U);

				const auto second_generation = generation_for(
					static_cast<std::uint8_t>(generation_seed + 1U));
				EXPECT_EQ(
					confirm_publisher_generation(slot, second_generation),
					PublisherGenerationUpdate::kChanged);
				EXPECT_FALSE(slot.latest_valid_snapshot.has_value());
				EXPECT_FALSE(slot.last_accepted_sequence.has_value());
				EXPECT_EQ(
					confirm_publisher_generation(slot, second_generation),
					PublisherGenerationUpdate::kUnchanged);
			}
		}

		TEST(PropertyInvariant, ArbiterIsPermutationInvariantAndSelectsQualifiedSnapshots)
		{
			constexpr std::uint64_t seed = 0xA8B1'0001ULL;
			RecordProperty("property_seed", std::to_string(seed));
			RecordProperty("generator_version", "arbiter-permutation-v1");
			RecordProperty("iterations", std::to_string(kArbiterIterations));

			const auto contract = load_contract();
			const auto policy = load_policy();
			const auto base_steady = std::chrono::steady_clock::time_point{60s};
			constexpr std::int64_t now_ros_ns = 60'000'000'000LL;
			std::mt19937_64 random{seed};
			std::optional<std::string> expected_source;

			std::vector<std::string> source_ids;
			for (const auto &[source_id, entry] : policy->sources)
			{
				(void)entry;
				source_ids.push_back(source_id);
			}

			for (std::uint64_t iteration = 0U; iteration < kArbiterIterations; ++iteration)
			{
				std::shuffle(source_ids.begin(), source_ids.end(), random);
				std::map<std::string, SourceSnapshot> snapshots;
				for (const auto &source_id : source_ids)
				{
					const auto &policy_entry = policy->sources.at(source_id);
					const auto age_ms = random() % 50U;
					const auto receive_ms = random() % 50U;
					snapshots.emplace(
						source_id,
						snapshot(
							source_id,
							policy_entry.priority,
							generation_for(static_cast<std::uint8_t>(iteration + source_id.size())),
							iteration + 1U,
							now_ros_ns - static_cast<std::int64_t>(age_ms) * kNanosecondsPerMillisecond,
							base_steady - std::chrono::milliseconds(receive_ms)));
				}

				SourceArbiter arbiter{contract, policy};
				const auto decision = arbiter.evaluate(
					ArbitrationInput{&snapshots, now_ros_ns, base_steady});
				ASSERT_TRUE(decision.selected.has_value()) << "iteration=" << iteration;
				ASSERT_EQ(decision.event, ArbitrationEvent::kFirstSelection);
				if (!expected_source.has_value())
				{
					expected_source = decision.selected->source_id;
				}
				EXPECT_EQ(decision.selected->source_id, expected_source.value())
					<< "iteration=" << iteration;
				const auto assessment = assess_source_snapshot(
					decision.selected->source_id,
					decision.selected.value(),
					*contract,
					*policy,
					now_ros_ns,
					base_steady);
				EXPECT_TRUE(assessment.qualified()) << "iteration=" << iteration;
			}
		}

		TEST(PropertyInvariant, StateMachineNeverProducesActiveForUnsafeHealth)
		{
			constexpr std::uint64_t seed = 0x57A7'0001ULL;
			RecordProperty("property_seed", std::to_string(seed));
			RecordProperty("generator_version", "state-health-gate-v1");
			RecordProperty("iterations", std::to_string(kStateMachineIterations));

			const auto contract = load_contract();
			const auto selected = snapshot(
				"planning",
				150U,
				generation_for(77U),
				1U,
				70'000'000'000LL,
				std::chrono::steady_clock::time_point{70s});
			for (std::uint64_t iteration = 0U; iteration < kStateMachineIterations; ++iteration)
			{
				GatewayStateMachine machine{contract};
				auto health = healthy_gateway();
				switch (iteration % 7U)
				{
				case 0U:
					health.critical_endpoints_healthy = false;
					break;
				case 1U:
					health.critical_qos_compatible = false;
					break;
				case 2U:
					health.ros_clock_healthy = false;
					break;
				case 3U:
					health.vehicle_state_valid = false;
					break;
				case 4U:
					health.vehicle_state_fresh = false;
					break;
				case 5U:
					health.vehicle_reports_safe_stop = true;
					break;
				case 6U:
					health.vehicle_reports_fault = true;
					break;
				}
				const auto decision = machine.evaluate(StateMachineInput{
					health,
					arbitration(selected, ArbitrationEvent::kFirstSelection),
					{},
					iteration + 1U});
				EXPECT_NE(decision.state, DataState::kActive) << "iteration=" << iteration;
				EXPECT_NE(decision.state, DataState::kError) << "iteration=" << iteration;
			}
		}

		TEST(PropertyInvariant, DecisionEngineIsDeterministicAndRejectedSamplesDoNotAdvanceRecovery)
		{
			constexpr std::uint64_t seed = 0xDEC1'5100ULL;
			RecordProperty("property_seed", std::to_string(seed));
			RecordProperty("generator_version", "decision-engine-event-sequence-v1");
			RecordProperty("iterations", "7");

			auto left = DecisionEngine{load_adas_bundle()};
			auto right = DecisionEngine{load_adas_bundle()};
			std::uint64_t event_sequence = 1U;
			auto apply_both = [&](DecisionEventPayload payload)
			{
				const DecisionEvent event{event_sequence++, std::move(payload)};
				const auto left_result = left.apply_event(event);
				const auto right_result = right.apply_event(event);
				expect_same_decision_result(left_result, right_result);
				return left_result;
			};

			apply_both(DecisionLifecycleEvent{
				DecisionLifecycleTransition::kConfigure,
				DecisionLifecycleResult::kSuccess});
			const auto planning_generation = generation_for(90U);
			const auto teleop_generation = generation_for(91U);
			auto health = healthy_gateway();
			apply_both(DecisionHealthSnapshotEvent{
				1U,
				0,
				health,
				{
					DecisionSourceEndpoint{
						"planning",
						DecisionSourceEndpointState::kUsable,
						planning_generation},
					DecisionSourceEndpoint{
						"teleop",
						DecisionSourceEndpointState::kUsable,
						teleop_generation}}});
			apply_both(DecisionLifecycleEvent{
				DecisionLifecycleTransition::kActivate,
				DecisionLifecycleResult::kSuccess});

			const std::vector<std::uint64_t> sequences{1U, 1U, 2U, 3U, 4U, 5U, 6U};
			std::optional<DecisionResult> last_result;
			for (std::size_t index = 0U; index < sequences.size(); ++index)
			{
				const auto offset_ns = static_cast<std::int64_t>(index + 1U) *
					20LL * kNanosecondsPerMillisecond;
				const auto now_ros_ns =
					kNanosecondsPerSecond + offset_ns;
				apply_both(DecisionSourceSampleEvent{
					"planning",
					planning_generation,
					valid_command(
						"planning",
						sequences[index],
						now_ros_ns - 10LL * kNanosecondsPerMillisecond),
					now_ros_ns,
					offset_ns - kNanosecondsPerMillisecond});

				const auto tick = left.describe_output_tick(offset_ns, now_ros_ns);
				const auto result = apply_both(tick);
				ASSERT_TRUE(result.has_value());
				last_result = result;
				if (result->state != DataState::kActive)
				{
					EXPECT_EQ(
						result->canonical_command.mode,
						control_link_interfaces::msg::ControlCommand::MODE_HOLD)
						<< "iteration=" << index;
					EXPECT_DOUBLE_EQ(result->canonical_command.linear_velocity_mps, 0.0);
					EXPECT_DOUBLE_EQ(result->canonical_command.angular_velocity_radps, 0.0);
				}
				const auto &planning_slot = left.source_slots().at("planning");
				if (sequences[index] == 1U && index == 1U)
				{
					EXPECT_EQ(planning_slot.last_accepted_sequence, std::optional<std::uint64_t>{1U});
					EXPECT_EQ(result->recovery_valid_count, 0U);
				}
			}

			ASSERT_TRUE(last_result.has_value());
			EXPECT_EQ(last_result->state, DataState::kActive);
			EXPECT_EQ(
				last_result->canonical_command.mode,
				control_link_interfaces::msg::ControlCommand::MODE_NORMAL);
			EXPECT_EQ(last_result->canonical_command.source_sequence, 6U);
		}
	}
}  // namespace control_link_gateway
