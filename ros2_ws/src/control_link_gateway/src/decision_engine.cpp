#include "control_link_gateway/decision_engine.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <utility>
#include <variant>

#include "rclcpp/time.hpp"

#include "control_link_gateway/source_binding.hpp"
#include "control_link_gateway/source_runtime.hpp"

namespace control_link_gateway
{
	namespace
	{
		constexpr std::int64_t kNanosecondsPerSecond = 1'000'000'000LL;
		constexpr std::uint64_t kNanosecondsPerMillisecond = 1'000'000ULL;

		std::map<std::string, SourceRuntimeSlot> make_source_slots(
			const control_link_contract::ContractBundle &bundle)
		{
			if (!bundle.profile)
			{
				throw std::invalid_argument("DecisionEngine requires a non-null ProfileConfig");
			}

			std::map<std::string, SourceRuntimeSlot> slots;
			std::visit(
				[&slots](const auto &profile)
				{
					for (const auto &source_id : profile.common.enabled_sources)
					{
						if (!slots.try_emplace(source_id).second)
						{
							throw std::logic_error(
								"duplicate enabled source reached DecisionEngine: " + source_id);
						}
					}
				},
				*bundle.profile);
			return slots;
		}

		std::map<std::string, DecisionSourceEndpoint> make_initial_source_endpoints(
			const std::map<std::string, SourceRuntimeSlot> &slots)
		{
			std::map<std::string, DecisionSourceEndpoint> result;
			for (const auto &[source_id, unused] : slots)
			{
				(void)unused;
				result.emplace(
					source_id,
					DecisionSourceEndpoint{
						source_id,
						DecisionSourceEndpointState::kMissing,
						std::nullopt});
			}
			return result;
		}

		bool recovery_health_is_healthy(
			const GatewayHealthSnapshot &health) noexcept
		{
			return health.critical_endpoints_healthy &&
				health.critical_qos_compatible &&
				health.ros_clock_healthy &&
				health.vehicle_state_valid &&
				health.vehicle_state_fresh &&
				!health.vehicle_reports_safe_stop &&
				!health.vehicle_reports_fault &&
				health.output_tick_healthy &&
				health.internal_invariants_healthy;
		}

		std::chrono::steady_clock::time_point steady_time_point(
			std::int64_t offset_ns)
		{
			if (offset_ns < 0)
			{
				throw std::invalid_argument("DecisionEvent steady offset must be non-negative");
			}
			const auto duration = std::chrono::duration_cast<std::chrono::steady_clock::duration>(
				std::chrono::nanoseconds{offset_ns});
			return std::chrono::steady_clock::time_point{duration};
		}

		std::int64_t checked_output_period_ns(double rate_hz)
		{
			if (!std::isfinite(rate_hz) || rate_hz <= 0.0)
			{
				throw std::invalid_argument("DecisionEngine output rate must be positive and finite");
			}
			const auto period = static_cast<long double>(kNanosecondsPerSecond) /
				static_cast<long double>(rate_hz);
			if (period < 1.0L ||
				period > static_cast<long double>(std::numeric_limits<std::int64_t>::max()))
			{
				throw std::out_of_range("DecisionEngine output period is not representable");
			}
			return static_cast<std::int64_t>(std::ceil(period));
		}

		std::int64_t checked_milliseconds_ns(
			std::uint64_t value,
			const char *field_name)
		{
			if (value == 0U ||
				value > static_cast<std::uint64_t>(
					std::numeric_limits<std::int64_t>::max()) /
					kNanosecondsPerMillisecond)
			{
				throw std::out_of_range(
					std::string(field_name) + " is not a positive representable duration");
			}
			return static_cast<std::int64_t>(value * kNanosecondsPerMillisecond);
		}

		builtin_interfaces::msg::Time ros_time_message(std::int64_t nanoseconds)
		{
			if (nanoseconds < 0)
			{
				throw std::logic_error("DecisionEngine cannot serialize negative canonical ROS time");
			}
			return static_cast<builtin_interfaces::msg::Time>(
				rclcpp::Time(nanoseconds, RCL_ROS_TIME));
		}

		control_link_interfaces::msg::ControlCommand make_canonical_command(
			const StateMachineDecision &decision)
		{
			using Command = control_link_interfaces::msg::ControlCommand;
			Command result;
			if (decision.state == DataState::kActive && !decision.selected.has_value())
			{
				throw std::logic_error("ACTIVE DecisionEngine result has no selected source");
			}
			if (decision.selected.has_value())
			{
				const auto &selected = decision.selected.value();
				result.source_stamp = ros_time_message(selected.source_stamp_ns);
				result.source_id = selected.source_id;
				result.source_sequence = selected.sequence;
			}
			if (decision.state != DataState::kActive)
			{
				result.mode = Command::MODE_HOLD;
				return result;
			}
			const auto &selected = decision.selected.value();
			result.mode = static_cast<std::uint8_t>(selected.mode);
			result.linear_velocity_mps = selected.linear_velocity_mps;
			result.angular_velocity_radps = selected.angular_velocity_radps;
			return result;
		}

		RejectReason binding_reject_reason(DecisionSourceEndpointState state) noexcept
		{
			return state == DecisionSourceEndpointState::kAmbiguous ?
				RejectReason::kSourceEndpointAmbiguous :
				RejectReason::kPublisherGenerationUnstable;
		}
	}  // namespace

	DecisionEngine::DecisionEngine(
		control_link_contract::ContractBundlePtr contract_bundle)
		: contract_bundle_(std::move(contract_bundle))
	{
		if (!contract_bundle_ || !contract_bundle_->gateway_contract ||
			!contract_bundle_->source_policy || !contract_bundle_->profile)
		{
			throw std::invalid_argument("DecisionEngine requires a complete ContractBundle");
		}
		source_slots_ = make_source_slots(*contract_bundle_);
		source_endpoints_ = make_initial_source_endpoints(source_slots_);
		command_validator_ = std::make_unique<CommandValidator>(
			contract_bundle_->gateway_contract);
		reset_active_decision_state();
	}

	std::optional<DecisionResult> DecisionEngine::apply_event(
		const DecisionEvent &event)
	{
		if (event.event_sequence != next_event_sequence())
		{
			throw std::invalid_argument(
				"DecisionEngine event_sequence must be contiguous: expected=" +
				std::to_string(next_event_sequence()) + ", actual=" +
				std::to_string(event.event_sequence));
		}

		std::optional<DecisionResult> result;
		std::visit(
			[this, &event, &result](const auto &payload)
			{
				using Payload = std::decay_t<decltype(payload)>;
				if constexpr (std::is_same_v<Payload, DecisionLifecycleEvent>)
				{
					apply_lifecycle_event(payload);
				}
				else if constexpr (std::is_same_v<Payload, DecisionHealthSnapshotEvent>)
				{
					apply_health_event(payload);
				}
				else if constexpr (std::is_same_v<Payload, DecisionSourceSampleEvent>)
				{
					apply_source_sample(payload);
				}
				else
				{
					result = apply_output_tick(event.event_sequence, payload);
				}
			},
			event.payload);
		last_event_sequence_ = event.event_sequence;
		return result;
	}

	std::uint64_t DecisionEngine::next_event_sequence() const
	{
		if (last_event_sequence_ == std::numeric_limits<std::uint64_t>::max())
		{
			throw std::overflow_error("DecisionEngine event_sequence is exhausted");
		}
		return last_event_sequence_ + 1U;
	}

	std::uint64_t DecisionEngine::next_health_revision() const
	{
		if (health_revision_ == std::numeric_limits<std::uint64_t>::max())
		{
			throw std::overflow_error("DecisionEngine health revision is exhausted");
		}
		return health_revision_ + 1U;
	}

	bool DecisionEngine::configured() const noexcept
	{
		return configured_;
	}

	bool DecisionEngine::active() const noexcept
	{
		return active_;
	}

	DecisionOutputTickEvent DecisionEngine::describe_output_tick(
		std::int64_t steady_offset_ns,
		std::int64_t now_ros_ns) const
	{
		validate_steady_offset(steady_offset_ns);
		std::int64_t interval_ns = 0;
		if (last_output_tick_offset_ns_.has_value())
		{
			interval_ns = steady_offset_ns - last_output_tick_offset_ns_.value();
		}
		const auto scheduled_period_ns = checked_output_period_ns(
			contract_bundle_->gateway_contract->gateway.output_rate_hz);
		return DecisionOutputTickEvent{
			steady_offset_ns,
			now_ros_ns,
			interval_ns,
			interval_ns > scheduled_period_ns ? interval_ns - scheduled_period_ns : 0,
			health_revision_};
	}

	const GatewayHealthSnapshot &DecisionEngine::health_snapshot() const noexcept
	{
		return health_snapshot_;
	}

	const std::map<std::string, SourceRuntimeSlot> &DecisionEngine::source_slots() const noexcept
	{
		return source_slots_;
	}

	std::uint64_t DecisionEngine::consecutive_late_output_ticks() const noexcept
	{
		return consecutive_late_output_ticks_;
	}

	void DecisionEngine::apply_lifecycle_event(const DecisionLifecycleEvent &event)
	{
		if (event.result != DecisionLifecycleResult::kSuccess)
		{
			return;
		}

		switch (event.transition)
		{
		case DecisionLifecycleTransition::kConfigure:
			if (configured_ || active_)
			{
				throw std::logic_error("DecisionEngine cannot configure twice without cleanup");
			}
			configured_ = true;
			return;

		case DecisionLifecycleTransition::kActivate:
			if (!configured_ || active_ || health_revision_ == 0U)
			{
				throw std::logic_error("DecisionEngine activation state/revision is invalid");
			}
			reset_active_decision_state();
			active_ = true;
			return;

		case DecisionLifecycleTransition::kDeactivate:
			if (!configured_ || !active_)
			{
				throw std::logic_error("DecisionEngine cannot deactivate an inactive configuration");
			}
			active_ = false;
			pending_recovery_evidences_.clear();
			last_output_tick_offset_ns_.reset();
			consecutive_late_output_ticks_ = 0U;
			return;

		case DecisionLifecycleTransition::kCleanup:
			if (!configured_ || active_)
			{
				throw std::logic_error("DecisionEngine cleanup requires an inactive configuration");
			}
			configured_ = false;
			return;

		case DecisionLifecycleTransition::kError:
			active_ = false;
			pending_recovery_evidences_.clear();
			last_output_tick_offset_ns_.reset();
			return;
		}
		throw std::logic_error("DecisionEngine received an unsupported lifecycle transition");
	}

	void DecisionEngine::apply_health_event(const DecisionHealthSnapshotEvent &event)
	{
		if (!configured_)
		{
			throw std::logic_error("DecisionEngine health event requires configured lifecycle state");
		}
		if (event.health_revision != next_health_revision())
		{
			throw std::invalid_argument(
				"DecisionEngine health revision must be contiguous: expected=" +
				std::to_string(next_health_revision()) + ", actual=" +
				std::to_string(event.health_revision));
		}
		validate_steady_offset(event.steady_observed_offset_ns);
		if (event.source_endpoints.size() != source_slots_.size())
		{
			throw std::invalid_argument(
				"DecisionEngine health event must cover every enabled source");
		}

		std::map<std::string, DecisionSourceEndpoint> next_endpoints;
		for (const auto &endpoint : event.source_endpoints)
		{
			if (source_slots_.count(endpoint.source_id) == 0U ||
				!next_endpoints.emplace(endpoint.source_id, endpoint).second)
			{
				throw std::invalid_argument(
					"DecisionEngine health event contains unknown or duplicate source: " +
					endpoint.source_id);
			}
			const bool usable = endpoint.state == DecisionSourceEndpointState::kUsable;
			if (usable != endpoint.publisher_generation.has_value())
			{
				throw std::invalid_argument(
					"DecisionEngine usable source endpoint must carry exactly one generation");
			}
		}

		// 前置校验完成后才修改 Slot，避免非法 health event 留下半提交 generation
		for (const auto &[source_id, endpoint] : next_endpoints)
		{
			auto &slot = source_slots_.at(source_id);
			if (endpoint.state == DecisionSourceEndpointState::kUsable)
			{
				(void)confirm_publisher_generation(
					slot, endpoint.publisher_generation.value());
			}
			else
			{
				invalidate_source_endpoint_snapshot(slot);
			}
		}

		source_endpoints_ = std::move(next_endpoints);
		health_snapshot_ = event.health;
		health_revision_ = event.health_revision;
		update_health_epoch();
		commit_steady_offset(event.steady_observed_offset_ns);
	}

	void DecisionEngine::apply_source_sample(const DecisionSourceSampleEvent &event)
	{
		if (!configured_ || !active_)
		{
			throw std::logic_error("DecisionEngine source sample requires active lifecycle state");
		}
		validate_steady_offset(event.steady_receive_offset_ns);
		auto slot_iterator = source_slots_.find(event.expected_source_id);
		auto endpoint_iterator = source_endpoints_.find(event.expected_source_id);
		if (slot_iterator == source_slots_.end() || endpoint_iterator == source_endpoints_.end())
		{
			throw std::invalid_argument(
				"DecisionEngine source sample targets a disabled source: " +
				event.expected_source_id);
		}

		auto &slot = slot_iterator->second;
		const auto &endpoint = endpoint_iterator->second;
		const bool generation_matches =
			endpoint.state == DecisionSourceEndpointState::kUsable &&
			endpoint.publisher_generation.has_value() &&
			endpoint.publisher_generation.value() == event.publisher_generation;
		if (!generation_matches)
		{
			const auto reason = binding_reject_reason(endpoint.state);
			commit_validation_result(
				slot,
				CommandValidationResult{reason, std::nullopt});
			pending_recovery_evidences_.push_back(
				RecoveryEvidence{
					RecoveryCandidateKey{
						event.expected_source_id,
						event.publisher_generation},
					reason,
					health_epoch_});
			commit_steady_offset(event.steady_receive_offset_ns);
			return;
		}

		if (!message_matches_confirmed_generation(slot, event.publisher_generation))
		{
			throw std::logic_error(
				"DecisionEngine usable endpoint generation does not match SourceRuntimeSlot");
		}
		const auto &source = contract_bundle_->source_policy->sources.at(
			event.expected_source_id);
		const CommandValidationContext context{
			event.expected_source_id,
			event.publisher_generation,
			slot.last_accepted_sequence,
			source.priority,
			event.now_ros_ns,
			health_snapshot_.ros_clock_healthy,
			steady_time_point(event.steady_receive_offset_ns)};
		const auto result = command_validator_->validate(event.command, context);
		commit_validation_result(slot, result);
		pending_recovery_evidences_.push_back(
			RecoveryEvidence{
				RecoveryCandidateKey{
					event.expected_source_id,
					event.publisher_generation},
				result.reason,
				health_epoch_});
		commit_steady_offset(event.steady_receive_offset_ns);
	}

	DecisionResult DecisionEngine::apply_output_tick(
		std::uint64_t event_sequence,
		const DecisionOutputTickEvent &event)
	{
		if (!configured_ || !active_)
		{
			throw std::logic_error("DecisionEngine output tick requires active lifecycle state");
		}
		if (event.health_revision != health_revision_ || health_revision_ == 0U)
		{
			throw std::invalid_argument(
				"DecisionEngine output tick consumed a stale or future health revision");
		}
		validate_steady_offset(event.steady_offset_ns);
		const auto described = describe_output_tick(event.steady_offset_ns, event.now_ros_ns);
		if (event.tick_interval_ns != described.tick_interval_ns ||
			event.tick_lateness_ns != described.tick_lateness_ns)
		{
			throw std::invalid_argument(
				"DecisionEngine output tick timing fields do not match logical steady time");
		}

		const auto &gateway = contract_bundle_->gateway_contract->gateway;
		if (last_output_tick_offset_ns_.has_value())
		{
			const auto late_threshold_ns = checked_milliseconds_ns(
				gateway.output_tick_late_threshold_ms,
				"gateway.output_tick_late_threshold_ms");
			if (event.tick_interval_ns > late_threshold_ns)
			{
				if (consecutive_late_output_ticks_ < std::numeric_limits<std::uint64_t>::max())
				{
					consecutive_late_output_ticks_ += 1U;
				}
			}
			else
			{
				consecutive_late_output_ticks_ = 0U;
			}
		}
		last_output_tick_offset_ns_ = event.steady_offset_ns;
		if (gateway.consecutive_late_ticks_to_safe_stop == 0U)
		{
			throw std::logic_error("DecisionEngine reached zero late-tick safety threshold");
		}
		health_snapshot_.output_tick_healthy = consecutive_late_output_ticks_ <
			gateway.consecutive_late_ticks_to_safe_stop;
		update_health_epoch();

		auto snapshots = collect_latest_valid_snapshots(source_slots_);
		const auto arbitration = source_arbiter_->evaluate(
			ArbitrationInput{
				&snapshots,
				event.now_ros_ns,
				steady_time_point(event.steady_offset_ns)});
		auto evidences = std::move(pending_recovery_evidences_);
		pending_recovery_evidences_.clear();
		const auto state_decision = state_machine_->evaluate(
			StateMachineInput{
				health_snapshot_,
				arbitration,
				std::move(evidences),
				health_epoch_});
		DecisionResult result;
		result.event_sequence = event_sequence;
		result.state = state_decision.state;
		result.reason = state_decision.reason;
		if (state_decision.state == DataState::kRecovering &&
			state_decision.selected.has_value())
		{
			result.recovery_candidate = RecoveryCandidateKey{
				state_decision.selected->source_id,
				state_decision.selected->publisher_generation};
		}
		result.recovery_valid_count = state_decision.recovery_valid_count;
		result.transition_sequence = state_decision.transition_sequence;
		result.canonical_command = make_canonical_command(state_decision);
		result.lifecycle_error_requested = state_decision.state == DataState::kError;

		result.sources.reserve(source_slots_.size());
		for (const auto &[source_id, slot] : source_slots_)
		{
			const auto &endpoint = source_endpoints_.at(source_id);
			const bool generation_matches =
				endpoint.state == DecisionSourceEndpointState::kUsable &&
				endpoint.publisher_generation.has_value() &&
				slot.confirmed_publisher_generation.has_value() &&
				endpoint.publisher_generation.value() ==
					slot.confirmed_publisher_generation.value();
			DecisionSourceStatus status;
			status.source_id = source_id;
			status.accepted_count = slot.accepted_count;
			status.rejected_count = slot.rejected_count;
			status.last_reject_reason = slot.last_reject_reason;
			status.last_accepted_sequence = slot.last_accepted_sequence;
			status.command_valid = generation_matches && slot.latest_valid_snapshot.has_value();
			if (slot.latest_valid_snapshot.has_value())
			{
				const auto assessment = assess_source_snapshot(
					source_id,
					slot.latest_valid_snapshot.value(),
					*contract_bundle_->gateway_contract,
					*contract_bundle_->source_policy,
					event.now_ros_ns,
					steady_time_point(event.steady_offset_ns));
				status.lease_valid = status.command_valid && assessment.lease_valid;
				status.command_age_ns = assessment.command_age_ns;
			}
			result.sources.push_back(std::move(status));
		}
		commit_steady_offset(event.steady_offset_ns);
		return result;
	}

	void DecisionEngine::reset_active_decision_state()
	{
		source_arbiter_ = std::make_unique<SourceArbiter>(
			contract_bundle_->gateway_contract,
			contract_bundle_->source_policy);
		state_machine_ = std::make_unique<GatewayStateMachine>(
			contract_bundle_->gateway_contract);
		pending_recovery_evidences_.clear();
		last_output_tick_offset_ns_.reset();
		consecutive_late_output_ticks_ = 0U;
		health_snapshot_.output_tick_healthy = true;
		update_health_epoch();
	}

	void DecisionEngine::update_health_epoch()
	{
		const bool healthy = recovery_health_is_healthy(health_snapshot_);
		if (healthy == recovery_health_healthy_)
		{
			return;
		}
		pending_recovery_evidences_.clear();
		if (health_epoch_ == std::numeric_limits<std::uint64_t>::max())
		{
			health_snapshot_.internal_invariants_healthy = false;
			recovery_health_healthy_ = false;
			return;
		}
		health_epoch_ += 1U;
		recovery_health_healthy_ = healthy;
	}

	void DecisionEngine::validate_steady_offset(std::int64_t offset_ns) const
	{
		if (offset_ns < 0 ||
			(last_steady_offset_ns_.has_value() && offset_ns < last_steady_offset_ns_.value()))
		{
			throw std::invalid_argument("DecisionEngine logical steady time moved backwards");
		}
	}

	void DecisionEngine::commit_steady_offset(std::int64_t offset_ns) noexcept
	{
		last_steady_offset_ns_ = offset_ns;
	}
}  // namespace control_link_gateway
