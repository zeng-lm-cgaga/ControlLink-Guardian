#include "control_link_gateway/control_gateway_node.hpp"

#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <filesystem>
#include <limits>
#include <map>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>
#include <utility>
#include <variant>
#include <vector>

#include "diagnostic_msgs/msg/diagnostic_status.hpp"
#include "diagnostic_msgs/msg/key_value.hpp"
#include "control_link_contract/fastdds_environment.hpp"
#include "control_link_gateway/source_binding.hpp"
#include "control_link_gateway/source_runtime.hpp"
#include "lifecycle_msgs/msg/transition.hpp"
#include "rclcpp/logging.hpp"
#include "rclcpp/parameter.hpp"
#include "rclcpp/subscription_options.hpp"
#include "rclcpp/time.hpp"
#include "rmw/rmw.h"

namespace control_link_gateway
{
	namespace
	{
		constexpr char kProfilePathParameter[] = "profile_path";
		constexpr char kConfigRootParameter[] = "config_root";
		constexpr char kDecisionTracePathParameter[] = "decision_trace_path";
		constexpr char kDecisionTraceQueueCapacityParameter[] =
			"decision_trace_queue_capacity";
		constexpr char kCanonicalConsumerId[] = "canonical_output_consumer";
		constexpr char kVehicleStateProducerId[] = "vehicle_state_producer";

		struct SourceGraphTransitionLog
		{
			std::string source_id;
			SourceEndpointState state;
			std::size_t publisher_count;
		};

		struct CriticalGraphTransitionLog
		{
			std::string endpoint_id;
			bool identity_healthy;
			bool qos_healthy;
			bool qos_observation_complete;
		};

		std::chrono::milliseconds checked_milliseconds(
			std::uint64_t value,
			const char *field_name)
		{
			if (value == 0U)
			{
				throw std::invalid_argument(
					std::string(field_name) + " must be greater than zero");
			}

			constexpr auto kMaximumMilliseconds =
				static_cast<std::uint64_t>(
					std::numeric_limits<std::chrono::milliseconds::rep>::max());
			if (value > kMaximumMilliseconds)
			{
				throw std::out_of_range(
					std::string(field_name) + " exceeds steady timer range");
			}

			return std::chrono::milliseconds{
				static_cast<std::chrono::milliseconds::rep>(value)};
		}

		std::chrono::nanoseconds checked_output_period(double rate_hz)
		{
			if (!std::isfinite(rate_hz) || rate_hz <= 0.0)
			{
				throw std::invalid_argument(
					"gateway.output_rate_hz must be finite and greater than zero");
			}

			constexpr long double kNanosecondsPerSecond = 1'000'000'000.0L;
			const auto period_ns =
				kNanosecondsPerSecond / static_cast<long double>(rate_hz);
			const auto maximum = static_cast<long double>(
				std::numeric_limits<std::chrono::nanoseconds::rep>::max());
			if (period_ns < 1.0L || period_ns > maximum)
			{
				throw std::out_of_range(
					"gateway.output_rate_hz cannot be represented by a steady nanosecond timer");
			}

			return std::chrono::nanoseconds{
				static_cast<std::chrono::nanoseconds::rep>(
					std::ceil(period_ns))};
		}

		builtin_interfaces::msg::Time ros_time_message(
			std::int64_t nanoseconds)
		{
			if (nanoseconds < 0)
			{
				throw std::logic_error(
					"Gateway cannot serialize a negative ROS time");
			}

			return static_cast<builtin_interfaces::msg::Time>(
				rclcpp::Time(nanoseconds, RCL_ROS_TIME));
		}

		void add_diagnostic_value(
			diagnostic_msgs::msg::DiagnosticStatus &status,
			std::string key,
			std::string value)
		{
			diagnostic_msgs::msg::KeyValue item;
			item.key = std::move(key);
			item.value = std::move(value);
			status.values.push_back(std::move(item));
		}

		diagnostic_msgs::msg::DiagnosticStatus make_diagnostic_status(
			std::string name,
			std::uint8_t level,
			std::string message)
		{
			diagnostic_msgs::msg::DiagnosticStatus status;
			status.name = std::move(name);
			status.level = level;
			status.message = std::move(message);
			status.hardware_id = "control_link/gateway";
			return status;
		}

		const char *data_state_name(DataState state) noexcept
		{
			switch (state)
			{
			case DataState::kStandby:
				return "STANDBY";
			case DataState::kActive:
				return "ACTIVE";
			case DataState::kDegraded:
				return "DEGRADED";
			case DataState::kSafeStop:
				return "SAFE_STOP";
			case DataState::kRecovering:
				return "RECOVERING";
			case DataState::kError:
				return "ERROR";
			}

			return "UNKNOWN";
		}

		const char *source_endpoint_state_name(
			SourceEndpointState state) noexcept
		{
			switch (state)
			{
			case SourceEndpointState::kMissing:
				return "MISSING";
			case SourceEndpointState::kAmbiguous:
				return "AMBIGUOUS";
			case SourceEndpointState::kUnexpectedDirection:
				return "UNEXPECTED_DIRECTION";
			case SourceEndpointState::kTypeMismatch:
				return "TYPE_MISMATCH";
			case SourceEndpointState::kQosMismatch:
				return "QOS_MISMATCH";
			case SourceEndpointState::kUsable:
				return "USABLE";
			}

			return "UNKNOWN";
		}

		std::uint8_t diagnostic_level_for_state(DataState state) noexcept
		{
			switch (state)
			{
			case DataState::kActive:
				return diagnostic_msgs::msg::DiagnosticStatus::OK;
			case DataState::kDegraded:
			case DataState::kRecovering:
			case DataState::kStandby:
				return diagnostic_msgs::msg::DiagnosticStatus::WARN;
			case DataState::kSafeStop:
			case DataState::kError:
				return diagnostic_msgs::msg::DiagnosticStatus::ERROR;
			}

			return diagnostic_msgs::msg::DiagnosticStatus::STALE;
		}

		double nanoseconds_to_milliseconds(std::int64_t nanoseconds) noexcept
		{
			constexpr double kNanosecondsPerMillisecond = 1'000'000.0;
			return nanoseconds <= 0 ?
				0.0 : static_cast<double>(nanoseconds) / kNanosecondsPerMillisecond;
		}

		control_link_interfaces::msg::GatewayState make_gateway_state(
			const DecisionResult &decision,
			std::int64_t now_ros_ns)
		{
			control_link_interfaces::msg::GatewayState message;
			message.observed_at = ros_time_message(now_ros_ns);
			message.state = static_cast<std::uint8_t>(decision.state);
			message.reason_code = static_cast<std::uint16_t>(decision.reason);
			message.recovery_valid_count = decision.recovery_valid_count;
			message.transition_sequence = decision.transition_sequence;

			if (!decision.canonical_command.source_id.empty())
			{
				const auto &selected = decision.canonical_command;
				message.active_source_id = selected.source_id;
				message.active_source_sequence = selected.source_sequence;
				const auto source_stamp_ns = rclcpp::Time(selected.source_stamp).nanoseconds();
				message.active_command_age_ms = nanoseconds_to_milliseconds(
					now_ros_ns - source_stamp_ns);
			}

			return message;
		}

		DecisionSourceEndpointState decision_endpoint_state(
			SourceEndpointState state)
		{
			switch (state)
			{
			case SourceEndpointState::kMissing:
				return DecisionSourceEndpointState::kMissing;
			case SourceEndpointState::kAmbiguous:
				return DecisionSourceEndpointState::kAmbiguous;
			case SourceEndpointState::kUnexpectedDirection:
				return DecisionSourceEndpointState::kUnexpectedDirection;
			case SourceEndpointState::kTypeMismatch:
				return DecisionSourceEndpointState::kTypeMismatch;
			case SourceEndpointState::kQosMismatch:
				return DecisionSourceEndpointState::kQosMismatch;
			case SourceEndpointState::kUsable:
				return DecisionSourceEndpointState::kUsable;
			}
			throw std::logic_error("unsupported SourceEndpointState reached Decision Trace");
		}

		DecisionTraceHeader make_decision_trace_header(
			const control_link_contract::ContractBundle &bundle,
			const std::string &rmw_implementation)
		{
			const char *ros_distro = std::getenv("ROS_DISTRO");
			return DecisionTraceHeader{
				kDecisionTraceSchemaVersion,
				CONTROL_LINK_GIT_COMMIT,
				CONTROL_LINK_GIT_DIRTY != 0,
				CONTROL_LINK_BUILD_TYPE,
				bundle.identity.decision_config.profile_id,
				bundle.identity.contract.contract_id,
				bundle.identity.contract.contract_version,
				bundle.identity.contract.contract_hash,
				bundle.identity.decision_config.decision_config_hash,
				rmw_implementation,
				ros_distro == nullptr || *ros_distro == '\0' ? "unknown" : ros_distro,
				"relative_ns_zero"};
		}

		std::map<std::string, SourceEndpointStabilityTracker>
		make_source_endpoint_trackers(
			const std::map<std::string, SourceRuntimeSlot> &slots)
		{
			std::map<std::string, SourceEndpointStabilityTracker> trackers;
			for (const auto &[source_id, slot] : slots)
			{
				(void)slot;
				trackers.try_emplace(source_id);
			}
			return trackers;
		}

		std::map<std::string, CriticalEndpointStabilityTracker>
		make_critical_endpoint_trackers(
			const control_link_contract::GatewayContract &contract)
		{
			std::map<std::string, CriticalEndpointStabilityTracker> trackers;
			for (const auto &endpoint : contract.critical_endpoints)
			{
				if (!trackers.try_emplace(endpoint.id).second)
				{
					throw std::logic_error(
						"duplicate critical endpoint reached Gateway runtime: " +
						endpoint.id);
				}
			}
			return trackers;
		}

		bool profile_uses_sim_time(
			const control_link_contract::ProfileConfig &profile)
		{
			return std::visit(
				[](const auto &typed_profile)
				{
					return typed_profile.common.use_sim_time;
				},
				profile);
		}

	} // namespace

	ControlGatewayNode::ControlGatewayNode(const rclcpp::NodeOptions &options)
		: rclcpp_lifecycle::LifecycleNode("gateway", options)
	{
		declare_parameter<std::string>(kProfilePathParameter, "");
		declare_parameter<std::string>(kConfigRootParameter, "");
		declare_parameter<std::string>(kDecisionTracePathParameter, "");
		declare_parameter<std::int64_t>(kDecisionTraceQueueCapacityParameter, 4096);
		// Executor add_node() 前固定调度拓扑，Lifecycle 只增删组内的 endpoint 和 timer
		data_plane_group_ = create_callback_group(
			rclcpp::CallbackGroupType::MutuallyExclusive);
		health_group_ = create_callback_group(
			rclcpp::CallbackGroupType::MutuallyExclusive);
	}

	std::int64_t ControlGatewayNode::decision_steady_offset_locked(
		std::chrono::steady_clock::time_point now) const
	{
		if (!decision_steady_origin_.has_value() ||
			now < decision_steady_origin_.value())
		{
			throw std::logic_error("Decision Trace steady origin is unavailable or in the future");
		}
		return std::chrono::duration_cast<std::chrono::nanoseconds>(
			now - decision_steady_origin_.value()).count();
	}

	std::optional<DecisionResult> ControlGatewayNode::submit_decision_event_locked(
		DecisionEventPayload payload)
	{
		if (!decision_engine_)
		{
			throw std::logic_error("Gateway cannot submit an event without DecisionEngine");
		}
		DecisionEvent event{
			decision_engine_->next_event_sequence(),
			std::move(payload)};
		std::optional<DecisionResult> result;
		try
		{
			result = decision_engine_->apply_event(event);
		}
		catch (...)
		{
			if (decision_trace_recorder_)
			{
				decision_trace_recorder_->invalidate(
					"DecisionEngine rejected a live event before it could be recorded");
				trace_recording_failed_ = true;
			}
			throw;
		}

		if (decision_trace_recorder_)
		{
			try
			{
				if (!decision_trace_recorder_->try_enqueue(
						DecisionTraceFrame{std::move(event), result}))
				{
					trace_recording_failed_ = true;
				}
			}
			catch (...)
			{
				// frame 复制属于旁路观测，失败只能使 trace INVALID，不能打断数据面
				decision_trace_recorder_->invalidate(
					"Decision Trace frame construction failed");
				trace_recording_failed_ = true;
			}
		}
		return result;
	}

	void ControlGatewayNode::submit_health_snapshot_locked(
		std::chrono::steady_clock::time_point observed_at)
	{
		if (!decision_engine_)
		{
			throw std::logic_error("Gateway cannot submit health without DecisionEngine");
		}
		health_snapshot_.output_tick_healthy =
			decision_engine_->health_snapshot().output_tick_healthy;
		std::vector<DecisionSourceEndpoint> endpoints;
		endpoints.reserve(source_endpoint_trackers_.size());
		for (const auto &[source_id, tracker] : source_endpoint_trackers_)
		{
			DecisionSourceEndpoint endpoint{
				source_id,
				DecisionSourceEndpointState::kMissing,
				std::nullopt};
			if (tracker.stable_assessment.has_value())
			{
				const auto &stable = tracker.stable_assessment.value();
				endpoint.state = decision_endpoint_state(stable.state);
				if (stable.usable())
				{
					endpoint.publisher_generation = stable.publisher_generation;
				}
			}
			endpoints.push_back(std::move(endpoint));
		}
		(void)submit_decision_event_locked(
			DecisionHealthSnapshotEvent{
				decision_engine_->next_health_revision(),
				decision_steady_offset_locked(observed_at),
				health_snapshot_,
				std::move(endpoints)});
	}

	void ControlGatewayNode::update_ros_clock_health_locked(
		std::chrono::steady_clock::time_point now_steady,
		std::int64_t now_ros_ns)
	{
		if (!contract_bundle_ || !contract_bundle_->profile)
		{
			throw std::logic_error(
				"ROS clock monitor reached an incomplete Gateway configuration");
		}

		const auto stall_timeout = checked_milliseconds(
			contract_bundle_->gateway_contract->gateway.ros_clock_stall_timeout_ms,
			"gateway.ros_clock_stall_timeout_ms");
		const bool using_sim_time = profile_uses_sim_time(*contract_bundle_->profile);

		bool sampled_backward_jump = false;
		if (!last_ros_time_ns_.has_value())
		{
			last_ros_time_ns_ = now_ros_ns;
			last_ros_time_progress_at_ = now_steady;
			ros_clock_backward_jump_ = false;
		}
		else
		{
			const auto previous_ros_ns = last_ros_time_ns_.value();
			if (now_ros_ns < previous_ros_ns)
			{
				sampled_backward_jump = true;
				ros_clock_backward_jump_ = true;
				last_ros_time_progress_at_ = now_steady;
			}
			else if (now_ros_ns > previous_ros_ns)
			{
				// 一次 backward jump 后必须先看到新的正向推进，才允许进入恢复路径
				ros_clock_backward_jump_ = false;
				last_ros_time_progress_at_ = now_steady;
			}
			last_ros_time_ns_ = now_ros_ns;
		}

		const auto callback_backward_jumps =
			pending_ros_clock_backward_jumps_.exchange(
				0U,
				std::memory_order_acq_rel);
		if (sampled_backward_jump || callback_backward_jumps > 0U)
		{
			// callback 与采样可能描述同一次跳变，优先采用 callback 的真实事件数量
			const auto observed_jump_count = callback_backward_jumps > 0U ?
				callback_backward_jumps : 1U;
			const auto maximum_count =
				std::numeric_limits<std::uint64_t>::max();
			if (observed_jump_count > maximum_count - ros_clock_backward_jump_count_)
			{
				ros_clock_backward_jump_count_ = maximum_count;
			}
			else
			{
				ros_clock_backward_jump_count_ += observed_jump_count;
			}
			ros_clock_backward_jump_ = true;
			last_ros_time_progress_at_ = now_steady;
		}

		bool healthy = now_ros_ns >= 0;
		if (using_sim_time && now_ros_ns <= 0)
		{
			healthy = false;
		}
		if (!last_ros_time_progress_at_.has_value() ||
			now_steady < last_ros_time_progress_at_.value() ||
			now_steady - last_ros_time_progress_at_.value() > stall_timeout)
		{
			healthy = false;
		}
		if (ros_clock_backward_jump_)
		{
			healthy = false;
		}

		health_snapshot_.ros_clock_healthy = healthy;
	}

	void ControlGatewayNode::poll_ros_clock() noexcept
	{
		try
		{
			std::scoped_lock lock(runtime_mutex_);
			if (!health_plane_configured_ || !contract_bundle_)
			{
				return;
			}

			const auto now_steady = std::chrono::steady_clock::now();
			update_ros_clock_health_locked(
				now_steady,
				get_clock()->now().nanoseconds());
			submit_health_snapshot_locked(now_steady);
		}
		catch (const std::exception &exception)
		{
			{
				std::scoped_lock lock(runtime_mutex_);
				health_snapshot_.ros_clock_healthy = false;
				health_snapshot_.internal_invariants_healthy = false;
			}
			RCLCPP_ERROR(
				get_logger(),
				"Gateway ROS clock monitor failed: %s",
				exception.what());
		}
		catch (...)
		{
			{
				std::scoped_lock lock(runtime_mutex_);
				health_snapshot_.ros_clock_healthy = false;
				health_snapshot_.internal_invariants_healthy = false;
			}
			RCLCPP_ERROR(
				get_logger(),
				"Gateway ROS clock monitor failed with an unknown exception");
		}
	}

	void ControlGatewayNode::publish_diagnostics() noexcept
	{
		try
		{
			diagnostic_msgs::msg::DiagnosticArray message;
			decltype(diagnostics_publisher_) publisher;

			{
				std::scoped_lock lock(runtime_mutex_);
				if (!health_plane_configured_ || !contract_bundle_ ||
					!decision_engine_ || !diagnostics_publisher_)
				{
					return;
				}

				// 多个 callback group 会竞争此锁，DecisionEvent 时间必须按实际提交顺序采样
				const auto now_steady = std::chrono::steady_clock::now();
				const auto now_ros_ns = get_clock()->now().nanoseconds();
				update_ros_clock_health_locked(now_steady, now_ros_ns);
				submit_health_snapshot_locked(now_steady);
				if (now_ros_ns >= 0)
				{
					message.header.stamp = ros_time_message(now_ros_ns);
				}

				diagnostic_msgs::msg::DiagnosticStatus state_status;
				if (last_decision_.has_value())
				{
					const auto &decision = last_decision_.value();
					state_status = make_diagnostic_status(
						"gateway/state",
						diagnostic_level_for_state(decision.state),
						data_state_name(decision.state));
					add_diagnostic_value(
						state_status,
						"data_state",
						data_state_name(decision.state));
					add_diagnostic_value(
						state_status,
						"state_code",
						std::to_string(static_cast<unsigned int>(decision.state)));
					add_diagnostic_value(
						state_status,
						"reason_code",
						std::to_string(static_cast<unsigned int>(decision.reason)));
					add_diagnostic_value(
						state_status,
						"transition_sequence",
						std::to_string(decision.transition_sequence));
					add_diagnostic_value(
						state_status,
						"recovery_valid_count",
						std::to_string(decision.recovery_valid_count));
					if (!decision.canonical_command.source_id.empty())
					{
						add_diagnostic_value(
							state_status,
							"active_source_id",
							decision.canonical_command.source_id);
						add_diagnostic_value(
							state_status,
							"active_source_sequence",
							std::to_string(
								decision.canonical_command.source_sequence));
					}
				}
				else
				{
					state_status = make_diagnostic_status(
						"gateway/state",
						diagnostic_msgs::msg::DiagnosticStatus::WARN,
						"data plane has not produced its first decision");
					add_diagnostic_value(
						state_status,
						"data_state",
						"NOT_STARTED");
				}
				add_diagnostic_value(
					state_status,
					"lifecycle_state",
					get_current_state().label());
				message.status.push_back(std::move(state_status));

				const auto &identity = contract_bundle_->identity;
				diagnostic_msgs::msg::DiagnosticStatus config_status =
					make_diagnostic_status(
						"gateway/config",
						diagnostic_msgs::msg::DiagnosticStatus::OK,
						"validated immutable configuration");
				add_diagnostic_value(
					config_status,
					"contract_id",
					identity.contract.contract_id);
				add_diagnostic_value(
					config_status,
					"contract_version",
					std::to_string(identity.contract.contract_version));
				add_diagnostic_value(
					config_status,
					"contract_hash",
					identity.contract.contract_hash);
				add_diagnostic_value(
					config_status,
					"profile_id",
					identity.decision_config.profile_id);
				add_diagnostic_value(
					config_status,
					"decision_config_hash",
					identity.decision_config.decision_config_hash);
				add_diagnostic_value(
					config_status,
					"fastdds_profile_hash",
					contract_bundle_->fastdds_profile_hash);
				message.status.push_back(std::move(config_status));

				const auto &gateway = contract_bundle_->gateway_contract->gateway;
				diagnostic_msgs::msg::DiagnosticStatus clock_status =
					make_diagnostic_status(
						"gateway/clock",
						health_snapshot_.ros_clock_healthy ?
							diagnostic_msgs::msg::DiagnosticStatus::OK :
							diagnostic_msgs::msg::DiagnosticStatus::ERROR,
						health_snapshot_.ros_clock_healthy ?
							"ROS clock healthy" : "ROS clock invalid");
				add_diagnostic_value(
					clock_status,
					"healthy",
					health_snapshot_.ros_clock_healthy ? "true" : "false");
				add_diagnostic_value(
					clock_status,
					"use_sim_time",
					profile_uses_sim_time(*contract_bundle_->profile) ?
						"true" : "false");
				add_diagnostic_value(
					clock_status,
					"now_ros_ns",
					std::to_string(now_ros_ns));
				add_diagnostic_value(
					clock_status,
					"last_ros_time_ns",
					last_ros_time_ns_.has_value() ?
						std::to_string(last_ros_time_ns_.value()) : "unset");
				add_diagnostic_value(
					clock_status,
					"backward_jump",
					ros_clock_backward_jump_ ? "true" : "false");
				add_diagnostic_value(
					clock_status,
					"backward_jump_count",
					std::to_string(ros_clock_backward_jump_count_));
				add_diagnostic_value(
					clock_status,
					"stall_timeout_ms",
					std::to_string(gateway.ros_clock_stall_timeout_ms));
				if (last_ros_time_progress_at_.has_value() &&
					now_steady >= last_ros_time_progress_at_.value())
				{
					add_diagnostic_value(
						clock_status,
						"age_since_progress_ms",
						std::to_string(
							std::chrono::duration_cast<std::chrono::milliseconds>(
								now_steady - last_ros_time_progress_at_.value())
								.count()));
				}
				message.status.push_back(std::move(clock_status));

				const bool output_tick_healthy = health_snapshot_.output_tick_healthy;
				const auto output_level = !data_plane_enabled_ ?
					diagnostic_msgs::msg::DiagnosticStatus::WARN :
					output_tick_healthy ?
						diagnostic_msgs::msg::DiagnosticStatus::OK :
						diagnostic_msgs::msg::DiagnosticStatus::ERROR;
				diagnostic_msgs::msg::DiagnosticStatus output_status =
					make_diagnostic_status(
						"gateway/output_tick",
						output_level,
						!data_plane_enabled_ ?
							"output scheduler inactive" :
						output_tick_healthy ?
							"output scheduler healthy" :
							"output scheduler exceeded safety threshold");
				add_diagnostic_value(
					output_status,
					"healthy",
					output_tick_healthy ? "true" : "false");
					add_diagnostic_value(
						output_status,
						"consecutive_late_ticks",
						std::to_string(
							decision_engine_->consecutive_late_output_ticks()));
				add_diagnostic_value(
					output_status,
					"late_threshold_ms",
					std::to_string(gateway.output_tick_late_threshold_ms));
				add_diagnostic_value(
					output_status,
					"safe_stop_after_ticks",
					std::to_string(gateway.consecutive_late_ticks_to_safe_stop));
				if (last_output_tick_at_.has_value() &&
					now_steady >= last_output_tick_at_.value())
				{
					add_diagnostic_value(
						output_status,
						"age_since_tick_ms",
						std::to_string(
							std::chrono::duration_cast<std::chrono::milliseconds>(
								now_steady - last_output_tick_at_.value())
								.count()));
				}
				message.status.push_back(std::move(output_status));

				const auto endpoint_level =
					!health_snapshot_.critical_endpoints_healthy ?
						diagnostic_msgs::msg::DiagnosticStatus::ERROR :
					!health_snapshot_.critical_qos_compatible ?
						diagnostic_msgs::msg::DiagnosticStatus::WARN :
						diagnostic_msgs::msg::DiagnosticStatus::OK;
				diagnostic_msgs::msg::DiagnosticStatus endpoint_status =
					make_diagnostic_status(
						"gateway/endpoints",
						endpoint_level,
						endpoint_level == diagnostic_msgs::msg::DiagnosticStatus::OK ?
							"critical endpoints healthy" :
							"critical endpoint health gate is not ready");
				add_diagnostic_value(
					endpoint_status,
					"identity_healthy",
					health_snapshot_.critical_endpoints_healthy ? "true" : "false");
				add_diagnostic_value(
					endpoint_status,
					"qos_compatible",
					health_snapshot_.critical_qos_compatible ? "true" : "false");
				for (const auto &expected : contract_bundle_->gateway_contract->critical_endpoints)
				{
					const auto tracker_iterator =
						critical_endpoint_trackers_.find(expected.id);
					const bool stable = tracker_iterator !=
						critical_endpoint_trackers_.end() &&
						tracker_iterator->second.stable_assessment.has_value();
					const auto prefix = "critical." + expected.id + ".";
					add_diagnostic_value(
						endpoint_status,
						prefix + "stable",
						stable ? "true" : "false");
					if (!stable)
					{
						continue;
					}
					const auto &assessment =
						tracker_iterator->second.stable_assessment.value();
					add_diagnostic_value(
						endpoint_status,
						prefix + "role_count",
						std::to_string(assessment.identity.matching_role_count));
					add_diagnostic_value(
						endpoint_status,
						prefix + "discovered_count",
						std::to_string(assessment.identity.discovered_count));
					add_diagnostic_value(
						endpoint_status,
						prefix + "identity_healthy",
						assessment.identity.healthy() ? "true" : "false");
					add_diagnostic_value(
						endpoint_status,
						prefix + "qos_healthy",
						assessment.qos.healthy(assessment.exact_qos_required) ?
							"true" : "false");
					add_diagnostic_value(
						endpoint_status,
						prefix + "qos_observation_complete",
						assessment.qos.observation_complete ? "true" : "false");
				}
				message.status.push_back(std::move(endpoint_status));

				const auto relative_now = std::chrono::steady_clock::time_point{
					std::chrono::duration_cast<std::chrono::steady_clock::duration>(
						std::chrono::nanoseconds{
							decision_steady_offset_locked(now_steady)})};
				for (const auto &[source_id, slot] : decision_engine_->source_slots())
				{
					const auto &source =
						contract_bundle_->source_policy->sources.at(source_id);
					const auto tracker_iterator =
						source_endpoint_trackers_.find(source_id);
					const bool stable_generation_matches =
						tracker_iterator != source_endpoint_trackers_.end() &&
						tracker_iterator->second.stable_assessment.has_value() &&
						tracker_iterator->second.stable_assessment->usable() &&
						tracker_iterator->second.stable_assessment->publisher_generation.has_value() &&
						slot.confirmed_publisher_generation.has_value() &&
						tracker_iterator->second.stable_assessment->publisher_generation.value() ==
							slot.confirmed_publisher_generation.value();
					const bool command_valid =
						stable_generation_matches && slot.latest_valid_snapshot.has_value();
					bool lease_valid = false;
					double command_age_ms = 0.0;
					if (command_valid)
					{
						const auto assessment = assess_source_snapshot(
							source_id,
							slot.latest_valid_snapshot.value(),
							*contract_bundle_->gateway_contract,
							*contract_bundle_->source_policy,
							now_ros_ns,
							relative_now);
						lease_valid = assessment.lease_valid;
						command_age_ms = nanoseconds_to_milliseconds(
							assessment.command_age_ns);
					}

					const auto source_level = command_valid && lease_valid ?
						diagnostic_msgs::msg::DiagnosticStatus::OK :
						source.required_for_activation ?
							diagnostic_msgs::msg::DiagnosticStatus::ERROR :
							diagnostic_msgs::msg::DiagnosticStatus::WARN;
					diagnostic_msgs::msg::DiagnosticStatus source_status =
						make_diagnostic_status(
							"gateway/sources/" + source_id,
							source_level,
							command_valid && lease_valid ?
								"source command healthy" :
								"source command is not qualified");
					add_diagnostic_value(
						source_status,
						"required_for_activation",
						source.required_for_activation ? "true" : "false");
					add_diagnostic_value(
						source_status,
						"command_valid",
						command_valid ? "true" : "false");
					add_diagnostic_value(
						source_status,
						"lease_valid",
						lease_valid ? "true" : "false");
					add_diagnostic_value(
						source_status,
						"command_age_ms",
						std::to_string(command_age_ms));
					add_diagnostic_value(
						source_status,
						"last_reject_reason",
						std::to_string(static_cast<unsigned int>(slot.last_reject_reason)));
					add_diagnostic_value(
						source_status,
						"accepted_count",
						std::to_string(slot.accepted_count));
					add_diagnostic_value(
						source_status,
						"rejected_count",
						std::to_string(slot.rejected_count));
					if (tracker_iterator != source_endpoint_trackers_.end() &&
						tracker_iterator->second.stable_assessment.has_value())
					{
						add_diagnostic_value(
							source_status,
							"endpoint_state",
							source_endpoint_state_name(
								tracker_iterator->second.stable_assessment->state));
					}
					message.status.push_back(std::move(source_status));
				}

				diagnostic_msgs::msg::DiagnosticStatus trace_status;
				if (!decision_trace_recorder_)
				{
					trace_status = make_diagnostic_status(
						"gateway/decision_trace",
						diagnostic_msgs::msg::DiagnosticStatus::OK,
						"decision trace disabled");
					add_diagnostic_value(trace_status, "enabled", "false");
				}
				else
				{
					const auto recorder_status = decision_trace_recorder_->status();
					trace_recording_failed_ = trace_recording_failed_ ||
						!recorder_status.trace_valid;
					trace_status = make_diagnostic_status(
						"gateway/decision_trace",
						trace_recording_failed_ ?
							diagnostic_msgs::msg::DiagnosticStatus::ERROR :
							diagnostic_msgs::msg::DiagnosticStatus::OK,
						trace_recording_failed_ ?
							"decision trace INVALID" :
							"decision trace recording");
					add_diagnostic_value(trace_status, "enabled", "true");
					add_diagnostic_value(
						trace_status,
						"accepting",
						recorder_status.accepting ? "true" : "false");
					add_diagnostic_value(
						trace_status,
						"trace_valid",
						recorder_status.trace_valid ? "true" : "false");
					add_diagnostic_value(
						trace_status,
						"trace_overflow",
						recorder_status.trace_overflow ? "true" : "false");
					add_diagnostic_value(
						trace_status,
						"writer_failed",
						recorder_status.writer_failed ? "true" : "false");
					add_diagnostic_value(
						trace_status,
						"accepted_event_count",
						std::to_string(recorder_status.accepted_event_count));
					add_diagnostic_value(
						trace_status,
						"accepted_result_count",
						std::to_string(recorder_status.accepted_result_count));
					if (!recorder_status.error_message.empty())
					{
						add_diagnostic_value(
							trace_status,
							"error",
							recorder_status.error_message);
					}
				}
				message.status.push_back(std::move(trace_status));

				publisher = diagnostics_publisher_;
			}

			{
				std::scoped_lock lock(runtime_mutex_, publisher_mutex_);
				if (!health_plane_configured_ || diagnostics_publisher_ != publisher)
				{
					return;
				}
				publisher->publish(message);
			}
		}
		catch (const std::exception &exception)
		{
			RCLCPP_ERROR(
				get_logger(),
				"Gateway diagnostics update failed: %s",
				exception.what());
		}
		catch (...)
		{
			RCLCPP_ERROR(
				get_logger(),
				"Gateway diagnostics update failed with an unknown exception");
		}
	}

	void ControlGatewayNode::request_lifecycle_error() noexcept
	{
		rclcpp::TimerBase::SharedPtr output_timer;
		{
			std::scoped_lock lock(runtime_mutex_);
			lifecycle_error_requested_ = true;
			data_plane_enabled_ = false;
			output_timer = output_timer_;
		}

		if (output_timer)
		{
			output_timer->cancel();
		}

		try
		{
			trigger_transition(
				lifecycle_msgs::msg::Transition::TRANSITION_DEACTIVATE);
		}
		catch (const std::exception &exception)
		{
			RCLCPP_ERROR(
				get_logger(),
				"Failed to request Lifecycle error processing: %s",
				exception.what());
		}
		catch (...)
		{
			RCLCPP_ERROR(
				get_logger(),
				"Failed to request Lifecycle error processing with an unknown exception");
		}
	}

	void ControlGatewayNode::run_output_tick() noexcept
	{
		ControlCommand canonical_message;
		control_link_interfaces::msg::GatewayState gateway_state_message;
		std::vector<control_link_interfaces::msg::SourceStatus>
			source_status_messages;
		decltype(canonical_output_publisher_) canonical_publisher;
		decltype(gateway_state_publisher_) gateway_state_publisher;
		decltype(source_status_publisher_) source_status_publisher;
		bool request_error = false;

		try
		{
			{
				std::scoped_lock lock(runtime_mutex_);
				if (!data_plane_enabled_)
				{
					return;
				}

				if (!contract_bundle_ || !decision_engine_ ||
					!canonical_output_publisher_ || !gateway_state_publisher_ ||
					!source_status_publisher_ ||
					!canonical_output_publisher_->is_activated() ||
					!gateway_state_publisher_->is_activated() ||
					!source_status_publisher_->is_activated())
				{
					throw std::logic_error(
						"output tick reached an incomplete active Gateway");
				}

				const auto now_steady = std::chrono::steady_clock::now();
				const auto now_ros_ns = get_clock()->now().nanoseconds();
				update_ros_clock_health_locked(now_steady, now_ros_ns);
				refresh_vehicle_state_health_locked(now_steady);
				submit_health_snapshot_locked(now_steady);
				const auto tick = decision_engine_->describe_output_tick(
					decision_steady_offset_locked(now_steady),
					now_ros_ns);
				auto produced = submit_decision_event_locked(tick);
				if (!produced.has_value())
				{
					throw std::logic_error("DecisionEngine output tick produced no DecisionResult");
				}
				const auto decision = std::move(produced.value());
				health_snapshot_.output_tick_healthy =
					decision_engine_->health_snapshot().output_tick_healthy;
				last_output_tick_at_ = now_steady;
				last_decision_ = decision;

				canonical_message = decision.canonical_command;
				gateway_state_message = make_gateway_state(decision, now_ros_ns);
				source_status_messages.reserve(decision.sources.size());
				for (const auto &source_result : decision.sources)
				{
					const auto &source =
						contract_bundle_->source_policy->sources.at(source_result.source_id);

					control_link_interfaces::msg::SourceStatus status;
					status.observed_at = ros_time_message(now_ros_ns);
					status.source_id = source_result.source_id;
					status.priority = source.priority;
					status.enabled = true;
					status.last_reject_reason =
						static_cast<std::uint16_t>(source_result.last_reject_reason);
					status.last_source_sequence =
						source_result.last_accepted_sequence.value_or(0U);
					status.accepted_count = source_result.accepted_count;
					status.rejected_count = source_result.rejected_count;
					status.command_valid = source_result.command_valid;
					status.lease_valid = source_result.lease_valid;
					status.command_age_ms = nanoseconds_to_milliseconds(
						source_result.command_age_ns);

					source_status_messages.push_back(std::move(status));
				}

				canonical_publisher = canonical_output_publisher_;
				gateway_state_publisher = gateway_state_publisher_;
				source_status_publisher = source_status_publisher_;
				request_error = decision.lifecycle_error_requested;
			}

			{
				std::scoped_lock lock(runtime_mutex_, publisher_mutex_);
				// Lifecycle 可能在决策组装后开始 deactivate/cleanup，发布前必须重新确认配置代次
				if (!data_plane_enabled_ ||
					canonical_output_publisher_ != canonical_publisher ||
					gateway_state_publisher_ != gateway_state_publisher ||
					source_status_publisher_ != source_status_publisher ||
					!canonical_publisher->is_activated() ||
					!gateway_state_publisher->is_activated() ||
					!source_status_publisher->is_activated())
				{
					return;
				}

				canonical_publisher->publish(canonical_message);
				gateway_state_publisher->publish(gateway_state_message);
				for (const auto &status : source_status_messages)
				{
					source_status_publisher->publish(status);
				}
			}

			if (request_error)
			{
				request_lifecycle_error();
			}
		}
		catch (const std::exception &exception)
		{
			{
				std::scoped_lock lock(runtime_mutex_);
				health_snapshot_.internal_invariants_healthy = false;
				try
				{
					if (decision_engine_ && decision_engine_->configured())
					{
						submit_health_snapshot_locked(std::chrono::steady_clock::now());
					}
				}
				catch (...)
				{
				}
			}
			RCLCPP_ERROR(
				get_logger(),
				"Gateway output tick failed: %s",
				exception.what());
			request_lifecycle_error();
		}
		catch (...)
		{
			{
				std::scoped_lock lock(runtime_mutex_);
				health_snapshot_.internal_invariants_healthy = false;
				try
				{
					if (decision_engine_ && decision_engine_->configured())
					{
						submit_health_snapshot_locked(std::chrono::steady_clock::now());
					}
				}
				catch (...)
				{
				}
			}
			RCLCPP_ERROR(
				get_logger(),
				"Gateway output tick failed with an unknown exception");
			request_lifecycle_error();
		}
	}

	void ControlGatewayNode::poll_graph() noexcept
	{
		try
		{
			control_link_contract::ContractBundlePtr bundle;
			std::optional<rclcpp::QoS> local_input_qos;
			std::optional<rclcpp::QoS> local_output_qos;
			std::optional<rclcpp::QoS> local_vehicle_state_qos;
			std::string rmw_implementation;
			std::vector<std::string> source_ids;

			{
				std::scoped_lock lock(runtime_mutex_);
				if (!health_plane_configured_ || !contract_bundle_ ||
					!source_input_qos_.has_value() ||
					!canonical_output_qos_.has_value() ||
					!vehicle_state_qos_.has_value())
					{
						return;
					}
					update_ros_clock_health_locked(
						std::chrono::steady_clock::now(),
						get_clock()->now().nanoseconds());

					bundle = contract_bundle_;
				local_input_qos = source_input_qos_;
				local_output_qos = canonical_output_qos_;
				local_vehicle_state_qos = vehicle_state_qos_;
				rmw_implementation = rmw_implementation_;
				source_ids.reserve(source_endpoint_trackers_.size());
				for (const auto &[source_id, tracker] : source_endpoint_trackers_)
				{
					(void)tracker;
					source_ids.push_back(source_id);
				}
			}

			const auto &contract = *bundle->gateway_contract;
			const auto &expected_remote_qos =
				contract.qos_profiles.at(contract.input.qos_profile);
			std::map<std::string, SourceEndpointAssessment> source_observations;

			// Graph 查询不持有 runtime_mutex_，避免阻塞未来的输入 callback 和 output tick
			for (const auto &source_id : source_ids)
			{
				const auto &source = bundle->source_policy->sources.at(source_id);
				auto publishers = get_publishers_info_by_topic(source.topic);
				source_observations.emplace(
					source_id,
					assess_source_publishers(
						publishers,
						source.type,
						rmw_implementation,
						local_input_qos.value(),
						expected_remote_qos));
			}

			std::map<std::string, CriticalEndpointAssessment>
				critical_observations;
			for (const auto &expected : contract.critical_endpoints)
			{
				std::vector<rclcpp::TopicEndpointInfo> endpoints;
				switch (expected.remote_direction)
				{
				case control_link_contract::RemoteDirection::kPublisher:
					endpoints = get_publishers_info_by_topic(expected.topic);
					break;

				case control_link_contract::RemoteDirection::kSubscription:
					endpoints = get_subscriptions_info_by_topic(expected.topic);
					break;

				default:
					throw std::logic_error(
						"unsupported critical endpoint direction reached Gateway runtime");
				}

				const rclcpp::QoS *local_qos = nullptr;
				const control_link_contract::QosProfile *expected_profile = nullptr;
				if (expected.id == kCanonicalConsumerId)
				{
					local_qos = &local_output_qos.value();
					expected_profile = &contract.qos_profiles.at(
						contract.output.qos_profile);
				}
				else if (expected.id == kVehicleStateProducerId)
				{
					local_qos = &local_vehicle_state_qos.value();
					const auto &vehicle_state =
						contract.state_topics.at("vehicle_state");
					if (!vehicle_state.qos_profile.has_value())
					{
						throw std::logic_error(
							"VehicleState critical endpoint has no QoS profile");
					}
					expected_profile = &contract.qos_profiles.at(
						vehicle_state.qos_profile.value());
				}
				else
				{
					throw std::logic_error(
						"unknown v1 critical endpoint reached Gateway runtime: " +
						expected.id);
				}

				auto identity = assess_critical_endpoint_identity(
					endpoints,
					expected);
				auto qos = assess_critical_endpoint_qos(
					identity,
					expected,
					*local_qos,
					*expected_profile);
				critical_observations.emplace(
					expected.id,
					CriticalEndpointAssessment{
						std::move(identity),
						std::move(qos),
						expected.exact_qos_required});
			}

			const auto stable_window = checked_milliseconds(
				contract.gateway.graph_stable_window_ms,
				"gateway.graph_stable_window_ms");

			std::vector<SourceGraphTransitionLog> transition_logs;
			transition_logs.reserve(source_observations.size());
			std::vector<CriticalGraphTransitionLog> critical_transition_logs;
			critical_transition_logs.reserve(critical_observations.size());
			{
				std::scoped_lock lock(runtime_mutex_);
				// cleanup 或重新 configure 已替换快照时，丢弃旧查询结果
				if (!health_plane_configured_ || contract_bundle_ != bundle)
				{
					return;
				}
				// Graph 查询保持在锁外，但观测提交时间必须在锁内采样，避免排队后时间倒退
				const auto observed_at = std::chrono::steady_clock::now();

				for (const auto &[source_id, observation] : source_observations)
				{
					auto &tracker = source_endpoint_trackers_.at(source_id);
					const auto event = update_source_endpoint_stability(
						tracker,
						observation,
						observed_at,
						stable_window);

					if (event != SourceEndpointStabilityEvent::kInitialized &&
						event != SourceEndpointStabilityEvent::kChanged)
					{
						continue;
					}

					const auto &stable = tracker.stable_assessment.value();
					transition_logs.push_back(SourceGraphTransitionLog{
						source_id,
						stable.state,
						stable.publisher_count});
				}

				for (const auto &[endpoint_id, observation] : critical_observations)
				{
					auto &tracker = critical_endpoint_trackers_.at(endpoint_id);
					const auto event = update_critical_endpoint_stability(
						tracker,
						observation,
						observed_at,
						stable_window);
					if (event != CriticalEndpointStabilityEvent::kInitialized &&
						event != CriticalEndpointStabilityEvent::kChanged)
					{
						continue;
					}

					const auto &stable = tracker.stable_assessment.value();
					critical_transition_logs.push_back(
						CriticalGraphTransitionLog{
							endpoint_id,
							stable.identity.healthy(),
							stable.qos.healthy(stable.exact_qos_required),
							stable.qos.observation_complete});
				}

				bool identity_healthy = true;
				bool qos_healthy = true;
				for (const auto &expected : contract.critical_endpoints)
				{
					const auto &tracker =
						critical_endpoint_trackers_.at(expected.id);
					if (!tracker.stable_assessment.has_value())
					{
						identity_healthy = false;
						qos_healthy = false;
						continue;
					}

					const auto &stable = tracker.stable_assessment.value();
					identity_healthy =
						identity_healthy && stable.identity.healthy();
					qos_healthy = qos_healthy &&
						stable.qos.healthy(stable.exact_qos_required);
				}
				health_snapshot_.critical_endpoints_healthy = identity_healthy;
				health_snapshot_.critical_qos_compatible = qos_healthy;

				const auto &vehicle_tracker =
					critical_endpoint_trackers_.at(kVehicleStateProducerId);
				if (vehicle_tracker.stable_assessment.has_value())
				{
					const auto &stable = vehicle_tracker.stable_assessment.value();
					if (stable.identity.healthy() &&
						stable.identity.matching_role_endpoints.size() == 1U)
					{
						auto generation = publisher_generation_from_endpoint_info(
							stable.identity.matching_role_endpoints.front(),
							rmw_implementation_);
						if (!vehicle_state_publisher_generation_.has_value() ||
							!(vehicle_state_publisher_generation_.value() == generation))
						{
							// 新 publisher generation 不能继承旧执行反馈与接收时间
							vehicle_state_publisher_generation_ = std::move(generation);
							vehicle_state_runtime_ = VehicleStateRuntime{};
						}
					}
				}

				refresh_vehicle_state_health_locked(observed_at);
				submit_health_snapshot_locked(observed_at);
			}

			// 日志可能触发格式化和 I/O，不占用 data plane 共用的 runtime_mutex_
			for (const auto &transition : transition_logs)
			{
				if (transition.state == SourceEndpointState::kUsable)
				{
					RCLCPP_INFO(
						get_logger(),
						"Source endpoint stable and usable: source=%s",
						transition.source_id.c_str());
					continue;
				}

				RCLCPP_WARN(
					get_logger(),
					"Source endpoint stable but unusable: source=%s, state=%u, publishers=%zu",
					transition.source_id.c_str(),
					static_cast<unsigned int>(transition.state),
					transition.publisher_count);
			}

			for (const auto &transition : critical_transition_logs)
			{
				const bool healthy =
					transition.identity_healthy && transition.qos_healthy;
				if (healthy && transition.qos_observation_complete)
				{
					RCLCPP_INFO(
						get_logger(),
						"Critical endpoint stable and healthy: id=%s",
						transition.endpoint_id.c_str());
					continue;
				}

				RCLCPP_WARN(
					get_logger(),
					"Critical endpoint stable: id=%s, state=%s, identity=%s, qos=%s, qos_observation=%s",
					transition.endpoint_id.c_str(),
					healthy ? "healthy" : "unhealthy",
					transition.identity_healthy ? "healthy" : "unhealthy",
					transition.qos_healthy ? "healthy" : "unhealthy",
					transition.qos_observation_complete ? "complete" : "partial");
			}
		}
		catch (const std::logic_error &exception)
		{
			{
				std::scoped_lock lock(runtime_mutex_);
				health_snapshot_.internal_invariants_healthy = false;
				try
				{
					if (decision_engine_ && decision_engine_->configured())
					{
						submit_health_snapshot_locked(std::chrono::steady_clock::now());
					}
				}
				catch (...)
				{
				}
			}
			RCLCPP_ERROR(
				get_logger(),
				"Graph monitor invariant failed: %s",
				exception.what());
		}
		catch (const std::exception &exception)
		{
			{
				std::scoped_lock lock(runtime_mutex_);
				health_snapshot_.critical_endpoints_healthy = false;
				health_snapshot_.critical_qos_compatible = false;
				try
				{
					if (decision_engine_ && decision_engine_->configured())
					{
						submit_health_snapshot_locked(std::chrono::steady_clock::now());
					}
				}
				catch (...)
				{
				}
			}
			RCLCPP_ERROR(
				get_logger(),
				"ROS Graph poll failed: %s",
				exception.what());
		}
		}

	void ControlGatewayNode::refresh_vehicle_state_health_locked(
		std::chrono::steady_clock::time_point now)
	{
		if (!contract_bundle_)
		{
			throw std::logic_error(
				"VehicleState health refresh reached an incomplete Gateway configuration");
		}

		const auto assessment = assess_vehicle_state_health(
			vehicle_state_runtime_,
			now,
			checked_milliseconds(
				contract_bundle_->gateway_contract->gateway
					.vehicle_state_topic_timeout_ms,
				"gateway.vehicle_state_topic_timeout_ms"));
		health_snapshot_.vehicle_state_valid = assessment.valid;
		health_snapshot_.vehicle_state_fresh = assessment.fresh;
		health_snapshot_.vehicle_reports_safe_stop =
			assessment.reports_safe_stop;
		health_snapshot_.vehicle_reports_fault = assessment.reports_fault;
	}

	void ControlGatewayNode::handle_vehicle_state(
		const VehicleState &state,
		const rclcpp::MessageInfo &message_info) noexcept
	{
		try
		{
			std::scoped_lock lock(runtime_mutex_);
			// VehicleState 是 activation gate 的输入，所以 INACTIVE 期间也必须接收
			if (!health_plane_configured_)
			{
				return;
			}

			if (!contract_bundle_ || !vehicle_state_validator_)
			{
				throw std::logic_error(
					"VehicleState callback reached an incomplete Gateway configuration");
			}

			const auto received_at = std::chrono::steady_clock::now();
			const auto actual_generation =
				publisher_generation_from_message_info(message_info);
			const auto &tracker =
				critical_endpoint_trackers_.at(kVehicleStateProducerId);

			// exact QoS mismatch 由独立健康字段映射 DEGRADED，不能污染发布者身份判断
			const bool stable_role_available =
				tracker.stable_assessment.has_value() &&
				tracker.stable_assessment->identity.healthy() &&
				tracker.stable_assessment->identity
					.matching_role_endpoints.size() == 1U;
			if (!stable_role_available)
			{
				commit_vehicle_state_validation_result(
					vehicle_state_runtime_,
					VehicleStateValidationResult{
						VehicleStateRejectReason::kPublisherGenerationUnstable,
						std::nullopt});
				refresh_vehicle_state_health_locked(received_at);
				submit_health_snapshot_locked(received_at);
				return;
			}

			const auto stable_generation =
				publisher_generation_from_endpoint_info(
					tracker.stable_assessment->identity
						.matching_role_endpoints.front(),
					rmw_implementation_);
			if (!(actual_generation == stable_generation))
			{
				commit_vehicle_state_validation_result(
					vehicle_state_runtime_,
					VehicleStateValidationResult{
						VehicleStateRejectReason::kPublisherGenerationUnstable,
						std::nullopt});
				refresh_vehicle_state_health_locked(received_at);
				submit_health_snapshot_locked(received_at);
				return;
			}

			// stable tracker 与已确认 generation 都由 poll_graph() 在同一临界区提交
			if (!vehicle_state_publisher_generation_.has_value() ||
				!(vehicle_state_publisher_generation_.value() == stable_generation))
			{
				throw std::logic_error(
					"stable VehicleState publisher generation does not match Gateway runtime");
			}

			const VehicleStateValidationContext context{
				actual_generation,
				get_clock()->now().nanoseconds(),
				health_snapshot_.ros_clock_healthy,
				received_at};
			const auto result = vehicle_state_validator_->validate(state, context);
			commit_vehicle_state_validation_result(vehicle_state_runtime_, result);
			refresh_vehicle_state_health_locked(received_at);
			submit_health_snapshot_locked(received_at);
		}
		catch (const std::exception &exception)
		{
			{
				std::scoped_lock lock(runtime_mutex_);
				health_snapshot_.internal_invariants_healthy = false;
				try
				{
					if (decision_engine_ && decision_engine_->configured())
					{
						submit_health_snapshot_locked(std::chrono::steady_clock::now());
					}
				}
				catch (...)
				{
				}
			}
			RCLCPP_ERROR(
				get_logger(),
				"VehicleState callback failed: %s",
				exception.what());
		}
	}

	void ControlGatewayNode::handle_source_command(
		const std::string &expected_source_id,
		const ControlCommand &command,
		const rclcpp::MessageInfo &message_info) noexcept
	{
		try
		{
			std::scoped_lock lock(runtime_mutex_);
			if (!data_plane_enabled_)
			{
				return;
			}

			if (!contract_bundle_ || !decision_engine_)
			{
				throw std::logic_error(
					"source callback reached an incomplete Gateway configuration");
			}
			if (source_endpoint_trackers_.count(expected_source_id) == 0U)
			{
				throw std::logic_error(
					"source callback has no matching endpoint tracker: " +
					expected_source_id);
			}
			const auto received_at = std::chrono::steady_clock::now();
			(void)submit_decision_event_locked(
				DecisionSourceSampleEvent{
					expected_source_id,
					publisher_generation_from_message_info(message_info),
					command,
					get_clock()->now().nanoseconds(),
					decision_steady_offset_locked(received_at)});
		}
		catch (const std::exception &exception)
		{
			{
				std::scoped_lock lock(runtime_mutex_);
				health_snapshot_.internal_invariants_healthy = false;
				try
				{
					if (decision_engine_ && decision_engine_->configured())
					{
						submit_health_snapshot_locked(std::chrono::steady_clock::now());
					}
				}
				catch (...)
				{
				}
			}
			RCLCPP_ERROR(
				get_logger(),
				"Source command callback failed: source=%s, error=%s",
				expected_source_id.c_str(),
				exception.what());
		}
	}

	ControlGatewayNode::CallbackReturn ControlGatewayNode::on_configure(
		const rclcpp_lifecycle::State &previous_state)
	{
		(void)previous_state;

		try
		{
			const std::filesystem::path profile_path{
				get_parameter(kProfilePathParameter).as_string()};
			const std::filesystem::path config_root{
				get_parameter(kConfigRootParameter).as_string()};

			// 先在局部变量中完成全部构造，避免失败时留下半配置成员
			auto bundle = control_link_contract::load_contract_bundle(
				profile_path,
				config_root);

			const std::string actual_fqn{
				get_node_base_interface()->get_fully_qualified_name()};
			const auto &expected_fqn = bundle->gateway_contract->gateway.node_fqn;
			if (actual_fqn != expected_fqn)
			{
				throw std::runtime_error(
					"gateway node FQN mismatch: expected=" + expected_fqn +
					", actual=" + actual_fqn);
			}
			// Participant 已在节点构造前后建立，只能检查启动环境，不能在 configure 时修复
			auto rmw_implementation =
				control_link_contract::validate_fastdds_process_environment(
					*bundle->profile,
					"ControlGatewayNode");

			auto qos_factory = std::make_unique<control_link_contract::QosFactory>(
				bundle->gateway_contract);
			auto decision_engine = std::make_unique<DecisionEngine>(bundle);
			auto vehicle_state_validator = std::make_unique<VehicleStateValidator>(
				bundle->gateway_contract);
			if (!data_plane_group_ || !health_group_)
			{
				throw std::logic_error(
					"ControlGatewayNode callback groups are not available");
			}
			const auto data_plane_group = data_plane_group_;
			const auto health_group = health_group_;
			auto source_endpoint_trackers =
				make_source_endpoint_trackers(decision_engine->source_slots());
			auto critical_endpoint_trackers =
				make_critical_endpoint_trackers(*bundle->gateway_contract);

			const auto &gateway_contract = *bundle->gateway_contract;
			const auto &gateway_state_topic =
				gateway_contract.state_topics.at("gateway_state");
			const auto &source_status_topic =
				gateway_contract.state_topics.at("source_status");
			const auto &vehicle_state_topic =
				gateway_contract.state_topics.at("vehicle_state");
			const auto &diagnostics_topic =
				gateway_contract.state_topics.at("diagnostics");

			if (!gateway_state_topic.qos_profile.has_value())
			{
				throw std::logic_error(
					"ControlGatewayNode requires state_topics.gateway_state.qos_profile");
			}
			if (!source_status_topic.qos_profile.has_value())
			{
				throw std::logic_error(
					"ControlGatewayNode requires state_topics.source_status.qos_profile");
			}
			if (!vehicle_state_topic.qos_profile.has_value())
			{
				throw std::logic_error(
					"ControlGatewayNode requires state_topics.vehicle_state.qos_profile");
			}
			if (diagnostics_topic.type != "diagnostic_msgs/msg/DiagnosticArray")
			{
				throw std::logic_error(
					"ControlGatewayNode requires diagnostic_msgs/msg/DiagnosticArray for diagnostics");
			}

			const auto canonical_output_qos = qos_factory->make(
				gateway_contract.output.qos_profile);
			const auto source_input_qos = qos_factory->make(
				gateway_contract.input.qos_profile);
			const auto gateway_state_qos = qos_factory->make(
				gateway_state_topic.qos_profile.value());
			const auto source_status_qos = qos_factory->make(
				source_status_topic.qos_profile.value());
			const auto vehicle_state_qos = qos_factory->make(
				vehicle_state_topic.qos_profile.value());

			auto canonical_output_publisher =
				create_publisher<control_link_interfaces::msg::ControlCommand>(
					gateway_contract.output.topic,
					canonical_output_qos);
			auto gateway_state_publisher =
				create_publisher<control_link_interfaces::msg::GatewayState>(
					gateway_state_topic.topic,
					gateway_state_qos);
			auto source_status_publisher =
				create_publisher<control_link_interfaces::msg::SourceStatus>(
					source_status_topic.topic,
					source_status_qos);
			auto diagnostics_publisher =
				create_publisher<diagnostic_msgs::msg::DiagnosticArray>(
					diagnostics_topic.topic,
					rclcpp::QoS(rclcpp::KeepLast(10U)));

			rclcpp::SubscriptionOptions source_subscription_options;
			source_subscription_options.callback_group = data_plane_group;
			std::map<std::string, SourceSubscription::SharedPtr>
				source_subscriptions;
			for (const auto &[source_id, slot] : decision_engine->source_slots())
			{
				(void)slot;
				const auto &source = bundle->source_policy->sources.at(source_id);
				auto subscription = create_subscription<ControlCommand>(
					source.topic,
					source_input_qos,
					[this, source_id](
						ControlCommand::ConstSharedPtr command,
						const rclcpp::MessageInfo &message_info)
					{
						handle_source_command(
							source_id,
							*command,
							message_info);
					},
					source_subscription_options);

				const bool inserted = source_subscriptions.emplace(
					source_id,
					std::move(subscription)).second;
				if (!inserted)
				{
					throw std::logic_error(
						"duplicate source subscription reached Gateway runtime: " +
						source_id);
				}
			}

			rclcpp::SubscriptionOptions vehicle_state_subscription_options;
			vehicle_state_subscription_options.callback_group = health_group;
			auto vehicle_state_subscription = create_subscription<VehicleState>(
				vehicle_state_topic.topic,
				vehicle_state_qos,
				[this](
					VehicleState::ConstSharedPtr state,
					const rclcpp::MessageInfo &message_info)
				{
					handle_vehicle_state(*state, message_info);
				},
				vehicle_state_subscription_options);

			auto graph_timer = create_wall_timer(
				checked_milliseconds(
					gateway_contract.gateway.graph_poll_ms,
					"gateway.graph_poll_ms"),
				[this]()
				{
					poll_graph();
				},
				health_group);
			auto clock_timer = create_wall_timer(
				checked_output_period(gateway_contract.gateway.output_rate_hz),
				[this]()
				{
					poll_ros_clock();
				},
				health_group);
			auto diagnostics_timer = create_wall_timer(
				checked_milliseconds(
					gateway_contract.gateway.graph_poll_ms,
					"gateway.graph_poll_ms"),
				[this]()
				{
					publish_diagnostics();
				},
				health_group);
			auto output_timer = create_wall_timer(
				checked_output_period(gateway_contract.gateway.output_rate_hz),
				[this]()
				{
					run_output_tick();
				},
				data_plane_group);
			// Lifecycle INACTIVE 期间 output timer 必须沉默，on_activate 再显式 reset
			output_timer->cancel();

			GatewayHealthSnapshot initial_health_snapshot;
			// 首次 ROS time 采样前统一保持 false，避免未确认的 clock 直接通过安全门
			initial_health_snapshot.ros_clock_healthy = false;
			// ROS clock 类型必须来自已通过严格解析的 Profile，不能在 launch 复制一份映射
			const auto use_sim_time_result = set_parameter(
				rclcpp::Parameter(
					"use_sim_time",
					profile_uses_sim_time(*bundle->profile)));
			if (!use_sim_time_result.successful)
			{
				throw std::runtime_error(
					"failed to apply Profile use_sim_time: " +
					use_sim_time_result.reason);
			}

			rcl_jump_threshold_t backward_jump_threshold{};
			backward_jump_threshold.on_clock_change = false;
			backward_jump_threshold.min_forward.nanoseconds = 0;
			backward_jump_threshold.min_backward.nanoseconds = -1;
			auto ros_clock_jump_handler = get_clock()->create_jump_callback(
				{},
				[this](const rcl_time_jump_t &jump) noexcept
				{
					if (jump.delta.nanoseconds >= 0)
					{
						return;
					}

					auto current = pending_ros_clock_backward_jumps_.load(
						std::memory_order_relaxed);
					const auto maximum =
						std::numeric_limits<std::uint64_t>::max();
					while (current != maximum &&
						!pending_ros_clock_backward_jumps_.compare_exchange_weak(
							current,
							current + 1U,
							std::memory_order_release,
							std::memory_order_relaxed))
					{
					}
				},
				backward_jump_threshold);

			const auto trace_path = std::filesystem::path{
				get_parameter(kDecisionTracePathParameter).as_string()};
			const auto trace_capacity_value =
				get_parameter(kDecisionTraceQueueCapacityParameter).as_int();
			if (trace_capacity_value <= 0)
			{
				throw std::invalid_argument(
					"decision_trace_queue_capacity must be positive");
			}
			if (!trace_path.empty() && !trace_path.is_absolute())
			{
				throw std::invalid_argument("decision_trace_path must be absolute");
			}
			if (!trace_path.empty())
			{
				const auto parent = trace_path.parent_path();
				if (parent.empty() || !std::filesystem::is_directory(parent))
				{
					throw std::invalid_argument(
						"decision_trace_path parent must be an existing directory");
				}
				if (std::filesystem::exists(trace_path) &&
					!std::filesystem::is_regular_file(trace_path))
				{
					throw std::invalid_argument(
						"decision_trace_path must target a regular file");
				}
			}
			const auto decision_steady_origin = std::chrono::steady_clock::now();
			std::unique_ptr<DecisionTraceRecorder> decision_trace_recorder;
			if (!trace_path.empty())
			{
				decision_trace_recorder = std::make_unique<DecisionTraceRecorder>(
					trace_path,
					make_decision_trace_header(*bundle, rmw_implementation),
					static_cast<std::size_t>(trace_capacity_value));
			}
			DecisionEvent configure_event{
				decision_engine->next_event_sequence(),
				DecisionLifecycleEvent{
					DecisionLifecycleTransition::kConfigure,
					DecisionLifecycleResult::kSuccess}};
			auto configure_result = decision_engine->apply_event(configure_event);
			if (configure_result.has_value())
			{
				throw std::logic_error("DecisionEngine configure event produced a result");
			}
			const bool trace_configure_accepted = !decision_trace_recorder ||
				decision_trace_recorder->try_enqueue(
					DecisionTraceFrame{std::move(configure_event), std::nullopt});

			std::scoped_lock lock(runtime_mutex_);
			contract_bundle_ = std::move(bundle);
			qos_factory_ = std::move(qos_factory);
			decision_engine_ = std::move(decision_engine);
			decision_trace_recorder_ = std::move(decision_trace_recorder);
			decision_steady_origin_ = decision_steady_origin;
			trace_recording_failed_ = !trace_configure_accepted;
			vehicle_state_validator_ = std::move(vehicle_state_validator);
			source_endpoint_trackers_ = std::move(source_endpoint_trackers);
			critical_endpoint_trackers_ =
				std::move(critical_endpoint_trackers);
			source_input_qos_ = source_input_qos;
			canonical_output_qos_ = canonical_output_qos;
			vehicle_state_qos_ = vehicle_state_qos;
			rmw_implementation_ = std::move(rmw_implementation);
			source_subscriptions_ = std::move(source_subscriptions);
			vehicle_state_subscription_ =
				std::move(vehicle_state_subscription);
			data_plane_enabled_ = false;
			health_snapshot_ = initial_health_snapshot;
			vehicle_state_runtime_ = VehicleStateRuntime{};
			vehicle_state_publisher_generation_.reset();
			graph_timer_ = std::move(graph_timer);
			clock_timer_ = std::move(clock_timer);
			diagnostics_timer_ = std::move(diagnostics_timer);
			output_timer_ = std::move(output_timer);
			last_ros_time_ns_.reset();
			last_ros_time_progress_at_.reset();
			pending_ros_clock_backward_jumps_.store(
				0U,
				std::memory_order_release);
			ros_clock_jump_handler_ = std::move(ros_clock_jump_handler);
			ros_clock_backward_jump_ = false;
			ros_clock_backward_jump_count_ = 0U;
			last_output_tick_at_.reset();
			lifecycle_error_requested_ = false;
			last_decision_.reset();
			canonical_output_publisher_ = std::move(canonical_output_publisher);
			gateway_state_publisher_ = std::move(gateway_state_publisher);
			source_status_publisher_ = std::move(source_status_publisher);
			diagnostics_publisher_ = std::move(diagnostics_publisher);
			// 必须最后开放，避免 callback 观察到只提交了一半的成员
			health_plane_configured_ = true;

			RCLCPP_INFO(
				get_logger(),
				"Gateway configured: profile=%s, contract=%s@%lu, contract_hash=%s, "
				"decision_config_hash=%s",
				profile_path.string().c_str(),
				contract_bundle_->identity.contract.contract_id.c_str(),
				static_cast<unsigned long>(
					contract_bundle_->identity.contract.contract_version),
				contract_bundle_->identity.contract.contract_hash.c_str(),
				contract_bundle_->identity.decision_config.decision_config_hash.c_str());
			return CallbackReturn::SUCCESS;
		}
		catch (const std::exception &exception)
		{
			RCLCPP_ERROR(
				get_logger(),
				"Gateway configuration failed: %s",
				exception.what());
			return CallbackReturn::FAILURE;
		}
	}

	ControlGatewayNode::CallbackReturn ControlGatewayNode::on_activate(
		const rclcpp_lifecycle::State &previous_state)
	{
		(void)previous_state;
		bool activation_committed = false;

		try
		{
			std::scoped_lock lock(runtime_mutex_, publisher_mutex_);
			if (!health_plane_configured_ || !contract_bundle_ ||
				!qos_factory_ || !decision_engine_ || !vehicle_state_validator_ ||
				!data_plane_group_ || !health_group_ || !graph_timer_ ||
				!clock_timer_ || !diagnostics_timer_ || !output_timer_ ||
				!ros_clock_jump_handler_ ||
				!vehicle_state_subscription_ ||
				!canonical_output_publisher_ || !gateway_state_publisher_ ||
				!source_status_publisher_ || !diagnostics_publisher_)
			{
				throw std::logic_error(
					"Gateway activation reached an incomplete configuration");
			}

			if (data_plane_enabled_)
			{
				throw std::logic_error(
					"Gateway data plane is already enabled");
			}

			if (canonical_output_publisher_->is_activated() ||
				gateway_state_publisher_->is_activated() ||
				source_status_publisher_->is_activated())
			{
				throw std::logic_error(
					"Gateway publisher activation state is inconsistent");
			}

			if (!health_snapshot_.internal_invariants_healthy)
			{
				throw std::logic_error(
					"Gateway internal invariant gate is not healthy");
			}

			// freshness 是随 steady time 衰减的值，不能直接信任上一次 callback 的缓存
			const auto activation_observed_at = std::chrono::steady_clock::now();
			refresh_vehicle_state_health_locked(activation_observed_at);
			submit_health_snapshot_locked(activation_observed_at);

			for (const auto &[source_id, slot] : decision_engine_->source_slots())
			{
				const auto &source =
					contract_bundle_->source_policy->sources.at(source_id);
				if (!source.required_for_activation)
				{
					continue;
				}

				const auto &tracker = source_endpoint_trackers_.at(source_id);
				if (!tracker.stable_assessment.has_value() ||
					!tracker.stable_assessment->usable())
				{
					throw std::runtime_error(
						"required source endpoint is not stable and usable: " +
						source_id);
				}

				const auto &stable_generation =
					tracker.stable_assessment->publisher_generation;
				if (!stable_generation.has_value() ||
					!slot.confirmed_publisher_generation.has_value() ||
					!(stable_generation.value() ==
						slot.confirmed_publisher_generation.value()))
				{
					throw std::logic_error(
						"required source generation is inconsistent: " +
						source_id);
				}
			}

			if (!health_snapshot_.critical_endpoints_healthy)
			{
				throw std::runtime_error(
					"critical endpoint Graph gate is not healthy");
			}

			if (!health_snapshot_.critical_qos_compatible)
			{
				throw std::runtime_error(
					"critical endpoint QoS gate is not compatible");
			}

			if (!health_snapshot_.vehicle_state_valid ||
				!health_snapshot_.vehicle_state_fresh)
			{
				throw std::runtime_error(
					"VehicleState gate is not valid and fresh");
			}

			if (!vehicle_state_publisher_generation_.has_value() ||
				!vehicle_state_runtime_.latest_valid_snapshot.has_value())
			{
				throw std::runtime_error(
					"VehicleState publisher generation is not ready");
			}

			const auto &vehicle_snapshot =
				vehicle_state_runtime_.latest_valid_snapshot.value();
			if (!(vehicle_snapshot.publisher_generation ==
					vehicle_state_publisher_generation_.value()))
			{
				throw std::logic_error(
					"VehicleState snapshot generation does not match the stable Graph generation");
			}

			// 只允许适配器因尚未收到 canonical command 而处于 SAFE_STOP 的启动状态
			const bool bootstrap_safe_stop =
				vehicle_snapshot.state == VehicleState::SAFE_STOP &&
				vehicle_snapshot.fault_code ==
				VehicleState::FAULT_ADAPTER_CANONICAL_TIMEOUT;
			if (!bootstrap_safe_stop)
			{
				throw std::runtime_error(
					"VehicleState is not the allowed adapter bootstrap SAFE_STOP");
			}

			bool canonical_output_activated = false;
			bool gateway_state_activated = false;
			bool source_status_activated = false;

			auto rollback_publishers = [this, &canonical_output_activated,
				&gateway_state_activated, &source_status_activated]() noexcept
			{
				auto rollback = [this](auto &publisher, bool &activated) noexcept
				{
					if (!activated || !publisher)
					{
						return;
					}

					try
					{
						publisher->on_deactivate();
					}
					catch (const std::exception &exception)
					{
						RCLCPP_ERROR(
							get_logger(),
							"Gateway publisher rollback failed: %s",
							exception.what());
					}
					catch (...)
					{
						RCLCPP_ERROR(
							get_logger(),
							"Gateway publisher rollback failed with an unknown exception");
					}
					activated = false;
				};

				rollback(source_status_publisher_, source_status_activated);
				rollback(gateway_state_publisher_, gateway_state_activated);
				rollback(canonical_output_publisher_, canonical_output_activated);
			};

			try
			{
				last_output_tick_at_.reset();
				lifecycle_error_requested_ = false;
				last_decision_.reset();
				canonical_output_publisher_->on_activate();
				canonical_output_activated = true;
				gateway_state_publisher_->on_activate();
				gateway_state_activated = true;
				source_status_publisher_->on_activate();
				source_status_activated = true;
				auto activation_result = submit_decision_event_locked(
					DecisionLifecycleEvent{
						DecisionLifecycleTransition::kActivate,
						DecisionLifecycleResult::kSuccess});
				if (activation_result.has_value() || !decision_engine_->active())
				{
					throw std::logic_error(
						"DecisionEngine activation event produced an invalid state");
				}
				activation_committed = true;
				health_snapshot_.output_tick_healthy =
					decision_engine_->health_snapshot().output_tick_healthy;
				data_plane_enabled_ = true;
				output_timer_->reset();
			}
			catch (...)
			{
				data_plane_enabled_ = false;
				output_timer_->cancel();
				if (activation_committed && decision_engine_->active())
				{
					try
					{
						(void)submit_decision_event_locked(
							DecisionLifecycleEvent{
								DecisionLifecycleTransition::kDeactivate,
								DecisionLifecycleResult::kSuccess});
						activation_committed = false;
					}
					catch (...)
					{
						health_snapshot_.internal_invariants_healthy = false;
					}
				}
				rollback_publishers();
				throw;
			}

			RCLCPP_INFO(get_logger(), "Gateway data plane activated");
			return CallbackReturn::SUCCESS;
		}
		catch (const std::logic_error &exception)
		{
			if (!activation_committed)
			{
				try
				{
					std::scoped_lock lock(runtime_mutex_);
					if (decision_engine_ && decision_engine_->configured() &&
						!decision_engine_->active())
					{
						(void)submit_decision_event_locked(
							DecisionLifecycleEvent{
								DecisionLifecycleTransition::kActivate,
								DecisionLifecycleResult::kError});
					}
				}
				catch (...)
				{
				}
			}
			RCLCPP_ERROR(
				get_logger(),
				"Gateway activation invariant failed: %s",
				exception.what());
			return CallbackReturn::ERROR;
		}
		catch (const std::exception &exception)
		{
			if (!activation_committed)
			{
				try
				{
					std::scoped_lock lock(runtime_mutex_);
					if (decision_engine_ && decision_engine_->configured() &&
						!decision_engine_->active())
					{
						(void)submit_decision_event_locked(
							DecisionLifecycleEvent{
								DecisionLifecycleTransition::kActivate,
								DecisionLifecycleResult::kFailure});
					}
				}
				catch (...)
				{
				}
			}
			RCLCPP_WARN(
				get_logger(),
				"Gateway activation rejected: %s",
				exception.what());
			return CallbackReturn::FAILURE;
		}
		catch (...)
		{
			if (!activation_committed)
			{
				try
				{
					std::scoped_lock lock(runtime_mutex_);
					if (decision_engine_ && decision_engine_->configured() &&
						!decision_engine_->active())
					{
						(void)submit_decision_event_locked(
							DecisionLifecycleEvent{
								DecisionLifecycleTransition::kActivate,
								DecisionLifecycleResult::kError});
					}
				}
				catch (...)
				{
				}
			}
			RCLCPP_ERROR(
				get_logger(),
				"Gateway activation failed with an unknown exception");
			return CallbackReturn::ERROR;
		}
	}

	ControlGatewayNode::CallbackReturn ControlGatewayNode::on_deactivate(
		const rclcpp_lifecycle::State &previous_state)
	{
		(void)previous_state;

		rclcpp::TimerBase::SharedPtr output_timer;
		bool lifecycle_error_requested = false;
		{
			std::scoped_lock lock(runtime_mutex_);
			// 普通 Subscription 不受 Lifecycle 自动停用，必须先关闭数据面写入门
			data_plane_enabled_ = false;
			last_output_tick_at_.reset();
			output_timer = output_timer_;
			lifecycle_error_requested = lifecycle_error_requested_;
		}
		if (output_timer)
		{
			output_timer->cancel();
		}

		// 先关闭入口让尚未进入发布边界的 tick 丢弃，再等待已进入边界的旧 tick 完成
		bool failed = false;
		{
			std::scoped_lock publisher_lock(publisher_mutex_);
			auto deactivate = [this, &failed](auto &publisher, const char *name)
			{
				if (!publisher || !publisher->is_activated())
				{
					return;
				}

				try
				{
					publisher->on_deactivate();
				}
				catch (const std::exception &exception)
				{
					failed = true;
					RCLCPP_ERROR(
						get_logger(),
						"Failed to deactivate publisher %s: %s",
						name,
						exception.what());
				}
				catch (...)
				{
					failed = true;
					RCLCPP_ERROR(
						get_logger(),
						"Failed to deactivate publisher %s with an unknown exception",
						name);
				}
			};

			deactivate(source_status_publisher_, "source_status");
			deactivate(gateway_state_publisher_, "gateway_state");
			deactivate(canonical_output_publisher_, "canonical_output");
		}

		try
		{
			std::scoped_lock lock(runtime_mutex_);
			if (!decision_engine_ || !decision_engine_->configured() ||
				!decision_engine_->active())
			{
				throw std::logic_error(
					"Gateway deactivation reached an invalid DecisionEngine state");
			}
			const auto result = failed || lifecycle_error_requested ?
				DecisionLifecycleResult::kError :
				DecisionLifecycleResult::kSuccess;
			(void)submit_decision_event_locked(
				DecisionLifecycleEvent{
					DecisionLifecycleTransition::kDeactivate,
					result});
		}
		catch (const std::exception &exception)
		{
			failed = true;
			RCLCPP_ERROR(
				get_logger(),
				"Failed to commit DecisionEngine deactivation: %s",
				exception.what());
		}

		if (failed || lifecycle_error_requested)
		{
			return CallbackReturn::ERROR;
		}

		RCLCPP_INFO(get_logger(), "Gateway data plane deactivated");
		return CallbackReturn::SUCCESS;
	}

	ControlGatewayNode::CallbackReturn ControlGatewayNode::on_error(
		const rclcpp_lifecycle::State &previous_state)
	{
		RCLCPP_ERROR(
			get_logger(),
			"Gateway entered Lifecycle error processing");
		try
		{
			std::scoped_lock lock(runtime_mutex_);
			if (decision_engine_ && decision_engine_->configured())
			{
				(void)submit_decision_event_locked(
					DecisionLifecycleEvent{
						DecisionLifecycleTransition::kError,
						DecisionLifecycleResult::kSuccess});
			}
		}
		catch (const std::exception &exception)
		{
			RCLCPP_ERROR(
				get_logger(),
				"Failed to commit DecisionEngine error transition: %s",
				exception.what());
		}
		return on_cleanup(previous_state);
	}

	ControlGatewayNode::CallbackReturn ControlGatewayNode::on_cleanup(
		const rclcpp_lifecycle::State &previous_state)
	{
		(void)previous_state;
		std::unique_ptr<DecisionTraceRecorder> trace_recorder;

		{
			std::scoped_lock lock(runtime_mutex_, publisher_mutex_);
			// cleanup 同时封闭 callback 入口和 Publisher 生命周期，避免旧配置资源被并发访问
			health_plane_configured_ = false;
			data_plane_enabled_ = false;
			if (output_timer_)
			{
				output_timer_->cancel();
			}
			output_timer_.reset();
			if (graph_timer_)
			{
				graph_timer_->cancel();
			}
			graph_timer_.reset();
			if (clock_timer_)
			{
				clock_timer_->cancel();
			}
			clock_timer_.reset();
			if (diagnostics_timer_)
			{
				diagnostics_timer_->cancel();
			}
			diagnostics_timer_.reset();
			vehicle_state_subscription_.reset();
			source_subscriptions_.clear();
			diagnostics_publisher_.reset();
			source_status_publisher_.reset();
			gateway_state_publisher_.reset();
			canonical_output_publisher_.reset();
			if (decision_engine_ && decision_engine_->configured())
			{
				(void)submit_decision_event_locked(
					DecisionLifecycleEvent{
						DecisionLifecycleTransition::kCleanup,
						DecisionLifecycleResult::kSuccess});
			}
			trace_recorder = std::move(decision_trace_recorder_);
			// CallbackGroup 属于节点，不属于单次配置，Executor spin 期间不能在这里销毁
			vehicle_state_publisher_generation_.reset();
			vehicle_state_runtime_ = VehicleStateRuntime{};
			last_ros_time_ns_.reset();
			last_ros_time_progress_at_.reset();
			ros_clock_jump_handler_.reset();
			pending_ros_clock_backward_jumps_.store(
				0U,
				std::memory_order_release);
			ros_clock_backward_jump_ = false;
			ros_clock_backward_jump_count_ = 0U;
			last_output_tick_at_.reset();
			lifecycle_error_requested_ = false;
			last_decision_.reset();
			vehicle_state_validator_.reset();
			critical_endpoint_trackers_.clear();
			source_endpoint_trackers_.clear();
			vehicle_state_qos_.reset();
			canonical_output_qos_.reset();
			source_input_qos_.reset();
			rmw_implementation_.clear();
			health_snapshot_ = GatewayHealthSnapshot{};
			decision_engine_.reset();
			decision_steady_origin_.reset();
			trace_recording_failed_ = false;
			qos_factory_.reset();
			contract_bundle_.reset();
		}

		if (trace_recorder)
		{
			try
			{
				const auto status = trace_recorder->stop();
				if (!status.trace_valid)
				{
					RCLCPP_ERROR(
						get_logger(),
						"Decision Trace closed INVALID: overflow=%s, writer_failed=%s, error=%s",
						status.trace_overflow ? "true" : "false",
						status.writer_failed ? "true" : "false",
						status.error_message.c_str());
				}
			}
			catch (const std::exception &exception)
			{
				RCLCPP_ERROR(
					get_logger(),
					"Failed to stop Decision Trace recorder: %s",
					exception.what());
			}
		}

		RCLCPP_INFO(get_logger(), "Gateway resources cleaned up");
		return CallbackReturn::SUCCESS;
	}
} // namespace control_link_gateway
