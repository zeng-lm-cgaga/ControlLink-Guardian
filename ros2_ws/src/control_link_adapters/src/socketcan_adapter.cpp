#include "control_link_adapters/socketcan_adapter.hpp"

#include <algorithm>
#include <filesystem>
#include <limits>
#include <stdexcept>
#include <utility>

#include "builtin_interfaces/msg/time.hpp"
#include "control_link_contract/fastdds_environment.hpp"
#include "control_link_contract/parser.hpp"
#include "control_link_contract/qos_factory.hpp"
#include "diagnostic_msgs/msg/diagnostic_status.hpp"
#include "diagnostic_msgs/msg/key_value.hpp"

namespace control_link_adapters
{
	namespace
	{
		constexpr char kProfilePathParameter[] = "profile_path";
		constexpr char kConfigRootParameter[] = "config_root";

		std::chrono::milliseconds checked_milliseconds(
			std::uint64_t value,
			const char *field_name)
		{
			const auto maximum = static_cast<std::uint64_t>(
				std::numeric_limits<std::chrono::milliseconds::rep>::max());
			if (value == 0U || value > maximum)
			{
				throw std::invalid_argument(
					std::string(field_name) +
					" must fit a positive milliseconds duration");
			}
			return std::chrono::milliseconds{
				static_cast<std::chrono::milliseconds::rep>(value)};
		}

		builtin_interfaces::msg::Time ros_time_message(std::int64_t nanoseconds)
		{
			if (nanoseconds <= 0)
			{
				throw std::invalid_argument(
					"SocketCanAdapter requires positive ROS time for state publication");
			}
			return static_cast<builtin_interfaces::msg::Time>(
				rclcpp::Time(nanoseconds, RCL_ROS_TIME));
		}

		CanonicalPublisherKey key_from_message_info(
			const rclcpp::MessageInfo &message_info)
		{
			const auto gid = message_info.get_rmw_message_info().publisher_gid;
			if (gid.implementation_identifier == nullptr)
			{
				throw std::invalid_argument(
					"canonical message has no publisher RMW implementation identifier");
			}

			CanonicalPublisherKey result;
			result.rmw_implementation = gid.implementation_identifier;
			std::copy_n(
				gid.data,
				RMW_GID_STORAGE_SIZE,
				result.publisher_gid.begin());
			return result;
		}

		bool same_publisher(
			const CanonicalPublisherKey &left,
			const CanonicalPublisherKey &right) noexcept
		{
			return left.rmw_implementation == right.rmw_implementation &&
				left.publisher_gid == right.publisher_gid;
		}

		const control_link_contract::CriticalEndpoint &canonical_consumer_role(
			const control_link_contract::GatewayContract &contract)
		{
			const auto endpoint = std::find_if(
				contract.critical_endpoints.begin(),
				contract.critical_endpoints.end(),
				[](const auto &candidate)
				{
					return candidate.id == "canonical_output_consumer";
				});
			if (endpoint == contract.critical_endpoints.end())
			{
				throw std::logic_error(
					"SocketCAN adapter requires canonical_output_consumer endpoint");
			}
			return *endpoint;
		}

		std::uint8_t vehicle_state_value(CanVehicleMode mode)
		{
			switch (mode)
			{
			case CanVehicleMode::kStandby:
				return control_link_interfaces::msg::VehicleState::STANDBY;
			case CanVehicleMode::kRunning:
				return control_link_interfaces::msg::VehicleState::RUNNING;
			case CanVehicleMode::kSafeStop:
				return control_link_interfaces::msg::VehicleState::SAFE_STOP;
			case CanVehicleMode::kFault:
				return control_link_interfaces::msg::VehicleState::FAULT;
			}
			throw std::logic_error("unknown decoded CAN vehicle mode");
		}

		const char *counter_observation_name(CounterObservation observation) noexcept
		{
			switch (observation)
			{
			case CounterObservation::kFirstValue:
				return "first_value";
			case CounterObservation::kExpected:
				return "expected";
			case CounterObservation::kDuplicate:
				return "duplicate";
			case CounterObservation::kJump:
				return "jump";
			case CounterObservation::kInvalid:
				return "invalid";
			}
			return "unknown";
		}

		const char *io_error_kind_name(SocketCanIoErrorKind kind) noexcept
		{
			switch (kind)
			{
			case SocketCanIoErrorKind::kPoll:
				return "poll";
			case SocketCanIoErrorKind::kCanRead:
				return "can_read";
			case SocketCanIoErrorKind::kCanWrite:
				return "can_write";
			case SocketCanIoErrorKind::kShortCanRead:
				return "short_can_read";
			case SocketCanIoErrorKind::kShortCanWrite:
				return "short_can_write";
			case SocketCanIoErrorKind::kEventRead:
				return "event_read";
			case SocketCanIoErrorKind::kReceiveHandler:
				return "receive_handler";
			case SocketCanIoErrorKind::kTransmitHandler:
				return "transmit_handler";
			case SocketCanIoErrorKind::kTransmitSuccessHandler:
				return "transmit_success_handler";
			}
			return "unknown";
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
	} // namespace

	SocketCanAdapter::SocketCanAdapter(const rclcpp::NodeOptions &options)
		: rclcpp::Node("vehicle_adapter", options),
		  state_counter_checker_(2U),
		  can_monitor_started_at_(std::chrono::steady_clock::now())
	{
		declare_parameter<std::string>(kProfilePathParameter, "");
		declare_parameter<std::string>(kConfigRootParameter, "");

		const auto profile_path = std::filesystem::path{
			get_parameter(kProfilePathParameter).as_string()};
		const auto config_root = std::filesystem::path{
			get_parameter(kConfigRootParameter).as_string()};
		bundle_ = control_link_contract::load_contract_bundle(
			profile_path,
			config_root);
		adas_profile_ = std::get_if<control_link_contract::AdasProfile>(
			bundle_->profile.get());
		if (adas_profile_ == nullptr)
		{
			throw std::invalid_argument(
				"SocketCanAdapter requires profile_id=adas");
		}

		const auto &canonical_role = canonical_consumer_role(
			*bundle_->gateway_contract);
		const std::string actual_fqn{
			get_node_base_interface()->get_fully_qualified_name()};
		if (actual_fqn != canonical_role.remote_node_fqn)
		{
			throw std::runtime_error(
				"SocketCanAdapter node FQN mismatch: expected=" +
				canonical_role.remote_node_fqn + ", actual=" + actual_fqn);
		}

		rmw_implementation_ =
			control_link_contract::validate_fastdds_process_environment(
				*bundle_->profile,
				"SocketCanAdapter");
		const bool actual_use_sim_time = get_parameter("use_sim_time").as_bool();
		if (actual_use_sim_time != adas_profile_->common.use_sim_time)
		{
			throw std::runtime_error(
				"SocketCanAdapter use_sim_time does not match ADAS Profile");
		}

		signal_map_ = bundle_->can_signal_map;
		if (!signal_map_)
		{
			throw std::logic_error("ADAS ContractBundle is missing its CAN signal map");
		}
		can_codec_ = std::make_unique<CanCodec>(*signal_map_);
		state_counter_checker_ = RollingCounterChecker(
			can_codec_->config().rolling_counter_modulo);
		can_state_timeout_ = checked_milliseconds(
			adas_profile_->adapter.can_state_frame_timeout_ms,
			"adapter.can_state_frame_timeout_ms");

		control_link_contract::QosFactory qos_factory{
			bundle_->gateway_contract};
		const auto canonical_qos = qos_factory.make(
			bundle_->gateway_contract->output.qos_profile);
		const auto &vehicle_state_contract =
			bundle_->gateway_contract->state_topics.at("vehicle_state");
		if (!vehicle_state_contract.qos_profile.has_value())
		{
			throw std::logic_error(
				"SocketCanAdapter requires vehicle_state QoS profile");
		}
		const auto vehicle_state_qos = qos_factory.make(
			vehicle_state_contract.qos_profile.value());

		canonical_guard_ = std::make_unique<CanonicalInputGuard>(
			bundle_->gateway_contract);
		local_watchdog_ = std::make_unique<LocalWatchdog>(
			adas_profile_->adapter.local_watchdog_timeout_ms);
		canonical_endpoint_tracker_ =
			std::make_unique<CanonicalEndpointTracker>(
				bundle_->gateway_contract->output.type,
				rmw_implementation_,
				checked_milliseconds(
					bundle_->gateway_contract->gateway.graph_stable_window_ms,
					"gateway.graph_stable_window_ms"));

		vehicle_state_publisher_ = create_publisher<VehicleState>(
			vehicle_state_contract.topic,
			vehicle_state_qos);
		diagnostics_publisher_ =
			create_publisher<diagnostic_msgs::msg::DiagnosticArray>(
				"/diagnostics",
				rclcpp::QoS(rclcpp::KeepLast(10U)));
		canonical_subscription_ = create_subscription<ControlCommand>(
			adas_profile_->adapter.canonical_input_topic,
			canonical_qos,
			[this](
				ControlCommand::ConstSharedPtr command,
				const rclcpp::MessageInfo &message_info)
			{
				handle_canonical_command(*command, message_info);
			});

		canonical_graph_timer_ = create_wall_timer(
			checked_milliseconds(
				bundle_->gateway_contract->gateway.graph_poll_ms,
				"gateway.graph_poll_ms"),
			[this]()
			{
				poll_canonical_endpoint();
			});
		vehicle_state_timer_ = create_wall_timer(
			checked_milliseconds(
				adas_profile_->adapter.vehicle_state_publish_period_ms,
				"adapter.vehicle_state_publish_period_ms"),
			[this]()
			{
				publish_vehicle_state();
			});
		diagnostics_timer_ = create_wall_timer(
			checked_milliseconds(
				bundle_->gateway_contract->gateway.graph_poll_ms,
				"gateway.graph_poll_ms"),
			[this]()
			{
				publish_diagnostics();
			});

		transport_ = std::make_unique<SocketCanTransport>(
			SocketCanTransportConfig{
				adas_profile_->adapter.interface,
				can_codec_->config().state_can_id,
				checked_milliseconds(
					adas_profile_->adapter.poll_timeout_ms,
					"adapter.poll_timeout_ms"),
				checked_milliseconds(
					adas_profile_->adapter.tx_period_ms,
					"adapter.tx_period_ms")});

		can_monitor_started_at_ = std::chrono::steady_clock::now();
		transport_->start(
			[this](
				const CanFrame &frame,
				std::chrono::steady_clock::time_point received_at)
			{
				handle_can_frame(frame, received_at);
			},
			[this](const SocketCanIoError &error)
			{
				handle_can_error(error);
			},
			[this](std::chrono::steady_clock::time_point)
			{
				return make_periodic_control_frame();
			},
			[this](const CanFrame &frame)
			{
				confirm_control_frame_transmitted(frame);
			});

		RCLCPP_INFO(
			get_logger(),
			"SocketCanAdapter ready: interface=%s, canonical=%s, vehicle_state=%s, tx_id=0x%03X, rx_id=0x%03X",
			adas_profile_->adapter.interface.c_str(),
			adas_profile_->adapter.canonical_input_topic.c_str(),
			vehicle_state_contract.topic.c_str(),
			can_codec_->config().control_can_id,
			can_codec_->config().state_can_id);
	}

	SocketCanAdapter::~SocketCanAdapter()
	{
		if (transport_)
		{
			transport_->stop();
		}
	}

	void SocketCanAdapter::poll_canonical_endpoint()
	{
		const auto publishers = get_publishers_info_by_topic(
			adas_profile_->adapter.canonical_input_topic);
		const auto snapshot = canonical_endpoint_tracker_->observe(
			publishers,
			std::chrono::steady_clock::now());

		bool require_immediate_hold = false;
		{
			std::lock_guard<std::mutex> lock{runtime_mutex_};
			const auto previous_state = canonical_endpoint_state_;
			canonical_endpoint_state_ = snapshot.state;

			if (snapshot.state == CanonicalEndpointState::kConfirmed &&
				snapshot.confirmed_publisher.has_value())
			{
				if (!confirmed_gateway_publisher_.has_value() ||
					!same_publisher(
						confirmed_gateway_publisher_.value(),
						snapshot.confirmed_publisher.value()))
				{
					// 新 Gateway generation 不能沿用旧命令或旧 watchdog lease
					confirmed_gateway_publisher_ = snapshot.confirmed_publisher;
					local_watchdog_->reset();
					latest_command_.reset();
					last_canonical_reject_reason_ = CanonicalRejectReason::kNone;
					require_immediate_hold = true;
				}
			}
			else if (previous_state == CanonicalEndpointState::kConfirmed)
			{
				latest_command_.reset();
				require_immediate_hold = true;
			}
		}

		if (require_immediate_hold)
		{
			request_immediate_hold();
		}
	}

	void SocketCanAdapter::handle_canonical_command(
		const ControlCommand &command,
		const rclcpp::MessageInfo &message_info)
	{
		const auto actual_publisher = key_from_message_info(message_info);
		const auto now_ros_ns = get_clock()->now().nanoseconds();
		const auto validation = canonical_guard_->validate(
			command,
			actual_publisher,
			canonical_endpoint_tracker_->current(),
			now_ros_ns,
			now_ros_ns > 0);

		if (!validation.accepted())
		{
			{
				std::lock_guard<std::mutex> lock{runtime_mutex_};
				latest_command_.reset();
				last_canonical_reject_reason_ = validation.reason;
			}
			request_immediate_hold();
			return;
		}

		{
			std::lock_guard<std::mutex> lock{runtime_mutex_};
			// 先完整复制命令，再刷新 steady watchdog，拒绝或复制失败都不能延长 lease
			latest_command_ = command;
			try
			{
				local_watchdog_->observe_valid_command(
					std::chrono::steady_clock::now());
			}
			catch (...)
			{
				latest_command_.reset();
				throw;
			}
			last_canonical_reject_reason_ = CanonicalRejectReason::kNone;
		}

		// command mailbox 已更新，eventfd 让 I/O thread 及时观察新状态
		transport_->notify();
	}

	void SocketCanAdapter::handle_can_frame(
		const CanFrame &frame,
		std::chrono::steady_clock::time_point received_at) noexcept
	{
		const auto decoded = can_codec_->decode_state(frame);
		std::lock_guard<std::mutex> lock{runtime_mutex_};
		if (!decoded.accepted())
		{
			last_can_reject_reason_ = decoded.reason;
			++rejected_can_state_frames_;
			can_recovery_valid_count_ = 0U;
			return;
		}

		const auto counter = state_counter_checker_.observe(
			decoded.signals->state_counter);
		last_counter_observation_ = counter;
		if (counter != CounterObservation::kFirstValue &&
			counter != CounterObservation::kExpected)
		{
			++rejected_can_state_frames_;
			can_recovery_valid_count_ = 0U;
			return;
		}

		latest_valid_can_state_ = ValidCanState{
			decoded.signals.value(),
			received_at};
		last_can_reject_reason_ = CanCodecRejectReason::kNone;
		++accepted_can_state_frames_;
		can_state_timed_out_ = false;
		if (!can_link_healthy_)
		{
			if (can_recovery_valid_count_ <
				adas_profile_->adapter.recovery_valid_frames)
			{
				++can_recovery_valid_count_;
			}
			if (can_recovery_valid_count_ >=
				adas_profile_->adapter.recovery_valid_frames)
			{
				can_link_healthy_ = true;
			}
		}
	}

	void SocketCanAdapter::handle_can_error(
		const SocketCanIoError &error)
	{
		std::lock_guard<std::mutex> lock{runtime_mutex_};
		if (error.fatal)
		{
			// detail 复制即使失败，也必须先保留 fail-closed 状态
			fatal_can_io_ = true;
			can_link_healthy_ = false;
			can_recovery_valid_count_ = 0U;
			state_counter_checker_.reset();
		}
		last_can_io_error_ = error;
	}

	std::optional<CanFrame> SocketCanAdapter::make_periodic_control_frame()
	{
		std::lock_guard<std::mutex> lock{runtime_mutex_};
		// canonical callback 也在此锁内刷新 watchdog，评估时间必须在锁内后取
		const auto now = std::chrono::steady_clock::now();
		if (force_next_hold_)
		{
			force_next_hold_ = false;
			return make_safe_hold_frame_locked();
		}
		const bool canonical_healthy =
			canonical_endpoint_state_ == CanonicalEndpointState::kConfirmed &&
			last_canonical_reject_reason_ == CanonicalRejectReason::kNone &&
			local_watchdog_->evaluate(now) == LocalWatchdogState::kHealthy &&
			latest_command_.has_value();
		if (!canonical_healthy ||
			latest_command_->mode == ControlCommand::MODE_HOLD)
		{
			return make_safe_hold_frame_locked();
		}

		return make_control_frame_locked(
			CanControlMode::kActive,
			latest_command_->linear_velocity_mps,
			latest_command_->angular_velocity_radps);
	}

	CanFrame SocketCanAdapter::make_control_frame_locked(
		CanControlMode mode,
		double speed_mps,
		double yaw_rate_radps)
	{
		const auto encoded = can_codec_->encode_control(
			CanControlSignals{
				mode,
				speed_mps,
				yaw_rate_radps,
				next_control_counter_});
		if (!encoded.accepted())
		{
			throw std::logic_error(
				std::string("validated canonical command failed CAN encoding: ") +
				can_codec_reject_reason_name(encoded.reason));
		}

		return encoded.frame.value();
	}

	CanFrame SocketCanAdapter::make_safe_hold_frame_locked()
	{
		return make_control_frame_locked(
			CanControlMode::kHold,
			0.0,
			0.0);
	}

	void SocketCanAdapter::confirm_control_frame_transmitted(
		const CanFrame &frame)
	{
		std::lock_guard<std::mutex> lock{runtime_mutex_};
		const auto transmitted_counter = static_cast<std::uint8_t>(
			frame.data[5U] & 0x0FU);
		if (transmitted_counter != next_control_counter_)
		{
			throw std::logic_error(
				"SocketCAN transmitted control counter does not match pending counter");
		}
		next_control_counter_ = static_cast<std::uint8_t>(
			(next_control_counter_ + 1U) %
			can_codec_->config().rolling_counter_modulo);
		if (last_can_io_error_.has_value() && !last_can_io_error_->fatal)
		{
			last_can_io_error_.reset();
		}
	}

	void SocketCanAdapter::request_immediate_hold() noexcept
	{
		{
			std::lock_guard<std::mutex> lock{runtime_mutex_};
			force_next_hold_ = true;
		}
		transport_->request_immediate_transmit();
	}

	void SocketCanAdapter::mark_can_timeout_locked(
		std::chrono::steady_clock::time_point now)
	{
		const auto reference = latest_valid_can_state_.has_value() ?
			latest_valid_can_state_->received_at : can_monitor_started_at_;
		if (now < reference)
		{
			throw std::logic_error(
				"SocketCanAdapter steady clock moved backwards");
		}
		if (now - reference < can_state_timeout_)
		{
			return;
		}

		if (!can_state_timed_out_)
		{
			// timeout 后允许 simulator/接口重启后的任意首个 counter 重新建立基线
			state_counter_checker_.reset();
		}
		can_state_timed_out_ = true;
		can_link_healthy_ = false;
		can_recovery_valid_count_ = 0U;
	}

	std::uint16_t SocketCanAdapter::canonical_fault_code_locked(
		std::chrono::steady_clock::time_point now) const
	{
		if (canonical_endpoint_state_ == CanonicalEndpointState::kAmbiguous ||
			last_canonical_reject_reason_ == CanonicalRejectReason::kEndpointAmbiguous)
		{
			return VehicleState::FAULT_ADAPTER_CANONICAL_SOURCE_AMBIGUOUS;
		}
		if (last_canonical_reject_reason_ != CanonicalRejectReason::kNone &&
			last_canonical_reject_reason_ !=
				CanonicalRejectReason::kEndpointUnavailable &&
			last_canonical_reject_reason_ !=
				CanonicalRejectReason::kEndpointUnstable)
		{
			return VehicleState::FAULT_ADAPTER_CANONICAL_INVALID;
		}
		if (local_watchdog_->evaluate(now) != LocalWatchdogState::kHealthy)
		{
			return VehicleState::FAULT_ADAPTER_CANONICAL_TIMEOUT;
		}
		if (canonical_endpoint_state_ != CanonicalEndpointState::kConfirmed)
		{
			return VehicleState::FAULT_ADAPTER_CANONICAL_TIMEOUT;
		}
		if (!latest_command_.has_value())
		{
			throw std::logic_error(
				"healthy SocketCAN adapter watchdog has no accepted canonical command");
		}
		return VehicleState::FAULT_NONE;
	}

	SocketCanAdapter::VehicleState SocketCanAdapter::compose_vehicle_state_locked(
		std::int64_t now_ros_ns,
		std::chrono::steady_clock::time_point now_steady) const
	{
		VehicleState state;
		state.observed_at = ros_time_message(now_ros_ns);

		if (latest_valid_can_state_.has_value())
		{
			const auto &signals = latest_valid_can_state_->signals;
			state.linear_velocity_mps = signals.measured_speed_mps;
			state.angular_velocity_radps = signals.measured_yaw_rate_radps;
			state.rolling_counter = signals.state_counter;
		}

		if (fatal_can_io_)
		{
			state.state = VehicleState::SAFE_STOP;
			state.fault_code = VehicleState::FAULT_ADAS_CAN_IO;
			return state;
		}
		if (!can_link_healthy_)
		{
			state.state = VehicleState::SAFE_STOP;
			state.fault_code = VehicleState::FAULT_ADAS_CAN_STATE_TIMEOUT;
			return state;
		}
		if (!latest_valid_can_state_.has_value())
		{
			throw std::logic_error(
				"healthy SocketCAN link has no valid state snapshot");
		}

		const auto &signals = latest_valid_can_state_->signals;
		if (signals.fault_code != 0U)
		{
			state.state = VehicleState::FAULT;
			state.fault_code = static_cast<std::uint16_t>(
				VehicleState::FAULT_ADAS_VEHICLE_REPORTED_BASE +
				signals.fault_code);
			return state;
		}
		if (signals.mode == CanVehicleMode::kFault)
		{
			state.state = vehicle_state_value(signals.mode);
			state.fault_code = VehicleState::FAULT_NONE;
			return state;
		}

		// simulator watchdog 导致的 SafeStop 在启动期必须保留 canonical timeout 语义
		// 否则 Gateway 无法识别“尚未收到首条合法 HOLD”的唯一 bootstrap 例外
		const auto canonical_fault = canonical_fault_code_locked(now_steady);
		if (canonical_fault != VehicleState::FAULT_NONE)
		{
			state.state = VehicleState::SAFE_STOP;
			state.fault_code = canonical_fault;
			return state;
		}
		if (signals.mode == CanVehicleMode::kSafeStop)
		{
			state.state = vehicle_state_value(signals.mode);
			state.fault_code = VehicleState::FAULT_NONE;
			return state;
		}

		state.state = vehicle_state_value(signals.mode);
		state.fault_code = VehicleState::FAULT_NONE;
		return state;
	}

	void SocketCanAdapter::publish_vehicle_state()
	{
		const auto now_ros_ns = get_clock()->now().nanoseconds();
		if (now_ros_ns <= 0)
		{
			return;
		}

		VehicleState state;
		{
			std::lock_guard<std::mutex> lock{runtime_mutex_};
			// 与 CAN snapshot 在同一临界区取时，避免锁等待期间的新帧晚于旧 now
			const auto now_steady = std::chrono::steady_clock::now();
			mark_can_timeout_locked(now_steady);
			state = compose_vehicle_state_locked(now_ros_ns, now_steady);
		}
		vehicle_state_publisher_->publish(state);
	}

	void SocketCanAdapter::publish_diagnostics()
	{
		const auto now_ros_ns = get_clock()->now().nanoseconds();
		if (now_ros_ns <= 0)
		{
			return;
		}

		diagnostic_msgs::msg::DiagnosticArray array;
		array.header.stamp = ros_time_message(now_ros_ns);
		diagnostic_msgs::msg::DiagnosticStatus status;
		status.name = "control_link/vehicle_adapter/socketcan";
		status.hardware_id = adas_profile_->adapter.interface;

		const auto transport_stats = transport_->stats();
		{
			std::lock_guard<std::mutex> lock{runtime_mutex_};
			const auto now_steady = std::chrono::steady_clock::now();
			mark_can_timeout_locked(now_steady);
			const auto canonical_fault = canonical_fault_code_locked(now_steady);

			if (fatal_can_io_)
			{
				status.level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
				status.message = "fatal CAN I/O failure";
			}
			else if (!can_link_healthy_)
			{
				status.level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
				status.message = can_state_timed_out_ ?
					"CAN state frame timeout" : "CAN state recovery pending";
			}
			else if (last_can_reject_reason_ != CanCodecRejectReason::kNone ||
				last_counter_observation_ == CounterObservation::kDuplicate ||
				last_counter_observation_ == CounterObservation::kJump ||
				last_counter_observation_ == CounterObservation::kInvalid ||
				last_can_io_error_.has_value())
			{
				status.level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
				status.message = "transient CAN frame or transport rejection";
			}
			else
			{
				status.level = diagnostic_msgs::msg::DiagnosticStatus::OK;
				status.message = "SocketCAN data path healthy";
			}

			add_diagnostic_value(
				status,
				"can_link_healthy",
				can_link_healthy_ ? "true" : "false");
			add_diagnostic_value(
				status,
				"can_state_timed_out",
				can_state_timed_out_ ? "true" : "false");
			add_diagnostic_value(
				status,
				"can_recovery_valid_count",
				std::to_string(can_recovery_valid_count_));
			add_diagnostic_value(
				status,
				"can_recovery_required",
				std::to_string(adas_profile_->adapter.recovery_valid_frames));
			add_diagnostic_value(
				status,
				"last_codec_reason",
				can_codec_reject_reason_name(last_can_reject_reason_));
			add_diagnostic_value(
				status,
				"last_counter_observation",
				counter_observation_name(last_counter_observation_));
			add_diagnostic_value(
				status,
				"accepted_can_state_frames",
				std::to_string(accepted_can_state_frames_));
			add_diagnostic_value(
				status,
				"rejected_can_state_frames",
				std::to_string(rejected_can_state_frames_));
			add_diagnostic_value(
				status,
				"canonical_fault_code",
				std::to_string(canonical_fault));
			add_diagnostic_value(
				status,
				"last_io_error_kind",
				last_can_io_error_.has_value() ?
					io_error_kind_name(last_can_io_error_->kind) : "none");
			add_diagnostic_value(
				status,
				"last_io_errno",
				last_can_io_error_.has_value() ?
					std::to_string(last_can_io_error_->error_number) : "0");
			add_diagnostic_value(
				status,
				"last_io_detail",
				last_can_io_error_.has_value() &&
					!last_can_io_error_->detail.empty() ?
					last_can_io_error_->detail : "none");
		}

		add_diagnostic_value(
			status,
			"transport_transmitted_frames",
			std::to_string(transport_stats.transmitted_frames));
		add_diagnostic_value(
			status,
			"transport_received_frames",
			std::to_string(transport_stats.received_frames));
		add_diagnostic_value(
			status,
			"transport_tx_would_block",
			std::to_string(transport_stats.transmit_would_block));
		add_diagnostic_value(
			status,
			"transport_poll_errors",
			std::to_string(transport_stats.poll_errors));
		add_diagnostic_value(
			status,
			"transport_read_errors",
			std::to_string(transport_stats.read_errors));
		add_diagnostic_value(
			status,
			"transport_write_errors",
			std::to_string(transport_stats.write_errors));
		array.status.push_back(std::move(status));
		diagnostics_publisher_->publish(array);
	}
} // namespace control_link_adapters
