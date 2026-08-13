#include "control_link_adapters/ros2_control_adapter.hpp"

#include <algorithm>
#include <cmath>
#include <filesystem>
#include <limits>
#include <stdexcept>
#include <utility>

#include "builtin_interfaces/msg/time.hpp"
#include "control_link_contract/fastdds_environment.hpp"
#include "control_link_contract/qos_factory.hpp"

namespace control_link_adapters
{
	namespace
	{
		constexpr char kProfilePathParameter[] = "profile_path";
		constexpr char kConfigRootParameter[] = "config_root";
		constexpr char kTwistStampedType[] = "geometry_msgs/msg/TwistStamped";
		constexpr std::uint64_t kNanosecondsPerMillisecond = 1'000'000ULL;
		constexpr std::int64_t kNanosecondsPerSecond = 1'000'000'000LL;

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

		std::chrono::nanoseconds checked_nanoseconds(
			std::uint64_t milliseconds,
			const char *field_name,
			bool allow_zero = false)
		{
			const auto maximum = static_cast<std::uint64_t>(
				std::numeric_limits<std::chrono::nanoseconds::rep>::max());
			if ((!allow_zero && milliseconds == 0U) ||
				milliseconds > maximum / kNanosecondsPerMillisecond)
			{
				throw std::invalid_argument(
					std::string(field_name) +
					" does not fit the supported nanoseconds duration");
			}
			return std::chrono::nanoseconds{
				static_cast<std::chrono::nanoseconds::rep>(
					milliseconds * kNanosecondsPerMillisecond)};
		}

		std::chrono::nanoseconds output_period(double output_rate_hz)
		{
			if (!std::isfinite(output_rate_hz) || output_rate_hz <= 0.0)
			{
				throw std::invalid_argument(
					"gateway.output_rate_hz must be positive and finite");
			}

			constexpr long double kNanosecondsPerSecond = 1'000'000'000.0L;
			const auto period_ns =
				kNanosecondsPerSecond / static_cast<long double>(output_rate_hz);
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

		std::optional<std::int64_t> stamp_nanoseconds(
			const builtin_interfaces::msg::Time &stamp) noexcept
		{
			if (stamp.sec < 0 ||
				stamp.nanosec >= static_cast<std::uint32_t>(kNanosecondsPerSecond))
			{
				return std::nullopt;
			}
			return static_cast<std::int64_t>(stamp.sec) * kNanosecondsPerSecond +
				static_cast<std::int64_t>(stamp.nanosec);
		}

		builtin_interfaces::msg::Time ros_time_message(std::int64_t nanoseconds)
		{
			if (nanoseconds <= 0)
			{
				throw std::invalid_argument(
					"Ros2ControlAdapter requires positive ROS time for state publication");
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
					"Robot adapter requires canonical_output_consumer endpoint");
			}
			return *endpoint;
		}
	} // namespace

	Ros2ControlAdapter::Ros2ControlAdapter(const rclcpp::NodeOptions &options)
		: rclcpp::Node("vehicle_adapter", options)
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
		robot_profile_ = std::get_if<control_link_contract::RobotProfile>(
			bundle_->profile.get());
		if (robot_profile_ == nullptr)
		{
			throw std::invalid_argument(
				"Ros2ControlAdapter requires profile_id=robot");
		}

		const auto &canonical_role = canonical_consumer_role(
			*bundle_->gateway_contract);
		const std::string actual_fqn{
			get_node_base_interface()->get_fully_qualified_name()};
		if (actual_fqn != canonical_role.remote_node_fqn)
		{
			throw std::runtime_error(
				"Ros2ControlAdapter node FQN mismatch: expected=" +
				canonical_role.remote_node_fqn + ", actual=" + actual_fqn);
		}

		if (robot_profile_->adapter.controller_command_type != kTwistStampedType)
		{
			throw std::runtime_error(
				"this build supports only geometry_msgs/msg/TwistStamped controller commands; actual=" +
				robot_profile_->adapter.controller_command_type);
		}

		rmw_implementation_ =
			control_link_contract::validate_fastdds_process_environment(
				*bundle_->profile,
				"Ros2ControlAdapter");

		const bool actual_use_sim_time = get_parameter("use_sim_time").as_bool();
		if (actual_use_sim_time != robot_profile_->common.use_sim_time)
		{
			throw std::runtime_error(
				"Ros2ControlAdapter use_sim_time does not match Robot Profile");
		}

		odometry_timeout_ = checked_nanoseconds(
			robot_profile_->health.odometry_timeout_ms,
			"health.odometry_timeout_ms");
		max_future_skew_ = checked_nanoseconds(
			bundle_->gateway_contract->limits.max_future_skew_ms,
			"limits.max_future_skew_ms",
			true);

		control_link_contract::QosFactory qos_factory{
			bundle_->gateway_contract};
		const auto canonical_qos = qos_factory.make(
			bundle_->gateway_contract->output.qos_profile);
		const auto &vehicle_state_contract =
			bundle_->gateway_contract->state_topics.at("vehicle_state");
		if (!vehicle_state_contract.qos_profile.has_value())
		{
			throw std::logic_error(
				"Robot adapter requires vehicle_state QoS profile");
		}
		const auto vehicle_state_qos = qos_factory.make(
			vehicle_state_contract.qos_profile.value());
		rclcpp::QoS controller_command_qos{rclcpp::KeepLast(1)};
		// controller Graph 明确回报 Automatic 与 infinite durations，本地 offered QoS 也必须明确
		const auto infinite_duration = rclcpp::Duration::max();
		controller_command_qos
			.reliable()
			.durability_volatile()
			.deadline(infinite_duration)
			.liveliness(rclcpp::LivelinessPolicy::Automatic)
			.liveliness_lease_duration(infinite_duration);

		canonical_guard_ = std::make_unique<CanonicalInputGuard>(
			bundle_->gateway_contract);
		local_watchdog_ = std::make_unique<LocalWatchdog>(
			robot_profile_->adapter.local_watchdog_timeout_ms);
		canonical_endpoint_tracker_ =
			std::make_unique<CanonicalEndpointTracker>(
				bundle_->gateway_contract->output.type,
				rmw_implementation_,
				checked_milliseconds(
					bundle_->gateway_contract->gateway.graph_stable_window_ms,
					"gateway.graph_stable_window_ms"));
		controller_endpoint_monitor_ =
			std::make_unique<ControllerEndpointMonitor>(
				robot_profile_->adapter.controller_node_fqn,
				robot_profile_->adapter.controller_command_type,
				controller_command_qos,
				checked_milliseconds(
					bundle_->gateway_contract->gateway.graph_stable_window_ms,
					"gateway.graph_stable_window_ms"),
				checked_milliseconds(
					robot_profile_->health.controller_state_timeout_ms,
					"health.controller_state_timeout_ms"));

		tf_buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
		// dedicated listener thread 让带 timeout 的 lookup 不会阻塞等待同一 executor 处理 /tf
		tf_listener_ = std::make_unique<tf2_ros::TransformListener>(
			*tf_buffer_,
			this,
			true);
		tf_health_monitor_ = std::make_unique<TfHealthMonitor>(
			*tf_buffer_,
			robot_profile_->frames.map,
			robot_profile_->frames.base_footprint,
			robot_profile_->health.tf_lookup_timeout_ms,
			robot_profile_->health.tf_max_age_ms);

		controller_command_publisher_ =
			create_publisher<geometry_msgs::msg::TwistStamped>(
				robot_profile_->adapter.controller_output_topic,
				controller_command_qos);
		vehicle_state_publisher_ = create_publisher<VehicleState>(
			vehicle_state_contract.topic,
			vehicle_state_qos);

		canonical_subscription_ = create_subscription<ControlCommand>(
			robot_profile_->adapter.canonical_input_topic,
			canonical_qos,
			[this](
				ControlCommand::ConstSharedPtr command,
				const rclcpp::MessageInfo &message_info)
			{
				handle_canonical_command(*command, message_info);
			});
		odometry_subscription_ = create_subscription<nav_msgs::msg::Odometry>(
			robot_profile_->adapter.odometry_topic,
			rclcpp::SensorDataQoS(),
			[this](nav_msgs::msg::Odometry::ConstSharedPtr odometry)
			{
				handle_odometry(*odometry);
			});

		const auto graph_period = checked_milliseconds(
			bundle_->gateway_contract->gateway.graph_poll_ms,
			"gateway.graph_poll_ms");
		canonical_graph_timer_ = create_wall_timer(
			graph_period,
			[this]()
			{
				poll_canonical_endpoint();
			});
		controller_graph_timer_ = create_wall_timer(
			graph_period,
			[this]()
			{
				poll_controller_endpoint();
			});
		controller_output_timer_ = create_wall_timer(
			output_period(bundle_->gateway_contract->gateway.output_rate_hz),
			[this]()
			{
				publish_controller_command();
			});
		vehicle_state_timer_ = create_wall_timer(
			checked_milliseconds(
				robot_profile_->health.vehicle_state_publish_period_ms,
				"health.vehicle_state_publish_period_ms"),
			[this]()
			{
				publish_vehicle_state();
			});

		RCLCPP_INFO(
			get_logger(),
			"Ros2ControlAdapter ready: canonical=%s, controller=%s, odometry=%s, vehicle_state=%s",
			robot_profile_->adapter.canonical_input_topic.c_str(),
			robot_profile_->adapter.controller_output_topic.c_str(),
			robot_profile_->adapter.odometry_topic.c_str(),
			vehicle_state_contract.topic.c_str());
	}

	void Ros2ControlAdapter::poll_canonical_endpoint()
	{
		const auto publishers = get_publishers_info_by_topic(
			robot_profile_->adapter.canonical_input_topic);
		const auto snapshot = canonical_endpoint_tracker_->observe(
			publishers,
			std::chrono::steady_clock::now());
		if (snapshot.state != CanonicalEndpointState::kConfirmed ||
			!snapshot.confirmed_publisher.has_value())
		{
			return;
		}

		if (!confirmed_gateway_publisher_.has_value() ||
			!same_publisher(
				confirmed_gateway_publisher_.value(),
				snapshot.confirmed_publisher.value()))
		{
			// 新 Gateway process/GID 必须从 bootstrap watchdog 重新开始
			confirmed_gateway_publisher_ = snapshot.confirmed_publisher;
			local_watchdog_->reset();
			latest_command_.reset();
			last_canonical_reject_reason_ = CanonicalRejectReason::kNone;
		}
	}

	void Ros2ControlAdapter::poll_controller_endpoint()
	{
		const auto subscriptions = get_subscriptions_info_by_topic(
			robot_profile_->adapter.controller_output_topic);
		controller_endpoint_snapshot_ = controller_endpoint_monitor_->observe(
			subscriptions,
			std::chrono::steady_clock::now());
	}

	void Ros2ControlAdapter::handle_canonical_command(
		const ControlCommand &command,
		const rclcpp::MessageInfo &message_info)
	{
		const auto actual_publisher = key_from_message_info(message_info);
		const auto now_ros_ns = get_clock()->now().nanoseconds();
		const auto result = canonical_guard_->validate(
			command,
			actual_publisher,
			canonical_endpoint_tracker_->current(),
			now_ros_ns,
			now_ros_ns > 0);
		if (!result.accepted())
		{
			latest_command_.reset();
			last_canonical_reject_reason_ = result.reason;
			return;
		}

		// 只有完整复制成功后才刷新 watchdog，避免半提交命令延长 lease
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

	void Ros2ControlAdapter::handle_odometry(
		const nav_msgs::msg::Odometry &odometry)
	{
		latest_odometry_sample_valid_ = false;
		const auto stamp_ns = stamp_nanoseconds(odometry.header.stamp);
		const auto now_ros_ns = get_clock()->now().nanoseconds();
		const auto linear_velocity = odometry.twist.twist.linear.x;
		const auto angular_velocity = odometry.twist.twist.angular.z;

		if (!stamp_ns.has_value() || stamp_ns.value() <= 0 || now_ros_ns <= 0 ||
			!std::isfinite(linear_velocity) || !std::isfinite(angular_velocity) ||
			odometry.header.frame_id != robot_profile_->frames.odom ||
			odometry.child_frame_id != robot_profile_->frames.base_footprint)
		{
			return;
		}

		if (stamp_ns.value() > now_ros_ns)
		{
			if (stamp_ns.value() - now_ros_ns > max_future_skew_.count())
			{
				return;
			}
		}
		else if (now_ros_ns - stamp_ns.value() > odometry_timeout_.count())
		{
			return;
		}

		latest_valid_odometry_ = OdomSnapshot{
			stamp_ns.value(),
			std::chrono::steady_clock::now(),
			linear_velocity,
			angular_velocity};
		latest_odometry_sample_valid_ = true;
	}

	bool Ros2ControlAdapter::odometry_is_healthy(
		std::chrono::steady_clock::time_point now_steady,
		std::int64_t now_ros_ns) const noexcept
	{
		if (!latest_odometry_sample_valid_ ||
			!latest_valid_odometry_.has_value() ||
			now_ros_ns <= 0)
		{
			return false;
		}

		const auto &odometry = latest_valid_odometry_.value();
		if (now_steady < odometry.received_at ||
			now_steady - odometry.received_at > odometry_timeout_)
		{
			return false;
		}

		if (odometry.stamp_ns > now_ros_ns)
		{
			return odometry.stamp_ns - now_ros_ns <= max_future_skew_.count();
		}
		return now_ros_ns - odometry.stamp_ns <= odometry_timeout_.count();
	}

	std::uint16_t Ros2ControlAdapter::platform_fault_code(
		std::chrono::steady_clock::time_point now_steady,
		std::int64_t now_ros_ns) const noexcept
	{
		if (!controller_endpoint_snapshot_.healthy())
		{
			return controller_endpoint_snapshot_.state ==
				ControllerEndpointState::kTimedOut ?
				VehicleState::FAULT_ROBOT_CONTROLLER_STATE_TIMEOUT :
				VehicleState::FAULT_ROBOT_CONTROLLER_UNAVAILABLE;
		}

		if (!odometry_is_healthy(now_steady, now_ros_ns))
		{
			return VehicleState::FAULT_ROBOT_ODOMETRY_TIMEOUT;
		}

		switch (tf_health_snapshot_.state)
		{
		case TfHealthState::kHealthy:
			return VehicleState::FAULT_NONE;

		case TfHealthState::kUnavailable:
			return VehicleState::FAULT_ROBOT_TF_UNAVAILABLE;

		case TfHealthState::kInvalidTime:
		case TfHealthState::kStale:
			return VehicleState::FAULT_ROBOT_TF_STALE;
		}

		return VehicleState::FAULT_ROBOT_TF_STALE;
	}

	std::uint16_t Ros2ControlAdapter::current_fault_code(
		std::chrono::steady_clock::time_point now_steady,
		std::int64_t now_ros_ns) const
	{
		const auto canonical_endpoint = canonical_endpoint_tracker_->current();
		if (canonical_endpoint.state == CanonicalEndpointState::kAmbiguous ||
			last_canonical_reject_reason_ == CanonicalRejectReason::kEndpointAmbiguous)
		{
			return VehicleState::FAULT_ADAPTER_CANONICAL_SOURCE_AMBIGUOUS;
		}
		if (canonical_endpoint.state != CanonicalEndpointState::kConfirmed)
		{
			// activate 前也要求 adapter 已稳定确认 Gateway generation，避免首条 HOLD 被拒绝
			return VehicleState::FAULT_ADAPTER_CANONICAL_INVALID;
		}

		if (last_canonical_reject_reason_ != CanonicalRejectReason::kNone)
		{
			return VehicleState::FAULT_ADAPTER_CANONICAL_INVALID;
		}

		const auto platform_fault = platform_fault_code(now_steady, now_ros_ns);
		if (platform_fault != VehicleState::FAULT_NONE)
		{
			return platform_fault;
		}

		if (local_watchdog_->evaluate(now_steady) != LocalWatchdogState::kHealthy)
		{
			return VehicleState::FAULT_ADAPTER_CANONICAL_TIMEOUT;
		}

		if (!latest_command_.has_value())
		{
			throw std::logic_error(
				"healthy Robot adapter watchdog has no accepted canonical command");
		}

		return VehicleState::FAULT_NONE;
	}

	void Ros2ControlAdapter::publish_controller_command()
	{
		const auto now_steady = std::chrono::steady_clock::now();
		const auto now_ros_ns = get_clock()->now().nanoseconds();
		const auto fault_code = current_fault_code(now_steady, now_ros_ns);

		geometry_msgs::msg::TwistStamped output;
		if (now_ros_ns > 0)
		{
			output.header.stamp = ros_time_message(now_ros_ns);
		}
		output.header.frame_id = robot_profile_->frames.base_footprint;
		if (fault_code == VehicleState::FAULT_NONE &&
			latest_command_.has_value() &&
			latest_command_->mode == ControlCommand::MODE_NORMAL)
		{
			output.twist.linear.x = latest_command_->linear_velocity_mps;
			output.twist.angular.z = latest_command_->angular_velocity_radps;
		}
		// 默认构造的 Twist 全为 0，HOLD 和任意 fault 都持续发布软件层零命令
		controller_command_publisher_->publish(output);
	}

	void Ros2ControlAdapter::publish_vehicle_state()
	{
		const auto now_steady = std::chrono::steady_clock::now();
		const auto now_ros_ns = get_clock()->now().nanoseconds();
		if (now_ros_ns <= 0)
		{
			// observed_at=0 会被 Gateway 结构化拒绝，等待仿真 ROS clock 启动再发布
			return;
		}

		tf_health_snapshot_ = tf_health_monitor_->assess(now_ros_ns);
		const auto fault_code = current_fault_code(now_steady, now_ros_ns);

		VehicleState state;
		state.observed_at = ros_time_message(now_ros_ns);
		state.fault_code = fault_code;
		state.rolling_counter = 0U;
		if (odometry_is_healthy(now_steady, now_ros_ns))
		{
			// 速度是执行端实际观测，不因 STANDBY/SAFE_STOP 就伪装为已经静止
			state.linear_velocity_mps =
				latest_valid_odometry_->linear_velocity_mps;
			state.angular_velocity_radps =
				latest_valid_odometry_->angular_velocity_radps;
		}
		if (fault_code != VehicleState::FAULT_NONE)
		{
			state.state = VehicleState::SAFE_STOP;
		}
		else if (!latest_command_.has_value())
		{
			throw std::logic_error(
				"healthy Robot adapter has no accepted canonical command");
		}
		else if (latest_command_->mode == ControlCommand::MODE_HOLD)
		{
			state.state = VehicleState::STANDBY;
		}
		else
		{
			state.state = VehicleState::RUNNING;
		}

		vehicle_state_publisher_->publish(state);
	}
} // namespace control_link_adapters
