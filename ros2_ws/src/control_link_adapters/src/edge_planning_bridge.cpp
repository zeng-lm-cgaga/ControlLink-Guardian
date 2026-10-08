#include "control_link_edge_protocol/edge_payload.hpp"

#include <algorithm>
#include <array>
#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <filesystem>
#include <limits>
#include <memory>
#include <mutex>
#include <optional>
#include <stdexcept>
#include <string>
#include <thread>
#include <utility>
#include <variant>

#include "builtin_interfaces/msg/time.hpp"
#include "control_link_contract/contract_bundle.hpp"
#include "control_link_contract/qos_factory.hpp"
#include "control_link_interfaces/msg/control_command.hpp"
#include "control_link_interfaces/msg/gateway_state.hpp"
#include "control_link_interfaces/msg/source_status.hpp"
#include "control_link_interfaces/msg/vehicle_state.hpp"
#include "edge_runtime/channel/channel_options.hpp"
#include "edge_runtime/channel/consumer.hpp"
#include "edge_runtime/channel/producer.hpp"
#include "edge_runtime/common/error.hpp"
#include "rclcpp/rclcpp.hpp"

namespace
{
	using namespace std::chrono_literals;
	using control_link_edge_protocol::CommandFrame;
	using control_link_edge_protocol::FeedbackFrame;
	using control_link_edge_protocol::SessionToken;
	using CommandConsumer = edge_runtime::Consumer<CommandFrame>;
	using FeedbackProducer = edge_runtime::Producer<FeedbackFrame>;
	using ControlCommand = control_link_interfaces::msg::ControlCommand;
	using GatewayState = control_link_interfaces::msg::GatewayState;
	using SourceStatus = control_link_interfaces::msg::SourceStatus;
	using VehicleState = control_link_interfaces::msg::VehicleState;

	struct TransportIdentity
	{
		std::uint64_t generation{0U};
		std::array<std::byte, 16U> instance_nonce{};

		bool operator==(const TransportIdentity &other) const noexcept
		{
			return generation == other.generation && instance_nonce == other.instance_nonce;
		}
		bool operator!=(const TransportIdentity &other) const noexcept
		{
			return !(*this == other);
		}
	};

	struct PendingCommand
	{
		CommandFrame frame{};
		std::uint64_t edge_sequence{0U};
		std::uint64_t epoch{0U};
	};

	std::string edge_error_text(const edge_runtime::Error &error)
	{
		std::string result = edge_runtime::to_string(error.code);
		if (error.operation != nullptr)
		{
			result += " operation=";
			result += error.operation;
		}
		return result;
	}

	bool profile_enables_source(
		const control_link_contract::ProfileConfig &profile,
		const std::string &source_id)
	{
		return std::visit(
			[&source_id](const auto &typed_profile)
			{
				return std::find(
					typed_profile.common.enabled_sources.begin(),
					typed_profile.common.enabled_sources.end(),
					source_id) != typed_profile.common.enabled_sources.end();
			},
			profile);
	}

	std::uint64_t ros_time_ns(const builtin_interfaces::msg::Time &time) noexcept
	{
		try
		{
			const auto value = rclcpp::Time(time).nanoseconds();
			return value > 0 ? static_cast<std::uint64_t>(value) : 0U;
		}
		catch (const std::exception &)
		{
			return 0U;
		}
	}

	bool to_ros_time(
		std::int64_t nanoseconds,
		builtin_interfaces::msg::Time *time) noexcept
	{
		if (time == nullptr || nanoseconds <= 0) return false;
		constexpr std::int64_t kNanosecondsPerSecond = 1'000'000'000LL;
		const auto seconds = nanoseconds / kNanosecondsPerSecond;
		const auto remainder = nanoseconds % kNanosecondsPerSecond;
		if (seconds > std::numeric_limits<std::int32_t>::max() || remainder < 0)
		{
			return false;
		}
		time->sec = static_cast<std::int32_t>(seconds);
		time->nanosec = static_cast<std::uint32_t>(remainder);
		return true;
	}

	control_link_edge_protocol::ActiveSource active_source_from_id(
		const std::string &source_id) noexcept
	{
		if (source_id == "planning") return control_link_edge_protocol::ActiveSource::kPlanning;
		if (source_id == "teleop") return control_link_edge_protocol::ActiveSource::kTeleop;
		if (source_id == "nav2") return control_link_edge_protocol::ActiveSource::kNav2;
		return control_link_edge_protocol::ActiveSource::kNone;
	}

	// Edge handle 只由 worker 拥有；ROS publisher 由 timer 串行重建，epoch 在发布前复核。
	class EdgePlanningBridge final : public rclcpp::Node
	{
	public:
		EdgePlanningBridge()
			: rclcpp::Node("edge_planning_bridge")
		{
			declare_parameter<std::string>("profile_path", "");
			declare_parameter<std::string>("config_root", "");
			declare_parameter<std::string>("command_channel", "control_link_planning_command");
			declare_parameter<std::string>("feedback_channel", "control_link_planning_feedback");
			declare_parameter<std::string>("source_id", "planning");

			load_configuration();
			if (!control_link_edge_protocol::generate_session_token(&session_token_))
			{
				throw std::runtime_error(
					"EdgePlanningBridge cannot obtain an operating-system session token");
			}
			session_cutoff_boot_ns_ = control_link_edge_protocol::boottime_now_ns();
			if (session_cutoff_boot_ns_ == 0U)
			{
				throw std::runtime_error("EdgePlanningBridge cannot read CLOCK_BOOTTIME");
			}

			publisher_ = create_publisher<ControlCommand>(source_topic_, input_qos_);
			create_feedback_subscriptions();
			publish_timer_ = create_wall_timer(5ms, [this]() { publish_pending_command(); });
			worker_ = std::thread([this]() { edge_worker_loop(); });

			RCLCPP_INFO(
				get_logger(),
				"Edge planning bridge ready: source=%s topic=%s command_channel=%s feedback_channel=%s",
				source_id_.c_str(), source_topic_.c_str(), command_channel_.c_str(),
				feedback_channel_.c_str());
		}

		~EdgePlanningBridge() override
		{
			{
				std::lock_guard<std::mutex> lock(mutex_);
				stop_requested_ = true;
			}
			stop_condition_.notify_all();
			if (worker_.joinable()) worker_.join();
			publish_timer_.reset();
			publisher_.reset();
		}

	private:
		void load_configuration()
		{
			const auto profile_path = std::filesystem::path{
				get_parameter("profile_path").as_string()};
			const auto config_root = std::filesystem::path{
				get_parameter("config_root").as_string()};
			command_channel_ = get_parameter("command_channel").as_string();
			feedback_channel_ = get_parameter("feedback_channel").as_string();
			source_id_ = get_parameter("source_id").as_string();
			if (profile_path.empty() || config_root.empty() || command_channel_.empty() ||
				feedback_channel_.empty())
			{
				throw std::invalid_argument(
					"EdgePlanningBridge requires profile_path, config_root and channel names");
			}
			if (source_id_ != "planning")
			{
				throw std::invalid_argument(
					"EdgePlanningBridge only exposes the existing planning source in A2");
			}

			bundle_ = control_link_contract::load_contract_bundle(profile_path, config_root);
			if (!profile_enables_source(*bundle_->profile, source_id_))
			{
				throw std::invalid_argument(
					"planning is not enabled by the selected Profile");
			}
			const auto source_iterator = bundle_->source_policy->sources.find(source_id_);
			if (source_iterator == bundle_->source_policy->sources.end())
			{
				throw std::out_of_range("planning is not defined by SourcePolicy");
			}
			if (source_iterator->second.type !=
				"control_link_interfaces/msg/ControlCommand")
			{
				throw std::invalid_argument(
					"planning SourcePolicy type is not ControlCommand");
			}
			source_topic_ = source_iterator->second.topic;
			control_link_contract::QosFactory qos_factory{bundle_->gateway_contract};
			input_qos_ = qos_factory.make(bundle_->gateway_contract->input.qos_profile);

			const auto &gateway_state_topic =
				bundle_->gateway_contract->state_topics.at("gateway_state");
			const auto &source_status_topic =
				bundle_->gateway_contract->state_topics.at("source_status");
			const auto &vehicle_state_topic =
				bundle_->gateway_contract->state_topics.at("vehicle_state");
			if (!gateway_state_topic.qos_profile.has_value() ||
				!source_status_topic.qos_profile.has_value() ||
				!vehicle_state_topic.qos_profile.has_value())
			{
				throw std::logic_error("EdgePlanningBridge requires state QoS profiles");
			}
			gateway_state_qos_ = qos_factory.make(gateway_state_topic.qos_profile.value());
			source_status_qos_ = qos_factory.make(source_status_topic.qos_profile.value());
			vehicle_state_qos_ = qos_factory.make(vehicle_state_topic.qos_profile.value());
			gateway_state_topic_ = gateway_state_topic.topic;
			source_status_topic_ = source_status_topic.topic;
			vehicle_state_topic_ = vehicle_state_topic.topic;
		}

		void create_feedback_subscriptions()
		{
			gateway_state_subscription_ = create_subscription<GatewayState>(
				gateway_state_topic_, gateway_state_qos_,
				[this](GatewayState::ConstSharedPtr message)
				{
					if (message == nullptr) return;
					const auto received_boot_ns =
						control_link_edge_protocol::boottime_now_ns();
					std::lock_guard<std::mutex> lock(mutex_);
					gateway_state_ = *message;
					gateway_receive_boot_ns_ = received_boot_ns;
				});
			source_status_subscription_ = create_subscription<SourceStatus>(
				source_status_topic_, source_status_qos_,
				[this](SourceStatus::ConstSharedPtr message)
				{
					if (message == nullptr || message->source_id != source_id_) return;
					const auto received_boot_ns =
						control_link_edge_protocol::boottime_now_ns();
					std::lock_guard<std::mutex> lock(mutex_);
					source_status_ = *message;
					source_status_receive_boot_ns_ = received_boot_ns;
				});
			vehicle_state_subscription_ = create_subscription<VehicleState>(
				vehicle_state_topic_, vehicle_state_qos_,
				[this](VehicleState::ConstSharedPtr message)
				{
					if (message == nullptr) return;
					const auto received_boot_ns =
						control_link_edge_protocol::boottime_now_ns();
					std::lock_guard<std::mutex> lock(mutex_);
					vehicle_state_ = *message;
					vehicle_receive_boot_ns_ = received_boot_ns;
				});
		}

		edge_runtime::ChannelOptions command_channel_options() const
		{
			edge_runtime::ChannelOptions options;
			options.name = command_channel_;
			options.stale_timeout = 500ms;
			options.reconnect_timeout = 500ms;
			return options;
		}

		edge_runtime::ChannelOptions feedback_channel_options() const
		{
			edge_runtime::ChannelOptions options;
			options.name = feedback_channel_;
			options.stale_timeout = 500ms;
			options.reconnect_timeout = 500ms;
			options.heartbeat_interval = 100ms;
			return options;
		}

		bool stopping()
		{
			std::lock_guard<std::mutex> lock(mutex_);
			return stop_requested_;
		}

		void wait_for_stop(std::chrono::milliseconds duration)
		{
			std::unique_lock<std::mutex> lock(mutex_);
			stop_condition_.wait_for(lock, duration, [this]() { return stop_requested_; });
		}

		void log_edge_error(const char *operation, const edge_runtime::Error &error)
		{
			const auto now = std::chrono::steady_clock::now();
			if (now - last_edge_log_ < 1s && last_edge_error_code_ == error.code) return;
			last_edge_log_ = now;
			last_edge_error_code_ = error.code;
			RCLCPP_WARN(
				get_logger(), "Edge %s failed: %s", operation, edge_error_text(error).c_str());
		}

		bool begin_new_session_locked(std::uint64_t cutoff_boot_ns)
		{
			// 新 token、BOOTTIME cutoff 和 epoch 必须一次性切换，旧 latest 只能被丢弃。
			SessionToken next_token;
			if (!control_link_edge_protocol::generate_session_token(&next_token))
			{
				fatal_error_ = true;
				pending_command_.reset();
				forwarding_ = false;
				return false;
			}
			if (epoch_ == std::numeric_limits<std::uint64_t>::max())
			{
				fatal_error_ = true;
				pending_command_.reset();
				forwarding_ = false;
				return false;
			}
			session_token_ = next_token;
			session_cutoff_boot_ns_ = cutoff_boot_ns;
			++epoch_;
			publisher_rebuild_requested_ = true;
			pending_command_.reset();
			forwarding_ = false;
			return true;
		}

		void reset_transport_identity_locked()
		{
			transport_identity_.reset();
			last_edge_sequence_ = 0U;
		}

		void try_create_feedback_producer()
		{
			if (feedback_producer_ != nullptr) return;
			auto created = FeedbackProducer::create(
				feedback_channel_options(), edge_runtime::feedback_schema());
			if (!created)
			{
				log_edge_error("feedback producer create", created.error());
				return;
			}
			feedback_producer_ = std::make_unique<FeedbackProducer>(
				std::move(created.value()));
			RCLCPP_INFO(get_logger(), "Edge feedback producer is ready");
		}

		void try_open_command_consumer()
		{
			if (command_consumer_ != nullptr || feedback_producer_ == nullptr) return;
			auto opened = CommandConsumer::open(
				command_channel_options(), edge_runtime::command_schema());
			if (!opened)
			{
				log_edge_error("command consumer open", opened.error());
				return;
			}
			command_consumer_ = std::make_unique<CommandConsumer>(
				std::move(opened.value()));
			{
				std::lock_guard<std::mutex> lock(mutex_);
				reset_transport_identity_locked();
				(void)begin_new_session_locked(control_link_edge_protocol::boottime_now_ns());
			}
			RCLCPP_INFO(get_logger(), "Edge command consumer is connected");
		}

		void reconnect_command_consumer()
		{
			if (command_consumer_ == nullptr) return;
			auto reconnected = command_consumer_->reconnect();
			if (!reconnected)
			{
				log_edge_error("command consumer reconnect", reconnected.error());
				return;
			}
			{
				std::lock_guard<std::mutex> lock(mutex_);
				reset_transport_identity_locked();
				if (!begin_new_session_locked(control_link_edge_protocol::boottime_now_ns()))
				{
					RCLCPP_ERROR(get_logger(), "new Edge session token generation failed");
				}
			}
			RCLCPP_INFO(
				get_logger(), "Edge command consumer reconnected; old session is invalid");
		}

		void handle_command_error(const edge_runtime::Error &error)
		{
			log_edge_error("command read", error);
			{
				std::lock_guard<std::mutex> lock(mutex_);
				pending_command_.reset();
				forwarding_ = false;
			}
			switch (error.code)
			{
			case edge_runtime::ErrorCode::kStaleHandle:
			case edge_runtime::ErrorCode::kProducerOffline:
			case edge_runtime::ErrorCode::kTransportFailed:
			case edge_runtime::ErrorCode::kRecoveryBlocked:
				reconnect_command_consumer();
				break;
			case edge_runtime::ErrorCode::kPayloadCorrupt:
			case edge_runtime::ErrorCode::kPayloadDecodeFailed:
			case edge_runtime::ErrorCode::kSchemaMismatch:
			{
				std::lock_guard<std::mutex> lock(mutex_);
				if (!begin_new_session_locked(control_link_edge_protocol::boottime_now_ns()))
				{
					RCLCPP_ERROR(get_logger(), "protocol error left Edge bridge fail-closed");
				}
				break;
			}
			default:
				break;
			}
		}

		void process_sample(const edge_runtime::Sample<CommandFrame> &sample)
		{
			// transport identity 与 Edge sequence 只做传输去重；业务 sequence 仍由 Gateway 校验。
			const auto now_boot_ns = control_link_edge_protocol::boottime_now_ns();
			if (now_boot_ns == 0U || sample.publish_boot_ns == 0U ||
				sample.publish_boot_ns > now_boot_ns)
			{
				std::lock_guard<std::mutex> lock(mutex_);
				pending_command_.reset();
				forwarding_ = false;
				return;
			}

			const TransportIdentity identity{sample.generation, sample.instance_nonce};
			std::lock_guard<std::mutex> lock(mutex_);
			if (!transport_identity_.has_value() || transport_identity_.value() != identity)
			{
				transport_identity_ = identity;
				last_edge_sequence_ = 0U;
				if (!begin_new_session_locked(now_boot_ns))
				{
					RCLCPP_ERROR(get_logger(), "Edge transport identity changed but session renewal failed");
				}
				return;
			}
			if (sample.publish_boot_ns <= session_cutoff_boot_ns_ ||
				sample.sequence <= last_edge_sequence_ ||
				sample.value.session_token != session_token_)
			{
				return;
			}

			builtin_interfaces::msg::Time source_stamp;
			if (!to_ros_time(sample.value.source_stamp_ns, &source_stamp))
			{
				pending_command_.reset();
				forwarding_ = false;
				return;
			}
			last_edge_sequence_ = sample.sequence;
			pending_command_ = PendingCommand{
				CommandFrame{sample.value}, sample.sequence, epoch_};
			pending_command_->frame = sample.value;
			forwarding_ = false;
		}

		void publish_pending_command()
		{
			// publisher 切换和实际 publish 同一把锁，避免检查旧 epoch 后向新资源发送。
			std::lock_guard<std::mutex> lock(mutex_);
			if (publisher_rebuild_requested_ && publisher_epoch_ != epoch_)
			{
				try
				{
					publisher_.reset();
					publisher_ = create_publisher<ControlCommand>(source_topic_, input_qos_);
					publisher_epoch_ = epoch_;
					publisher_rebuild_requested_ = false;
					fatal_error_ = false;
				}
				catch (const std::exception &exception)
				{
					fatal_error_ = true;
					RCLCPP_ERROR(
						get_logger(), "Edge publisher rebuild failed: %s", exception.what());
					return;
				}
			}
			if (fatal_error_ || !publisher_ || !pending_command_.has_value() ||
				publisher_epoch_ != epoch_ || pending_command_->epoch != epoch_ ||
				pending_command_->frame.session_token != session_token_)
			{
				return;
			}

			ControlCommand command;
			if (!to_ros_time(
				pending_command_->frame.source_stamp_ns, &command.source_stamp))
			{
				pending_command_.reset();
				return;
			}
			command.source_id = source_id_;
			command.source_sequence = pending_command_->frame.command_sequence;
			command.mode = static_cast<std::uint8_t>(pending_command_->frame.mode);
			command.linear_velocity_mps = pending_command_->frame.linear_velocity_mps;
			command.angular_velocity_radps = pending_command_->frame.angular_velocity_radps;
			try
			{
				publisher_->publish(command);
				forwarding_ = true;
				pending_command_.reset();
			}
			catch (const std::exception &exception)
			{
				pending_command_.reset();
				forwarding_ = false;
				fatal_error_ = true;
				RCLCPP_ERROR(get_logger(), "Edge command publish failed: %s", exception.what());
			}
		}

		FeedbackFrame make_feedback_frame()
		{
			// 每个 ROS 状态保留 observed_at 与本地 BOOTTIME 接收时间，缺失组保持 invalid。
			FeedbackFrame frame;
			const auto now_boot_ns = control_link_edge_protocol::boottime_now_ns();
			std::lock_guard<std::mutex> lock(mutex_);
			frame.session_token = session_token_;
			if (feedback_sequence_ == std::numeric_limits<std::uint64_t>::max())
			{
				fatal_error_ = true;
			}
			frame.feedback_sequence = ++feedback_sequence_;
			frame.bridge_publish_boot_ns = now_boot_ns;
			frame.transport_state = fatal_error_
				? control_link_edge_protocol::TransportState::kProtocolError
				: (forwarding_
					? control_link_edge_protocol::TransportState::kForwarding
					: control_link_edge_protocol::TransportState::kWaitingForCommand);
			if (gateway_state_.has_value())
			{
				frame.gateway_state = gateway_state_->state;
				frame.gateway_reason_code = gateway_state_->reason_code;
				frame.active_source_sequence = gateway_state_->active_source_sequence;
				frame.active_source = active_source_from_id(gateway_state_->active_source_id);
				if (ros_time_ns(gateway_state_->observed_at) != 0U &&
					gateway_receive_boot_ns_ != 0U)
				{
					frame.validity_flags |= control_link_edge_protocol::kGatewayStateValid;
					frame.gateway_observed_ros_ns = ros_time_ns(gateway_state_->observed_at);
					frame.gateway_receive_boot_ns = gateway_receive_boot_ns_;
				}
			}
			if (source_status_.has_value())
			{
				frame.planning_accepted_count = source_status_->accepted_count;
				frame.planning_rejected_count = source_status_->rejected_count;
				frame.planning_reject_reason = source_status_->last_reject_reason;
				if (ros_time_ns(source_status_->observed_at) != 0U &&
					source_status_receive_boot_ns_ != 0U)
				{
					frame.validity_flags |= control_link_edge_protocol::kPlanningStatusValid;
					frame.planning_observed_ros_ns = ros_time_ns(source_status_->observed_at);
					frame.planning_receive_boot_ns = source_status_receive_boot_ns_;
				}
			}
			if (vehicle_state_.has_value())
			{
				frame.vehicle_state = vehicle_state_->state;
				frame.vehicle_fault_code = vehicle_state_->fault_code;
				frame.vehicle_linear_velocity_mps = vehicle_state_->linear_velocity_mps;
				frame.vehicle_angular_velocity_radps = vehicle_state_->angular_velocity_radps;
				if (ros_time_ns(vehicle_state_->observed_at) != 0U &&
					vehicle_receive_boot_ns_ != 0U)
				{
					frame.validity_flags |= control_link_edge_protocol::kVehicleStateValid;
					frame.vehicle_observed_ros_ns = ros_time_ns(vehicle_state_->observed_at);
					frame.vehicle_receive_boot_ns = vehicle_receive_boot_ns_;
				}
			}
			if (!fatal_error_)
			{
				frame.validity_flags |= control_link_edge_protocol::kSessionReady;
			}
			if (transport_identity_.has_value())
			{
				frame.transport_generation = transport_identity_->generation;
				frame.transport_instance_nonce = transport_identity_->instance_nonce;
			}
			frame.command_edge_sequence = last_edge_sequence_;
			return frame;
		}

		void publish_feedback()
		{
			if (feedback_producer_ == nullptr) return;
			const auto frame = make_feedback_frame();
			auto published = feedback_producer_->publish(frame);
			if (!published)
			{
				log_edge_error("feedback publish", published.error());
				std::lock_guard<std::mutex> lock(mutex_);
				fatal_error_ = true;
				pending_command_.reset();
				forwarding_ = false;
			}
		}

		void edge_worker_loop()
		{
			auto next_feedback = std::chrono::steady_clock::now();
			auto next_heartbeat = next_feedback;
			auto next_reconnect = next_feedback;
			while (!stopping())
			{
				try_create_feedback_producer();
				try_open_command_consumer();
				if (command_consumer_ != nullptr)
				{
					auto sample = command_consumer_->wait_latest(50ms);
					if (sample)
					{
						process_sample(sample.value());
					}
					else if (sample.error().code != edge_runtime::ErrorCode::kNoNewSample &&
							sample.error().code != edge_runtime::ErrorCode::kTimeout &&
							std::chrono::steady_clock::now() >= next_reconnect)
					{
						next_reconnect = std::chrono::steady_clock::now() + 100ms;
						handle_command_error(sample.error());
					}
				}
				const auto now = std::chrono::steady_clock::now();
				if (feedback_producer_ != nullptr && now >= next_feedback)
				{
					publish_feedback();
					next_feedback = now + 100ms;
				}
				if (feedback_producer_ != nullptr && now >= next_heartbeat)
				{
					auto heartbeat = feedback_producer_->heartbeat();
					if (!heartbeat) log_edge_error("feedback heartbeat", heartbeat.error());
					next_heartbeat = now + 100ms;
				}
				if (feedback_producer_ == nullptr) wait_for_stop(100ms);
			}
			if (feedback_producer_ != nullptr)
			{
				const auto removed = feedback_producer_->remove_if_owner();
				if (!removed)
				{
					log_edge_error("feedback producer cleanup", removed.error());
				}
			}
			command_consumer_.reset();
			feedback_producer_.reset();
		}

		control_link_contract::ContractBundlePtr bundle_;
		std::string source_id_;
		std::string source_topic_;
		std::string command_channel_;
		std::string feedback_channel_;
		std::string gateway_state_topic_;
		std::string source_status_topic_;
		std::string vehicle_state_topic_;
		rclcpp::QoS input_qos_{rclcpp::KeepLast(1U)};
		rclcpp::QoS gateway_state_qos_{rclcpp::KeepLast(1U)};
		rclcpp::QoS source_status_qos_{rclcpp::KeepLast(1U)};
		rclcpp::QoS vehicle_state_qos_{rclcpp::KeepLast(1U)};

		rclcpp::Publisher<ControlCommand>::SharedPtr publisher_;
		rclcpp::Subscription<GatewayState>::SharedPtr gateway_state_subscription_;
		rclcpp::Subscription<SourceStatus>::SharedPtr source_status_subscription_;
		rclcpp::Subscription<VehicleState>::SharedPtr vehicle_state_subscription_;
		rclcpp::TimerBase::SharedPtr publish_timer_;

		std::unique_ptr<CommandConsumer> command_consumer_;
		std::unique_ptr<FeedbackProducer> feedback_producer_;
		std::thread worker_;
		std::mutex mutex_;
		std::condition_variable stop_condition_;
		bool stop_requested_{false};
		bool fatal_error_{false};

		SessionToken session_token_{};
		std::uint64_t session_cutoff_boot_ns_{0U};
		std::uint64_t epoch_{1U};
		std::uint64_t publisher_epoch_{1U};
		bool publisher_rebuild_requested_{false};
		std::optional<TransportIdentity> transport_identity_;
		std::uint64_t last_edge_sequence_{0U};
		bool forwarding_{false};
		std::optional<PendingCommand> pending_command_;

		std::optional<GatewayState> gateway_state_;
		std::uint64_t gateway_receive_boot_ns_{0U};
		std::optional<SourceStatus> source_status_;
		std::uint64_t source_status_receive_boot_ns_{0U};
		std::optional<VehicleState> vehicle_state_;
		std::uint64_t vehicle_receive_boot_ns_{0U};
		std::uint64_t feedback_sequence_{0U};

		std::chrono::steady_clock::time_point last_edge_log_{};
		edge_runtime::ErrorCode last_edge_error_code_{edge_runtime::ErrorCode::kSystemError};
	};
}

int main(int argc, char **argv)
{
	rclcpp::init(argc, argv);
	try
	{
		auto node = std::make_shared<EdgePlanningBridge>();
		rclcpp::spin(node);
	}
	catch (const std::exception &exception)
	{
		RCLCPP_FATAL(
			rclcpp::get_logger("edge_planning_bridge"),
			"Edge planning bridge failed: %s", exception.what());
		rclcpp::shutdown();
		return 1;
	}
	rclcpp::shutdown();
	return 0;
}
