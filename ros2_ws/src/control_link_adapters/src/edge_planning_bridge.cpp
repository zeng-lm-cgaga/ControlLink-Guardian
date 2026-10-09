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
#include <vector>

#include "builtin_interfaces/msg/time.hpp"
#include "control_link_adapters/graph_admission_policy.hpp"
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
#include "rmw/types.h"

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

	using control_link_adapters::GraphAdmissionProbeResult;

	bool graph_publisher_set_matches(
		const std::vector<rclcpp::TopicEndpointInfo> &publishers,
		bool verify_own_publisher,
		bool expected_gid_valid,
		const std::array<std::uint8_t, RMW_GID_STORAGE_SIZE> &expected_gid) noexcept
	{
		if (!verify_own_publisher)
		{
			return publishers.empty();
		}

		return expected_gid_valid && publishers.size() == 1U &&
			publishers.front().endpoint_gid() == expected_gid;
	}

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
			declare_parameter<std::int64_t>("graph_admission_timeout_ms", 30'000);
			declare_parameter<std::int64_t>("graph_admission_quiet_ms", 250);
			declare_parameter<std::int64_t>("graph_admission_poll_ms", 50);
			declare_parameter<std::int64_t>("gateway_readiness_timeout_ms", 1'000);

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

				{
					std::lock_guard<std::mutex> lock(mutex_);
					publisher_rebuild_requested_ = true;
					graph_admission_phase_ = "startup";
				}

			feedback_callback_group_ = create_callback_group(
				rclcpp::CallbackGroupType::MutuallyExclusive);
			publisher_callback_group_ = create_callback_group(
				rclcpp::CallbackGroupType::MutuallyExclusive);
			create_feedback_subscriptions();
			publish_timer_ = create_wall_timer(
				5ms, [this]() { publish_pending_command(); }, publisher_callback_group_);
			worker_ = std::thread([this]() { edge_worker_loop(); });

			RCLCPP_INFO(
				get_logger(),
				"Edge planning bridge started: source=%s topic=%s command_channel=%s feedback_channel=%s; "
				"Graph admission is asynchronous",
				source_id_.c_str(), source_topic_.c_str(), command_channel_.c_str(),
				feedback_channel_.c_str());
		}

		~EdgePlanningBridge() override
		{
			{
				std::lock_guard<std::mutex> lock(mutex_);
				stop_requested_ = true;
				graph_admission_pending_ = false;
				graph_admission_verify_own_publisher_ = false;
				publisher_rebuild_requested_ = false;
				gateway_source_ready_ = false;
			}
			publish_timer_.reset();
			stop_condition_.notify_all();
			if (worker_.joinable()) worker_.join();
			{
				std::lock_guard<std::mutex> lock(mutex_);
				publisher_.reset();
				publisher_gid_valid_ = false;
			}
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
		const auto graph_admission_timeout_ms =
			get_parameter("graph_admission_timeout_ms").as_int();
		const auto graph_admission_quiet_ms =
			get_parameter("graph_admission_quiet_ms").as_int();
		const auto graph_admission_poll_ms =
			get_parameter("graph_admission_poll_ms").as_int();
		const auto gateway_readiness_timeout_ms =
			get_parameter("gateway_readiness_timeout_ms").as_int();
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
		if (graph_admission_timeout_ms <= 0 || graph_admission_timeout_ms > 120'000 ||
			graph_admission_quiet_ms <= 0 ||
			graph_admission_quiet_ms > graph_admission_timeout_ms ||
			graph_admission_poll_ms <= 0 || graph_admission_poll_ms > 1'000)
		{
			throw std::invalid_argument(
				"EdgePlanningBridge graph admission timing parameters are out of range");
		}
		if (gateway_readiness_timeout_ms <= 0 ||
			gateway_readiness_timeout_ms > graph_admission_timeout_ms)
		{
			throw std::invalid_argument(
				"EdgePlanningBridge gateway readiness timeout is out of range");
		}
		graph_admission_timeout_ = std::chrono::milliseconds(graph_admission_timeout_ms);
		graph_admission_quiet_ = std::chrono::milliseconds(graph_admission_quiet_ms);
			graph_admission_poll_ = std::chrono::milliseconds(graph_admission_poll_ms);
		gateway_readiness_timeout_ =
			std::chrono::milliseconds(gateway_readiness_timeout_ms);

			bundle_ = control_link_contract::load_contract_bundle(profile_path, config_root);
			const auto gateway_stable_ms =
				bundle_->gateway_contract->gateway.graph_stable_window_ms;
			const auto gateway_poll_ms = bundle_->gateway_contract->gateway.graph_poll_ms;
			if (gateway_stable_ms >
				std::numeric_limits<std::uint64_t>::max() - gateway_poll_ms)
			{
				throw std::out_of_range(
					"Gateway Graph stability window cannot be represented by bridge timing");
			}
			const auto gateway_admission_quiet_ms = gateway_stable_ms + gateway_poll_ms;
			if (gateway_admission_quiet_ms >
				static_cast<std::uint64_t>(graph_admission_timeout_ms))
			{
				throw std::invalid_argument(
					"graph_admission_timeout_ms is shorter than the Gateway Graph stability window");
			}
			graph_admission_quiet_ = std::max(
				graph_admission_quiet_,
				std::chrono::milliseconds{
					static_cast<std::chrono::milliseconds::rep>(gateway_admission_quiet_ms)});
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

		// 只有 Gateway 在稳定窗口后报告了当前 bridge GID，才允许对外宣告 SessionReady。
		// 本地 Graph 的空窗/自有 GID 检查不能替代 Gateway 的实际准入结论。
		bool gateway_source_status_matches_publisher_locked(
			const SourceStatus &status) const noexcept
		{
			if (publisher_rebuild_requested_ || !publisher_gid_valid_ ||
				publisher_ == nullptr || !status.endpoint_stable ||
				status.endpoint_state != SourceStatus::ENDPOINT_USABLE ||
				status.endpoint_publisher_count != 1U ||
				!status.endpoint_generation_valid)
			{
				return false;
			}
			return std::equal(
				status.endpoint_publisher_gid.begin(),
				status.endpoint_publisher_gid.end(),
				publisher_gid_.begin());
		}

		void create_feedback_subscriptions()
		{
			rclcpp::SubscriptionOptions subscription_options;
			subscription_options.callback_group = feedback_callback_group_;
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
					}, subscription_options);
			source_status_subscription_ = create_subscription<SourceStatus>(
				source_status_topic_, source_status_qos_,
				[this](SourceStatus::ConstSharedPtr message)
				{
					if (message == nullptr || message->source_id != source_id_) return;
					const auto received_boot_ns =
						control_link_edge_protocol::boottime_now_ns();
					bool readiness_changed = false;
					bool ready = false;
					{
						std::lock_guard<std::mutex> lock(mutex_);
						source_status_ = *message;
						source_status_receive_boot_ns_ = received_boot_ns;
						ready = gateway_source_status_matches_publisher_locked(*message);
						readiness_changed = ready != gateway_source_ready_;
						gateway_source_ready_ = ready;
					}
					if (readiness_changed)
					{
						RCLCPP_INFO(
							get_logger(),
							"Gateway source admission changed: ready=%s endpoint_state=%u "
							"publishers=%u generation_match=%s",
							ready ? "true" : "false",
							static_cast<unsigned int>(message->endpoint_state),
							static_cast<unsigned int>(message->endpoint_publisher_count),
							ready ? "true" : "false");
					}
					}, subscription_options);
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
					}, subscription_options);
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

		void wait_for_stop_until(std::chrono::steady_clock::time_point deadline)
		{
			std::unique_lock<std::mutex> lock(mutex_);
			stop_condition_.wait_until(lock, deadline, [this]() { return stop_requested_; });
		}

		void start_graph_admission_locked(
			const char *phase,
			bool verify_own_publisher = false)
		{
			const auto now = std::chrono::steady_clock::now();
			graph_admission_pending_ = true;
			graph_admission_phase_ = phase != nullptr ? phase : "unknown";
			graph_admission_epoch_ = epoch_;
			graph_admission_verify_own_publisher_ = verify_own_publisher;
			graph_admission_started_ = now;
			graph_admission_deadline_ = now + graph_admission_timeout_;
			graph_admission_quiet_since_ =
				std::chrono::steady_clock::time_point::min();
			graph_admission_next_probe_ = now;
			graph_admission_next_report_ = now;
			graph_admission_last_publisher_count_ = std::numeric_limits<std::size_t>::max();
			graph_admission_zero_logged_ = false;

			RCLCPP_INFO(
				get_logger(),
				"Graph admission waiting: phase=%s topic=%s timeout_ms=%lld quiet_ms=%lld",
				graph_admission_phase_.c_str(), source_topic_.c_str(),
				static_cast<long long>(graph_admission_timeout_.count()),
				static_cast<long long>(graph_admission_quiet_.count()));
		}

		GraphAdmissionProbeResult handle_graph_admission_probe_failure(
			const std::string &phase,
			std::uint64_t admission_epoch,
			std::chrono::steady_clock::time_point deadline,
			bool final_probe)
		{
			const auto failure_now = std::chrono::steady_clock::now();
			GraphAdmissionProbeResult result;
			{
				std::lock_guard<std::mutex> lock(mutex_);
				result = control_link_adapters::graph_admission_probe_failure_result(
					failure_now,
					deadline,
					graph_admission_pending_,
					graph_admission_epoch_ == admission_epoch,
					!rclcpp::ok() || stop_requested_);
				if (result == GraphAdmissionProbeResult::kWaiting)
				{
					graph_admission_next_probe_ = failure_now + graph_admission_poll_;
					// 查询失败使此前 quiet-window 观测失去连续性，必须重新稳定观察。
					graph_admission_quiet_since_ =
						std::chrono::steady_clock::time_point::min();
					graph_admission_zero_logged_ = false;
				}
			}
			if (result == GraphAdmissionProbeResult::kTimedOut)
			{
				RCLCPP_ERROR(
					get_logger(),
					"Graph admission timeout after %sprobe exception: phase=%s; "
					"staying fail-closed",
					final_probe ? "final " : "",
					phase.c_str());
			}
			return result;
		}

		GraphAdmissionProbeResult probe_graph_admission()
		{
			std::string phase;
			std::uint64_t admission_epoch = 0U;
			bool verify_own_publisher = false;
			bool expected_gid_valid = false;
			std::array<std::uint8_t, RMW_GID_STORAGE_SIZE> expected_gid{};
			std::chrono::steady_clock::time_point deadline;
			{
				std::lock_guard<std::mutex> lock(mutex_);
				if (!graph_admission_pending_)
				{
					return GraphAdmissionProbeResult::kReady;
				}
				const auto now = std::chrono::steady_clock::now();
				if (now < graph_admission_next_probe_)
				{
					return GraphAdmissionProbeResult::kWaiting;
				}
				phase = graph_admission_phase_;
				admission_epoch = graph_admission_epoch_;
				verify_own_publisher = graph_admission_verify_own_publisher_;
				expected_gid_valid = publisher_gid_valid_;
				if (expected_gid_valid)
				{
					expected_gid = publisher_gid_;
				}
				deadline = graph_admission_deadline_;
			}

			if (!rclcpp::ok() || stopping())
			{
				return GraphAdmissionProbeResult::kStopped;
			}

			std::vector<rclcpp::TopicEndpointInfo> publishers;
			try
			{
				publishers = get_publishers_info_by_topic(source_topic_);
			}
			catch (const std::exception &exception)
			{
				RCLCPP_WARN_THROTTLE(
					get_logger(), *get_clock(), 1000,
					"Graph admission probe failed for phase=%s: %s",
					phase.c_str(), exception.what());
				return handle_graph_admission_probe_failure(
					phase, admission_epoch, deadline, false);
			}

			const auto now = std::chrono::steady_clock::now();
			bool final_probe = false;
			bool timed_out = now >= deadline;
			bool report_zero = false;
			bool report_blocked = false;
			std::size_t publisher_count = publishers.size();
			{
				std::lock_guard<std::mutex> lock(mutex_);
				if (!graph_admission_pending_ || graph_admission_epoch_ != admission_epoch)
				{
					return GraphAdmissionProbeResult::kSuperseded;
				}
				graph_admission_next_probe_ = now + graph_admission_poll_;
				if (graph_publisher_set_matches(
					publishers, verify_own_publisher, expected_gid_valid, expected_gid))
				{
					if (graph_admission_quiet_since_ ==
						std::chrono::steady_clock::time_point::min())
					{
						graph_admission_quiet_since_ = now;
					}
					if (!graph_admission_zero_logged_)
					{
						graph_admission_zero_logged_ = true;
						report_zero = true;
					}
					final_probe = control_link_adapters::graph_admission_quiet_window_complete(
						true,
						graph_admission_quiet_since_,
						now,
						graph_admission_quiet_,
						graph_admission_deadline_);
				}
				else
				{
					graph_admission_quiet_since_ =
						std::chrono::steady_clock::time_point::min();
					graph_admission_zero_logged_ = false;
					if (publishers.size() != graph_admission_last_publisher_count_ ||
						now >= graph_admission_next_report_)
					{
						graph_admission_last_publisher_count_ = publishers.size();
						graph_admission_next_report_ = now + 1s;
						report_blocked = true;
					}
				}
			}

			if (report_zero)
			{
				RCLCPP_INFO(
					get_logger(),
					"Graph admission observed expected publisher set; settling: phase=%s "
					"verify_own_publisher=%s",
					phase.c_str(), verify_own_publisher ? "true" : "false");
			}
			if (report_blocked)
			{
				RCLCPP_WARN(
					get_logger(),
					"Graph admission blocked: phase=%s planning_publishers=%zu",
					phase.c_str(), publisher_count);
			}

			if (final_probe && !timed_out && rclcpp::ok() && !stopping())
			{
				std::vector<rclcpp::TopicEndpointInfo> final_publishers;
				try
				{
					final_publishers = get_publishers_info_by_topic(source_topic_);
				}
				catch (const std::exception &exception)
				{
					RCLCPP_WARN_THROTTLE(
						get_logger(), *get_clock(), 1000,
						"Final Graph admission probe failed for phase=%s: %s",
						phase.c_str(), exception.what());
					return handle_graph_admission_probe_failure(
						phase, admission_epoch, deadline, true);
				}
				const auto final_now = std::chrono::steady_clock::now();
				if (!rclcpp::ok() || stopping())
				{
					return GraphAdmissionProbeResult::kStopped;
				}
				std::lock_guard<std::mutex> lock(mutex_);
				if (!graph_admission_pending_ || graph_admission_epoch_ != admission_epoch)
				{
					return GraphAdmissionProbeResult::kSuperseded;
				}
				if (graph_publisher_set_matches(
						final_publishers,
						verify_own_publisher,
						expected_gid_valid,
						expected_gid) &&
					final_now < graph_admission_deadline_)
				{
					graph_admission_pending_ = false;
					if (graph_admission_verify_own_publisher_)
					{
						publisher_rebuild_requested_ = false;
						graph_admission_verify_own_publisher_ = false;
						gateway_source_ready_ = false;
						publisher_graph_ready_boot_ns_ =
							control_link_edge_protocol::boottime_now_ns();
						RCLCPP_INFO(
							get_logger(),
							"Local publisher Graph admission complete; waiting for Gateway "
							"stable generation before SessionReady");
					}
					const auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
						final_now - graph_admission_started_);
					RCLCPP_INFO(
						get_logger(),
						"Graph admission ready: phase=%s elapsed_ms=%lld",
						phase.c_str(), static_cast<long long>(elapsed.count()));
					return GraphAdmissionProbeResult::kReady;
				}
				publisher_count = final_publishers.size();
				graph_admission_quiet_since_ =
					std::chrono::steady_clock::time_point::min();
				graph_admission_zero_logged_ = false;
				graph_admission_next_probe_ = final_now + graph_admission_poll_;
				timed_out = final_now >= graph_admission_deadline_;
			}

			if (timed_out)
			{
				RCLCPP_ERROR(
					get_logger(),
					"Graph admission timeout: phase=%s planning_publishers=%zu; staying fail-closed",
					phase.c_str(), publisher_count);
				return GraphAdmissionProbeResult::kTimedOut;
			}
			return GraphAdmissionProbeResult::kWaiting;
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
			gateway_source_ready_ = false;
			publisher_graph_ready_boot_ns_ = 0U;
			source_status_.reset();
			source_status_receive_boot_ns_ = 0U;
			graph_admission_pending_ = false;
			graph_admission_phase_ = "publisher_rebuild";
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

		bool try_open_command_consumer()
		{
			if (command_consumer_ != nullptr || feedback_producer_ == nullptr) return false;
			auto opened = CommandConsumer::open(
				command_channel_options(), edge_runtime::command_schema());
			if (!opened)
			{
				log_edge_error("command consumer open", opened.error());
				return false;
			}
			command_consumer_ = std::make_unique<CommandConsumer>(
				std::move(opened.value()));
			{
				std::lock_guard<std::mutex> lock(mutex_);
				reset_transport_identity_locked();
				(void)begin_new_session_locked(control_link_edge_protocol::boottime_now_ns());
			}
			RCLCPP_INFO(get_logger(), "Edge command consumer is connected");
			return true;
		}

		bool reconnect_command_consumer()
		{
			if (command_consumer_ == nullptr) return false;
			auto reconnected = command_consumer_->reconnect();
			if (!reconnected)
			{
				log_edge_error("command consumer reconnect", reconnected.error());
				return false;
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
			return true;
		}

		bool handle_command_error(const edge_runtime::Error &error)
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
				return reconnect_command_consumer();
			case edge_runtime::ErrorCode::kPayloadCorrupt:
			case edge_runtime::ErrorCode::kPayloadDecodeFailed:
			case edge_runtime::ErrorCode::kSchemaMismatch:
			{
				std::lock_guard<std::mutex> lock(mutex_);
				const bool session_reset =
					begin_new_session_locked(control_link_edge_protocol::boottime_now_ns());
				if (!session_reset)
				{
					RCLCPP_ERROR(get_logger(), "protocol error left Edge bridge fail-closed");
				}
				return false;
			}
			default:
				return false;
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
			if (!rclcpp::ok() || stopping()) return;

			{
				std::lock_guard<std::mutex> lock(mutex_);
				if (fatal_error_) return;
				if (publisher_rebuild_requested_ && !graph_admission_pending_)
				{
					// 先释放旧 GID，再由后续 timer tick 在锁外做一次有限 Graph 探测。
					publisher_.reset();
					publisher_gid_valid_ = false;
					gateway_source_ready_ = false;
					publisher_graph_ready_boot_ns_ = 0U;
					start_graph_admission_locked(graph_admission_phase_.c_str());
				}
			}

			GraphAdmissionProbeResult admission = GraphAdmissionProbeResult::kReady;
			std::uint64_t admission_epoch = 0U;
			{
				std::lock_guard<std::mutex> lock(mutex_);
				if (publisher_rebuild_requested_ && graph_admission_pending_)
				{
					admission = GraphAdmissionProbeResult::kWaiting;
					admission_epoch = graph_admission_epoch_;
				}
			}
			if (admission == GraphAdmissionProbeResult::kWaiting)
			{
				admission = probe_graph_admission();
				if (admission == GraphAdmissionProbeResult::kStopped ||
					admission == GraphAdmissionProbeResult::kSuperseded ||
					admission == GraphAdmissionProbeResult::kWaiting)
				{
					return;
				}
				if (admission == GraphAdmissionProbeResult::kTimedOut)
				{
					std::lock_guard<std::mutex> lock(mutex_);
					if (!control_link_adapters::graph_admission_timeout_commit_allowed(
							graph_admission_pending_, graph_admission_epoch_, admission_epoch) ||
						!rclcpp::ok() || stop_requested_)
					{
						return;
					}
					graph_admission_pending_ = false;
					graph_admission_verify_own_publisher_ = false;
					fatal_error_ = true;
					publisher_rebuild_requested_ = false;
					publisher_.reset();
					publisher_gid_valid_ = false;
					gateway_source_ready_ = false;
					publisher_graph_ready_boot_ns_ = 0U;
					pending_command_.reset();
					forwarding_ = false;
					return;
				}
			}

			{
				// publisher 切换和实际 publish 同一把锁，避免检查旧 epoch 后向新资源发送。
				std::lock_guard<std::mutex> lock(mutex_);
				if (!rclcpp::ok() || fatal_error_ ||
					(publisher_rebuild_requested_ && graph_admission_pending_))
				{
					return;
				}
				if (publisher_rebuild_requested_)
				{
					if (publisher_ != nullptr)
					{
						// 当前 publisher 仍处于 post-create GID verification，不能发布。
						return;
					}
					try
					{
						publisher_ = create_publisher<ControlCommand>(source_topic_, input_qos_);
						publisher_epoch_ = epoch_;
						const auto &gid = publisher_->get_gid();
						std::copy_n(
							gid.data,
							RMW_GID_STORAGE_SIZE,
							publisher_gid_.begin());
						publisher_gid_valid_ = true;
						// 创建后再次要求 Graph 只看到本次 publisher 的 GID，
						// 并保持至少 Gateway 的 stable window，避免旧 lease 造成 TOCTOU。
						start_graph_admission_locked("publisher_verify", true);
						return;
					}
					catch (const std::exception &exception)
					{
						publisher_.reset();
						publisher_gid_valid_ = false;
						gateway_source_ready_ = false;
						publisher_graph_ready_boot_ns_ = 0U;
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
			const bool source_status_current =
				gateway_source_ready_ && publisher_graph_ready_boot_ns_ != 0U &&
				source_status_receive_boot_ns_ >= publisher_graph_ready_boot_ns_ &&
				now_boot_ns >= source_status_receive_boot_ns_ &&
				now_boot_ns - source_status_receive_boot_ns_ <=
					static_cast<std::uint64_t>(gateway_readiness_timeout_.count()) * 1'000'000U;
			const bool gateway_state_current =
				gateway_state_.has_value() && gateway_receive_boot_ns_ != 0U &&
				now_boot_ns >= gateway_receive_boot_ns_ &&
				now_boot_ns - gateway_receive_boot_ns_ <=
					static_cast<std::uint64_t>(gateway_readiness_timeout_.count()) * 1'000'000U &&
					gateway_state_->state != GatewayState::ERROR;
			if (!fatal_error_ && !publisher_rebuild_requested_ && publisher_ != nullptr &&
				source_status_current && gateway_state_current)
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
			auto next_command_open = next_feedback;
			auto command_open_backoff = 20ms;
			auto command_reconnect_backoff = 100ms;
			while (!stopping())
			{
				try_create_feedback_producer();
				const auto before_command_open = std::chrono::steady_clock::now();
				if (feedback_producer_ != nullptr && command_consumer_ == nullptr &&
					before_command_open >= next_command_open)
				{
					if (try_open_command_consumer())
					{
						command_open_backoff = 20ms;
						command_reconnect_backoff = 100ms;
						next_command_open = before_command_open;
						next_reconnect = before_command_open;
					}
					else
					{
						next_command_open = before_command_open + command_open_backoff;
						command_open_backoff = std::min(command_open_backoff * 2, 1000ms);
					}
				}
				const auto before_command_wait = std::chrono::steady_clock::now();
				if (command_consumer_ != nullptr && before_command_wait >= next_reconnect)
				{
					auto sample = command_consumer_->wait_latest(50ms);
					if (sample)
					{
						process_sample(sample.value());
						command_reconnect_backoff = 100ms;
						next_reconnect = std::chrono::steady_clock::now();
					}
					else if (sample.error().code != edge_runtime::ErrorCode::kNoNewSample &&
							sample.error().code != edge_runtime::ErrorCode::kTimeout &&
							std::chrono::steady_clock::now() >= next_reconnect)
					{
						const auto attempt_at = std::chrono::steady_clock::now();
						const bool recovered = handle_command_error(sample.error());
						if (recovered)
						{
							command_reconnect_backoff = 100ms;
							next_reconnect = attempt_at;
						}
						else
						{
							next_reconnect = attempt_at + command_reconnect_backoff;
							command_reconnect_backoff = std::min(
								command_reconnect_backoff * 2, 1000ms);
						}
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
				if (feedback_producer_ == nullptr)
				{
					wait_for_stop(100ms);
				}
				else
				{
					const auto wake_at = command_consumer_ == nullptr
						? std::min(std::min(next_feedback, next_heartbeat), next_command_open)
						: std::min(std::min(next_feedback, next_heartbeat), next_reconnect);
					if (wake_at > now) wait_for_stop_until(wake_at);
				}
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
		std::chrono::milliseconds graph_admission_timeout_{30'000};
		std::chrono::milliseconds graph_admission_quiet_{250};
		std::chrono::milliseconds graph_admission_poll_{50};
		std::chrono::milliseconds gateway_readiness_timeout_{1'000};
		rclcpp::QoS input_qos_{rclcpp::KeepLast(1U)};
		rclcpp::QoS gateway_state_qos_{rclcpp::KeepLast(1U)};
		rclcpp::QoS source_status_qos_{rclcpp::KeepLast(1U)};
		rclcpp::QoS vehicle_state_qos_{rclcpp::KeepLast(1U)};

			rclcpp::Publisher<ControlCommand>::SharedPtr publisher_;
		rclcpp::Subscription<GatewayState>::SharedPtr gateway_state_subscription_;
		rclcpp::Subscription<SourceStatus>::SharedPtr source_status_subscription_;
		rclcpp::Subscription<VehicleState>::SharedPtr vehicle_state_subscription_;
		rclcpp::TimerBase::SharedPtr publish_timer_;
		rclcpp::CallbackGroup::SharedPtr feedback_callback_group_;
		rclcpp::CallbackGroup::SharedPtr publisher_callback_group_;

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
		std::uint64_t publisher_epoch_{0U};
		bool publisher_rebuild_requested_{false};
		bool graph_admission_pending_{false};
		bool graph_admission_verify_own_publisher_{false};
		std::string graph_admission_phase_;
		std::uint64_t graph_admission_epoch_{0U};
		std::chrono::steady_clock::time_point graph_admission_started_{};
		std::chrono::steady_clock::time_point graph_admission_deadline_{};
		std::chrono::steady_clock::time_point graph_admission_quiet_since_{
			std::chrono::steady_clock::time_point::min()};
		std::chrono::steady_clock::time_point graph_admission_next_probe_{};
		std::chrono::steady_clock::time_point graph_admission_next_report_{};
		std::size_t graph_admission_last_publisher_count_{
			std::numeric_limits<std::size_t>::max()};
		bool graph_admission_zero_logged_{false};
		std::array<std::uint8_t, RMW_GID_STORAGE_SIZE> publisher_gid_{};
		bool publisher_gid_valid_{false};
		std::uint64_t publisher_graph_ready_boot_ns_{0U};
		bool gateway_source_ready_{false};
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
		rclcpp::executors::MultiThreadedExecutor executor(
			rclcpp::ExecutorOptions(), 2U);
		executor.add_node(node);
		executor.spin();
		executor.remove_node(node);
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
