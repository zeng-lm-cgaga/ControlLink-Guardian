#include "control_link_edge_protocol/client.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstring>
#include <cstdio>
#include <exception>
#include <limits>
#include <memory>
#include <optional>
#include <thread>
#include <utility>

#include "control_link_edge_protocol/edge_payload.hpp"
#include "edge_runtime/channel/channel_options.hpp"
#include "edge_runtime/channel/consumer.hpp"
#include "edge_runtime/channel/producer.hpp"
#include "edge_runtime/common/error.hpp"

namespace control_link_edge_client
{
	using namespace std::chrono_literals;

	namespace
	{
		using CommandConsumer = edge_runtime::Consumer<control_link_edge_protocol::CommandFrame>;
		using CommandProducer = edge_runtime::Producer<control_link_edge_protocol::CommandFrame>;
		using FeedbackConsumer = edge_runtime::Consumer<FeedbackFrame>;

		bool should_reconnect(edge_runtime::ErrorCode code) noexcept
		{
			return code == edge_runtime::ErrorCode::kStaleHandle ||
#if CONTROL_LINK_EDGE_HAS_CONTROL_CONTEXT
				code == edge_runtime::ErrorCode::kClosed ||
#endif
				code == edge_runtime::ErrorCode::kProducerOffline ||
				code == edge_runtime::ErrorCode::kTransportFailed ||
				code == edge_runtime::ErrorCode::kRecoveryBlocked;
		}

		bool transport_allows_commands(const FeedbackFrame &frame) noexcept
		{
			return frame.transport_state !=
					control_link_edge_protocol::TransportState::kStopped &&
				frame.transport_state !=
					control_link_edge_protocol::TransportState::kProtocolError;
		}

		std::chrono::nanoseconds nonnegative_timeout(
			std::chrono::nanoseconds timeout) noexcept
		{
			return timeout.count() > 0 ? timeout : std::chrono::nanoseconds::zero();
		}

		std::uint64_t freshness_limit_ns(std::chrono::milliseconds freshness) noexcept
		{
			const auto count = static_cast<std::uint64_t>(
				std::max<std::int64_t>(0, freshness.count()));
			constexpr auto kNanosecondsPerMillisecond = std::int64_t{1'000'000};
			if (count > std::numeric_limits<std::uint64_t>::max() /
				static_cast<std::uint64_t>(kNanosecondsPerMillisecond))
			{
				return std::numeric_limits<std::uint64_t>::max();
			}
			return static_cast<std::uint64_t>(count) *
				static_cast<std::uint64_t>(kNanosecondsPerMillisecond);
		}

		bool timestamp_fresh(
			bool valid,
			std::uint64_t timestamp_boot_ns,
			std::uint64_t now_boot_ns,
			std::uint64_t limit_ns) noexcept
		{
			return valid && timestamp_boot_ns != 0U && now_boot_ns >= timestamp_boot_ns &&
				now_boot_ns - timestamp_boot_ns <= limit_ns;
		}

		bool feedback_metadata_valid(
			const FeedbackFrame &frame,
			const edge_runtime::Sample<FeedbackFrame> &sample,
			std::uint64_t now_boot_ns) noexcept
		{
			return now_boot_ns != 0U && sample.publish_boot_ns != 0U &&
				sample.receive_boot_ns != 0U && frame.bridge_publish_boot_ns != 0U &&
				now_boot_ns >= sample.publish_boot_ns &&
				now_boot_ns >= sample.receive_boot_ns &&
				now_boot_ns >= frame.bridge_publish_boot_ns;
		}

		FeedbackSnapshot make_feedback_snapshot(
			const FeedbackFrame &frame,
			std::uint64_t publish_boot_ns,
			std::uint64_t receive_boot_ns,
			std::uint64_t missed_samples,
			std::uint64_t now_boot_ns,
			std::uint64_t freshness_limit) noexcept
		{
			const auto valid = frame.validity_flags;
			return FeedbackSnapshot{
				frame,
				publish_boot_ns,
				receive_boot_ns,
				missed_samples,
				timestamp_fresh(true, publish_boot_ns, now_boot_ns, freshness_limit),
				timestamp_fresh(
					(valid & control_link_edge_protocol::kGatewayStateValid) != 0U,
					frame.gateway_receive_boot_ns, now_boot_ns, freshness_limit),
				timestamp_fresh(
					(valid & control_link_edge_protocol::kPlanningStatusValid) != 0U,
					frame.planning_receive_boot_ns, now_boot_ns, freshness_limit),
				timestamp_fresh(
					(valid & control_link_edge_protocol::kVehicleStateValid) != 0U,
					frame.vehicle_receive_boot_ns, now_boot_ns, freshness_limit)};
		}

		edge_runtime::Error client_error(
			edge_runtime::ErrorCode code,
			const char *operation,
			const char *context) noexcept
		{
			return edge_runtime::make_error(code, operation, context);
		}
	}

	class Client::Impl final
	{
	public:
		Impl(
			ClientOptions options,
			std::unique_ptr<CommandProducer> command_producer,
			std::unique_ptr<FeedbackConsumer> feedback_consumer)
			: options_(std::move(options)),
				command_producer_(std::move(command_producer)),
				feedback_consumer_(std::move(feedback_consumer))
				, feedback_wait_(options_.wait_mode)
		{
			status_state_ = ClientState::kWaitingForSession;
		}

#if CONTROL_LINK_EDGE_HAS_CONTROL_CONTEXT
		explicit Impl(ClientOptions options)
			: options_(std::move(options)), feedback_wait_(options_.wait_mode), controlled_(true)
		{
		}

		edge_runtime::Result<void> connect(const ControlContext &context)
		{
			if (!controlled_ || closing_ || status_state_ == ClientState::kClosed)
				return client_error(edge_runtime::ErrorCode::kClosed, "Client::connect", "owner stopped or legacy client");
			auto checked = context.check();
			if (!checked) return checked;
			if (!command_producer_)
			{
				auto ready = control_.begin(context);
				if (!ready) return ready;
				auto options = channel_options(options_.command_channel);
				options.heartbeat_interval = 100ms;
				auto created = CommandProducer::create(options, edge_runtime::command_schema(), context, control_.receipt());
				if (!created) return created.error();
				command_producer_ = std::make_unique<CommandProducer>(std::move(created.value()));
			}
			if (!command_notifications_enabled_ && options_.wait_mode == WaitMode::kEventFd)
			{
				auto ready = control_.begin(context);
				if (!ready) return ready;
				auto enabled = command_producer_->enable_event_notifications(context, control_.receipt());
				if (!enabled) return enabled;
				command_notifications_enabled_ = true;
			}
			while (!feedback_consumer_)
			{
				auto checked = context.check();
				if (!checked) return checked;
				auto opened = reopen_feedback_consumer(&context);
				if (opened) break;
				if (control_.pending()) return opened;
				if (opened.error().code != edge_runtime::ErrorCode::kNotFound &&
					opened.error().code != edge_runtime::ErrorCode::kProducerOffline) return opened;
				auto heartbeat = command_producer_->heartbeat();
				if (!heartbeat) return heartbeat;
				std::this_thread::sleep_for(1ms);
			}
			if (!feedback_seen_) status_state_ = ClientState::kWaitingForSession;
			return edge_runtime::Result<void>::ok();
		}

		edge_runtime::Result<CloseStatus> close(const ControlContext &context) noexcept
		{
			if (!controlled_) return client_error(edge_runtime::ErrorCode::kInvalidOptions, "Client::close", "legacy client requires legacy close");
			if (status_state_ == ClientState::kClosed) return CloseStatus::kClosed;
			closing_ = true;
			status_state_ = ClientState::kClosing;
			session_.ready = false;
			if (command_producer_) command_producer_->request_stop();
			if (feedback_consumer_) feedback_consumer_->request_stop();
			auto settled = feedback_wait_.finalize(context);
			if (!settled) return settled.error();
			settled = control_.settle(context);
			if (!settled) return settled.error();
			if (feedback_consumer_)
			{
				auto ready = control_.begin(context);
				if (!ready) return ready.error();
				auto closed = feedback_consumer_->finalize(context, control_.receipt());
				if (!closed || closed.value() != CloseStatus::kClosed) return closed;
				feedback_consumer_.reset();
			}
			if (command_producer_)
			{
				if (!command_removed_)
				{
					auto ready = control_.begin(context);
					if (!ready) return ready.error();
					auto removed = command_producer_->remove_if_owner(context, control_.receipt());
					if (!removed) return removed.error();
					command_removed_ = true;
				}
				auto ready = control_.begin(context);
				if (!ready) return ready.error();
				auto closed = command_producer_->finalize(context, control_.receipt());
				if (!closed || closed.value() != CloseStatus::kClosed) return closed;
				command_producer_.reset();
			}
			status_state_ = ClientState::kClosed;
			return CloseStatus::kClosed;
		}
#endif

		bool strict_cleanup() const noexcept
		{
#if CONTROL_LINK_EDGE_HAS_CONTROL_CONTEXT
			return controlled_;
#else
			return false;
#endif
		}

		edge_runtime::Result<FeedbackSnapshot> read_feedback(
			std::chrono::nanoseconds timeout)
		{
			if (status_state_ == ClientState::kClosed || status_state_ == ClientState::kClosing)
			{
				return client_error(
					edge_runtime::ErrorCode::kStaleHandle,
					"Client::read_feedback", "client closed");
			}
			const auto read_deadline = std::chrono::steady_clock::now() + nonnegative_timeout(timeout);
#if CONTROL_LINK_EDGE_HAS_CONTROL_CONTEXT
			const auto context = control_budget(nonnegative_timeout(timeout));
			if (controlled_ && !feedback_consumer_ && timeout.count() <= 0)
				return client_error(edge_runtime::ErrorCode::kProducerOffline, "Client::try_feedback", "reconnect requires positive operation budget");
#endif

			if (!feedback_consumer_)
			{
				const auto now = std::chrono::steady_clock::now();
				if (now < next_reconnect_)
				{
					const auto retry_wait = std::chrono::duration_cast<std::chrono::nanoseconds>(
						next_reconnect_ - now);
					const auto caller_wait = nonnegative_timeout(timeout);
					if (caller_wait.count() > 0 && retry_wait.count() > 0)
					{
						std::this_thread::sleep_for(std::min(caller_wait, retry_wait));
					}
					return client_error(
						edge_runtime::ErrorCode::kProducerOffline,
						"Client::read_feedback", "feedback producer unavailable");
				}
				next_reconnect_ = now + options_.reconnect_period;
				auto reopened = reopen_feedback_consumer(
#if CONTROL_LINK_EDGE_HAS_CONTROL_CONTEXT
					controlled_ ? &context : nullptr
#endif
				);
				if (!reopened)
				{
					return reopened.error();
				}
			}

			auto result = [&]() -> edge_runtime::Result<edge_runtime::Sample<FeedbackFrame>>
			{
#if CONTROL_LINK_EDGE_HAS_CONTROL_CONTEXT
				if (controlled_ && timeout.count() > 0)
				{
					auto registered = feedback_wait_.ensure(*feedback_consumer_, context);
					if (!registered) return registered.error();
				}
#endif
				return feedback_wait_.read(*feedback_consumer_, nonnegative_timeout(
					std::chrono::duration_cast<std::chrono::nanoseconds>(read_deadline - std::chrono::steady_clock::now())));
			}();
			if (!result)
			{
				if (should_reconnect(result.error().code))
				{
#if CONTROL_LINK_EDGE_HAS_CONTROL_CONTEXT
					if (controlled_)
					{
						invalidate_session();
						if (timeout.count() <= 0) return result.error();
					}
#endif
					const auto now = std::chrono::steady_clock::now();
					if (now >= next_reconnect_)
					{
						next_reconnect_ = now + options_.reconnect_period;
						auto reopened = reopen_feedback_consumer(
#if CONTROL_LINK_EDGE_HAS_CONTROL_CONTEXT
						controlled_ ? &context : nullptr
#endif
						);
						if (!reopened) return reopened.error();
						status_state_ = ClientState::kTransportLost;
					}
				}
				else if (result.error().code == edge_runtime::ErrorCode::kDataStale ||
					result.error().code == edge_runtime::ErrorCode::kProducerStalled)
				{
					if (feedback_seen_)
					{
						refresh_cached_feedback_state();
					}
				}
				else if (result.error().code != edge_runtime::ErrorCode::kTimeout &&
#if CONTROL_LINK_EDGE_HAS_CONTROL_CONTEXT
					result.error().code != edge_runtime::ErrorCode::kCancelled &&
#endif
					result.error().code != edge_runtime::ErrorCode::kNoNewSample &&
					result.error().code != edge_runtime::ErrorCode::kDataStale)
				{
					status_state_ = ClientState::kProtocolError;
				}
				return result.error();
			}

			const auto &sample = result.value();
			const auto now_boot_ns = control_link_edge_protocol::boottime_now_ns();
			if (!feedback_metadata_valid(sample.value, sample, now_boot_ns))
			{
				status_state_ = ClientState::kTransportLost;
				return client_error(
					edge_runtime::ErrorCode::kClockAnomaly,
					"Client::read_feedback", "invalid BOOTTIME feedback metadata");
			}

			if (!feedback_seen_ || sample.value.session_token != session_.id.token ||
				!transport_identity_seen_ ||
				sample.generation != session_.transport_generation ||
				sample.instance_nonce != session_.transport_instance_nonce)
			{
				++session_.id.epoch;
				session_.id.token = sample.value.session_token;
				session_.has_token =
					!control_link_edge_protocol::is_zero(sample.value.session_token);
				last_submitted_sequence_ = 0U;
			}
			session_.transport_generation = sample.generation;
			session_.transport_instance_nonce = sample.instance_nonce;
			transport_identity_seen_ = true;
			feedback_seen_ = true;
			last_feedback_sequence_ = sample.value.feedback_sequence;
			last_feedback_publish_boot_ns_ = sample.publish_boot_ns;
			last_feedback_receive_boot_ns_ = sample.receive_boot_ns;

			const auto freshness_limit = freshness_limit_ns(options_.feedback_freshness);
			latest_feedback_ = make_feedback_snapshot(
				sample.value, sample.publish_boot_ns, sample.receive_boot_ns,
				sample.missed_samples,
				now_boot_ns, freshness_limit);
			session_.feedback_fresh = latest_feedback_->fresh;
			session_.ready =
				(sample.value.validity_flags & control_link_edge_protocol::kSessionReady) != 0U &&
				command_notification_errno_ == 0;
			session_.transport_allowed = transport_allows_commands(sample.value) && command_notification_errno_ == 0;
			status_state_ = command_notification_errno_ != 0 ? ClientState::kTransportLost : (!session_.ready || !session_.transport_allowed
				? ClientState::kWaitingForSession
				: (latest_feedback_->fresh ? ClientState::kReady : ClientState::kFeedbackStale));
			return latest_feedback_.value();
		}

		edge_runtime::Result<SubmissionReceipt> submit(const CommandTarget &target)
		{
			if (status_state_ == ClientState::kClosed || status_state_ == ClientState::kClosing || !command_producer_)
			{
				return client_error(
					edge_runtime::ErrorCode::kStaleHandle,
					"Client::submit", "client closed");
			}
			// 已提交样本不重发；通知服务故障须由 owner close/open，反馈不能解除锁存。
			if (command_notification_errno_ != 0)
				return client_error(edge_runtime::ErrorCode::kTransportFailed,
					"Client::submit", "notification failure latched; close/open required");
			if (target.session != session_.id)
			{
				return client_error(
					edge_runtime::ErrorCode::kStaleHandle,
					"Client::submit", "session mismatch; target must be rebuilt");
			}
			if (!feedback_seen_ || !session_.has_token || !session_.ready ||
				!session_.transport_allowed)
			{
				return client_error(
					edge_runtime::ErrorCode::kProducerOffline,
					"Client::submit", "session is not ready");
			}

			const auto now_boot_ns = control_link_edge_protocol::boottime_now_ns();
			if (now_boot_ns == 0U || last_feedback_publish_boot_ns_ == 0U ||
				last_feedback_receive_boot_ns_ == 0U ||
				now_boot_ns < last_feedback_publish_boot_ns_ ||
				now_boot_ns < last_feedback_receive_boot_ns_)
			{
				status_state_ = ClientState::kTransportLost;
				return client_error(
					edge_runtime::ErrorCode::kClockAnomaly,
					"Client::submit", "invalid feedback BOOTTIME metadata");
			}
			const auto freshness_limit = freshness_limit_ns(options_.feedback_freshness);
			if (now_boot_ns - last_feedback_publish_boot_ns_ > freshness_limit)
			{
				session_.feedback_fresh = false;
				status_state_ = ClientState::kFeedbackStale;
				return client_error(
					edge_runtime::ErrorCode::kDataStale,
					"Client::submit", "feedback is stale");
			}
			if (target.command_sequence == 0U ||
				target.command_sequence <= last_submitted_sequence_)
			{
				return client_error(
					edge_runtime::ErrorCode::kInvalidOptions,
					"Client::submit", "business sequence must increase per session");
			}
			if (target.command_sequence == std::numeric_limits<std::uint64_t>::max())
			{
				return client_error(
					edge_runtime::ErrorCode::kSequenceExhausted,
					"Client::submit", "business sequence exhausted");
			}
			if (target.source_stamp_ns <= 0 ||
				(target.mode != CommandMode::kNormal && target.mode != CommandMode::kHold) ||
				target.clock_domain != ClockDomain::kSystem ||
				!std::isfinite(target.linear_velocity_mps) ||
				!std::isfinite(target.angular_velocity_radps))
			{
				return client_error(
					edge_runtime::ErrorCode::kInvalidOptions,
					"Client::submit", "target fields are invalid");
			}

			control_link_edge_protocol::CommandFrame frame;
			frame.session_token = target.session.token;
			frame.command_sequence = target.command_sequence;
			frame.source_stamp_ns = target.source_stamp_ns;
			frame.clock_domain = target.clock_domain;
			frame.mode = target.mode;
			frame.linear_velocity_mps = target.linear_velocity_mps;
			frame.angular_velocity_radps = target.angular_velocity_radps;
			auto published = command_producer_->publish(frame);
			if (!published)
			{
				status_state_ = ClientState::kTransportLost;
				return published.error();
			}
			last_submitted_sequence_ = target.command_sequence;
			command_notification_errno_ = notification_error(*command_producer_, options_.wait_mode);
			if (command_notification_errno_ != 0)
			{
				invalidate_session();
				status_state_ = ClientState::kTransportLost;
			}
			return SubmissionReceipt{target.command_sequence, published.value()};
		}

		edge_runtime::Result<void> heartbeat() noexcept
		{
			if (status_state_ == ClientState::kClosed || status_state_ == ClientState::kClosing || !command_producer_)
			{
				return client_error(
					edge_runtime::ErrorCode::kStaleHandle,
					"Client::heartbeat", "client closed");
			}
			auto result = command_producer_->heartbeat();
			if (!result) status_state_ = ClientState::kTransportLost;
			return result;
		}

		SessionSnapshot current_session() const noexcept
		{
			return session_;
		}

		ClientStatus status() const noexcept
		{
			ClientStatus result;
			result.state = status_state_;
			result.cleanup_pending = status_state_ == ClientState::kClosing;
#if CONTROL_LINK_EDGE_HAS_CONTROL_CONTEXT
			result.cleanup_pending = result.cleanup_pending || control_.pending() || feedback_wait_.cleanup_pending();
			result.last_control_committed = controlled_ && control_.committed();
#endif
			result.session = session_;
			result.feedback_seen = feedback_seen_;
			result.notification_registrations = feedback_wait_.registrations();
			result.notification_registration_failures = feedback_wait_.registration_failures();
			result.command_notification_errno = command_notification_errno_;
			result.last_feedback_sequence = last_feedback_sequence_;
			result.last_feedback_publish_boot_ns = last_feedback_publish_boot_ns_;
			result.last_feedback_receive_boot_ns = last_feedback_receive_boot_ns_;
			result.last_submitted_sequence = last_submitted_sequence_;
			if (latest_feedback_.has_value())
			{
				const auto now_boot_ns = control_link_edge_protocol::boottime_now_ns();
				if (now_boot_ns != 0U &&
					now_boot_ns >= latest_feedback_->publish_boot_ns &&
					now_boot_ns >= latest_feedback_->receive_boot_ns)
				{
					result.latest_feedback = make_feedback_snapshot(
						latest_feedback_->frame,
						latest_feedback_->publish_boot_ns,
						latest_feedback_->receive_boot_ns,
						latest_feedback_->missed_samples,
						now_boot_ns,
						freshness_limit_ns(options_.feedback_freshness));
				}
				else
				{
					result.latest_feedback = latest_feedback_;
					result.latest_feedback->fresh = false;
					result.latest_feedback->gateway_fresh = false;
					result.latest_feedback->planning_fresh = false;
					result.latest_feedback->vehicle_fresh = false;
				}
			}
			if (status_state_ != ClientState::kClosed && feedback_seen_)
			{
				const auto now_boot_ns = control_link_edge_protocol::boottime_now_ns();
				const auto freshness_limit = freshness_limit_ns(options_.feedback_freshness);
					if (now_boot_ns == 0U ||
						now_boot_ns < last_feedback_publish_boot_ns_ ||
						now_boot_ns < last_feedback_receive_boot_ns_ ||
						(now_boot_ns - last_feedback_publish_boot_ns_) > freshness_limit)
					{
						result.session.feedback_fresh = false;
					if (result.state == ClientState::kReady)
					{
						result.state = ClientState::kFeedbackStale;
					}
					else if (result.latest_feedback.has_value())
					{
						result.session.feedback_fresh = result.latest_feedback->fresh;
						if (result.state == ClientState::kFeedbackStale &&
							result.session.ready && result.session.transport_allowed &&
							result.latest_feedback->fresh)
						{
							result.state = ClientState::kReady;
						}
					}
				}
			}
			return result;
		}

		edge_runtime::Result<void> close() noexcept
		{
#if CONTROL_LINK_EDGE_HAS_CONTROL_CONTEXT
			if (controlled_)
			{
				auto result = close(control_budget(100ms));
				if (!result) return result.error();
				if (result.value() == CloseStatus::kPending)
					return client_error(edge_runtime::ErrorCode::kRecoveryBlocked, "Client::close", "cleanup pending; retain owner and retry with fresh context");
				return edge_runtime::Result<void>::ok();
			}
#endif
			if (status_state_ == ClientState::kClosed) return edge_runtime::Result<void>::ok();
			feedback_wait_.reset();
			feedback_consumer_.reset();
			edge_runtime::Result<void> removed = edge_runtime::Result<void>::ok();
			if (command_producer_)
			{
				removed = command_producer_->remove_if_owner();
				command_producer_.reset();
			}
			status_state_ = ClientState::kClosed;
			return removed;
		}

	private:
		edge_runtime::Result<void> reopen_feedback_consumer(
#if CONTROL_LINK_EDGE_HAS_CONTROL_CONTEXT
			const ControlContext *supplied_context = nullptr
#endif
		)
		{
#if CONTROL_LINK_EDGE_HAS_CONTROL_CONTEXT
			if (controlled_)
			{
				const auto context = supplied_context ? *supplied_context : control_budget(20ms);
				auto settled = feedback_wait_.finalize(context);
				if (!settled) return settled;
				if (feedback_consumer_)
				{
					feedback_consumer_->request_stop();
					auto ready = control_.begin(context);
					if (!ready) return ready;
					auto closed = feedback_consumer_->finalize(context, control_.receipt());
					if (!closed) return closed.error();
					if (closed.value() != CloseStatus::kClosed)
						return client_error(edge_runtime::ErrorCode::kRecoveryBlocked, "Client::reopen", "old consumer cleanup pending");
					feedback_consumer_.reset();
				}
				auto ready = control_.begin(context);
				if (!ready) return ready;
				auto opened = FeedbackConsumer::open(feedback_channel_options(), edge_runtime::feedback_schema(), context, control_.receipt());
				if (!opened) return opened.error();
				feedback_consumer_ = std::make_unique<FeedbackConsumer>(std::move(opened.value()));
				invalidate_session();
				return edge_runtime::Result<void>::ok();
			}
#endif
			// 先释放旧映射再按名称重开，避免把同一进程的旧 consumer owner
			// 传给 EdgeRuntime::Consumer::reconnect；新 producer 可能已经替换实例。
			feedback_wait_.reset();
			feedback_consumer_.reset();
			auto reopened = FeedbackConsumer::open(
				feedback_channel_options(), edge_runtime::feedback_schema());
			if (!reopened)
			{
				status_state_ = ClientState::kTransportLost;
				return reopened.error();
			}
			auto channel_status = reopened.value().status();
			if (!channel_status)
			{
				status_state_ = ClientState::kTransportLost;
				return channel_status.error();
			}
			const auto &candidate_status = channel_status.value();
			std::array<std::byte, 16U> candidate_nonce{};
			std::memcpy(candidate_nonce.data(), &candidate_status.instance_nonce_hi, 8U);
			std::memcpy(candidate_nonce.data() + 8U, &candidate_status.instance_nonce_lo, 8U);
			if (transport_identity_seen_ &&
				candidate_status.generation == session_.transport_generation &&
				candidate_nonce == session_.transport_instance_nonce)
			{
				status_state_ = ClientState::kTransportLost;
				return client_error(
					edge_runtime::ErrorCode::kProducerOffline,
					"Client::reopen_feedback",
					"feedback producer replacement has not changed transport identity");
			}
			if (!candidate_status.ready || !candidate_status.producer_alive)
			{
				status_state_ = ClientState::kTransportLost;
				return client_error(
					edge_runtime::ErrorCode::kProducerOffline,
					"Client::reopen_feedback", "feedback producer is not alive");
			}
			feedback_consumer_ = std::make_unique<FeedbackConsumer>(
				std::move(reopened.value()));
			invalidate_session();
			return edge_runtime::Result<void>::ok();
		}

		edge_runtime::ChannelOptions feedback_channel_options() const
		{
			return channel_options(options_.feedback_channel);
		}

		void invalidate_session() noexcept
		{
			if (session_.id.epoch != std::numeric_limits<std::uint64_t>::max())
			{
				++session_.id.epoch;
			}
			session_.ready = false;
			session_.transport_allowed = false;
			session_.feedback_fresh = false;
			last_submitted_sequence_ = 0U;
		}

		void refresh_cached_feedback_state() noexcept
		{
			if (!latest_feedback_.has_value()) return;
			const auto now_boot_ns = control_link_edge_protocol::boottime_now_ns();
			latest_feedback_ = make_feedback_snapshot(
				latest_feedback_->frame,
				latest_feedback_->publish_boot_ns,
				latest_feedback_->receive_boot_ns,
				latest_feedback_->missed_samples,
				now_boot_ns,
				freshness_limit_ns(options_.feedback_freshness));
			session_.feedback_fresh = latest_feedback_->fresh;
			if (!session_.ready || !session_.transport_allowed)
			{
				status_state_ = ClientState::kWaitingForSession;
			}
			else
			{
				status_state_ = latest_feedback_->fresh
					? ClientState::kReady
					: ClientState::kFeedbackStale;
			}
		}

		ClientOptions options_;
		std::unique_ptr<CommandProducer> command_producer_;
		std::unique_ptr<FeedbackConsumer> feedback_consumer_;
		ChannelWait<FeedbackFrame> feedback_wait_;
		int command_notification_errno_{0};
		SessionSnapshot session_{};
		std::optional<FeedbackSnapshot> latest_feedback_;
		ClientState status_state_{ClientState::kOpening};
		bool feedback_seen_{false};
		bool transport_identity_seen_{false};
		std::uint64_t last_feedback_sequence_{0U};
		std::uint64_t last_feedback_publish_boot_ns_{0U};
		std::uint64_t last_feedback_receive_boot_ns_{0U};
		std::uint64_t last_submitted_sequence_{0U};
		std::chrono::steady_clock::time_point next_reconnect_{};
#if CONTROL_LINK_EDGE_HAS_CONTROL_CONTEXT
		ControlOperation control_;
		bool controlled_{false};
		bool closing_{false};
		bool command_removed_{false};
		bool command_notifications_enabled_{false};
#endif
	};

#if CONTROL_LINK_EDGE_HAS_CONTROL_CONTEXT
	edge_runtime::Result<Client> Client::prepare(const ClientOptions &options)
	{
		if (options.command_channel.empty() || options.feedback_channel.empty() ||
			options.feedback_freshness.count() <= 0 || options.feedback_open_timeout.count() <= 0 || options.reconnect_period.count() <= 0 ||
			!wait_mode_supported(options.wait_mode))
			return client_error(edge_runtime::ErrorCode::kInvalidOptions, "Client::prepare", "invalid channel, freshness or wait mode");
		return Client(std::make_unique<Impl>(options));
	}
	edge_runtime::Result<void> Client::connect(const ControlContext &context)
	{
		return impl_ ? impl_->connect(context) : client_error(edge_runtime::ErrorCode::kClosed, "Client::connect", "moved client");
	}
	edge_runtime::Result<CloseStatus> Client::close(const ControlContext &context) noexcept
	{
		return impl_ ? impl_->close(context) : edge_runtime::Result<CloseStatus>(CloseStatus::kClosed);
	}
#endif

	edge_runtime::Result<Client> Client::open(const ClientOptions &options)
	{
		if (options.command_channel.empty() || options.feedback_channel.empty() ||
			options.feedback_open_timeout.count() <= 0 ||
			options.reconnect_period.count() <= 0 || options.feedback_freshness.count() <= 0)
		{
			return client_error(
				edge_runtime::ErrorCode::kInvalidOptions,
				"Client::open", "invalid channel or timeout options");
		}
		if (!wait_mode_supported(options.wait_mode))
			return client_error(edge_runtime::ErrorCode::kUnsupportedPlatform, "Client::open", "eventfd unsupported by selected SDK");

		auto command_options = channel_options(options.command_channel);
		command_options.heartbeat_interval = 100ms;
		auto producer = CommandProducer::create(
			command_options, edge_runtime::command_schema());
		if (!producer) return producer.error();
		auto enabled = enable_notifications(producer.value(), options.wait_mode);
		if (!enabled) { (void)producer.value().remove_if_owner(); return enabled.error(); }

		auto feedback_options = channel_options(options.feedback_channel);
		const auto deadline = std::chrono::steady_clock::now() + options.feedback_open_timeout;
		while (true)
		{
			auto heartbeat = producer.value().heartbeat();
			if (!heartbeat)
			{
				(void)producer.value().remove_if_owner();
				return heartbeat.error();
			}
			auto consumer = FeedbackConsumer::open(
				feedback_options, edge_runtime::feedback_schema());
			if (consumer)
			{
				return Client(std::make_unique<Impl>(
					options,
					std::make_unique<CommandProducer>(std::move(producer.value())),
					std::make_unique<FeedbackConsumer>(std::move(consumer.value()))));
			}
			if (std::chrono::steady_clock::now() >= deadline)
			{
				(void)producer.value().remove_if_owner();
				return consumer.error();
			}
			std::this_thread::sleep_for(100ms);
		}
	}

	Client::Client(std::unique_ptr<Impl> implementation) noexcept
		: impl_(std::move(implementation))
	{
	}

	Client::Client(Client &&other) noexcept
		: impl_(std::move(other.impl_))
	{
	}

	Client &Client::operator=(Client &&other) noexcept
	{
		if (this == &other) return *this;
		if (impl_ != nullptr && !impl_->close() && impl_->strict_cleanup())
		{
			std::fputs("ControlLink Client move requires closed owner; cleanup remains pending\n", stderr);
			std::terminate();
		}
		impl_ = std::move(other.impl_);
		return *this;
	}

	Client::~Client()
	{
		if (impl_ != nullptr && !impl_->close() && impl_->strict_cleanup())
		{
			std::fputs("ControlLink Client requires explicit close/finalize before destruction; retain pending owner\n", stderr);
			std::terminate();
		}
	}

	edge_runtime::Result<FeedbackSnapshot> Client::wait_feedback(
		std::chrono::nanoseconds timeout)
	{
		return impl_ != nullptr
			? impl_->read_feedback(timeout)
			: client_error(
				edge_runtime::ErrorCode::kStaleHandle,
				"Client::wait_feedback", "client moved or closed");
	}

	edge_runtime::Result<FeedbackSnapshot> Client::try_feedback()
	{
		return wait_feedback(std::chrono::nanoseconds::zero());
	}

	edge_runtime::Result<SubmissionReceipt> Client::submit(const CommandTarget &target)
	{
		return impl_ != nullptr
			? impl_->submit(target)
			: client_error(
				edge_runtime::ErrorCode::kStaleHandle,
				"Client::submit", "client moved or closed");
	}

	edge_runtime::Result<void> Client::heartbeat() noexcept
	{
		return impl_ != nullptr
			? impl_->heartbeat()
			: client_error(
				edge_runtime::ErrorCode::kStaleHandle,
				"Client::heartbeat", "client moved or closed");
	}

	SessionSnapshot Client::current_session() const noexcept
	{
		return impl_ != nullptr ? impl_->current_session() : SessionSnapshot{};
	}

	ClientStatus Client::status() const noexcept
	{
		return impl_ != nullptr ? impl_->status() : ClientStatus{};
	}

	edge_runtime::Result<void> Client::close() noexcept
	{
		return impl_ != nullptr ? impl_->close() : edge_runtime::Result<void>::ok();
	}
} // namespace control_link_edge_client
