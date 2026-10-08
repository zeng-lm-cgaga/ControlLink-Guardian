#include "control_link_edge_protocol/edge_payload.hpp"

#include <chrono>
#include <cmath>
#include <cerrno>
#include <csignal>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <iostream>
#include <limits>
#include <memory>
#include <optional>
#include <sstream>
#include <string>
#include <thread>

#include "edge_runtime/channel/channel_options.hpp"
#include "edge_runtime/channel/consumer.hpp"
#include "edge_runtime/channel/producer.hpp"
#include "edge_runtime/common/error.hpp"

namespace
{
	using control_link_edge_protocol::CommandFrame;
	using control_link_edge_protocol::FeedbackFrame;
	using control_link_edge_protocol::SessionToken;
	using FeedbackConsumer = edge_runtime::Consumer<FeedbackFrame>;
	using CommandProducer = edge_runtime::Producer<CommandFrame>;

	volatile std::sig_atomic_t stop_requested = 0;

	void handle_signal(int) noexcept
	{
		stop_requested = 1;
	}

	struct Options
	{
		std::string command_channel;
		std::string feedback_channel;
		double linear_velocity_mps{0.25};
		double angular_velocity_radps{0.10};
		std::uint64_t period_ms{50U};
		std::uint64_t duration_ms{0U};
	};

	void usage(const char *program)
	{
		std::cerr << "usage: " << program
			<< " --command-channel NAME --feedback-channel NAME"
			<< " [--linear-velocity-mps VALUE]"
			<< " [--angular-velocity-radps VALUE]"
			<< " [--period-ms VALUE] [--duration-ms VALUE]\n";
	}

	bool parse_unsigned(const char *text, std::uint64_t *value)
	{
		if (text == nullptr || *text == '\0' || value == nullptr) return false;
		char *end = nullptr;
		errno = 0;
		const auto parsed = std::strtoull(text, &end, 10);
		if (errno != 0 || end == text || *end != '\0') return false;
		*value = static_cast<std::uint64_t>(parsed);
		return true;
	}

	bool parse_double(const char *text, double *value)
	{
		if (text == nullptr || *text == '\0' || value == nullptr) return false;
		char *end = nullptr;
		errno = 0;
		const auto parsed = std::strtod(text, &end);
		if (errno != 0 || end == text || *end != '\0') return false;
		if (!std::isfinite(parsed)) return false;
		*value = parsed;
		return true;
	}

	bool parse_options(int argc, char **argv, Options *options)
	{
		if (options == nullptr) return false;
		for (int index = 1; index < argc; ++index)
		{
			const std::string argument = argv[index];
			if (argument == "--help")
			{
				usage(argv[0]);
				return false;
			}
			if (index + 1 >= argc)
			{
				std::cerr << "missing value for " << argument << '\n';
				return false;
			}
			const char *value = argv[++index];
			if (argument == "--command-channel") options->command_channel = value;
			else if (argument == "--feedback-channel") options->feedback_channel = value;
			else if (argument == "--linear-velocity-mps")
			{
				if (!parse_double(value, &options->linear_velocity_mps)) return false;
			}
			else if (argument == "--angular-velocity-radps")
			{
				if (!parse_double(value, &options->angular_velocity_radps)) return false;
			}
			else if (argument == "--period-ms")
			{
				if (!parse_unsigned(value, &options->period_ms)) return false;
			}
			else if (argument == "--duration-ms")
			{
				if (!parse_unsigned(value, &options->duration_ms)) return false;
			}
			else
			{
				std::cerr << "unknown option: " << argument << '\n';
				return false;
			}
		}
		return !options->command_channel.empty() && !options->feedback_channel.empty() &&
			options->period_ms > 0U && options->period_ms <= 60'000U;
	}

	std::string error_text(const edge_runtime::Error &error)
	{
		std::ostringstream stream;
		stream << edge_runtime::to_string(error.code);
		if (error.operation != nullptr) stream << " operation=" << error.operation;
		return stream.str();
	}

	edge_runtime::ChannelOptions command_options(const Options &options)
	{
		edge_runtime::ChannelOptions result;
		result.name = options.command_channel;
		result.stale_timeout = std::chrono::milliseconds(500);
		result.reconnect_timeout = std::chrono::milliseconds(500);
		result.heartbeat_interval = std::chrono::milliseconds(100);
		return result;
	}

	edge_runtime::ChannelOptions feedback_options(const Options &options)
	{
		edge_runtime::ChannelOptions result;
		result.name = options.feedback_channel;
		result.stale_timeout = std::chrono::milliseconds(500);
		result.reconnect_timeout = std::chrono::milliseconds(500);
		return result;
	}

	bool same_token(const SessionToken &left, const SessionToken &right) noexcept
	{
		return left == right;
	}

	bool should_reconnect(edge_runtime::ErrorCode code) noexcept
	{
		return code == edge_runtime::ErrorCode::kStaleHandle ||
			code == edge_runtime::ErrorCode::kProducerOffline ||
			code == edge_runtime::ErrorCode::kTransportFailed ||
			code == edge_runtime::ErrorCode::kRecoveryBlocked;
	}

	void print_feedback(const FeedbackFrame &feedback)
	{
		std::cout << "feedback sequence=" << feedback.feedback_sequence
			<< " transport=" << static_cast<unsigned>(feedback.transport_state)
			<< " gateway=" << static_cast<unsigned>(feedback.gateway_state)
			<< " active_source=" << static_cast<unsigned>(feedback.active_source)
			<< " planning_accepted=" << feedback.planning_accepted_count
			<< " planning_rejected=" << feedback.planning_rejected_count << '\n';
	}
}

int main(int argc, char **argv)
{
	Options options;
	if (!parse_options(argc, argv, &options))
	{
		usage(argv[0]);
		return 2;
	}
	if (std::signal(SIGINT, handle_signal) == SIG_ERR ||
		std::signal(SIGTERM, handle_signal) == SIG_ERR)
	{
		std::cerr << "cannot install termination signal handlers\n";
		return 2;
	}

	const auto command_channel_options = command_options(options);
	const auto feedback_channel_options = feedback_options(options);
	auto command_result = CommandProducer::create(
		command_channel_options, edge_runtime::command_schema());
	if (!command_result)
	{
		std::cerr << "command channel create failed: " << error_text(command_result.error()) << '\n';
		return 3;
	}
	CommandProducer command_producer = std::move(command_result.value());

	std::unique_ptr<FeedbackConsumer> feedback_consumer;
	const auto feedback_retry_deadline =
		std::chrono::steady_clock::now() + std::chrono::seconds(30);
	while (!feedback_consumer && !stop_requested)
	{
		auto opened = FeedbackConsumer::open(
			feedback_channel_options, edge_runtime::feedback_schema());
		if (opened)
		{
			feedback_consumer = std::make_unique<FeedbackConsumer>(
				std::move(opened.value()));
			break;
		}
		if (std::chrono::steady_clock::now() >= feedback_retry_deadline)
		{
			std::cerr << "feedback channel open timed out: "
				<< error_text(opened.error()) << '\n';
			(void)command_producer.remove_if_owner();
			return 4;
		}
		std::this_thread::sleep_for(std::chrono::milliseconds(100));
	}
	if (!feedback_consumer)
	{
		(void)command_producer.remove_if_owner();
		return 0;
	}

	std::optional<SessionToken> active_token;
	bool feedback_session_ready = false;
	bool feedback_transport_allowed = false;
	std::uint64_t next_command_sequence = 0U;
	std::uint64_t last_feedback_boot_ns = 0U;
	std::uint64_t last_feedback_sequence = 0U;
	const auto started_at = std::chrono::steady_clock::now();
	auto next_publish = started_at;
	auto next_reconnect_attempt = started_at;

	while (!stop_requested)
	{
		if (options.duration_ms != 0U &&
			std::chrono::steady_clock::now() - started_at >=
				std::chrono::milliseconds(options.duration_ms))
		{
			break;
		}

		const auto feedback = feedback_consumer->wait_latest(std::chrono::milliseconds(50));
		if (feedback)
		{
			const auto &sample = feedback.value();
			const auto &value = sample.value;
			last_feedback_boot_ns = sample.receive_boot_ns;
			if (last_feedback_sequence != value.feedback_sequence)
			{
				last_feedback_sequence = value.feedback_sequence;
				if (!active_token.has_value() || !same_token(active_token.value(), value.session_token))
				{
					// token 变化使旧业务目标失效；客户端必须重新生成 sequence 和目标时间。
					active_token = value.session_token;
					next_command_sequence = 0U;
					std::cout << "new bridge session token observed; old command target discarded\n";
				}
				feedback_session_ready =
					(value.validity_flags & control_link_edge_protocol::kSessionReady) != 0U;
				feedback_transport_allowed =
					value.transport_state != control_link_edge_protocol::TransportState::kStopped &&
					value.transport_state != control_link_edge_protocol::TransportState::kProtocolError;
				print_feedback(value);
			}
		}
		else if (should_reconnect(feedback.error().code) &&
			std::chrono::steady_clock::now() >= next_reconnect_attempt)
		{
			next_reconnect_attempt = std::chrono::steady_clock::now() +
				std::chrono::milliseconds(100);
			auto reconnected = feedback_consumer->reconnect();
			if (!reconnected)
			{
				active_token.reset();
				feedback_session_ready = false;
				feedback_transport_allowed = false;
				last_feedback_boot_ns = 0U;
			}
			else
			{
				active_token.reset();
				feedback_session_ready = false;
				feedback_transport_allowed = false;
				last_feedback_boot_ns = 0U;
				std::cout << "feedback channel reconnected; waiting for new token\n";
			}
		}

		const auto now_boot_ns = control_link_edge_protocol::boottime_now_ns();
		const bool feedback_fresh = last_feedback_boot_ns != 0U &&
			now_boot_ns >= last_feedback_boot_ns &&
			now_boot_ns - last_feedback_boot_ns <= 500'000'000U;
		const bool transport_ready = active_token.has_value() && feedback_fresh &&
			feedback_session_ready && feedback_transport_allowed;
		if (transport_ready && std::chrono::steady_clock::now() >= next_publish)
		{
			CommandFrame command;
			command.session_token = active_token.value();
			command.command_sequence = ++next_command_sequence;
			command.source_stamp_ns = control_link_edge_protocol::system_time_now_ns();
			command.linear_velocity_mps = options.linear_velocity_mps;
			command.angular_velocity_radps = options.angular_velocity_radps;
			if (command.source_stamp_ns <= 0)
			{
				std::cerr << "system clock unavailable; command forwarding stopped\n";
				active_token.reset();
				feedback_session_ready = false;
				feedback_transport_allowed = false;
			}
			else
			{
				auto published = command_producer.publish(command);
				if (!published)
				{
					std::cerr << "command publish failed: " << error_text(published.error()) << '\n';
					active_token.reset();
					feedback_session_ready = false;
					feedback_transport_allowed = false;
				}
			}
			next_publish = std::chrono::steady_clock::now() +
				std::chrono::milliseconds(options.period_ms);
		}

		auto heartbeat = command_producer.heartbeat();
		if (!heartbeat)
		{
			std::cerr << "command heartbeat failed: " << error_text(heartbeat.error()) << '\n';
			active_token.reset();
			feedback_session_ready = false;
			feedback_transport_allowed = false;
		}
	}

	feedback_consumer.reset();
	const auto removed = command_producer.remove_if_owner();
	if (!removed)
	{
		std::cerr << "command channel cleanup failed: " << error_text(removed.error()) << '\n';
		return 5;
	}
	return 0;
}
