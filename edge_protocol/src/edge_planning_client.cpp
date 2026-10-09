#include "control_link_edge_protocol/client.hpp"

#include <cerrno>
#include <csignal>
#include <cstdlib>
#include <cmath>
#include <cstdint>
#include <iostream>
#include <sstream>
#include <string>
#include <thread>

namespace
{
	using control_link_edge_client::Client;
	using control_link_edge_client::ClientOptions;
	using control_link_edge_client::ClientState;
	using control_link_edge_client::CommandTarget;
	using control_link_edge_client::FeedbackSnapshot;
	using control_link_edge_client::SessionId;
	using control_link_edge_protocol::CommandMode;

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
		if (errno != 0 || end == text || *end != '\0' || !std::isfinite(parsed))
		{
			return false;
		}
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

	void print_feedback(const FeedbackSnapshot &snapshot)
	{
		const auto &feedback = snapshot.frame;
		std::cout << "feedback=" << feedback.feedback_sequence
			<< " transport=" << static_cast<unsigned>(feedback.transport_state)
			<< " transport_fresh=" << (snapshot.fresh ? "yes" : "no")
			<< " gateway=" << static_cast<unsigned>(feedback.gateway_state)
			<< " gateway_reason=" << feedback.gateway_reason_code
			<< " gateway_current=" << (snapshot.gateway_state_current() ? "yes" : "no")
			<< " planning_accepted=" << feedback.planning_accepted_count
			<< " planning_rejected=" << feedback.planning_rejected_count
			<< " planning_current=" << (snapshot.planning_status_current() ? "yes" : "no")
			<< " active_source=" << static_cast<unsigned>(feedback.active_source)
			<< " vehicle_state=" << static_cast<unsigned>(feedback.vehicle_state)
			<< " vehicle_fault=" << feedback.vehicle_fault_code
			<< " vehicle_current=" << (snapshot.vehicle_state_current() ? "yes" : "no")
			<< '\n' << std::flush;
	}

	const char *client_state_name(ClientState state) noexcept
	{
		switch (state)
		{
		case ClientState::kOpening: return "opening";
		case ClientState::kWaitingForSession: return "waiting_for_session";
		case ClientState::kReady: return "ready";
		case ClientState::kFeedbackStale: return "feedback_stale";
		case ClientState::kTransportLost: return "transport_lost";
		case ClientState::kProtocolError: return "protocol_error";
		case ClientState::kClosed: return "closed";
		}
		return "unknown";
	}

	bool session_changed(
		const std::optional<SessionId> &previous,
		const SessionId &current) noexcept
	{
		return !previous.has_value() || previous.value() != current;
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

	ClientOptions client_options;
	client_options.command_channel = options.command_channel;
	client_options.feedback_channel = options.feedback_channel;
	client_options.feedback_open_timeout = std::chrono::seconds(30);
	client_options.reconnect_period = std::chrono::milliseconds(100);
	client_options.feedback_freshness = std::chrono::milliseconds(500);
		auto opened = Client::open(client_options);
	if (!opened)
	{
		std::cerr << "edge client open failed: " << error_text(opened.error()) << '\n';
		return 3;
	}
	Client client = std::move(opened.value());

	std::optional<SessionId> target_session;
	std::uint64_t next_command_sequence = 0U;
	const auto started_at = std::chrono::steady_clock::now();
	auto next_publish = started_at;

	while (!stop_requested)
	{
		if (options.duration_ms != 0U &&
			std::chrono::steady_clock::now() - started_at >=
				std::chrono::milliseconds(options.duration_ms))
		{
			break;
		}

		const auto feedback = client.wait_feedback(std::chrono::milliseconds(10));
		if (feedback)
		{
			const auto current_session = client.current_session();
			if (session_changed(target_session, current_session.id))
			{
				target_session = current_session.id;
				next_command_sequence = 0U;
				std::cout << "new SDK session observed; old target discarded\n" << std::flush;
			}
			print_feedback(feedback.value());

		}
		else if (feedback.error().code == edge_runtime::ErrorCode::kDataStale ||
			feedback.error().code == edge_runtime::ErrorCode::kProducerStalled)
		{
			const auto status = client.status();
			if (status.state == ClientState::kFeedbackStale)
			{
				std::cerr << "SDK feedback stale: state="
					<< client_state_name(status.state) << " transport_fresh=no\n";
			}
		}
		else if (feedback.error().code != edge_runtime::ErrorCode::kTimeout &&
			feedback.error().code != edge_runtime::ErrorCode::kNoNewSample)
		{
			std::cerr << "SDK feedback read failed: " << error_text(feedback.error()) << '\n';
		}

		const auto status = client.status();
		if (status.session.ready && status.session.transport_allowed &&
			status.session.feedback_fresh &&
			std::chrono::steady_clock::now() >= next_publish)
		{
			CommandTarget target;
			target.session = status.session.id;
			target.command_sequence = ++next_command_sequence;
			target.source_stamp_ns =
				control_link_edge_protocol::system_time_now_ns();
			target.clock_domain = control_link_edge_protocol::ClockDomain::kSystem;
			target.mode = CommandMode::kNormal;
			target.linear_velocity_mps = options.linear_velocity_mps;
			target.angular_velocity_radps = options.angular_velocity_radps;
			auto submitted = client.submit(target);
			if (!submitted)
			{
				std::cerr << "SDK target submit rejected: "
					<< error_text(submitted.error()) << '\n';
			}
			next_publish = std::chrono::steady_clock::now() +
				std::chrono::milliseconds(options.period_ms);
		}

		auto heartbeat = client.heartbeat();
		if (!heartbeat)
		{
			std::cerr << "SDK transport heartbeat failed: "
				<< error_text(heartbeat.error()) << '\n';
			break;
		}
	}

	const auto closed = client.close();
	if (!closed)
	{
		std::cerr << "SDK close failed: " << error_text(closed.error()) << '\n';
		return 5;
	}
	return 0;
}
