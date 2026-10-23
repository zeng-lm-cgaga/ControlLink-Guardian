#include "control_link_edge_protocol/client.hpp"
#include <csignal>
#include <fstream>
#include <iostream>
#include <thread>

namespace
{
using namespace control_link_edge_client;
using namespace control_link_edge_protocol;
using namespace std::chrono_literals;
volatile std::sig_atomic_t stopped = 0;
void signal_stop(int) { stopped = 1; }
std::uint64_t monotonic_ns()
{
	return std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::steady_clock::now().time_since_epoch()).count();
}
void error(const char *event, const edge_runtime::Error &failure)
{
	std::cout << "{\"event\":\"" << event << "\",\"monotonic_ns\":" << monotonic_ns()
		<< ",\"code\":\"" << edge_runtime::to_string(failure.code) << "\",\"errno\":" << failure.errno_value << "}\n" << std::flush;
}
#if CONTROL_LINK_EDGE_HAS_CONTROL_CONTEXT
bool close_retained(Client &client)
{
	while (true)
	{
		auto closed = client.close(control_budget(100ms));
		if (closed && closed.value() == CloseStatus::kClosed) return true;
		if (!closed) error("cleanup_pending", closed.error());
		std::this_thread::sleep_for(100ms);
	}
}
edge_runtime::Result<Client> open_owned(const ClientOptions &options)
{
	auto prepared = Client::prepare(options);
	if (!prepared) return prepared.error();
	auto client = std::move(prepared.value());
	const auto deadline = std::chrono::steady_clock::now() + options.feedback_open_timeout;
	while (!stopped && std::chrono::steady_clock::now() < deadline)
	{
		auto ready = client.connect(control_budget(std::min(
			std::chrono::duration_cast<std::chrono::nanoseconds>(deadline - std::chrono::steady_clock::now()), std::chrono::nanoseconds{20'000'000})));
		if (ready) return client;
		if (ready.error().code != edge_runtime::ErrorCode::kTimeout && ready.error().code != edge_runtime::ErrorCode::kNotFound &&
			ready.error().code != edge_runtime::ErrorCode::kProducerOffline)
		{
			const auto failure = ready.error();
			close_retained(client);
			return failure;
		}
	}
	close_retained(client);
	return edge_runtime::make_error(edge_runtime::ErrorCode::kTimeout, "driver connect", "operation stopped or expired");
}
#else
bool close_retained(Client &client) { return static_cast<bool>(client.close()); }
edge_runtime::Result<Client> open_owned(const ClientOptions &options) { return Client::open(options); }
#endif
}

int main(int argc, char **argv)
{
	if (argc != 5) return 2;
	ClientOptions options;
	options.command_channel = argv[1];
	options.feedback_channel = argv[2];
	if (!parse_wait_mode(argv[3], options.wait_mode)) return 2;
	options.feedback_open_timeout = 1500ms;
	const std::string control_path = argv[4];
	std::signal(SIGINT, signal_stop);
	std::signal(SIGTERM, signal_stop);
	auto opened = open_owned(options);
	if (!opened) { error("open_failed", opened.error()); return 1; }
	auto client = std::move(opened.value());
	std::string previous;
	SessionId session{};
	std::optional<CommandTarget> old_target;
	std::uint64_t sequence = 0, submitted = 0;
	auto next = std::chrono::steady_clock::now();
	const auto deadline = next + 150s;
	while (!stopped && std::chrono::steady_clock::now() < deadline)
	{
		std::string phase;
		std::ifstream input(control_path);
		input >> phase;
		if (phase != previous)
		{
			previous = phase;
			std::cout << "{\"event\":\"phase\",\"phase\":\"" << phase << "\",\"monotonic_ns\":" << monotonic_ns() << "}\n" << std::flush;
			if (phase == "reopen")
			{
				CommandTarget target;
				target.session = client.current_session().id;
				target.command_sequence = sequence + 1;
				target.source_stamp_ns = system_time_now_ns();
				old_target = target;
				if (!close_retained(client)) return 1;
				auto replacement = open_owned(options);
				if (!replacement) { error("reopen_failed", replacement.error()); return 1; }
				client = std::move(replacement.value());
				sequence = 0;
			}
		}
		auto feedback = client.wait_feedback(10ms);
		if (feedback)
		{
			const auto current = client.current_session();
			if (current.id != session)
			{
				session = current.id;
				sequence = 0;
				std::cout << "{\"event\":\"session\",\"epoch\":" << session.epoch
					<< ",\"token\":\"";
				static constexpr char digits[] = "0123456789abcdef";
				for (const auto value : session.token.bytes)
				{
					const auto byte = std::to_integer<unsigned>(value);
					std::cout << digits[byte >> 4] << digits[byte & 15];
				}
				std::cout << "\",\"monotonic_ns\":" << monotonic_ns() << "}\n" << std::flush;
			}
			std::cout << "{\"event\":\"feedback\",\"monotonic_ns\":" << monotonic_ns()
				<< ",\"feedback_sequence\":" << feedback.value().frame.feedback_sequence
				<< ",\"publish_boot_ns\":" << feedback.value().publish_boot_ns
				<< ",\"gateway_state\":" << static_cast<unsigned>(feedback.value().frame.gateway_state)
				<< ",\"accepted\":" << feedback.value().frame.planning_accepted_count
				<< ",\"registrations\":" << client.status().notification_registrations << "}\n" << std::flush;
			if (old_target && current.id.token != old_target->session.token && current.ready)
			{
				auto rejected = client.submit(*old_target);
				if (rejected) { (void)close_retained(client); return 3; }
				error("old_target_rejected", rejected.error());
				old_target.reset();
			}
		}
		else if (feedback.error().code != edge_runtime::ErrorCode::kNoNewSample &&
			feedback.error().code != edge_runtime::ErrorCode::kTimeout && feedback.error().code != edge_runtime::ErrorCode::kDataStale)
			error("feedback_unavailable", feedback.error());
		const auto now = std::chrono::steady_clock::now();
		if (now >= next)
		{
			next += 50ms;
			if (next <= now) next = now + 50ms;
			auto heartbeat = client.heartbeat();
			if (!heartbeat) error("heartbeat_error", heartbeat.error());
			if (phase != "pause" && client.current_session().ready)
			{
				CommandTarget target;
				target.session = client.current_session().id;
				target.command_sequence = ++sequence;
				target.source_stamp_ns = system_time_now_ns();
				target.linear_velocity_mps = 0.25;
				target.angular_velocity_radps = 0.05;
				auto receipt = client.submit(target);
				if (!receipt) error("submit_rejected", receipt.error());
				else
				{
					++submitted;
					std::cout << "{\"event\":\"submit\",\"monotonic_ns\":" << monotonic_ns()
						<< ",\"sequence\":" << sequence << ",\"stamp_ns\":" << target.source_stamp_ns << "}\n" << std::flush;
				}
			}
		}
		std::this_thread::sleep_for(1ms);
	}
	const auto status = client.status();
	if (!close_retained(client)) return 1;
	std::cout << "{\"event\":\"closed\",\"monotonic_ns\":" << monotonic_ns()
		<< ",\"submitted\":" << submitted << ",\"registrations\":" << status.notification_registrations
		<< ",\"registration_failures\":" << status.notification_registration_failures
		<< ",\"command_notification_errno\":" << status.command_notification_errno << "}\n" << std::flush;
	return stopped ? 0 : 124;
}
