#include <chrono>
#include <cerrno>
#include <cstdint>
#include <cstdlib>
#include <iostream>
#include <limits>
#include <string>
#include <vector>

#include "edge_runtime/process/supervisor.hpp"

namespace
{
	struct Options
	{
		std::string command_channel;
		std::string feedback_channel;
		std::string client_path;
		std::string linear_velocity_mps{"0.25"};
		std::string angular_velocity_radps{"0.10"};
		std::uint32_t max_restarts{3U};
		std::uint64_t create_timeout_ms{10'000U};
	};

	void usage(const char *program)
	{
		std::cerr << "usage: " << program
			<< " --command-channel NAME --feedback-channel NAME --client PATH"
			<< " [--linear-velocity-mps VALUE] [--angular-velocity-radps VALUE]"
			<< " [--max-restarts COUNT] [--create-timeout-ms VALUE]\n";
	}

	bool parse_unsigned(const char *text, std::uint64_t *value)
	{
		if (text == nullptr || *text == '\0' || value == nullptr) return false;
		char *end = nullptr;
		errno = 0;
		const auto parsed = std::strtoull(text, &end, 10);
		if (errno != 0 || end == text || *end != '\0' ||
			parsed > std::numeric_limits<std::uint64_t>::max())
		{
			return false;
		}
		*value = static_cast<std::uint64_t>(parsed);
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
			else if (argument == "--client") options->client_path = value;
			else if (argument == "--linear-velocity-mps") options->linear_velocity_mps = value;
			else if (argument == "--angular-velocity-radps") options->angular_velocity_radps = value;
			else if (argument == "--max-restarts")
			{
				std::uint64_t parsed = 0U;
				if (!parse_unsigned(value, &parsed) || parsed > std::numeric_limits<std::uint32_t>::max())
				{
					return false;
				}
				options->max_restarts = static_cast<std::uint32_t>(parsed);
			}
			else if (argument == "--create-timeout-ms")
			{
				if (!parse_unsigned(value, &options->create_timeout_ms)) return false;
			}
			else
			{
				std::cerr << "unknown option: " << argument << '\n';
				return false;
			}
		}
		return !options->command_channel.empty() && !options->feedback_channel.empty() &&
			!options->client_path.empty() && options->create_timeout_ms > 0U;
	}

	const char *event_name(edge_runtime::SupervisorEvent event) noexcept
	{
		switch (event)
		{
		case edge_runtime::SupervisorEvent::kSupervised: return "supervised";
		case edge_runtime::SupervisorEvent::kStallDetected: return "stall_detected";
		case edge_runtime::SupervisorEvent::kKilled: return "killed";
		case edge_runtime::SupervisorEvent::kRestartArmed: return "restart_armed";
		}
		return "unknown";
	}

	void report_event(const edge_runtime::SupervisorEventInfo &event, void *)
	{
		std::cout << "supervisor event=" << event_name(event.event)
			<< " pid=" << event.pid << " generation=" << event.generation
			<< " attempt=" << event.attempt << " delay_ns=" << event.delay_ns
			<< " signal=" << event.signal << '\n' << std::flush;
	}

	const char *outcome_name(edge_runtime::SupervisionOutcome outcome) noexcept
	{
		switch (outcome)
		{
		case edge_runtime::SupervisionOutcome::kCleanExit: return "clean_exit";
		case edge_runtime::SupervisionOutcome::kStopped: return "stopped";
		case edge_runtime::SupervisionOutcome::kRestartsExhausted: return "restarts_exhausted";
		}
		return "unknown";
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

	edge_runtime::SupervisorOptions supervisor_options;
	supervisor_options.channel_name = options.command_channel;
	supervisor_options.producer_argv = {
		options.client_path,
		"--command-channel", options.command_channel,
		"--feedback-channel", options.feedback_channel,
		"--linear-velocity-mps", options.linear_velocity_mps,
		"--angular-velocity-radps", options.angular_velocity_radps,
	};
	supervisor_options.initial_delay = std::chrono::milliseconds(100);
	supervisor_options.max_delay = std::chrono::seconds(2);
	supervisor_options.multiplier = 2U;
	supervisor_options.max_restarts = options.max_restarts;
	supervisor_options.stable_reset_window = std::chrono::seconds(60);
	supervisor_options.stall_grace = std::chrono::seconds(2);
	supervisor_options.watch_interval = std::chrono::milliseconds(100);
	supervisor_options.create_timeout =
		std::chrono::milliseconds(options.create_timeout_ms);
	supervisor_options.on_event = report_event;

	auto supervisor = edge_runtime::ProducerSupervisor::create(supervisor_options);
	if (!supervisor)
	{
		std::cerr << "supervisor create failed: "
			<< edge_runtime::to_string(supervisor.error().code) << '\n';
		return 3;
	}
	auto result = supervisor.value().run();
	if (!result)
	{
		std::cerr << "supervisor run failed: "
			<< edge_runtime::to_string(result.error().code) << '\n';
		return 4;
	}
	std::cout << "supervisor outcome=" << outcome_name(result.value().outcome)
		<< " spawn_attempts=" << result.value().spawn_attempts
		<< " restarts=" << result.value().restarts
		<< " last_generation=" << result.value().last_generation
		<< " last_exit_status=" << result.value().last_child_exit_status << '\n';
	if (!result.value().stdout_tail.empty())
	{
		std::cout << "child_stdout_tail_begin\n" << result.value().stdout_tail
			<< "\nchild_stdout_tail_end\n";
	}
	return result.value().outcome == edge_runtime::SupervisionOutcome::kRestartsExhausted
		? 5
		: 0;
}
