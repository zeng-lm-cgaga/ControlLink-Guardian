#include <charconv>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <stdexcept>
#include <string>
#include <string_view>
#include <system_error>

#include <nlohmann/json.hpp>

#include "control_link_contract/contract_bundle.hpp"
#include "control_link_gateway/decision_replay.hpp"
#include "control_link_gateway/decision_trace.hpp"

namespace
{
	constexpr int kExitSuccess = 0;
	constexpr int kExitUsage = 2;
	constexpr int kExitInvalid = 3;
	constexpr int kExitDivergence = 4;
	constexpr int kExitIo = 5;

	struct Options
	{
		std::filesystem::path profile_path;
		std::filesystem::path config_root;
		std::filesystem::path trace_path;
		std::filesystem::path result_path;
		std::uint32_t repeat{1U};
	};

	std::string_view require_value(int argc, char **argv, int &index)
	{
		if (index + 1 >= argc)
		{
			throw std::invalid_argument(
				std::string("missing value for argument: ") + argv[index]);
		}
		index += 1;
		return argv[index];
	}

	std::uint32_t parse_repeat(std::string_view text)
	{
		std::uint32_t result = 0U;
		const auto parsed = std::from_chars(
			text.data(),
			text.data() + text.size(),
			result);
		if (parsed.ec != std::errc{} || parsed.ptr != text.data() + text.size() ||
			result == 0U || result > 10'000U)
		{
			throw std::invalid_argument("--repeat must be an integer in [1, 10000]");
		}
		return result;
	}

	Options parse_options(int argc, char **argv)
	{
		Options result;
		bool saw_profile = false;
		bool saw_config_root = false;
		bool saw_trace = false;
		bool saw_result = false;
		bool saw_repeat = false;

		for (int index = 1; index < argc; ++index)
		{
			const std::string_view argument{argv[index]};
			if (argument == "--profile-path" && !saw_profile)
			{
				result.profile_path = require_value(argc, argv, index);
				saw_profile = true;
			}
			else if (argument == "--config-root" && !saw_config_root)
			{
				result.config_root = require_value(argc, argv, index);
				saw_config_root = true;
			}
			else if (argument == "--trace" && !saw_trace)
			{
				result.trace_path = require_value(argc, argv, index);
				saw_trace = true;
			}
			else if (argument == "--result" && !saw_result)
			{
				result.result_path = require_value(argc, argv, index);
				saw_result = true;
			}
			else if (argument == "--repeat" && !saw_repeat)
			{
				result.repeat = parse_repeat(require_value(argc, argv, index));
				saw_repeat = true;
			}
			else
			{
				throw std::invalid_argument(
					"unknown or duplicate argument: " + std::string(argument));
			}
		}

		if (!saw_profile || !saw_config_root || !saw_trace || !saw_result)
		{
			throw std::invalid_argument(
				"required arguments: --profile-path --config-root --trace --result");
		}
		return result;
	}

	void write_result(
		const std::filesystem::path &path,
		const std::string &content)
	{
		std::ofstream output{path, std::ios::binary | std::ios::trunc};
		if (!output)
		{
			throw std::runtime_error("failed to open replay result: " + path.string());
		}
		output << content << '\n';
		output.close();
		if (!output)
		{
			throw std::runtime_error("failed to write replay result: " + path.string());
		}
	}

	std::string error_result(
		std::string category,
		std::string message,
		std::uint64_t event_sequence = 0U)
	{
		return nlohmann::json{
			{"valid", false},
			{"matched", false},
			{"error_category", std::move(category)},
			{"event_sequence", event_sequence == 0U ?
				nlohmann::json(nullptr) : nlohmann::json(event_sequence)},
			{"message", std::move(message)}}.dump();
	}

	int run(const Options &options)
	{
		try
		{
			const auto bundle = control_link_contract::load_contract_bundle(
				options.profile_path,
				options.config_root);
			const auto trace = control_link_gateway::read_decision_trace_file(
				options.trace_path);
			const auto report = control_link_gateway::replay_decision_trace(
				bundle,
				trace,
				options.repeat);
			write_result(
				options.result_path,
				control_link_gateway::serialize_decision_replay_report(report));
			if (!report.matched)
			{
				std::cerr << "Decision Replay diverged at event "
					<< report.first_difference_event_sequence.value_or(0U)
					<< ": " << report.first_difference_field << '\n';
				return kExitDivergence;
			}
			std::cout << "Decision Replay matched " << report.result_count
				<< " results for " << report.completed_repetitions
				<< " repetitions\n";
			return kExitSuccess;
		}
		catch (const control_link_gateway::DecisionReplayError &exception)
		{
			write_result(
				options.result_path,
				error_result("invalid_replay", exception.what(), exception.event_sequence()));
			std::cerr << exception.what() << '\n';
			return kExitInvalid;
		}
		catch (const control_link_gateway::DecisionTraceError &exception)
		{
			write_result(
				options.result_path,
				error_result("invalid_trace", exception.what()));
			std::cerr << exception.what() << '\n';
			return kExitInvalid;
		}
	}
}  // namespace

int main(int argc, char **argv)
{
	try
	{
		return run(parse_options(argc, argv));
	}
	catch (const std::invalid_argument &exception)
	{
		std::cerr << "replay_decisions: " << exception.what() << '\n';
		return kExitUsage;
	}
	catch (const std::exception &exception)
	{
		std::cerr << "replay_decisions: " << exception.what() << '\n';
		return kExitIo;
	}
}
