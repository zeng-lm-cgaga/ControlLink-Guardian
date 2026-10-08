#include <fstream>
#include <iostream>
#include <iterator>
#include <cstddef>
#include <cstdint>
#include <stdexcept>
#include <sstream>
#include <string>

#include "control_link_gateway/decision_trace.hpp"

namespace
{
	std::string read_file(const std::string &path)
	{
		std::ifstream input(path, std::ios::binary);
		if (!input)
		{
			throw std::runtime_error("cannot open fuzz input: " + path);
		}
		return std::string(
			std::istreambuf_iterator<char>(input),
			std::istreambuf_iterator<char>());
	}

	void exercise(const std::string &input)
	{
		try
		{
			(void)control_link_gateway::parse_decision_trace_record(input, 1U);
		}
		catch (const std::exception &)
		{
		}

		try
		{
			std::istringstream stream{input};
			(void)control_link_gateway::read_decision_trace(stream);
		}
		catch (const std::exception &)
		{
		}
	}

	std::string mutate(const std::string &seed, std::uint32_t round)
	{
		std::uint32_t state = 0xC2B2AE35U ^ round;
		state = state * 1'664'525U + 1'013'904'223U;
		std::string result = seed;
		if (result.empty())
		{
			result.push_back(static_cast<char>(state & 0xFFU));
			return result;
		}

		const auto position = static_cast<std::size_t>(state % result.size());
		switch (round % 4U)
		{
		case 0U:
			result[position] = static_cast<char>(
				static_cast<unsigned char>(result[position]) ^
				static_cast<unsigned char>(1U << (state % 8U)));
			break;
		case 1U:
			if (result.size() > 1U)
				result.erase(position, 1U);
			break;
		case 2U:
			result.insert(
				position,
				1U,
				static_cast<char>(state & 0xFFU));
			break;
		case 3U:
			result[position] = '\0';
			break;
		}
		return result;
	}

	void exercise_seed(const std::string &input)
	{
		exercise(input);
		for (std::uint32_t round = 0U; round < 64U; ++round)
			exercise(mutate(input, round));
	}
}

int main(int argc, char **argv)
{
	try
	{
		if (argc == 1)
		{
			exercise_seed(
				std::string(
					std::istreambuf_iterator<char>(std::cin),
					std::istreambuf_iterator<char>()));
			return 0;
		}

		for (int index = 1; index < argc; ++index)
		{
			exercise_seed(read_file(argv[index]));
		}
		return 0;
	}
	catch (const std::exception &error)
	{
		std::cerr << error.what() << '\n';
		return 2;
	}
}
