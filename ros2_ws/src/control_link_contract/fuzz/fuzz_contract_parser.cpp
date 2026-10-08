#include <fstream>
#include <iostream>
#include <iterator>
#include <cstdint>
#include <stdexcept>
#include <string>

#include "control_link_contract/parser.hpp"

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

	void exercise(const std::string &input, const std::string &source_name)
	{
		try
		{
			(void)control_link_contract::parse_gateway_contract_text(input, source_name);
		}
		catch (const std::exception &)
		{
		}

		try
		{
			(void)control_link_contract::parse_source_policy_text(input, source_name);
		}
		catch (const std::exception &)
		{
		}
	}

	std::string mutate(const std::string &seed, std::uint32_t round)
	{
		std::uint32_t state = 0x9E3779B9U ^ round;
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

	void exercise_seed(const std::string &input, const std::string &source_name)
	{
		exercise(input, source_name);
		for (std::uint32_t round = 0U; round < 64U; ++round)
			exercise(mutate(input, round), source_name);
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
					std::istreambuf_iterator<char>()),
				"stdin");
			return 0;
		}

		for (int index = 1; index < argc; ++index)
		{
			const std::string path{argv[index]};
			exercise_seed(read_file(path), path);
		}
		return 0;
	}
	catch (const std::exception &error)
	{
		std::cerr << error.what() << '\n';
		return 2;
	}
}
