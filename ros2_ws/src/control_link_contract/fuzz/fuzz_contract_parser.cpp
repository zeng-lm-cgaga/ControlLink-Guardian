#include <fstream>
#include <iostream>
#include <iterator>
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
}

int main(int argc, char **argv)
{
	try
	{
		if (argc == 1)
		{
			exercise(
				std::string(
					std::istreambuf_iterator<char>(std::cin),
					std::istreambuf_iterator<char>()),
				"stdin");
			return 0;
		}

		for (int index = 1; index < argc; ++index)
		{
			const std::string path{argv[index]};
			exercise(read_file(path), path);
		}
		return 0;
	}
	catch (const std::exception &error)
	{
		std::cerr << error.what() << '\n';
		return 2;
	}
}
