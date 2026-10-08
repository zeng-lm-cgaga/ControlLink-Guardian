#include <fstream>
#include <iostream>
#include <iterator>
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
					std::istreambuf_iterator<char>()));
			return 0;
		}

		for (int index = 1; index < argc; ++index)
		{
			exercise(read_file(argv[index]));
		}
		return 0;
	}
	catch (const std::exception &error)
	{
		std::cerr << error.what() << '\n';
		return 2;
	}
}
