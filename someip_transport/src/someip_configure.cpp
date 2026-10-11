#include <fstream>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <string>

#include "control_link_someip_transport/config.hpp"

int main(int argc, char **argv)
{
	try
	{
		if (argc != 8)
		{
			throw std::invalid_argument(
				"usage: someip_configure NETWORK_YAML OUTPUT_JSON APP_NAME APP_ID NETWORK_NAME server|client BIND_ADDRESS");
		}
		auto config = control_link::someip::load_config(argv[1]);
		std::size_t consumed = 0;
		const auto identity = std::stoul(argv[4], &consumed, 0);
		if (consumed != std::string{argv[4]}.size() || identity == 0 || identity >= 0xffff)
		{
			throw std::invalid_argument("application identity must be uint16 in [1,65534]");
		}
		const std::string role{argv[6]};
		if (role != "server" && role != "client") throw std::invalid_argument("invalid role");
		config.bind_address = argv[7];
		control_link::someip::validate_config(config);
		const auto json = control_link::someip::vsomeip_configuration_json(
			config, argv[3], static_cast<std::uint16_t>(identity), argv[5], role == "server");
		if (std::ifstream{argv[2]}.good()) throw std::invalid_argument("refusing to overwrite existing JSON evidence");
		std::ofstream output{argv[2]};
		output.exceptions(std::ios::failbit | std::ios::badbit);
		output << json << '\n';
		output.flush();
		output.close();
		return 0;
	}
	catch (const std::exception &error)
	{
		std::cerr << "someip_configure: " << error.what() << '\n';
		return 1;
	}
}
