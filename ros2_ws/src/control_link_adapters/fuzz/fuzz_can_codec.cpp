#include <algorithm>
#include <cstdint>
#include <fstream>
#include <iostream>
#include <iterator>
#include <limits>
#include <stdexcept>
#include <string>

#include "control_link_adapters/can_codec.hpp"
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

	void exercise(
		const std::string &input,
		const control_link_adapters::CanCodec &codec)
	{
		control_link_adapters::CanFrame frame{};
		const auto byte_at = [&input](std::size_t index) -> std::uint8_t
		{
			return index < input.size() ?
				static_cast<std::uint8_t>(static_cast<unsigned char>(input[index])) :
				0U;
		};
		frame.can_id = static_cast<std::uint32_t>(byte_at(0U)) |
			(static_cast<std::uint32_t>(byte_at(1U)) << 8U);
		frame.dlc = byte_at(2U);
		frame.is_extended = (byte_at(3U) & 0x01U) != 0U;
		frame.is_remote = (byte_at(3U) & 0x02U) != 0U;
		for (std::size_t index = 0U; index < frame.data.size(); ++index)
		{
			frame.data[index] = byte_at(index + 4U);
		}

		(void)codec.decode_control(frame);
		(void)codec.decode_state(frame);

		control_link_adapters::RollingCounterChecker checker{
			codec.config().rolling_counter_modulo};
		for (std::size_t index = 4U; index < input.size(); ++index)
		{
			(void)checker.observe(static_cast<std::uint8_t>(
				static_cast<unsigned char>(input[index])));
		}

		const auto speed_byte = byte_at(4U);
		const auto yaw_byte = byte_at(5U);
		const double speed = (speed_byte & 0x80U) != 0U ?
			std::numeric_limits<double>::quiet_NaN() :
			(static_cast<double>(static_cast<std::int8_t>(speed_byte)) / 10.0);
		const double yaw_rate = (yaw_byte & 0x80U) != 0U ?
			std::numeric_limits<double>::infinity() :
			(static_cast<double>(static_cast<std::int8_t>(yaw_byte)) / 10.0);
		(void)codec.encode_control(
			control_link_adapters::CanControlSignals{
				static_cast<control_link_adapters::CanControlMode>(byte_at(6U) % 3U),
				speed,
				yaw_rate,
				byte_at(7U)});
		(void)codec.encode_state(
			control_link_adapters::CanVehicleStateSignals{
				static_cast<control_link_adapters::CanVehicleMode>(byte_at(6U) % 5U),
				speed,
				yaw_rate,
				byte_at(7U),
				byte_at(8U),
				byte_at(9U)});
	}
}

int main(int argc, char **argv)
{
	try
	{
		const auto signal_map = control_link_contract::load_can_signal_map(
			CONTROL_LINK_FUZZ_CAN_SIGNAL_MAP_PATH);
		control_link_adapters::CanCodec codec{*signal_map};
		if (argc == 1)
		{
			exercise(
				std::string(
					std::istreambuf_iterator<char>(std::cin),
					std::istreambuf_iterator<char>()),
				codec);
			return 0;
		}

		for (int index = 1; index < argc; ++index)
		{
			const std::string path{argv[index]};
			exercise(read_file(path), codec);
		}
		return 0;
	}
	catch (const std::exception &error)
	{
		std::cerr << error.what() << '\n';
		return 2;
	}
}
