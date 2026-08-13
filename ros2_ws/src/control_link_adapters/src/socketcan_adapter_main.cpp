#include <exception>
#include <memory>

#include "control_link_adapters/socketcan_adapter.hpp"
#include "rclcpp/rclcpp.hpp"

int main(int argc, char **argv)
{
	rclcpp::init(argc, argv);
	try
	{
		auto node = std::make_shared<control_link_adapters::SocketCanAdapter>();
		// ROS callbacks 串行执行，CAN fd 只由 adapter 内部的独立 I/O thread 访问
		rclcpp::spin(node);
	}
	catch (const std::exception &exception)
	{
		RCLCPP_FATAL(
			rclcpp::get_logger("socketcan_adapter"),
			"SocketCanAdapter failed: %s",
			exception.what());
		rclcpp::shutdown();
		return 1;
	}

	rclcpp::shutdown();
	return 0;
}
