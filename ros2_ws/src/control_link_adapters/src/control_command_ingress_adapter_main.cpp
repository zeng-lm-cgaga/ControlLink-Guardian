#include <memory>

#include "control_link_adapters/control_command_ingress_adapter.hpp"

int main(int argc, char ** argv)
{
	rclcpp::init(argc, argv);
	try
	{
		rclcpp::spin(std::make_shared<control_link_adapters::ControlCommandIngressAdapter>());
	}
	catch (const std::exception & exception)
	{
		RCLCPP_FATAL(rclcpp::get_logger("control_command_ingress_adapter"),
			"ControlCommand ingress failed: %s", exception.what());
		rclcpp::shutdown();
		return 1;
	}
	rclcpp::shutdown();
	return 0;
}
