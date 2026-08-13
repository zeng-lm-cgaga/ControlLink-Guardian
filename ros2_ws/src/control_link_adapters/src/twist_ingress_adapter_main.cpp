#include <memory>

#include "control_link_adapters/twist_ingress_adapter.hpp"
#include "rclcpp/rclcpp.hpp"

int main(int argc, char ** argv)
{
	rclcpp::init(argc, argv);
	try
	{
		auto node = std::make_shared<control_link_adapters::TwistIngressAdapter>();
		rclcpp::spin(node);
	}
	catch (const std::exception & exception)
	{
		RCLCPP_FATAL(
			rclcpp::get_logger("twist_ingress_adapter"),
			"Twist ingress failed: %s",
			exception.what());
		rclcpp::shutdown();
		return 1;
	}
	rclcpp::shutdown();
	return 0;
}
