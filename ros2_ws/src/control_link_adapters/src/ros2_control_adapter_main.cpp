#include <exception>
#include <memory>

#include "control_link_adapters/ros2_control_adapter.hpp"
#include "rclcpp/rclcpp.hpp"

int main(int argc, char **argv)
{
	rclcpp::init(argc, argv);
	try
	{
		auto node = std::make_shared<control_link_adapters::Ros2ControlAdapter>();
		// Node 自身的 canonical、odom、Graph、output 与 state callback 保持串行
		// TransformListener 使用自己的受控线程，只写入线程安全的 tf2 Buffer
		rclcpp::spin(node);
	}
	catch (const std::exception &exception)
	{
		RCLCPP_FATAL(
			rclcpp::get_logger("ros2_control_adapter"),
			"Ros2ControlAdapter failed: %s",
			exception.what());
		rclcpp::shutdown();
		return 1;
	}

	rclcpp::shutdown();
	return 0;
}
