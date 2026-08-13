#include <exception>
#include <memory>

#include "control_link_gateway/control_gateway_node.hpp"
#include "rclcpp/executors/multi_threaded_executor.hpp"
#include "rclcpp/rclcpp.hpp"

int main(int argc, char **argv)
{
	rclcpp::init(argc, argv);
	int exit_code = 0;

	try
	{
		auto node = std::make_shared<control_link_gateway::ControlGatewayNode>();
		rclcpp::ExecutorOptions executor_options;
		rclcpp::executors::MultiThreadedExecutor executor(
			executor_options,
			2U);
		executor.add_node(node->get_node_base_interface());
		executor.spin();
	}
	catch (const std::exception &exception)
	{
		RCLCPP_FATAL(
			rclcpp::get_logger("control_link_gateway"),
			"Gateway process terminated with an exception: %s",
			exception.what());
		exit_code = 1;
	}
	catch (...)
	{
		RCLCPP_FATAL(
			rclcpp::get_logger("control_link_gateway"),
			"Gateway process terminated with an unknown exception");
		exit_code = 1;
	}

	if (rclcpp::ok())
	{
		rclcpp::shutdown();
	}

	return exit_code;
}
