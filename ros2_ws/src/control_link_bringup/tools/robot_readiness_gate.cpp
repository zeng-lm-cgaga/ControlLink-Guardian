#include <chrono>
#include <cstdint>
#include <map>
#include <memory>
#include <optional>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

#include "controller_manager_msgs/srv/list_controllers.hpp"
#include "lifecycle_msgs/msg/state.hpp"
#include "lifecycle_msgs/srv/get_state.hpp"
#include "rclcpp/executors/single_threaded_executor.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2/time.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

namespace
{
	using namespace std::chrono_literals;
	using ListControllers = controller_manager_msgs::srv::ListControllers;
	using GetState = lifecycle_msgs::srv::GetState;

	class RosClockProgress final
	{
	public:
		void sample(std::int64_t now_ns) noexcept
		{
			if (now_ns <= 0)
			{
				return;
			}

			if (!first_positive_ns_.has_value())
			{
				first_positive_ns_ = now_ns;
				return;
			}
			if (now_ns > first_positive_ns_.value())
			{
				progressed_ = true;
			}
		}

		[[nodiscard]] bool progressed() const noexcept
		{
			return progressed_;
		}

	private:
		std::optional<std::int64_t> first_positive_ns_;
		bool progressed_{false};
	};

	template<typename FutureT>
	bool future_ready(
		rclcpp::executors::SingleThreadedExecutor & executor,
		const FutureT & future,
		std::chrono::milliseconds timeout)
	{
		return executor.spin_until_future_complete(future, timeout) ==
			rclcpp::FutureReturnCode::SUCCESS;
	}

	bool controllers_are_active(
		const rclcpp::Client<ListControllers>::SharedPtr & client,
		rclcpp::executors::SingleThreadedExecutor & executor)
	{
		if (!client->service_is_ready())
		{
			return false;
		}

		auto future = client->async_send_request(
			std::make_shared<ListControllers::Request>());
		if (!future_ready(executor, future, 500ms))
		{
			return false;
		}

		bool joint_state_active = false;
		bool diff_drive_active = false;
		// range-for 必须由局部 shared_ptr 保持 Response 生命周期，不能引用 future.get() 临时结果的成员
		const auto response = future.get();
		for (const auto & controller : response->controller)
		{
			if (controller.name == "joint_state_broadcaster")
			{
				joint_state_active = controller.state == "active";
			}
			else if (controller.name == "diff_drive_controller")
			{
				diff_drive_active = controller.state == "active";
			}
		}
		return joint_state_active && diff_drive_active;
	}

	bool lifecycle_nodes_are_active(
		const std::map<std::string, rclcpp::Client<GetState>::SharedPtr> & clients,
		rclcpp::executors::SingleThreadedExecutor & executor)
	{
		for (const auto & [node_name, client] : clients)
		{
			(void)node_name;
			if (!client->service_is_ready())
			{
				return false;
			}

			auto future = client->async_send_request(
				std::make_shared<GetState::Request>());
			if (!future_ready(executor, future, 300ms) ||
				future.get()->current_state.id !=
				lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE)
			{
				return false;
			}
		}
		return true;
	}

	int wait_for_readiness(const rclcpp::Node::SharedPtr & node)
	{
		const auto phase = node->declare_parameter<std::string>("phase", "");
		const auto timeout_ms = node->declare_parameter<std::int64_t>(
			"timeout_ms", 60000);
		const auto map_frame = node->declare_parameter<std::string>(
			"map_frame", "map");
		const auto base_frame = node->declare_parameter<std::string>(
			"base_frame", "base_footprint");
		const auto controller_manager_fqn = node->declare_parameter<std::string>(
			"controller_manager_fqn", "");
		if (phase != "controllers" && phase != "nav2")
		{
			throw std::invalid_argument(
				"phase must be controllers or nav2; actual=" + phase);
		}
		if (timeout_ms <= 0)
		{
			throw std::invalid_argument("timeout_ms must be positive");
		}
		if (phase == "controllers" &&
			(controller_manager_fqn.empty() || controller_manager_fqn.front() != '/'))
		{
			throw std::invalid_argument(
				"controller_manager_fqn must be an absolute node FQN");
		}

		rclcpp::executors::SingleThreadedExecutor executor;
		executor.add_node(node);
		RosClockProgress clock_progress;
		const auto deadline = std::chrono::steady_clock::now() +
			std::chrono::milliseconds{timeout_ms};
		auto next_log = std::chrono::steady_clock::now();

		if (phase == "controllers")
		{
			auto controllers_client = node->create_client<ListControllers>(
				controller_manager_fqn + "/list_controllers");
			while (rclcpp::ok() && std::chrono::steady_clock::now() < deadline)
			{
				executor.spin_some();
				clock_progress.sample(node->get_clock()->now().nanoseconds());
				if (clock_progress.progressed() &&
					controllers_are_active(controllers_client, executor))
				{
					RCLCPP_INFO(
						node->get_logger(),
						"Robot readiness passed: controllers ACTIVE and ROS clock advancing");
					return 0;
				}

				if (std::chrono::steady_clock::now() >= next_log)
				{
					RCLCPP_INFO(
						node->get_logger(),
						"Waiting for controllers ACTIVE and advancing ROS clock");
					next_log = std::chrono::steady_clock::now() + 2s;
				}
				std::this_thread::sleep_for(100ms);
			}
		}
		else
		{
			const std::vector<std::string> lifecycle_node_names{
				"map_server",
				"amcl",
				"planner_server",
				"controller_server",
				"smoother_server",
				"behavior_server",
				"bt_navigator",
				"waypoint_follower",
				"velocity_smoother"};
			std::map<std::string, rclcpp::Client<GetState>::SharedPtr> state_clients;
			for (const auto & node_name : lifecycle_node_names)
			{
				state_clients.emplace(
					node_name,
					node->create_client<GetState>("/" + node_name + "/get_state"));
			}

			auto tf_buffer = std::make_shared<tf2_ros::Buffer>(node->get_clock());
			auto tf_listener = std::make_shared<tf2_ros::TransformListener>(
				*tf_buffer,
				node,
				false);
			while (rclcpp::ok() && std::chrono::steady_clock::now() < deadline)
			{
				executor.spin_some();
				clock_progress.sample(node->get_clock()->now().nanoseconds());
				const bool lifecycle_ready = lifecycle_nodes_are_active(
					state_clients,
					executor);
				const bool tf_ready = tf_buffer->canTransform(
					map_frame,
					base_frame,
					tf2::TimePointZero);
				if (clock_progress.progressed() && lifecycle_ready && tf_ready)
				{
					RCLCPP_INFO(
						node->get_logger(),
						"Robot readiness passed: Nav2 ACTIVE, ROS clock advancing and %s -> %s available",
						map_frame.c_str(),
						base_frame.c_str());
					(void)tf_listener;
					return 0;
				}

				if (std::chrono::steady_clock::now() >= next_log)
				{
					RCLCPP_INFO(
						node->get_logger(),
						"Waiting for Nav2 ACTIVE, advancing ROS clock and complete TF");
					next_log = std::chrono::steady_clock::now() + 2s;
				}
				std::this_thread::sleep_for(100ms);
			}
		}

		RCLCPP_ERROR(
			node->get_logger(),
			"Robot readiness timed out: phase=%s, timeout_ms=%ld",
			phase.c_str(),
			static_cast<long>(timeout_ms));
		return 1;
	}
}  // namespace

int main(int argc, char ** argv)
{
	rclcpp::init(argc, argv);
	int exit_code = 1;
	try
	{
		auto node = std::make_shared<rclcpp::Node>("robot_readiness_gate");
		exit_code = wait_for_readiness(node);
	}
	catch (const std::exception & exception)
	{
		RCLCPP_FATAL(
			rclcpp::get_logger("robot_readiness_gate"),
			"Robot readiness gate failed: %s",
			exception.what());
	}
	rclcpp::shutdown();
	return exit_code;
}
