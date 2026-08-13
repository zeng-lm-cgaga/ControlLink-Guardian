#include <chrono>
#include <cstdint>
#include <memory>
#include <optional>
#include <stdexcept>
#include <string>
#include <thread>

#include "lifecycle_msgs/msg/state.hpp"
#include "lifecycle_msgs/msg/transition.hpp"
#include "lifecycle_msgs/srv/change_state.hpp"
#include "lifecycle_msgs/srv/get_state.hpp"
#include "rclcpp/executors/single_threaded_executor.hpp"
#include "rclcpp/rclcpp.hpp"

namespace
{
	using namespace std::chrono_literals;
	using ChangeState = lifecycle_msgs::srv::ChangeState;
	using GetState = lifecycle_msgs::srv::GetState;

	template<typename FutureT>
	bool future_ready(
		rclcpp::executors::SingleThreadedExecutor & executor,
		const FutureT & future,
		std::chrono::milliseconds timeout)
	{
		return executor.spin_until_future_complete(future, timeout) ==
			rclcpp::FutureReturnCode::SUCCESS;
	}

	std::optional<std::uint8_t> try_read_state(
		const rclcpp::Client<GetState>::SharedPtr & client,
		rclcpp::executors::SingleThreadedExecutor & executor)
	{
		auto future = client->async_send_request(
			std::make_shared<GetState::Request>());
		if (!future_ready(executor, future, 2s))
		{
			return std::nullopt;
		}
		return future.get()->current_state.id;
	}

	std::optional<bool> try_request_transition(
		const rclcpp::Client<ChangeState>::SharedPtr & client,
		rclcpp::executors::SingleThreadedExecutor & executor,
		std::uint8_t transition_id)
	{
		auto request = std::make_shared<ChangeState::Request>();
		request->transition.id = transition_id;
		auto future = client->async_send_request(request);
		if (!future_ready(executor, future, 5s))
		{
			return std::nullopt;
		}
		return future.get()->success;
	}

	bool is_transitional_state(std::uint8_t state) noexcept
	{
		return state >= lifecycle_msgs::msg::State::TRANSITION_STATE_CONFIGURING &&
			state <= lifecycle_msgs::msg::State::TRANSITION_STATE_ERRORPROCESSING;
	}

	int activate_gateway(const rclcpp::Node::SharedPtr & node)
	{
		const auto target_fqn = node->declare_parameter<std::string>(
			"target_fqn", "/control_link/gateway");
		const auto timeout_ms = node->declare_parameter<std::int64_t>(
			"timeout_ms", 30000);
		if (target_fqn.empty() || target_fqn.front() != '/')
		{
			throw std::invalid_argument("target_fqn must be an absolute node FQN");
		}
		if (timeout_ms <= 0)
		{
			throw std::invalid_argument("timeout_ms must be positive");
		}

		auto get_state_client = node->create_client<GetState>(
			target_fqn + "/get_state");
		auto change_state_client = node->create_client<ChangeState>(
			target_fqn + "/change_state");
		rclcpp::executors::SingleThreadedExecutor executor;
		executor.add_node(node);
		const auto deadline = std::chrono::steady_clock::now() +
			std::chrono::milliseconds{timeout_ms};

		while (rclcpp::ok() && std::chrono::steady_clock::now() < deadline &&
			(!get_state_client->service_is_ready() ||
			!change_state_client->service_is_ready()))
		{
			executor.spin_some();
			std::this_thread::sleep_for(100ms);
		}
		if (!get_state_client->service_is_ready() ||
			!change_state_client->service_is_ready())
		{
			throw std::runtime_error("gateway Lifecycle services are unavailable");
		}

		auto next_log = std::chrono::steady_clock::now();
		while (rclcpp::ok() && std::chrono::steady_clock::now() < deadline)
		{
			const auto state_result = try_read_state(get_state_client, executor);
			if (!state_result.has_value())
			{
				if (std::chrono::steady_clock::now() >= next_log)
				{
					RCLCPP_WARN(
						node->get_logger(),
						"Gateway GetState timed out, retrying within bounded startup timeout");
					next_log = std::chrono::steady_clock::now() + 2s;
				}
				continue;
			}

			const auto state = state_result.value();
			if (state == lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE)
			{
				RCLCPP_INFO(
					node->get_logger(),
					"Gateway Lifecycle ACTIVE: %s",
					target_fqn.c_str());
				return 0;
			}
			if (is_transitional_state(state))
			{
				std::this_thread::sleep_for(100ms);
				continue;
			}
			if (state == lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED)
			{
				RCLCPP_INFO(node->get_logger(), "Configuring %s", target_fqn.c_str());
				const auto configured = try_request_transition(
					change_state_client,
					executor,
					lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);
				if (!configured.has_value() || !configured.value())
				{
					RCLCPP_WARN(
						node->get_logger(),
						"Gateway configure response unavailable, rechecking server state");
				}
				std::this_thread::sleep_for(100ms);
				continue;
			}
			if (state != lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE)
			{
				throw std::runtime_error(
					"gateway left INACTIVE while waiting for activation prerequisites");
			}

			const auto activated = try_request_transition(
				change_state_client,
				executor,
				lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE);
			if ((!activated.has_value() || !activated.value()) &&
				std::chrono::steady_clock::now() >= next_log)
			{
				RCLCPP_INFO(
					node->get_logger(),
					"Gateway activation gate not ready, retrying within bounded timeout");
				next_log = std::chrono::steady_clock::now() + 2s;
			}
			std::this_thread::sleep_for(200ms);
		}

		throw std::runtime_error("gateway activation timed out");
	}
}  // namespace

int main(int argc, char ** argv)
{
	rclcpp::init(argc, argv);
	int exit_code = 1;
	try
	{
		auto node = std::make_shared<rclcpp::Node>("gateway_lifecycle_activator");
		exit_code = activate_gateway(node);
	}
	catch (const std::exception & exception)
	{
		RCLCPP_FATAL(
			rclcpp::get_logger("gateway_lifecycle_activator"),
			"Gateway Lifecycle activation failed: %s",
			exception.what());
	}
	rclcpp::shutdown();
	return exit_code;
}
