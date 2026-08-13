#include "control_link_adapters/controller_endpoint_monitor.hpp"

#include <algorithm>
#include <stdexcept>
#include <utility>

namespace control_link_adapters
{
	namespace
	{
		std::string endpoint_fqn(const rclcpp::TopicEndpointInfo &endpoint)
		{
			const auto node_namespace = endpoint.node_namespace();
			if (node_namespace.empty() || node_namespace == "/")
			{
				return "/" + endpoint.node_name();
			}

			if (node_namespace.back() == '/')
			{
				return node_namespace + endpoint.node_name();
			}

			return node_namespace + "/" + endpoint.node_name();
		}
	} // namespace

	ControllerEndpointMonitor::ControllerEndpointMonitor(
		std::string expected_node_fqn,
		std::string expected_type,
		rclcpp::QoS publisher_qos,
		std::chrono::milliseconds stable_window,
		std::chrono::milliseconds state_timeout)
		: expected_node_fqn_(std::move(expected_node_fqn)),
		  expected_type_(std::move(expected_type)),
		  publisher_qos_(std::move(publisher_qos)),
		  stable_window_(stable_window),
		  state_timeout_(state_timeout)
	{
		if (expected_node_fqn_.empty() || expected_type_.empty())
		{
			throw std::invalid_argument(
				"ControllerEndpointMonitor requires non-empty endpoint identity");
		}
		if (stable_window_ <= std::chrono::milliseconds::zero() ||
			state_timeout_ <= std::chrono::milliseconds::zero())
		{
			throw std::invalid_argument(
				"ControllerEndpointMonitor durations must be positive");
		}
	}

	std::vector<rclcpp::TopicEndpointInfo>
	ControllerEndpointMonitor::matching_endpoints(
		const std::vector<rclcpp::TopicEndpointInfo> &subscriptions) const
	{
		std::vector<rclcpp::TopicEndpointInfo> matches;
		for (const auto &endpoint : subscriptions)
		{
			const auto qos_result = rclcpp::qos_check_compatible(
				publisher_qos_,
				endpoint.qos_profile());
			if (endpoint.endpoint_type() == rclcpp::EndpointType::Subscription &&
				endpoint.topic_type() == expected_type_ &&
				endpoint_fqn(endpoint) == expected_node_fqn_ &&
				qos_result.compatibility == rclcpp::QoSCompatibility::Ok)
			{
				matches.push_back(endpoint);
			}
		}
		return matches;
	}

	ControllerEndpointSnapshot ControllerEndpointMonitor::observe(
		const std::vector<rclcpp::TopicEndpointInfo> &subscriptions,
		std::chrono::steady_clock::time_point observed_at)
	{
		if (last_observed_at_.has_value() &&
			observed_at < last_observed_at_.value())
		{
			throw std::invalid_argument(
				"controller endpoint observation time moved backwards");
		}
		last_observed_at_ = observed_at;

		const auto matches = matching_endpoints(subscriptions);
		if (matches.size() > 1U)
		{
			candidate_gid_.reset();
			candidate_since_.reset();
			current_ = ControllerEndpointSnapshot{
				ControllerEndpointState::kAmbiguous,
				std::nullopt};
			return current_;
		}

		if (matches.empty())
		{
			candidate_gid_.reset();
			candidate_since_.reset();
			const bool expected_role_present = std::any_of(
				subscriptions.begin(),
				subscriptions.end(),
				[this](const rclcpp::TopicEndpointInfo &endpoint)
				{
					return endpoint.endpoint_type() ==
						rclcpp::EndpointType::Subscription &&
						endpoint_fqn(endpoint) == expected_node_fqn_;
				});

			// 无目标 role 时允许 discovery 短暂漏报，其他观察订阅不应改变目标失联语义
			if (!expected_role_present && stable_gid_.has_value() &&
				last_confirmed_at_.has_value() &&
				observed_at - last_confirmed_at_.value() <= state_timeout_)
			{
				current_ = ControllerEndpointSnapshot{
					ControllerEndpointState::kHealthy,
					stable_gid_};
				return current_;
			}

			stable_gid_.reset();
			current_ = ControllerEndpointSnapshot{
				!expected_role_present && ever_confirmed_ ?
					ControllerEndpointState::kTimedOut :
					ControllerEndpointState::kUnavailable,
				std::nullopt};
			return current_;
		}

		const auto observed_gid = matches.front().endpoint_gid();
		if (stable_gid_.has_value() && stable_gid_.value() == observed_gid)
		{
			last_confirmed_at_ = observed_at;
			candidate_gid_.reset();
			candidate_since_.reset();
			current_ = ControllerEndpointSnapshot{
				ControllerEndpointState::kHealthy,
				stable_gid_};
			return current_;
		}

		if (!candidate_gid_.has_value() || candidate_gid_.value() != observed_gid)
		{
			candidate_gid_ = observed_gid;
			candidate_since_ = observed_at;
			current_ = ControllerEndpointSnapshot{
				ControllerEndpointState::kStabilizing,
				std::nullopt};
			return current_;
		}

		if (!candidate_since_.has_value())
		{
			throw std::logic_error(
				"controller endpoint candidate has no start time");
		}

		if (observed_at - candidate_since_.value() < stable_window_)
		{
			current_ = ControllerEndpointSnapshot{
				ControllerEndpointState::kStabilizing,
				std::nullopt};
			return current_;
		}

		stable_gid_ = observed_gid;
		candidate_gid_.reset();
		candidate_since_.reset();
		last_confirmed_at_ = observed_at;
		ever_confirmed_ = true;
		current_ = ControllerEndpointSnapshot{
			ControllerEndpointState::kHealthy,
			stable_gid_};
		return current_;
	}

	ControllerEndpointSnapshot ControllerEndpointMonitor::current() const noexcept
	{
		return current_;
	}
} // namespace control_link_adapters
