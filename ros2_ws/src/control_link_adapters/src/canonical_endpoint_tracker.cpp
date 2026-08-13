#include "control_link_adapters/canonical_endpoint_tracker.hpp"

#include <algorithm>
#include <stdexcept>
#include <utility>

namespace control_link_adapters
{
	namespace
	{
		CanonicalPublisherKey publisher_key(
			const rclcpp::TopicEndpointInfo &endpoint,
			std::string_view rmw_implementation)
		{
			return CanonicalPublisherKey{
				std::string(rmw_implementation),
				endpoint.endpoint_gid()};
		}

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

	CanonicalEndpointTracker::CanonicalEndpointTracker(
		std::string expected_type,
		std::string rmw_implementation,
		std::chrono::milliseconds stable_window)
		: expected_type_(std::move(expected_type)),
		  rmw_implementation_(std::move(rmw_implementation)),
		  stable_window_(stable_window)
	{
		if (expected_type_.empty() || rmw_implementation_.empty())
		{
			throw std::invalid_argument(
				"CanonicalEndpointTracker requires non-empty endpoint identity");
		}
		if (stable_window_ <= std::chrono::milliseconds::zero())
		{
			throw std::invalid_argument(
				"CanonicalEndpointTracker stable window must be positive");
		}
	}

	CanonicalEndpointSnapshot CanonicalEndpointTracker::assess(
		const std::vector<rclcpp::TopicEndpointInfo> &publishers) const
	{
		if (publishers.empty())
		{
			return CanonicalEndpointSnapshot{
				CanonicalEndpointState::kUnavailable,
				{},
				std::nullopt};
		}

		if (publishers.size() != 1U)
		{
			// canonical output 没有多个控制 authority 的合并语义
			return CanonicalEndpointSnapshot{
				CanonicalEndpointState::kAmbiguous,
				{},
				std::nullopt};
		}

		const auto &publisher = publishers.front();
		if (publisher.endpoint_type() != rclcpp::EndpointType::Publisher ||
			std::string_view(publisher.topic_type()) != expected_type_)
		{
			return CanonicalEndpointSnapshot{
				CanonicalEndpointState::kUnavailable,
				{},
				std::nullopt};
		}

		return CanonicalEndpointSnapshot{
			CanonicalEndpointState::kConfirmed,
			endpoint_fqn(publisher),
			publisher_key(publisher, rmw_implementation_)};
	}

	bool CanonicalEndpointTracker::same_snapshot(
		const CanonicalEndpointSnapshot &left,
		const CanonicalEndpointSnapshot &right) const noexcept
	{
		if (left.state != right.state)
		{
			return false;
		}

		if (left.state != CanonicalEndpointState::kConfirmed)
		{
			return true;
		}

		if (left.node_fqn != right.node_fqn ||
			left.confirmed_publisher.has_value() !=
				right.confirmed_publisher.has_value())
		{
			return false;
		}

		if (!left.confirmed_publisher.has_value())
		{
			return true;
		}

		return left.confirmed_publisher->rmw_implementation ==
				right.confirmed_publisher->rmw_implementation &&
			left.confirmed_publisher->publisher_gid ==
				right.confirmed_publisher->publisher_gid;
	}

	CanonicalEndpointSnapshot CanonicalEndpointTracker::observe(
		const std::vector<rclcpp::TopicEndpointInfo> &publishers,
		std::chrono::steady_clock::time_point observed_at)
	{
		if (last_observed_at_.has_value() &&
			observed_at < last_observed_at_.value())
		{
			throw std::invalid_argument(
				"canonical endpoint observation time moved backwards");
		}
		last_observed_at_ = observed_at;

		const auto observation = assess(publishers);
		if (stable_snapshot_.has_value() &&
			same_snapshot(observation, stable_snapshot_.value()))
		{
			candidate_snapshot_.reset();
			candidate_since_.reset();
			return stable_snapshot_.value();
		}

		if (!candidate_snapshot_.has_value() ||
			!same_snapshot(observation, candidate_snapshot_.value()))
		{
			candidate_snapshot_ = observation;
			candidate_since_ = observed_at;
			return CanonicalEndpointSnapshot{
				CanonicalEndpointState::kUnstable,
				{},
				std::nullopt};
		}

		if (!candidate_since_.has_value())
		{
			throw std::logic_error(
				"canonical endpoint candidate has no start time");
		}

		if (observed_at - candidate_since_.value() < stable_window_)
		{
			return CanonicalEndpointSnapshot{
				CanonicalEndpointState::kUnstable,
				{},
				std::nullopt};
		}

		stable_snapshot_ = candidate_snapshot_;
		candidate_snapshot_.reset();
		candidate_since_.reset();
		return stable_snapshot_.value();
	}

	CanonicalEndpointSnapshot CanonicalEndpointTracker::current() const
	{
		if (candidate_snapshot_.has_value())
		{
			return CanonicalEndpointSnapshot{
				CanonicalEndpointState::kUnstable,
				{},
				std::nullopt};
		}

		if (stable_snapshot_.has_value())
		{
			return stable_snapshot_.value();
		}

		return CanonicalEndpointSnapshot{
			CanonicalEndpointState::kUnstable,
			{},
			std::nullopt};
	}
} // namespace control_link_adapters
