#include "control_link_gateway/critical_endpoint_monitor.hpp"

#include <algorithm>
#include <array>
#include <cstdint>
#include <stdexcept>
#include <string_view>
#include <utility>

#include <rmw/types.h>

namespace control_link_gateway
{
	namespace
	{
		bool same_generation_set(
			const std::vector<rclcpp::TopicEndpointInfo> &left,
			const std::vector<rclcpp::TopicEndpointInfo> &right) noexcept
		{
			if (left.size() != right.size())
			{
				return false;
			}

			std::vector<std::array<std::uint8_t, RMW_GID_STORAGE_SIZE>> left_gids;
			std::vector<std::array<std::uint8_t, RMW_GID_STORAGE_SIZE>> right_gids;
			left_gids.reserve(left.size());
			right_gids.reserve(right.size());
			for (const auto &endpoint : left)
			{
				left_gids.push_back(endpoint.endpoint_gid());
			}
			for (const auto &endpoint : right)
			{
				right_gids.push_back(endpoint.endpoint_gid());
			}
			std::sort(left_gids.begin(), left_gids.end());
			std::sort(right_gids.begin(), right_gids.end());
			return left_gids == right_gids;
		}

		bool same_critical_endpoint_state(
			const CriticalEndpointAssessment &left,
			const CriticalEndpointAssessment &right) noexcept
		{
			return left.exact_qos_required == right.exact_qos_required &&
				left.identity.role_count_healthy == right.identity.role_count_healthy &&
				left.identity.additional_endpoints_healthy ==
					right.identity.additional_endpoints_healthy &&
				left.identity.matching_role_count == right.identity.matching_role_count &&
				left.qos.dds_compatible == right.qos.dds_compatible &&
				left.qos.observed_policy_match == right.qos.observed_policy_match &&
				left.qos.observation_complete == right.qos.observation_complete &&
				same_generation_set(
					left.identity.matching_role_endpoints,
					right.identity.matching_role_endpoints);
		}

		rclcpp::EndpointType expected_endpoint_type(
			control_link_contract::RemoteDirection direction)
		{
			switch (direction)
			{
			case control_link_contract::RemoteDirection::kPublisher:
				return rclcpp::EndpointType::Publisher;

			case control_link_contract::RemoteDirection::kSubscription:
				return rclcpp::EndpointType::Subscription;

			default:
				throw std::invalid_argument(
					"unsupported critical endpoint remote direction");
			}
		}

		bool matches_role(
			const rclcpp::TopicEndpointInfo &endpoint,
			const control_link_contract::CriticalEndpoint &expected)
		{
			return endpoint.endpoint_type() ==
				expected_endpoint_type(expected.remote_direction) &&
				std::string_view(endpoint.topic_type()) == expected.type &&
				endpoint_node_fqn(endpoint) == expected.remote_node_fqn;
		}
	} // namespace

	std::string endpoint_node_fqn(
		const rclcpp::TopicEndpointInfo &endpoint)
	{
		const auto &node_namespace = endpoint.node_namespace();
		const auto &node_name = endpoint.node_name();
		if (node_namespace.empty() || node_namespace == "/")
		{
			return "/" + node_name;
		}

		if (node_namespace.back() == '/')
		{
			return node_namespace + node_name;
		}

		return node_namespace + "/" + node_name;
	}

	CriticalEndpointIdentityAssessment assess_critical_endpoint_identity(
		const std::vector<rclcpp::TopicEndpointInfo> &endpoints,
		const control_link_contract::CriticalEndpoint &expected)
	{
		CriticalEndpointIdentityAssessment assessment;
		assessment.discovered_count = endpoints.size();
		assessment.matching_role_endpoints.reserve(endpoints.size());

		for (const auto &endpoint : endpoints)
		{
			if (matches_role(endpoint, expected))
			{
				assessment.matching_role_endpoints.push_back(endpoint);
				continue;
			}

			++assessment.additional_endpoint_count;
		}

		assessment.matching_role_count =
			assessment.matching_role_endpoints.size();
		assessment.role_count_healthy =
			assessment.matching_role_count >= expected.min_count &&
			(!expected.max_count.has_value() ||
				assessment.matching_role_count <= expected.max_count.value());
		assessment.additional_endpoints_healthy =
			expected.allow_additional_endpoints ||
			assessment.additional_endpoint_count == 0U;
		return assessment;
	}

	CriticalEndpointQosAssessment assess_critical_endpoint_qos(
		const CriticalEndpointIdentityAssessment &identity,
		const control_link_contract::CriticalEndpoint &expected,
		const rclcpp::QoS &local_qos,
		const control_link_contract::QosProfile &expected_remote_profile)
	{
		if (identity.matching_role_count != identity.matching_role_endpoints.size())
		{
			throw std::logic_error(
				"critical endpoint identity count does not match role endpoint list");
		}

		CriticalEndpointQosAssessment assessment;
		if (identity.matching_role_endpoints.empty())
		{
			return assessment;
		}

		assessment.endpoint_reports.reserve(identity.matching_role_endpoints.size());
		assessment.dds_compatible = true;
		assessment.observed_policy_match = true;
		assessment.observation_complete = true;
		for (const auto &endpoint : identity.matching_role_endpoints)
		{
			auto report = control_link_contract::assess_endpoint_qos(
				expected.remote_direction,
				local_qos,
				endpoint.qos_profile(),
				expected_remote_profile);
			assessment.dds_compatible =
				assessment.dds_compatible &&
				report.dds_compatibility == rclcpp::QoSCompatibility::Ok;
			assessment.observed_policy_match =
				assessment.observed_policy_match &&
				report.observed_policies_match();
			assessment.observation_complete =
				assessment.observation_complete &&
				report.unobservable_policies.empty();
			assessment.endpoint_reports.push_back(std::move(report));
		}

		return assessment;
	}

	CriticalEndpointStabilityEvent update_critical_endpoint_stability(
		CriticalEndpointStabilityTracker &tracker,
		const CriticalEndpointAssessment &observation,
		std::chrono::steady_clock::time_point observed_at,
		std::chrono::milliseconds stable_window)
	{
		if (stable_window <= std::chrono::milliseconds::zero())
		{
			throw std::invalid_argument(
				"critical endpoint stable window must be greater than zero");
		}

		const bool has_candidate = tracker.candidate_assessment.has_value();
		const bool has_candidate_since = tracker.candidate_since.has_value();
		if (has_candidate != has_candidate_since)
		{
			throw std::logic_error(
				"critical endpoint candidate and start time are inconsistent");
		}

		if (tracker.last_observed_at.has_value() &&
			observed_at < tracker.last_observed_at.value())
		{
			throw std::invalid_argument(
				"critical endpoint observation time must not move backwards");
		}

		if (tracker.candidate_since.has_value() &&
			observed_at < tracker.candidate_since.value())
		{
			throw std::logic_error(
				"critical endpoint candidate start time is after observation time");
		}
		tracker.last_observed_at = observed_at;

		if (tracker.stable_assessment.has_value() &&
			same_critical_endpoint_state(
				observation,
				tracker.stable_assessment.value()))
		{
			// 同一控制语义下只更新最新的 Graph/QoS 诊断细节
			tracker.stable_assessment = observation;
			tracker.candidate_assessment.reset();
			tracker.candidate_since.reset();
			return CriticalEndpointStabilityEvent::kUnchanged;
		}

		if (!tracker.candidate_assessment.has_value())
		{
			tracker.candidate_assessment = observation;
			tracker.candidate_since = observed_at;
			return CriticalEndpointStabilityEvent::kPending;
		}

		if (!same_critical_endpoint_state(
				tracker.candidate_assessment.value(),
				observation))
		{
			tracker.candidate_assessment = observation;
			tracker.candidate_since = observed_at;
			return CriticalEndpointStabilityEvent::kPending;
		}

		tracker.candidate_assessment = observation;
		if (observed_at - tracker.candidate_since.value() < stable_window)
		{
			return CriticalEndpointStabilityEvent::kPending;
		}

		const bool initializing = !tracker.stable_assessment.has_value();
		tracker.stable_assessment = observation;
		tracker.candidate_assessment.reset();
		tracker.candidate_since.reset();
		return initializing ?
			CriticalEndpointStabilityEvent::kInitialized :
			CriticalEndpointStabilityEvent::kChanged;
	}
} // namespace control_link_gateway
