#include "control_link_gateway/graph_monitor.hpp"

#include <stdexcept>
#include <string>
#include <string_view>
#include <utility>

namespace control_link_gateway
{
	namespace
	{
		bool same_stability_state(
			const SourceEndpointAssessment &left,
			const SourceEndpointAssessment &right) noexcept
		{
			if (left.state != right.state)
			{
				return false;
			}

			if (left.state != SourceEndpointState::kUsable)
			{
				return true;
			}

			return left.publisher_generation.has_value() &&
				right.publisher_generation.has_value() &&
				left.publisher_generation.value() ==
				right.publisher_generation.value();
		}
	} // namespace

	bool SourceEndpointAssessment::usable() const noexcept
	{
		return state == SourceEndpointState::kUsable;
	}

	PublisherGenerationKey publisher_generation_from_endpoint_info(
		const rclcpp::TopicEndpointInfo &endpoint,
		std::string_view rmw_implementation)
	{
		const auto gid = endpoint.endpoint_gid();
		return PublisherGenerationKey{
			std::string(rmw_implementation),
			gid};
	}

	SourceEndpointAssessment assess_source_publishers(
		const std::vector<rclcpp::TopicEndpointInfo> &publishers,
		std::string_view expected_type,
		std::string_view rmw_implementation,
		const rclcpp::QoS &local_qos,
		const control_link_contract::QosProfile &expected_remote_profile)
	{
		const auto publisher_count = publishers.size();
		if (publishers.empty())
		{
			return SourceEndpointAssessment{
				SourceEndpointState::kMissing,
				publisher_count,
				std::nullopt,
				std::nullopt};
		}

		if (publisher_count > 1U)
		{
			// 多个 Publisher 时来源身份不唯一，不能任选一个 GID 继续接收控制命令
			return SourceEndpointAssessment{
				SourceEndpointState::kAmbiguous,
				publisher_count,
				std::nullopt,
				std::nullopt};
		}

		const auto &publisher = publishers.front();
		if (publisher.endpoint_type() != rclcpp::EndpointType::Publisher)
		{
			return SourceEndpointAssessment{
				SourceEndpointState::kUnexpectedDirection,
				publisher_count,
				std::nullopt,
				std::nullopt};
		}

		if (std::string_view(publisher.topic_type()) != expected_type)
		{
			return SourceEndpointAssessment{
				SourceEndpointState::kTypeMismatch,
				publisher_count,
				std::nullopt,
				std::nullopt};
		}

		auto qos_report = control_link_contract::assess_endpoint_qos(
			control_link_contract::RemoteDirection::kPublisher,
			local_qos,
			publisher.qos_profile(),
			expected_remote_profile);

		const bool dds_compatible =
			qos_report.dds_compatibility == rclcpp::QoSCompatibility::Ok;
		if (!dds_compatible || !qos_report.observed_policies_match())
		{
			return SourceEndpointAssessment{
				SourceEndpointState::kQosMismatch,
				publisher_count,
				std::nullopt,
				std::move(qos_report)};
		}

		return SourceEndpointAssessment{
			SourceEndpointState::kUsable,
			publisher_count,
			publisher_generation_from_endpoint_info(
				publisher,
				rmw_implementation),
			std::move(qos_report)};
	}

	SourceEndpointStabilityEvent
	update_source_endpoint_stability(
		SourceEndpointStabilityTracker &tracker,
		const SourceEndpointAssessment &observation,
		std::chrono::steady_clock::time_point observed_at,
		std::chrono::milliseconds stable_window)
	{
		if (stable_window <= std::chrono::milliseconds::zero())
		{
			throw std::invalid_argument(
				"source endpoint stable window must be greater than zero");
		}

		const bool has_candidate = tracker.candidate_assessment.has_value();
		const bool has_candidate_since = tracker.candidate_since.has_value();
		if (has_candidate != has_candidate_since)
		{
			throw std::logic_error(
				"source endpoint stability candidate and start time are inconsistent");
		}

		if (
			tracker.last_observed_at.has_value() &&
			observed_at < tracker.last_observed_at.value())
		{
			throw std::invalid_argument(
				"source endpoint observation time must not move backwards");
		}

		if (
			tracker.candidate_since.has_value() &&
			observed_at < tracker.candidate_since.value())
		{
			throw std::logic_error(
				"source endpoint candidate start time is after the observation time");
		}
		tracker.last_observed_at = observed_at;

		if (
			tracker.stable_assessment.has_value() &&
			same_stability_state(
				observation,
				tracker.stable_assessment.value()))
		{
			// 健康状态未改变时刷新最新诊断细节，并取消尚未稳定的相反候选
			tracker.stable_assessment = observation;
			tracker.candidate_assessment.reset();
			tracker.candidate_since.reset();
			return SourceEndpointStabilityEvent::kUnchanged;
		}

		if (!tracker.candidate_assessment.has_value())
		{
			tracker.candidate_assessment = observation;
			tracker.candidate_since = observed_at;
			return SourceEndpointStabilityEvent::kPending;
		}

		if (!same_stability_state(
				tracker.candidate_assessment.value(),
				observation))
		{
			tracker.candidate_assessment = observation;
			tracker.candidate_since = observed_at;
			return SourceEndpointStabilityEvent::kPending;
		}

		// 保留首次观察时间，只刷新同一候选的最新诊断数据
		tracker.candidate_assessment = observation;
		const auto candidate_age = observed_at - tracker.candidate_since.value();
		if (candidate_age < stable_window)
		{
			return SourceEndpointStabilityEvent::kPending;
		}

		const bool initializing = !tracker.stable_assessment.has_value();
		tracker.stable_assessment = observation;
		tracker.candidate_assessment.reset();
		tracker.candidate_since.reset();

		return initializing ?
			SourceEndpointStabilityEvent::kInitialized :
			SourceEndpointStabilityEvent::kChanged;
	}
} // namespace control_link_gateway
