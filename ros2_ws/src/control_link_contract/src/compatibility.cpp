#include "control_link_contract/compatibility.hpp"

#include <cstdint>
#include <limits>
#include <stdexcept>
#include <string>
#include <string_view>
#include <utility>

namespace control_link_contract
{

	namespace
	{
		std::string unsupported_policy_name(int policy)
		{
			return "unsupported(" + std::to_string(policy) + ")";
		}

		void add_mismatch(
			QosCompatibilityReport & report,
			std::string policy,
			std::string expected,
			std::string actual)
		{
			report.exact_mismatches.push_back(
				QosPolicyMismatch{
					std::move(policy),
					std::move(expected),
					std::move(actual)});
		}

		void add_observation_gap(
			QosCompatibilityReport & report,
			std::string policy,
			std::string expected,
			std::string actual)
		{
			report.unobservable_policies.push_back(
				QosPolicyObservationGap{
					std::move(policy),
					std::move(expected),
					std::move(actual)});
		}

		std::string reliability_name(rclcpp::ReliabilityPolicy policy)
		{
			switch (policy) {
				case rclcpp::ReliabilityPolicy::Reliable:
					return "reliable";
				case rclcpp::ReliabilityPolicy::BestEffort:
					return "best_effort";
				case rclcpp::ReliabilityPolicy::SystemDefault:
					return "system_default";
				case rclcpp::ReliabilityPolicy::Unknown:
					return "unknown";
			}

			// actual policy 来自远端 Graph，未命名值应进入 mismatch，而不是中断监测
			return unsupported_policy_name(static_cast<int>(policy));
		}

		std::string durability_name(rclcpp::DurabilityPolicy policy)
		{
			switch (policy) {
				case rclcpp::DurabilityPolicy::Volatile:
					return "volatile";
				case rclcpp::DurabilityPolicy::TransientLocal:
					return "transient_local";
				case rclcpp::DurabilityPolicy::SystemDefault:
					return "system_default";
				case rclcpp::DurabilityPolicy::Unknown:
					return "unknown";
			}

			return unsupported_policy_name(static_cast<int>(policy));
		}

		std::string history_name(rclcpp::HistoryPolicy policy)
		{
			switch (policy) {
				case rclcpp::HistoryPolicy::KeepLast:
					return "keep_last";
				case rclcpp::HistoryPolicy::KeepAll:
					return "keep_all";
				case rclcpp::HistoryPolicy::SystemDefault:
					return "system_default";
				case rclcpp::HistoryPolicy::Unknown:
					return "unknown";
			}

			return unsupported_policy_name(static_cast<int>(policy));
		}

		std::string liveliness_name(rclcpp::LivelinessPolicy policy)
		{
			switch (policy) {
				case rclcpp::LivelinessPolicy::Automatic:
					return "automatic";
				case rclcpp::LivelinessPolicy::ManualByTopic:
					return "manual_by_topic";
				case rclcpp::LivelinessPolicy::SystemDefault:
					return "system_default";
				case rclcpp::LivelinessPolicy::Unknown:
					return "unknown";
			}

			return unsupported_policy_name(static_cast<int>(policy));
		}

		std::int64_t duration_nanoseconds_from_ms(
			std::uint64_t milliseconds,
			std::string_view policy_name)
		{
			// rclcpp::Duration 使用有符号纳秒，乘法前先验证范围以避免无符号溢出
			constexpr std::uint64_t kNanosecondsPerMillisecond = 1'000'000U;
			constexpr auto kMaxNanoseconds =
				static_cast<std::uint64_t>(std::numeric_limits<rcl_duration_value_t>::max());

			if (milliseconds == 0U) {
				throw std::invalid_argument(std::string(policy_name) + " must be greater than zero");
			}

			if (milliseconds > kMaxNanoseconds / kNanosecondsPerMillisecond) {
				throw std::out_of_range(
					std::string(policy_name) + " exceeds rclcpp::Duration range");
			}

			return static_cast<std::int64_t>(milliseconds * kNanosecondsPerMillisecond);
		}

		std::string duration_name(std::int64_t nanoseconds)
		{
			return std::to_string(nanoseconds) + "ns";
		}

	} // namespace

	bool QosCompatibilityReport::exact_match() const noexcept
	{
		return exact_mismatches.empty() && unobservable_policies.empty();
	}

	bool QosCompatibilityReport::observed_policies_match() const noexcept
	{
		return exact_mismatches.empty();
	}

	QosCompatibilityReport assess_endpoint_qos(
		RemoteDirection remote_direction,
		const rclcpp::QoS & local_qos,
		const rclcpp::QoS & actual_remote_qos,
		const QosProfile & expected_remote_profile)
	{
		rclcpp::QoSCheckCompatibleResult dds_result;

		// qos_check_compatible 的参数顺序固定为 offered publisher -> requested subscription
		switch (remote_direction) {
			case RemoteDirection::kPublisher:
				dds_result = rclcpp::qos_check_compatible(
					actual_remote_qos,
					local_qos);
				break;

			case RemoteDirection::kSubscription:
				dds_result = rclcpp::qos_check_compatible(
					local_qos,
					actual_remote_qos);
				break;

			default:
				throw std::invalid_argument("unsupported remote endpoint direction");
		}

		QosCompatibilityReport report{
			dds_result.compatibility,
			dds_result.reason,
			{},
			{}};

		const auto actual_reliability = actual_remote_qos.reliability();
		switch (expected_remote_profile.reliability) {
			case ReliabilityPolicy::kReliable:
				if (actual_reliability != rclcpp::ReliabilityPolicy::Reliable) {
					add_mismatch(
						report,
						"reliability",
						"reliable",
						reliability_name(actual_reliability));
				}
				break;

			case ReliabilityPolicy::kBestEffort:
				if (actual_reliability != rclcpp::ReliabilityPolicy::BestEffort) {
					add_mismatch(
						report,
						"reliability",
						"best_effort",
						reliability_name(actual_reliability));
				}
				break;

			default:
				throw std::invalid_argument("unsupported expected reliability policy");
		}

		const auto actual_durability = actual_remote_qos.durability();
		switch (expected_remote_profile.durability) {
			case DurabilityPolicy::kVolatile:
				if (actual_durability != rclcpp::DurabilityPolicy::Volatile) {
					add_mismatch(
						report,
						"durability",
						"volatile",
						durability_name(actual_durability));
				}
				break;

			case DurabilityPolicy::kTransientLocal:
				if (actual_durability != rclcpp::DurabilityPolicy::TransientLocal) {
					add_mismatch(
						report,
						"durability",
						"transient_local",
						durability_name(actual_durability));
				}
				break;

			default:
				throw std::invalid_argument("unsupported expected durability policy");
		}

		const auto actual_history = actual_remote_qos.history();
		const bool history_unobservable =
			actual_history == rclcpp::HistoryPolicy::Unknown ||
			actual_history == rclcpp::HistoryPolicy::SystemDefault;
		switch (expected_remote_profile.history) {
			case HistoryPolicy::kKeepLast:
				if (history_unobservable) {
					// Humble/FastDDS 的 Graph endpoint info 会将 history/depth 回报为 UNKNOWN/0
					add_observation_gap(
						report,
						"history",
						"keep_last",
						history_name(actual_history));
					add_observation_gap(
						report,
						"depth",
						std::to_string(expected_remote_profile.depth),
						std::to_string(actual_remote_qos.depth()));
				} else if (actual_history != rclcpp::HistoryPolicy::KeepLast) {
					add_mismatch(
						report,
						"history",
						"keep_last",
						history_name(actual_history));
				} else if (actual_remote_qos.depth() != expected_remote_profile.depth) {
					add_mismatch(
						report,
						"depth",
						std::to_string(expected_remote_profile.depth),
						std::to_string(actual_remote_qos.depth()));
				}
				break;

			case HistoryPolicy::kKeepAll:
				if (history_unobservable) {
					add_observation_gap(
						report,
						"history",
						"keep_all",
						history_name(actual_history));
				} else if (actual_history != rclcpp::HistoryPolicy::KeepAll) {
					add_mismatch(
						report,
						"history",
						"keep_all",
						history_name(actual_history));
				}
				break;

			default:
				throw std::invalid_argument("unsupported expected history policy");
		}

		if (expected_remote_profile.deadline_ms.has_value()) {
			const auto expected_nanoseconds = duration_nanoseconds_from_ms(
				expected_remote_profile.deadline_ms.value(),
				"deadline_ms");
			const auto actual_nanoseconds = actual_remote_qos.deadline().nanoseconds();
			if (actual_nanoseconds != expected_nanoseconds) {
				add_mismatch(
					report,
					"deadline",
					duration_name(expected_nanoseconds),
					duration_name(actual_nanoseconds));
			}
		}

		if (expected_remote_profile.lifespan_ms.has_value()) {
			const auto expected_nanoseconds = duration_nanoseconds_from_ms(
				expected_remote_profile.lifespan_ms.value(),
				"lifespan_ms");
			const auto actual_nanoseconds = actual_remote_qos.lifespan().nanoseconds();
			if (actual_nanoseconds != expected_nanoseconds) {
				add_mismatch(
					report,
					"lifespan",
					duration_name(expected_nanoseconds),
					duration_name(actual_nanoseconds));
			}
		}

		if (expected_remote_profile.liveliness.has_value()) {
			const auto actual_liveliness = actual_remote_qos.liveliness();
			switch (expected_remote_profile.liveliness.value()) {
				case LivelinessPolicy::kAutomatic:
					if (actual_liveliness != rclcpp::LivelinessPolicy::Automatic) {
						add_mismatch(
							report,
							"liveliness",
							"automatic",
							liveliness_name(actual_liveliness));
					}
					break;

				case LivelinessPolicy::kManualByTopic:
					if (actual_liveliness != rclcpp::LivelinessPolicy::ManualByTopic) {
						add_mismatch(
							report,
							"liveliness",
							"manual_by_topic",
							liveliness_name(actual_liveliness));
					}
					break;

				default:
					throw std::invalid_argument("unsupported expected liveliness policy");
			}
		}

		if (expected_remote_profile.liveliness_lease_duration_ms.has_value()) {
			const auto expected_nanoseconds = duration_nanoseconds_from_ms(
				expected_remote_profile.liveliness_lease_duration_ms.value(),
				"liveliness_lease_duration_ms");
			const auto actual_nanoseconds =
				actual_remote_qos.liveliness_lease_duration().nanoseconds();
			if (actual_nanoseconds != expected_nanoseconds) {
				add_mismatch(
					report,
					"liveliness_lease_duration",
					duration_name(expected_nanoseconds),
					duration_name(actual_nanoseconds));
			}
		}

		return report;
	}

}  // namespace control_link_contract
