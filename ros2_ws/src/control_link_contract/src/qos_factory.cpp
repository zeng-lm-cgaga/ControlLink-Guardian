#include "control_link_contract/qos_factory.hpp"

#include <cstdint>
#include <limits>
#include <stdexcept>
#include <string>
#include <string_view>
#include <utility>

#include <rclcpp/duration.hpp>

namespace control_link_contract
{
	namespace
	{
		rclcpp::QoS make_history_qos(const QosProfile &profile)
		{
			switch (profile.history)
			{
				case HistoryPolicy::kKeepLast:
					if (profile.depth == 0U)
					{
						throw std::invalid_argument("keep_last depth must be at least 1");
					}
					return rclcpp::QoS(rclcpp::KeepLast(profile.depth));

				case HistoryPolicy::kKeepAll:
					return rclcpp::QoS(rclcpp::KeepAll());
			}

			throw std::invalid_argument("unsupported history policy");
		}

		rclcpp::Duration duration_from_ms(
			std::uint64_t milliseconds,
			std::string_view policy_name)
		{
			constexpr std::uint64_t kNanosecondsPerMillisecond = 1'000'000U;

			if (milliseconds == 0)
			{
				throw std::invalid_argument(std::string(policy_name) + " must be greater than zero");
			}

			constexpr auto kMaxNanoseconds =
				static_cast<std::uint64_t>(
					std::numeric_limits<rcl_duration_value_t>::max());

			if (milliseconds > kMaxNanoseconds / kNanosecondsPerMillisecond)
			{
				throw std::out_of_range(std::string(policy_name) + " exceeds rclcpp::Duration range");
			}

			const auto nanoseconds = static_cast<rcl_duration_value_t>(milliseconds * kNanosecondsPerMillisecond);
			return rclcpp::Duration::from_nanoseconds(nanoseconds);
		}
	} // namespace

	QosFactory::QosFactory(GatewayContractPtr contract) : contract_(std::move(contract))
	{
		if (!contract_)
		{
			throw std::invalid_argument("QosFactory requires a non-null GatewayContract");
		}
	}

	rclcpp::QoS QosFactory::make(std::string_view profile_name) const
	{
		const std::string profile_key(profile_name);

		const auto profile_iterator = contract_->qos_profiles.find(profile_key);
		if (profile_iterator == contract_->qos_profiles.end())
		{
			throw std::out_of_range("unknown Qos profile: " + profile_key);
		}
		const QosProfile &profile = profile_iterator->second;
		auto qos = make_history_qos(profile);

		switch (profile.reliability)
		{
			case ReliabilityPolicy::kReliable:

				qos.reliable();
				break;

			case ReliabilityPolicy::kBestEffort:
				qos.best_effort();
				break;

			default:
				throw std::invalid_argument("unsupported reliability policy");
		}

		switch (profile.durability)
		{
			case DurabilityPolicy::kVolatile:
				qos.durability_volatile();
				break;

			case DurabilityPolicy::kTransientLocal:
				qos.transient_local();
				break;

			default:
				throw std::invalid_argument("unsupported durability policy");
		}

		if (profile.deadline_ms.has_value())
		{
			qos.deadline(
				duration_from_ms(
					profile.deadline_ms.value(),
					"deadline_ms"));
		}

		if (profile.lifespan_ms.has_value())
		{
			qos.lifespan(
				duration_from_ms(
					profile.lifespan_ms.value(),
					"lifespan_ms"));
		}

		const bool has_liveliness = profile.liveliness.has_value();
		const bool has_liveliness_lease = profile.liveliness_lease_duration_ms.has_value();
		if (has_liveliness != has_liveliness_lease) {
			throw std::invalid_argument("liveliness and liveliness_lease_duration_ms must be configured together");
		}

		if(has_liveliness) {
			switch(profile.liveliness.value())
			{
				case LivelinessPolicy::kAutomatic:
					qos.liveliness(rclcpp::LivelinessPolicy::Automatic);
					break;

				case LivelinessPolicy::kManualByTopic:
					qos.liveliness(rclcpp::LivelinessPolicy::ManualByTopic);
					break;

				default:
					throw std::invalid_argument("unsupported liveliness policy");
			}

			qos.liveliness_lease_duration(
				duration_from_ms(
					profile.liveliness_lease_duration_ms.value(),
					"liveliness_lease_duration_ms"
					)
				);
		}

		return qos;
	}
} // namespace control_link_contract