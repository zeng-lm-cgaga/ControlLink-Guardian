#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <filesystem>
#include <limits>
#include <memory>
#include <stdexcept>
#include <string>
#include <variant>
#include <utility>

#include "control_link_contract/contract_bundle.hpp"
#include "control_link_contract/qos_factory.hpp"
#include "control_link_interfaces/msg/control_command.hpp"
#include "rclcpp/rclcpp.hpp"

namespace
{
	using ControlCommand = control_link_interfaces::msg::ControlCommand;

	std::chrono::nanoseconds checked_period(double rate_hz)
	{
		if (!std::isfinite(rate_hz) || rate_hz < 20.0)
		{
			throw std::invalid_argument(
				"mock source publish_rate_hz must be finite and at least 20Hz");
		}

		const auto period_ns = 1'000'000'000.0L /
			static_cast<long double>(rate_hz);
		if (period_ns < 1.0L ||
			period_ns > static_cast<long double>(
				std::numeric_limits<std::chrono::nanoseconds::rep>::max()))
		{
			throw std::out_of_range(
				"mock source publish_rate_hz cannot be represented by a timer");
		}

		return std::chrono::nanoseconds{
			static_cast<std::chrono::nanoseconds::rep>(std::ceil(period_ns))};
	}

	class MockControlSource final : public rclcpp::Node
	{
	public:
		MockControlSource()
			: rclcpp::Node("mock_control_source")
		{
			declare_parameter<std::string>("profile_path", "");
			declare_parameter<std::string>("config_root", "");
			declare_parameter<std::string>("source_id", "");
			declare_parameter<double>("publish_rate_hz", 25.0);
			declare_parameter<double>("linear_velocity_mps", 0.25);
			declare_parameter<double>("angular_velocity_radps", 0.10);
			declare_parameter<std::int64_t>("startup_delay_ms", 0);
			declare_parameter<std::int64_t>("minimum_subscription_count", 0);

			const auto profile_path = std::filesystem::path{
				get_parameter("profile_path").as_string()};
			const auto config_root = std::filesystem::path{
				get_parameter("config_root").as_string()};
			source_id_ = get_parameter("source_id").as_string();
			if (source_id_.empty())
			{
				throw std::invalid_argument(
					"mock source requires a non-empty source_id");
			}

			bundle_ = control_link_contract::load_contract_bundle(
				profile_path,
				config_root);
			const auto source_iterator =
				bundle_->source_policy->sources.find(source_id_);
			if (source_iterator == bundle_->source_policy->sources.end())
			{
				throw std::out_of_range(
					"mock source is not defined by SourcePolicy: " + source_id_);
			}

			if (!source_is_enabled(source_id_))
			{
				throw std::invalid_argument(
					"mock source is not enabled by Profile: " + source_id_);
			}

			const auto rate_hz = get_parameter("publish_rate_hz").as_double();
			const auto startup_delay_ms =
				get_parameter("startup_delay_ms").as_int();
			const auto minimum_subscription_count =
				get_parameter("minimum_subscription_count").as_int();
			if (startup_delay_ms < 0 || startup_delay_ms > 60'000)
			{
				throw std::out_of_range(
					"mock source startup_delay_ms must be in [0,60000]");
			}
			if (minimum_subscription_count < 0 ||
				minimum_subscription_count > 1024)
			{
				throw std::out_of_range(
					"mock source minimum_subscription_count must be in [0,1024]");
			}
			publish_not_before_ = std::chrono::steady_clock::now() +
				std::chrono::milliseconds{startup_delay_ms};
			minimum_subscription_count_ =
				static_cast<std::size_t>(minimum_subscription_count);
			linear_velocity_mps_ =
				get_parameter("linear_velocity_mps").as_double();
			angular_velocity_radps_ =
				get_parameter("angular_velocity_radps").as_double();
			const auto &limits = bundle_->gateway_contract->limits;
			if (!std::isfinite(linear_velocity_mps_) ||
				!std::isfinite(angular_velocity_radps_) ||
				std::abs(linear_velocity_mps_) > limits.max_abs_linear_velocity_mps ||
				std::abs(angular_velocity_radps_) >
					limits.max_abs_angular_velocity_radps)
			{
				throw std::out_of_range(
					"mock source motion exceeds Gateway Contract limits");
			}

			control_link_contract::QosFactory qos_factory{
				bundle_->gateway_contract};
			const auto qos = qos_factory.make(
				bundle_->gateway_contract->input.qos_profile);
			publisher_ = create_publisher<ControlCommand>(
				source_iterator->second.topic,
				qos);
			timer_ = create_wall_timer(
				checked_period(rate_hz),
				[this]()
				{
					publish_command();
				});

			RCLCPP_INFO(
				get_logger(),
				"Mock source ready: source=%s, topic=%s, rate_hz=%.2f, "
				"startup_delay_ms=%ld, minimum_subscriptions=%zu",
				source_id_.c_str(),
				source_iterator->second.topic.c_str(),
				rate_hz,
				static_cast<long>(startup_delay_ms),
				minimum_subscription_count_);
		}

	private:
		bool source_is_enabled(const std::string &source_id) const
		{
			return std::visit(
				[&source_id](const auto &profile)
				{
					for (const auto &enabled : profile.common.enabled_sources)
					{
						if (enabled == source_id)
						{
							return true;
						}
					}
					return false;
				},
				*bundle_->profile);
		}

		void publish_command()
		{
			const auto now_steady = std::chrono::steady_clock::now();
			if (now_steady < publish_not_before_ ||
				publisher_->get_subscription_count() < minimum_subscription_count_)
			{
				return;
			}
			if (!publishing_started_)
			{
				publishing_started_ = true;
				RCLCPP_INFO(
					get_logger(),
					"Mock source publishing started: source=%s, matched_subscriptions=%zu",
					source_id_.c_str(),
					publisher_->get_subscription_count());
			}

			if (source_sequence_ == std::numeric_limits<std::uint64_t>::max())
			{
				RCLCPP_ERROR(
					get_logger(),
					"source sequence exhausted, stopping mock source");
				timer_->cancel();
				return;
			}

			const auto now = get_clock()->now();
			if (now.nanoseconds() <= 0)
			{
				RCLCPP_WARN_THROTTLE(
					get_logger(),
					*get_clock(),
					2000,
					"ROS clock has not started, mock source is waiting");
				return;
			}

			ControlCommand command;
			command.source_stamp =
				static_cast<builtin_interfaces::msg::Time>(now);
			command.source_id = source_id_;
			command.source_sequence = ++source_sequence_;
			command.mode = ControlCommand::MODE_NORMAL;
			command.linear_velocity_mps = linear_velocity_mps_;
			command.angular_velocity_radps = angular_velocity_radps_;
			publisher_->publish(command);
		}

		control_link_contract::ContractBundlePtr bundle_;
		rclcpp::Publisher<ControlCommand>::SharedPtr publisher_;
		rclcpp::TimerBase::SharedPtr timer_;
		std::string source_id_;
		std::uint64_t source_sequence_{0};
		double linear_velocity_mps_{0.0};
		double angular_velocity_radps_{0.0};
		std::chrono::steady_clock::time_point publish_not_before_{};
		std::size_t minimum_subscription_count_{0U};
		bool publishing_started_{false};
	};
} // namespace

int main(int argc, char **argv)
{
	rclcpp::init(argc, argv);
	try
	{
		auto node = std::make_shared<MockControlSource>();
		rclcpp::spin(node);
	}
	catch (const std::exception &exception)
	{
		RCLCPP_FATAL(
			rclcpp::get_logger("mock_control_source"),
			"Mock source failed: %s",
			exception.what());
		rclcpp::shutdown();
		return 1;
	}
	rclcpp::shutdown();
	return 0;
}
