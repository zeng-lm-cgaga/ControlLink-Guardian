#include "control_link_adapters/twist_ingress_adapter.hpp"

#include <algorithm>
#include <limits>
#include <stdexcept>
#include <variant>

#include "control_link_contract/fastdds_environment.hpp"
#include "control_link_contract/qos_factory.hpp"

namespace control_link_adapters
{
	namespace
	{
		const control_link_contract::ProfileCommon & profile_common(
			const control_link_contract::ProfileConfig & profile)
		{
			return std::visit(
				[](const auto & concrete_profile)
					-> const control_link_contract::ProfileCommon &
				{
					return concrete_profile.common;
				},
				profile);
		}
	}  // namespace

	TwistIngressAdapter::TwistIngressAdapter(
		const rclcpp::NodeOptions & options)
		: rclcpp::Node("twist_ingress_adapter", options)
	{
		declare_parameter<std::string>("profile_path", "");
		declare_parameter<std::string>("config_root", "");
		declare_parameter<std::string>("source_id", "");

		const auto profile_path = std::filesystem::path{
			get_parameter("profile_path").as_string()};
		const auto config_root = std::filesystem::path{
			get_parameter("config_root").as_string()};
		source_id_ = get_parameter("source_id").as_string();
		if (profile_path.empty() || config_root.empty())
		{
			throw std::invalid_argument(
				"TwistIngressAdapter requires profile_path and config_root");
		}
		if (source_id_.empty())
		{
			throw std::invalid_argument(
				"TwistIngressAdapter requires a non-empty source_id");
		}

		bundle_ = control_link_contract::load_contract_bundle(
			profile_path,
			config_root);
		const auto rmw_implementation =
			control_link_contract::validate_fastdds_process_environment(
				*bundle_->profile,
				"TwistIngressAdapter");
		if (!source_is_enabled())
		{
			throw std::invalid_argument(
				"ingress source is not enabled by Profile: " + source_id_);
		}

		const auto source_iterator = bundle_->source_policy->sources.find(source_id_);
		if (source_iterator == bundle_->source_policy->sources.end())
		{
			throw std::out_of_range(
				"ingress source is not defined by SourcePolicy: " + source_id_);
		}
		const auto & common = profile_common(*bundle_->profile);
		const auto ingress_iterator = common.ingress.find(source_id_);
		if (ingress_iterator == common.ingress.end())
		{
			throw std::out_of_range(
				"ingress binding is not defined by Profile: " + source_id_);
		}
		const auto & ingress = ingress_iterator->second;
		if (ingress.input_type != "geometry_msgs/msg/Twist")
		{
			throw std::invalid_argument(
				"TwistIngressAdapter requires geometry_msgs/msg/Twist input; actual=" +
				ingress.input_type);
		}

		control_link_contract::QosFactory qos_factory{
			bundle_->gateway_contract};
		command_publisher_ = create_publisher<ControlCommand>(
			ingress.output_topic,
			qos_factory.make(bundle_->gateway_contract->input.qos_profile));

		// 外部 Twist 的 QoS 不属于 Gateway Contract，使用可靠的短队列承接上游
		// 转换后的 ControlCommand 才使用 Contract QoS
		const auto input_qos = rclcpp::QoS{rclcpp::KeepLast(10)}
			.reliable()
			.durability_volatile();
		twist_subscription_ = create_subscription<Twist>(
			ingress.input_topic,
			input_qos,
			[this](const Twist & twist)
			{
				handle_twist(twist);
			});

		RCLCPP_INFO(
			get_logger(),
			"Twist ingress ready: source=%s, input=%s, output=%s, rmw=%s",
			source_id_.c_str(),
			ingress.input_topic.c_str(),
			ingress.output_topic.c_str(),
			rmw_implementation.c_str());
	}

	bool TwistIngressAdapter::source_is_enabled() const
	{
		const auto & enabled_sources =
			profile_common(*bundle_->profile).enabled_sources;
		return std::find(
			enabled_sources.begin(),
			enabled_sources.end(),
			source_id_) != enabled_sources.end();
	}

	void TwistIngressAdapter::handle_twist(const Twist & twist)
	{
		if (sequence_exhausted_)
		{
			return;
		}
		if (source_sequence_ == std::numeric_limits<std::uint64_t>::max())
		{
			sequence_exhausted_ = true;
			RCLCPP_ERROR(
				get_logger(),
				"source sequence exhausted, stopping ingress publication");
			return;
		}

		const auto now = get_clock()->now();
		if (now.nanoseconds() <= 0)
		{
			RCLCPP_WARN_THROTTLE(
				get_logger(),
				*get_clock(),
				2000,
				"ROS clock has not started, ingress is waiting");
			return;
		}

		ControlCommand command;
		command.source_stamp = static_cast<builtin_interfaces::msg::Time>(now);
		command.source_id = source_id_;
		command.source_sequence = ++source_sequence_;
		command.mode = ControlCommand::MODE_NORMAL;
		command.linear_velocity_mps = twist.linear.x;
		command.angular_velocity_radps = twist.angular.z;
		command_publisher_->publish(command);
	}
}  // namespace control_link_adapters
