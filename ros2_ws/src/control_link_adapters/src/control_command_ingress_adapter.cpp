#include "control_link_adapters/control_command_ingress_adapter.hpp"

#include <algorithm>
#include <filesystem>
#include <stdexcept>
#include <variant>

#include "control_link_contract/fastdds_environment.hpp"
#include "control_link_contract/qos_factory.hpp"

namespace control_link_adapters
{
	ControlCommandIngressAdapter::ControlCommandIngressAdapter(
		const rclcpp::NodeOptions & options)
		: rclcpp::Node("control_command_ingress_adapter", options)
	{
		const auto profile_path = declare_parameter<std::string>("profile_path", "");
		const auto config_root = declare_parameter<std::string>("config_root", "");
		const auto source_id = declare_parameter<std::string>("source_id", "");
		if (profile_path.empty() || config_root.empty() || source_id.empty())
		{
			throw std::invalid_argument(
				"ControlCommandIngressAdapter requires profile_path, config_root and source_id");
		}
		bundle_ = control_link_contract::load_contract_bundle(profile_path, config_root);
		const auto rmw_implementation =
			control_link_contract::validate_fastdds_process_environment(
				*bundle_->profile, "ControlCommandIngressAdapter");
		const auto & common = std::visit(
			[](const auto & profile) -> const control_link_contract::ProfileCommon &
			{
				return profile.common;
			}, *bundle_->profile);
		if (std::find(common.enabled_sources.begin(), common.enabled_sources.end(), source_id) ==
			common.enabled_sources.end())
		{
			throw std::invalid_argument("ingress source is not enabled by Profile: " + source_id);
		}
		const auto & ingress = common.ingress.at(source_id);
		if (ingress.input_type != "control_link_interfaces/msg/ControlCommand")
		{
			throw std::invalid_argument(
				"ControlCommandIngressAdapter requires ControlCommand input; actual=" +
				ingress.input_type);
		}
		control_link_contract::QosFactory qos_factory{bundle_->gateway_contract};
		const auto qos = qos_factory.make(bundle_->gateway_contract->input.qos_profile);
		command_publisher_ = create_publisher<ControlCommand>(ingress.output_topic, qos);
		command_subscription_ = create_subscription<ControlCommand>(
			ingress.input_topic, qos,
			[this, source_id](const ControlCommand & input)
			{
				// 不重新盖章、补序号或过滤旧/非法值；拒绝样本也必须由唯一 Gateway 计数。
				auto command = input;
				command.source_id = source_id;
				command_publisher_->publish(command);
			});
		RCLCPP_INFO(get_logger(),
			"ControlCommand ingress ready: source=%s, input=%s, output=%s, rmw=%s",
			source_id.c_str(), ingress.input_topic.c_str(), ingress.output_topic.c_str(),
			rmw_implementation.c_str());
	}
}  // namespace control_link_adapters
