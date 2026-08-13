#pragma once

#include <cstdint>
#include <filesystem>
#include <memory>
#include <string>

#include "control_link_contract/contract_bundle.hpp"
#include "control_link_interfaces/msg/control_command.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"

namespace control_link_adapters
{
	// 将 Robot Profile 的外部 Twist 输入转换为 source 专属 ControlCommand
	// 只负责来源绑定、序号和接收时刻，不复制 Gateway 的数值校验与仲裁规则
	class TwistIngressAdapter final : public rclcpp::Node
	{
	public:
		explicit TwistIngressAdapter(
			const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

	private:
		using Twist = geometry_msgs::msg::Twist;
		using ControlCommand = control_link_interfaces::msg::ControlCommand;

		void handle_twist(const Twist & twist);
		[[nodiscard]] bool source_is_enabled() const;

		control_link_contract::ContractBundlePtr bundle_;
		std::string source_id_;
		rclcpp::Publisher<ControlCommand>::SharedPtr command_publisher_;
		rclcpp::Subscription<Twist>::SharedPtr twist_subscription_;
		std::uint64_t source_sequence_{0};
		bool sequence_exhausted_{false};
	};
}  // namespace control_link_adapters
