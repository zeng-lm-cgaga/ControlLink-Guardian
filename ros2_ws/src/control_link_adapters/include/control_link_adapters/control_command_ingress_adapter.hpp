#pragma once

#include "control_link_contract/contract_bundle.hpp"
#include "control_link_interfaces/msg/control_command.hpp"
#include "rclcpp/rclcpp.hpp"

namespace control_link_adapters
{

	// 按 Profile 绑定已有 ControlCommand 的来源；业务 stamp/sequence/值原样传递，
	// 只装配 I/O，不替代 Gateway 的 Binding、Validator 或 freshness/lease 判断。
	class ControlCommandIngressAdapter final : public rclcpp::Node
	{
	public:
		explicit ControlCommandIngressAdapter(
			const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

	private:
		using ControlCommand = control_link_interfaces::msg::ControlCommand;
		control_link_contract::ContractBundlePtr bundle_;
		rclcpp::Publisher<ControlCommand>::SharedPtr command_publisher_;
		rclcpp::Subscription<ControlCommand>::SharedPtr command_subscription_;
	};
}  // namespace control_link_adapters
