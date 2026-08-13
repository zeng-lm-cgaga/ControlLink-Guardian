#pragma once

#include <string_view>

#include <rclcpp/qos.hpp>

#include "control_link_contract/model.hpp"

namespace control_link_contract
{

	class QosFactory final
	{
		public:
		// contract 是 Parser 已完成校验后发布的只读配置快照
		explicit QosFactory(GatewayContractPtr contract);

		// profile_name 是 qos_profiles 中的配置键，例如 control_input
		// 每次返回独立 rclcpp::QoS，未配置的 optional policy 保留 ROS2 默认语义
		rclcpp::QoS make(std::string_view profile_name) const;

		private:
		GatewayContractPtr contract_;
	};

}  // namespace control_link_contract
