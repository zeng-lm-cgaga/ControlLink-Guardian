#pragma once

#include <string>
#include <string_view>

#include "control_link_contract/model.hpp"

namespace control_link_contract
{
	// FastDDS participant 在 rclcpp::init() 期间建立，本函数只能核对启动环境，不能事后修复
	// Gateway 与执行 adapter 共用该规则，避免 transport 与 Topic QoS owner 在不同进程漂移
	[[nodiscard]] std::string validate_fastdds_process_environment(
		const ProfileConfig &profile,
		std::string_view component_name);
} // namespace control_link_contract
