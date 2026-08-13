#pragma once

#include <string>
#include <vector>

#include <rclcpp/qos.hpp>

#include "control_link_contract/model.hpp"

namespace control_link_contract
{
	// 一项 exact mismatch 同时保留机器可聚合的 policy 名和面向 diagnostics 的值
	struct QosPolicyMismatch
	{
		std::string policy;
		std::string expected;
		std::string actual;
	};

	// RMW Graph 未暴露某项 actual policy 时单独记录，不能伪装成远端配置错误
	struct QosPolicyObservationGap
	{
		std::string policy;
		std::string expected;
		std::string actual;
	};

	// DDS compatibility、可观测 policy mismatch 和 RMW observation gap 分层报告
	// 三层结论不能互相替代，例如 reliable publisher -> best-effort subscription 可兼容但不 exact
	struct QosCompatibilityReport
	{
		rclcpp::QoSCompatibility dds_compatibility;
		std::string dds_reason;
		std::vector<QosPolicyMismatch> exact_mismatches;
		std::vector<QosPolicyObservationGap> unobservable_policies;

		// 完整 exact 需要所有显式 Contract policy 都可观测且相等
		[[nodiscard]] bool exact_match() const noexcept;
		// Graph runtime gate 只回答 RMW 已暴露字段是否存在明确 mismatch
		[[nodiscard]] bool observed_policies_match() const noexcept;
	};

	// remote_direction 始终从网关视角描述远端，内部再保证 publisher QoS 位于 API 第一个参数
	QosCompatibilityReport assess_endpoint_qos(
		RemoteDirection remote_direction,
		const rclcpp::QoS & local_qos,
		const rclcpp::QoS & actual_remote_qos,
		const QosProfile & expected_remote_profile);

} // namespace control_link_contract
