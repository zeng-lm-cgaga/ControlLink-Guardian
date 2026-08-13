#pragma once

#include <chrono>
#include <cstdint>
#include <optional>
#include <string_view>

#include "control_link_contract/model.hpp"
#include "control_link_gateway/model.hpp"
#include "control_link_interfaces/msg/control_command.hpp"

namespace control_link_gateway
{
	// Gateway callback 在完成 subscription/source 与稳定 publisher generation 绑定后组装本上下文
	// expected_source_id 是非 owning view，只要求其底层字符串覆盖本次 validate 调用
	struct CommandValidationContext
	{
		std::string_view expected_source_id;
		PublisherGenerationKey publisher_generation;
		// sequence 基线只属于当前 source + publisher generation，超时或状态恢复不得清零
		std::optional<std::uint64_t> last_accepted_sequence;
		std::uint8_t priority;
		std::int64_t now_ros_ns;
		bool ros_clock_valid;
		std::chrono::steady_clock::time_point received_at;
	};

	// rejected 结果不携带 snapshot，调用方只能在 accepted() 后更新单槽和 lease
	struct CommandValidationResult
	{
		RejectReason reason;
		std::optional<SourceSnapshot> snapshot;

		[[nodiscard]] bool accepted() const noexcept;
	};

	// 纯决策类，不持有 sequence、接收时间或 source slot，运行状态由 Gateway callback 管理
	class CommandValidator final
	{
	public:
		explicit CommandValidator(
			control_link_contract::GatewayContractPtr contract);

		[[nodiscard]] CommandValidationResult validate(
			const control_link_interfaces::msg::ControlCommand &command,
			const CommandValidationContext &context) const;

	private:
		control_link_contract::GatewayContractPtr contract_;
	};
} // namespace control_link_gateway
