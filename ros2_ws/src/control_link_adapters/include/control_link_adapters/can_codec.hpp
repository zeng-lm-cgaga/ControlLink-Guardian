#pragma once

#include <array>
#include <cstdint>
#include <optional>

#include "control_link_contract/model.hpp"

namespace control_link_adapters
{
	// 中立 CAN 帧表示，不依赖 Linux can_frame，SocketCAN 只在边界处负责转换
	struct CanFrame final
	{
		std::uint32_t can_id;
		std::uint8_t dlc;
		bool is_extended;
		bool is_remote;
		std::array<std::uint8_t, 8U> data;
	};

	enum class CanControlMode : std::uint8_t
	{
		kHold = 0U,
		kActive = 1U,
	};

	struct CanControlSignals final
	{
		CanControlMode mode;
		double target_speed_mps;
		double target_yaw_rate_radps;
		std::uint8_t rolling_counter;
	};

	enum class CanVehicleMode : std::uint8_t
	{
		kStandby = 0U,
		kRunning = 1U,
		kSafeStop = 2U,
		kFault = 3U,
	};

	struct CanVehicleStateSignals final
	{
		CanVehicleMode mode;
		double measured_speed_mps;
		double measured_yaw_rate_radps;
		std::uint8_t fault_code;
		std::uint8_t state_counter;
		std::uint8_t echoed_control_counter;
	};

	// 由 can_signal_map.yaml 的加载层构造，codec 不直接读取 YAML
	// v1 的布局固定为 8-byte、16-bit little-endian 物理量和 CRC byte 7
	struct CanCodecConfig final
	{
		std::uint32_t control_can_id;
		std::uint32_t state_can_id;
		std::uint8_t dlc;
		double speed_scale_mps;
		double yaw_rate_scale_radps;
		double speed_min_mps;
		double speed_max_mps;
		double yaw_rate_min_radps;
		double yaw_rate_max_radps;
		std::uint8_t rolling_counter_modulo;
		std::uint8_t crc_polynomial;
		std::uint8_t crc_initial_value;
		std::uint8_t crc_final_xor;
	};

	enum class CanCodecRejectReason : std::uint8_t
	{
		kNone,
		kWrongCanId,
		kWrongDlc,
		kExtendedFrame,
		kRemoteFrame,
		kCrcMismatch,
		kReservedBitsNonzero,
		kInvalidMode,
		kInvalidCommandValid,
		kHoldNonzero,
		kNonFinite,
		kOutOfRange,
		kCounterOutOfRange,
	};

	[[nodiscard]] const char *can_codec_reject_reason_name(
		CanCodecRejectReason reason) noexcept;

	struct ControlEncodeResult final
	{
		std::optional<CanFrame> frame;
		CanCodecRejectReason reason{CanCodecRejectReason::kNone};

		[[nodiscard]] bool accepted() const noexcept
		{
			return frame.has_value() && reason == CanCodecRejectReason::kNone;
		}
	};

	struct StateEncodeResult final
	{
		std::optional<CanFrame> frame;
		CanCodecRejectReason reason{CanCodecRejectReason::kNone};

		[[nodiscard]] bool accepted() const noexcept
		{
			return frame.has_value() && reason == CanCodecRejectReason::kNone;
		}
	};

	struct ControlDecodeResult final
	{
		std::optional<CanControlSignals> signals;
		CanCodecRejectReason reason{CanCodecRejectReason::kNone};

		[[nodiscard]] bool accepted() const noexcept
		{
			return signals.has_value() && reason == CanCodecRejectReason::kNone;
		}
	};

	struct StateDecodeResult final
	{
		std::optional<CanVehicleStateSignals> signals;
		CanCodecRejectReason reason{CanCodecRejectReason::kNone};

		[[nodiscard]] bool accepted() const noexcept
		{
			return signals.has_value() && reason == CanCodecRejectReason::kNone;
		}
	};

	// 纯确定性协议转换器，不读取 YAML、不创建 ROS endpoint、不访问 CAN fd
	class CanCodec final
	{
	public:
		// 运行路径直接消费 Contract parser 发布的不可变 signal map，禁止 adapter 重抄协议常量
		explicit CanCodec(const control_link_contract::CanSignalMap &signal_map);

		[[nodiscard]] ControlEncodeResult encode_control(
			const CanControlSignals &signals) const noexcept;
		[[nodiscard]] StateEncodeResult encode_state(
			const CanVehicleStateSignals &signals) const noexcept;
		[[nodiscard]] ControlDecodeResult decode_control(
			const CanFrame &frame) const noexcept;
		[[nodiscard]] StateDecodeResult decode_state(
			const CanFrame &frame) const noexcept;

		[[nodiscard]] const CanCodecConfig &config() const noexcept
		{
			return config_;
		}

	private:
		explicit CanCodec(CanCodecConfig config);

		[[nodiscard]] CanCodecRejectReason validate_frame_shape(
			const CanFrame &frame,
			std::uint32_t expected_can_id) const noexcept;
		[[nodiscard]] std::uint8_t calculate_crc(
			const CanFrame &frame) const noexcept;
		[[nodiscard]] CanCodecRejectReason validate_common_signals(
			double speed_mps,
			double yaw_rate_radps) const noexcept;
		[[nodiscard]] std::optional<std::int32_t> quantize(
			double value,
			double scale,
			double minimum,
			double maximum) const noexcept;

		CanCodecConfig config_;
	};

	enum class CounterObservation : std::uint8_t
	{
		kFirstValue,
		kExpected,
		kDuplicate,
		kJump,
		kInvalid,
	};

	// control counter 和 state counter 各自拥有一个实例，CRC/结构错误不得调用 observe
	// jump 当前帧仍被拒绝并清空连续性基线，下一条合法帧按首帧规则重新建立基线
	class RollingCounterChecker final
	{
	public:
		explicit RollingCounterChecker(std::uint8_t modulo);

		[[nodiscard]] CounterObservation observe(
			std::uint8_t counter) noexcept;
		void reset() noexcept;

	private:
		std::uint8_t modulo_;
		std::optional<std::uint8_t> last_counter_;
	};
}  // namespace control_link_adapters
