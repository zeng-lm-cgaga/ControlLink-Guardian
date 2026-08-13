#include "control_link_adapters/can_codec.hpp"

#include <cmath>
#include <limits>
#include <stdexcept>
#include <vector>

namespace control_link_adapters
{
	namespace
	{
		constexpr std::uint32_t kMaxStandardCanId = 0x7FFU;
		constexpr std::uint8_t kPayloadCrcIndex = 7U;

		std::int32_t decode_signed_le(
			const std::array<std::uint8_t, 8U> &data,
			std::uint8_t offset) noexcept
		{
			const auto raw = static_cast<std::uint16_t>(
				static_cast<std::uint16_t>(data[offset]) |
				(static_cast<std::uint16_t>(data[offset + 1U]) << 8U));
			return raw <= 0x7FFFU ? static_cast<std::int32_t>(raw) :
				static_cast<std::int32_t>(raw) - 0x10000;
		}

		void encode_signed_le(
			std::array<std::uint8_t, 8U> &data,
			std::uint8_t offset,
			std::int32_t value) noexcept
		{
			const auto raw = static_cast<std::uint32_t>(value) & 0xFFFFU;
			data[offset] = static_cast<std::uint8_t>(raw & 0xFFU);
			data[offset + 1U] = static_cast<std::uint8_t>((raw >> 8U) & 0xFFU);
		}

		std::uint8_t update_crc(
			std::uint8_t crc,
			std::uint8_t input,
			std::uint8_t polynomial) noexcept
		{
			crc ^= input;
			for (std::uint8_t bit = 0U; bit < 8U; ++bit)
			{
				crc = (crc & 0x80U) != 0U ?
					static_cast<std::uint8_t>((crc << 1U) ^ polynomial) :
					static_cast<std::uint8_t>(crc << 1U);
			}
			return crc;
		}

		bool is_valid_mode(CanControlMode mode) noexcept
		{
			return mode == CanControlMode::kHold || mode == CanControlMode::kActive;
		}

		bool is_valid_vehicle_mode(CanVehicleMode mode) noexcept
		{
			return mode == CanVehicleMode::kStandby ||
				mode == CanVehicleMode::kRunning ||
				mode == CanVehicleMode::kSafeStop ||
				mode == CanVehicleMode::kFault;
		}

		bool same_physical_signal(
			const control_link_contract::CanPhysicalSignalConfig &left,
			const control_link_contract::CanPhysicalSignalConfig &right) noexcept
		{
			return left.scale == right.scale && left.offset == right.offset &&
				left.minimum == right.minimum && left.maximum == right.maximum;
		}

		CanCodecConfig codec_config_from_signal_map(
			const control_link_contract::CanSignalMap &signal_map)
		{
			if (signal_map.byte_order != control_link_contract::CanByteOrder::kLittleEndian ||
				signal_map.crc.algorithm !=
					control_link_contract::CanCrcAlgorithm::kCrc8SaeJ1850 ||
				signal_map.crc.reflect_input || signal_map.crc.reflect_output ||
				!signal_map.crc.include_can_id_lsb_first ||
				signal_map.crc.protected_payload_bytes !=
					std::vector<std::uint8_t>{0U, 1U, 2U, 3U, 4U, 5U, 6U})
			{
				throw std::invalid_argument(
					"CAN signal map uses protocol rules unsupported by the v1 codec");
			}
			if (!same_physical_signal(
					signal_map.control_frame.target_speed_mps,
					signal_map.state_frame.measured_speed_mps) ||
				!same_physical_signal(
					signal_map.control_frame.target_yaw_rate_radps,
					signal_map.state_frame.measured_yaw_rate_radps))
			{
				throw std::invalid_argument(
					"CAN v1 codec requires matching control and state physical signal rules");
			}
			if (signal_map.control_frame.target_speed_mps.offset != 0.0 ||
				signal_map.control_frame.target_yaw_rate_radps.offset != 0.0)
			{
				throw std::invalid_argument(
					"CAN v1 codec requires zero physical signal offsets");
			}
			return CanCodecConfig{
				signal_map.control_frame.can_id,
				signal_map.state_frame.can_id,
				signal_map.dlc,
				signal_map.control_frame.target_speed_mps.scale,
				signal_map.control_frame.target_yaw_rate_radps.scale,
				signal_map.control_frame.target_speed_mps.minimum,
				signal_map.control_frame.target_speed_mps.maximum,
				signal_map.control_frame.target_yaw_rate_radps.minimum,
				signal_map.control_frame.target_yaw_rate_radps.maximum,
				signal_map.rolling_counter.modulo,
				signal_map.crc.polynomial,
				signal_map.crc.initial_value,
				signal_map.crc.final_xor};
		}
	}

	const char *can_codec_reject_reason_name(CanCodecRejectReason reason) noexcept
	{
		switch (reason)
		{
			case CanCodecRejectReason::kNone:
				return "none";
			case CanCodecRejectReason::kWrongCanId:
				return "wrong_can_id";
			case CanCodecRejectReason::kWrongDlc:
				return "wrong_dlc";
			case CanCodecRejectReason::kExtendedFrame:
				return "extended_frame";
			case CanCodecRejectReason::kRemoteFrame:
				return "remote_frame";
			case CanCodecRejectReason::kCrcMismatch:
				return "crc_mismatch";
			case CanCodecRejectReason::kReservedBitsNonzero:
				return "reserved_bits_nonzero";
			case CanCodecRejectReason::kInvalidMode:
				return "invalid_mode";
			case CanCodecRejectReason::kInvalidCommandValid:
				return "invalid_command_valid";
			case CanCodecRejectReason::kHoldNonzero:
				return "hold_nonzero";
			case CanCodecRejectReason::kNonFinite:
				return "non_finite";
			case CanCodecRejectReason::kOutOfRange:
				return "out_of_range";
			case CanCodecRejectReason::kCounterOutOfRange:
				return "counter_out_of_range";
		}
		return "unknown";
	}

	CanCodec::CanCodec(CanCodecConfig config)
		: config_(config)
	{
		if (config_.control_can_id > kMaxStandardCanId ||
			config_.state_can_id > kMaxStandardCanId ||
			config_.control_can_id == config_.state_can_id)
		{
			throw std::invalid_argument(
				"CAN codec IDs must be distinct standard 11-bit identifiers");
		}
		if (config_.dlc != 8U || config_.rolling_counter_modulo < 2U ||
			config_.rolling_counter_modulo > 16U)
		{
			throw std::invalid_argument(
				"CAN codec requires DLC 8 and rolling counter modulo in [2,16]");
		}
		if (!std::isfinite(config_.speed_scale_mps) ||
			!std::isfinite(config_.yaw_rate_scale_radps) ||
			config_.speed_scale_mps <= 0.0 || config_.yaw_rate_scale_radps <= 0.0 ||
			!std::isfinite(config_.speed_min_mps) ||
			!std::isfinite(config_.speed_max_mps) ||
			!std::isfinite(config_.yaw_rate_min_radps) ||
			!std::isfinite(config_.yaw_rate_max_radps) ||
			config_.speed_min_mps > config_.speed_max_mps ||
			config_.yaw_rate_min_radps > config_.yaw_rate_max_radps)
		{
			throw std::invalid_argument(
				"CAN codec scales and physical ranges must be finite and valid");
		}
	}

	CanCodec::CanCodec(const control_link_contract::CanSignalMap &signal_map)
		: CanCodec(codec_config_from_signal_map(signal_map))
	{
	}

	CanCodecRejectReason CanCodec::validate_frame_shape(
		const CanFrame &frame,
		std::uint32_t expected_can_id) const noexcept
	{
		if (frame.can_id != expected_can_id)
			return CanCodecRejectReason::kWrongCanId;
		if (frame.dlc != config_.dlc)
			return CanCodecRejectReason::kWrongDlc;
		if (frame.is_extended)
			return CanCodecRejectReason::kExtendedFrame;
		if (frame.is_remote)
			return CanCodecRejectReason::kRemoteFrame;
		return CanCodecRejectReason::kNone;
	}

	std::uint8_t CanCodec::calculate_crc(const CanFrame &frame) const noexcept
	{
		std::uint8_t crc = config_.crc_initial_value;
		crc = update_crc(
			crc,
			static_cast<std::uint8_t>(frame.can_id & 0xFFU),
			config_.crc_polynomial);
		crc = update_crc(
			crc,
			static_cast<std::uint8_t>((frame.can_id >> 8U) & 0xFFU),
			config_.crc_polynomial);
		for (std::uint8_t index = 0U; index < kPayloadCrcIndex; ++index)
			crc = update_crc(crc, frame.data[index], config_.crc_polynomial);
		return static_cast<std::uint8_t>(crc ^ config_.crc_final_xor);
	}

	CanCodecRejectReason CanCodec::validate_common_signals(
		double speed_mps,
		double yaw_rate_radps) const noexcept
	{
		if (!std::isfinite(speed_mps) || !std::isfinite(yaw_rate_radps))
			return CanCodecRejectReason::kNonFinite;
		if (speed_mps < config_.speed_min_mps || speed_mps > config_.speed_max_mps ||
			yaw_rate_radps < config_.yaw_rate_min_radps ||
			yaw_rate_radps > config_.yaw_rate_max_radps)
			return CanCodecRejectReason::kOutOfRange;
		return CanCodecRejectReason::kNone;
	}

	std::optional<std::int32_t> CanCodec::quantize(
		double value,
		double scale,
		double minimum,
		double maximum) const noexcept
	{
		if (!std::isfinite(value) || value < minimum || value > maximum)
			return std::nullopt;
		const double raw = std::round(value / scale);
		if (!std::isfinite(raw) || raw < -32768.0 || raw > 32767.0)
			return std::nullopt;
		return static_cast<std::int32_t>(raw);
	}

	ControlEncodeResult CanCodec::encode_control(
		const CanControlSignals &signals) const noexcept
	{
		if (!is_valid_mode(signals.mode))
			return {std::nullopt, CanCodecRejectReason::kInvalidMode};
		if (signals.rolling_counter >= config_.rolling_counter_modulo)
			return {std::nullopt, CanCodecRejectReason::kCounterOutOfRange};
		const auto signal_result = validate_common_signals(
			signals.target_speed_mps, signals.target_yaw_rate_radps);
		if (signal_result != CanCodecRejectReason::kNone)
			return {std::nullopt, signal_result};
		if (signals.mode == CanControlMode::kHold &&
			(signals.target_speed_mps != 0.0 || signals.target_yaw_rate_radps != 0.0))
			return {std::nullopt, CanCodecRejectReason::kHoldNonzero};
		const auto speed = quantize(
			signals.target_speed_mps,
			config_.speed_scale_mps,
			config_.speed_min_mps,
			config_.speed_max_mps);
		const auto yaw_rate = quantize(
			signals.target_yaw_rate_radps,
			config_.yaw_rate_scale_radps,
			config_.yaw_rate_min_radps,
			config_.yaw_rate_max_radps);
		if (!speed.has_value() || !yaw_rate.has_value())
			return {std::nullopt, CanCodecRejectReason::kOutOfRange};

		CanFrame frame{
			config_.control_can_id,
			config_.dlc,
			false,
			false,
			{0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U}};
		encode_signed_le(frame.data, 0U, speed.value());
		encode_signed_le(frame.data, 2U, yaw_rate.value());
		frame.data[4U] = static_cast<std::uint8_t>(signals.mode);
		frame.data[5U] = signals.rolling_counter;
		frame.data[6U] = 0x01U;
		frame.data[7U] = calculate_crc(frame);
		return {frame, CanCodecRejectReason::kNone};
	}

	StateEncodeResult CanCodec::encode_state(
		const CanVehicleStateSignals &signals) const noexcept
	{
		if (!is_valid_vehicle_mode(signals.mode))
			return {std::nullopt, CanCodecRejectReason::kInvalidMode};
		if (signals.state_counter >= config_.rolling_counter_modulo ||
			signals.echoed_control_counter >= config_.rolling_counter_modulo)
			return {std::nullopt, CanCodecRejectReason::kCounterOutOfRange};
		const auto signal_result = validate_common_signals(
			signals.measured_speed_mps, signals.measured_yaw_rate_radps);
		if (signal_result != CanCodecRejectReason::kNone)
			return {std::nullopt, signal_result};
		const auto speed = quantize(
			signals.measured_speed_mps,
			config_.speed_scale_mps,
			config_.speed_min_mps,
			config_.speed_max_mps);
		const auto yaw_rate = quantize(
			signals.measured_yaw_rate_radps,
			config_.yaw_rate_scale_radps,
			config_.yaw_rate_min_radps,
			config_.yaw_rate_max_radps);
		if (!speed.has_value() || !yaw_rate.has_value())
			return {std::nullopt, CanCodecRejectReason::kOutOfRange};

		CanFrame frame{
			config_.state_can_id,
			config_.dlc,
			false,
			false,
			{0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U}};
		encode_signed_le(frame.data, 0U, speed.value());
		encode_signed_le(frame.data, 2U, yaw_rate.value());
		frame.data[4U] = static_cast<std::uint8_t>(signals.mode);
		frame.data[5U] = signals.fault_code;
		frame.data[6U] = static_cast<std::uint8_t>(
			(signals.state_counter & 0x0FU) |
			((signals.echoed_control_counter & 0x0FU) << 4U));
		frame.data[7U] = calculate_crc(frame);
		return {frame, CanCodecRejectReason::kNone};
	}

	ControlDecodeResult CanCodec::decode_control(
		const CanFrame &frame) const noexcept
	{
		const auto shape_reason = validate_frame_shape(frame, config_.control_can_id);
		if (shape_reason != CanCodecRejectReason::kNone)
			return {std::nullopt, shape_reason};
		if (frame.data[5U] & 0xF0U || frame.data[6U] & 0xFEU)
			return {std::nullopt, CanCodecRejectReason::kReservedBitsNonzero};
		if (frame.data[6U] != 0x01U)
			return {std::nullopt, CanCodecRejectReason::kInvalidCommandValid};
		if (frame.data[7U] != calculate_crc(frame))
			return {std::nullopt, CanCodecRejectReason::kCrcMismatch};
		const auto mode_value = frame.data[4U];
		if (mode_value > static_cast<std::uint8_t>(CanControlMode::kActive))
			return {std::nullopt, CanCodecRejectReason::kInvalidMode};
		const double speed = static_cast<double>(decode_signed_le(frame.data, 0U)) *
			config_.speed_scale_mps;
		const double yaw_rate = static_cast<double>(decode_signed_le(frame.data, 2U)) *
			config_.yaw_rate_scale_radps;
		const auto signal_reason = validate_common_signals(speed, yaw_rate);
		if (signal_reason != CanCodecRejectReason::kNone)
			return {std::nullopt, signal_reason};
		if (mode_value == static_cast<std::uint8_t>(CanControlMode::kHold) &&
			(speed != 0.0 || yaw_rate != 0.0))
			return {std::nullopt, CanCodecRejectReason::kHoldNonzero};
		return {
			CanControlSignals{
				static_cast<CanControlMode>(mode_value),
				speed,
				yaw_rate,
				static_cast<std::uint8_t>(frame.data[5U] & 0x0FU)},
			CanCodecRejectReason::kNone};
	}

	StateDecodeResult CanCodec::decode_state(const CanFrame &frame) const noexcept
	{
		const auto shape_reason = validate_frame_shape(frame, config_.state_can_id);
		if (shape_reason != CanCodecRejectReason::kNone)
			return {std::nullopt, shape_reason};
		if (frame.data[7U] != calculate_crc(frame))
			return {std::nullopt, CanCodecRejectReason::kCrcMismatch};
		if (frame.data[4U] > static_cast<std::uint8_t>(CanVehicleMode::kFault))
			return {std::nullopt, CanCodecRejectReason::kInvalidMode};
		const auto state_counter = static_cast<std::uint8_t>(frame.data[6U] & 0x0FU);
		const auto echoed_control_counter = static_cast<std::uint8_t>(frame.data[6U] >> 4U);
		if (state_counter >= config_.rolling_counter_modulo ||
			echoed_control_counter >= config_.rolling_counter_modulo)
			return {std::nullopt, CanCodecRejectReason::kCounterOutOfRange};
		const double speed = static_cast<double>(decode_signed_le(frame.data, 0U)) *
			config_.speed_scale_mps;
		const double yaw_rate = static_cast<double>(decode_signed_le(frame.data, 2U)) *
			config_.yaw_rate_scale_radps;
		const auto signal_reason = validate_common_signals(speed, yaw_rate);
		if (signal_reason != CanCodecRejectReason::kNone)
			return {std::nullopt, signal_reason};
		return {
			CanVehicleStateSignals{
				static_cast<CanVehicleMode>(frame.data[4U]),
				speed,
				yaw_rate,
				frame.data[5U],
				state_counter,
				echoed_control_counter},
			CanCodecRejectReason::kNone};
	}

	RollingCounterChecker::RollingCounterChecker(std::uint8_t modulo)
		: modulo_(modulo)
	{
		if (modulo_ < 2U || modulo_ > 16U)
			throw std::invalid_argument(
				"rolling counter modulo must be in [2,16]");
	}

	CounterObservation RollingCounterChecker::observe(
		std::uint8_t counter) noexcept
	{
		if (counter >= modulo_)
			return CounterObservation::kInvalid;
		if (!last_counter_.has_value())
		{
			last_counter_ = counter;
			return CounterObservation::kFirstValue;
		}
		if (counter == last_counter_.value())
			return CounterObservation::kDuplicate;
		const auto expected = static_cast<std::uint8_t>(
			(last_counter_.value() + 1U) % modulo_);
		if (counter != expected)
		{
			// 当前 jump 不成为可信样本，清空后由下一条合法帧重新建立连续性基线
			last_counter_.reset();
			return CounterObservation::kJump;
		}
		last_counter_ = counter;
		return CounterObservation::kExpected;
	}

	void RollingCounterChecker::reset() noexcept
	{
		last_counter_.reset();
	}
}  // namespace control_link_adapters
