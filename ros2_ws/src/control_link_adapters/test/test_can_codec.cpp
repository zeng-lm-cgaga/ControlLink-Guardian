#include <array>
#include <cmath>
#include <cstdint>
#include <limits>
#include <string>
#include <vector>

#include <gtest/gtest.h>

#include "control_link_adapters/can_codec.hpp"
#include "control_link_contract/parser.hpp"

namespace control_link_adapters
{
	namespace
	{
		control_link_contract::CanSignalMapPtr load_signal_map()
		{
			return control_link_contract::load_can_signal_map(
				CONTROL_LINK_TEST_CAN_SIGNAL_MAP_PATH);
		}

		std::uint8_t reference_crc_update(
			std::uint8_t crc,
			std::uint8_t input,
			std::uint8_t polynomial) noexcept
		{
			crc ^= input;
			for (std::uint8_t bit = 0U; bit < 8U; ++bit)
			{
				const bool high_bit_set = (crc & 0x80U) != 0U;
				crc = static_cast<std::uint8_t>(crc << 1U);
				if (high_bit_set)
				{
					crc ^= polynomial;
				}
			}
			return crc;
		}

		// 独立参考实现只消费协议参数和原始帧，不调用 CanCodec 的私有 CRC 路径
		std::uint8_t reference_crc(
			const CanFrame &frame,
			const control_link_contract::CanSignalMap &signal_map) noexcept
		{
			std::uint8_t crc = signal_map.crc.initial_value;
			crc = reference_crc_update(
				crc,
				static_cast<std::uint8_t>(frame.can_id & 0xFFU),
				signal_map.crc.polynomial);
			crc = reference_crc_update(
				crc,
				static_cast<std::uint8_t>((frame.can_id >> 8U) & 0xFFU),
				signal_map.crc.polynomial);
			for (const auto payload_index : signal_map.crc.protected_payload_bytes)
			{
				crc = reference_crc_update(
					crc,
					frame.data[payload_index],
					signal_map.crc.polynomial);
			}
			return static_cast<std::uint8_t>(crc ^ signal_map.crc.final_xor);
		}

		void refresh_crc(
			CanFrame &frame,
			const control_link_contract::CanSignalMap &signal_map) noexcept
		{
			frame.data[7U] = reference_crc(frame, signal_map);
		}

		CanFrame encode_control_or_fail(
			const CanCodec &codec,
			const CanControlSignals &signals)
		{
			const auto result = codec.encode_control(signals);
			EXPECT_TRUE(result.accepted()) << can_codec_reject_reason_name(result.reason);
			return result.frame.value_or(CanFrame{});
		}

		CanFrame encode_state_or_fail(
			const CanCodec &codec,
			const CanVehicleStateSignals &signals)
		{
			const auto result = codec.encode_state(signals);
			EXPECT_TRUE(result.accepted()) << can_codec_reject_reason_name(result.reason);
			return result.frame.value_or(CanFrame{});
		}

		TEST(CanCodecTest, LoadsRealSignalMapAndMatchesControlGoldenVector)
		{
			const auto signal_map = load_signal_map();
			const CanCodec codec{*signal_map};
			EXPECT_EQ(codec.config().control_can_id, 0x180U);
			EXPECT_EQ(codec.config().state_can_id, 0x280U);
			EXPECT_EQ(codec.config().dlc, 8U);
			EXPECT_EQ(codec.config().rolling_counter_modulo, 16U);

			const auto frame = encode_control_or_fail(
				codec,
				CanControlSignals{
					CanControlMode::kActive,
					0.42,
					-0.125,
					15U});
			const std::array<std::uint8_t, 8U> expected{
				0x2AU, 0x00U, 0x83U, 0xFFU, 0x01U, 0x0FU, 0x01U, 0xDEU};
			EXPECT_EQ(frame.can_id, 0x180U);
			EXPECT_EQ(frame.data, expected);
			EXPECT_EQ(frame.data[7U], reference_crc(frame, *signal_map));

			const auto decoded = codec.decode_control(frame);
			ASSERT_TRUE(decoded.accepted());
			ASSERT_TRUE(decoded.signals.has_value());
			EXPECT_EQ(decoded.signals->mode, CanControlMode::kActive);
			EXPECT_NEAR(decoded.signals->target_speed_mps, 0.42, 1.0e-12);
			EXPECT_NEAR(decoded.signals->target_yaw_rate_radps, -0.125, 1.0e-12);
			EXPECT_EQ(decoded.signals->rolling_counter, 15U);
		}

		TEST(CanCodecTest, MatchesStateGoldenVectorAndPreservesCounterEcho)
		{
			const auto signal_map = load_signal_map();
			const CanCodec codec{*signal_map};
			const auto frame = encode_state_or_fail(
				codec,
				CanVehicleStateSignals{
					CanVehicleMode::kRunning,
					-0.37,
					0.642,
					23U,
					15U,
					9U});
			const std::array<std::uint8_t, 8U> expected{
				0xDBU, 0xFFU, 0x82U, 0x02U, 0x01U, 0x17U, 0x9FU, 0x8AU};
			EXPECT_EQ(frame.can_id, 0x280U);
			EXPECT_EQ(frame.data, expected);
			EXPECT_EQ(frame.data[7U], reference_crc(frame, *signal_map));

			const auto decoded = codec.decode_state(frame);
			ASSERT_TRUE(decoded.accepted());
			ASSERT_TRUE(decoded.signals.has_value());
			EXPECT_EQ(decoded.signals->mode, CanVehicleMode::kRunning);
			EXPECT_NEAR(decoded.signals->measured_speed_mps, -0.37, 1.0e-12);
			EXPECT_NEAR(decoded.signals->measured_yaw_rate_radps, 0.642, 1.0e-12);
			EXPECT_EQ(decoded.signals->fault_code, 23U);
			EXPECT_EQ(decoded.signals->state_counter, 15U);
			EXPECT_EQ(decoded.signals->echoed_control_counter, 9U);
		}

		TEST(CanCodecTest, EncodesHoldAndPhysicalBoundsWithExplicitLittleEndianBytes)
		{
			const auto signal_map = load_signal_map();
			const CanCodec codec{*signal_map};
			const auto hold = encode_control_or_fail(
				codec,
				CanControlSignals{CanControlMode::kHold, 0.0, 0.0, 0U});
			EXPECT_EQ(hold.data[0U], 0U);
			EXPECT_EQ(hold.data[1U], 0U);
			EXPECT_EQ(hold.data[2U], 0U);
			EXPECT_EQ(hold.data[3U], 0U);
			EXPECT_EQ(hold.data[4U], 0U);
			EXPECT_EQ(hold.data[6U], 1U);

			const auto minimum = encode_control_or_fail(
				codec,
				CanControlSignals{CanControlMode::kActive, -1.0, -1.5, 14U});
			EXPECT_EQ(minimum.data[0U], 0x9CU);
			EXPECT_EQ(minimum.data[1U], 0xFFU);
			EXPECT_EQ(minimum.data[2U], 0x24U);
			EXPECT_EQ(minimum.data[3U], 0xFAU);

			const auto maximum = encode_control_or_fail(
				codec,
				CanControlSignals{CanControlMode::kActive, 1.0, 1.5, 15U});
			EXPECT_EQ(maximum.data[0U], 0x64U);
			EXPECT_EQ(maximum.data[1U], 0x00U);
			EXPECT_EQ(maximum.data[2U], 0xDCU);
			EXPECT_EQ(maximum.data[3U], 0x05U);
		}

		TEST(CanCodecTest, RejectsInvalidControlAndStateSignalsBeforeQuantization)
		{
			const auto signal_map = load_signal_map();
			const CanCodec codec{*signal_map};
			auto control = CanControlSignals{CanControlMode::kActive, 0.1, -0.2, 1U};

			control.mode = static_cast<CanControlMode>(2U);
			EXPECT_EQ(codec.encode_control(control).reason, CanCodecRejectReason::kInvalidMode);
			control = {CanControlMode::kActive, 0.1, -0.2, 16U};
			EXPECT_EQ(
				codec.encode_control(control).reason,
				CanCodecRejectReason::kCounterOutOfRange);
			control = {
				CanControlMode::kActive,
				std::numeric_limits<double>::quiet_NaN(),
				0.0,
				1U};
			EXPECT_EQ(codec.encode_control(control).reason, CanCodecRejectReason::kNonFinite);
			control = {CanControlMode::kActive, 1.01, 0.0, 1U};
			EXPECT_EQ(codec.encode_control(control).reason, CanCodecRejectReason::kOutOfRange);
			control = {CanControlMode::kHold, 0.01, 0.0, 1U};
			EXPECT_EQ(codec.encode_control(control).reason, CanCodecRejectReason::kHoldNonzero);

			auto state = CanVehicleStateSignals{
				static_cast<CanVehicleMode>(4U), 0.0, 0.0, 0U, 0U, 0U};
			EXPECT_EQ(codec.encode_state(state).reason, CanCodecRejectReason::kInvalidMode);
			state = {CanVehicleMode::kRunning, 0.0, 0.0, 0U, 16U, 0U};
			EXPECT_EQ(codec.encode_state(state).reason, CanCodecRejectReason::kCounterOutOfRange);
			state = {CanVehicleMode::kRunning, 0.0, 0.0, 0U, 0U, 16U};
			EXPECT_EQ(codec.encode_state(state).reason, CanCodecRejectReason::kCounterOutOfRange);
			state = {
				CanVehicleMode::kRunning,
				0.0,
				std::numeric_limits<double>::infinity(),
				0U,
				0U,
				0U};
			EXPECT_EQ(codec.encode_state(state).reason, CanCodecRejectReason::kNonFinite);
		}

		TEST(CanCodecTest, RejectsControlFrameShapeIntegrityAndSemanticFailures)
		{
			const auto signal_map = load_signal_map();
			const CanCodec codec{*signal_map};
			const auto valid = encode_control_or_fail(
				codec,
				CanControlSignals{CanControlMode::kActive, 0.2, -0.1, 3U});

			auto frame = valid;
			frame.can_id = codec.config().state_can_id;
			EXPECT_EQ(codec.decode_control(frame).reason, CanCodecRejectReason::kWrongCanId);
			frame = valid;
			frame.dlc = 7U;
			EXPECT_EQ(codec.decode_control(frame).reason, CanCodecRejectReason::kWrongDlc);
			frame = valid;
			frame.is_extended = true;
			EXPECT_EQ(codec.decode_control(frame).reason, CanCodecRejectReason::kExtendedFrame);
			frame = valid;
			frame.is_remote = true;
			EXPECT_EQ(codec.decode_control(frame).reason, CanCodecRejectReason::kRemoteFrame);

			frame = valid;
			frame.data[5U] |= 0x10U;
			refresh_crc(frame, *signal_map);
			EXPECT_EQ(
				codec.decode_control(frame).reason,
				CanCodecRejectReason::kReservedBitsNonzero);
			frame = valid;
			frame.data[6U] |= 0x02U;
			refresh_crc(frame, *signal_map);
			EXPECT_EQ(
				codec.decode_control(frame).reason,
				CanCodecRejectReason::kReservedBitsNonzero);
			frame = valid;
			frame.data[6U] = 0U;
			refresh_crc(frame, *signal_map);
			EXPECT_EQ(
				codec.decode_control(frame).reason,
				CanCodecRejectReason::kInvalidCommandValid);

			frame = valid;
			frame.data[7U] ^= 0x01U;
			EXPECT_EQ(codec.decode_control(frame).reason, CanCodecRejectReason::kCrcMismatch);
			frame = valid;
			frame.data[4U] = 2U;
			refresh_crc(frame, *signal_map);
			EXPECT_EQ(codec.decode_control(frame).reason, CanCodecRejectReason::kInvalidMode);
			frame = valid;
			frame.data[4U] = static_cast<std::uint8_t>(CanControlMode::kHold);
			refresh_crc(frame, *signal_map);
			EXPECT_EQ(codec.decode_control(frame).reason, CanCodecRejectReason::kHoldNonzero);
		}

		TEST(CanCodecTest, RejectsStateFrameShapeCrcModeAndPhysicalRangeFailures)
		{
			const auto signal_map = load_signal_map();
			const CanCodec codec{*signal_map};
			const auto valid = encode_state_or_fail(
				codec,
				CanVehicleStateSignals{
					CanVehicleMode::kRunning, 0.2, 0.1, 0U, 3U, 7U});

			auto frame = valid;
			frame.can_id = codec.config().control_can_id;
			EXPECT_EQ(codec.decode_state(frame).reason, CanCodecRejectReason::kWrongCanId);
			frame = valid;
			frame.dlc = 9U;
			EXPECT_EQ(codec.decode_state(frame).reason, CanCodecRejectReason::kWrongDlc);
			frame = valid;
			frame.is_extended = true;
			EXPECT_EQ(codec.decode_state(frame).reason, CanCodecRejectReason::kExtendedFrame);
			frame = valid;
			frame.is_remote = true;
			EXPECT_EQ(codec.decode_state(frame).reason, CanCodecRejectReason::kRemoteFrame);
			frame = valid;
			frame.data[7U] ^= 0x80U;
			EXPECT_EQ(codec.decode_state(frame).reason, CanCodecRejectReason::kCrcMismatch);

			frame = valid;
			frame.data[4U] = 4U;
			refresh_crc(frame, *signal_map);
			EXPECT_EQ(codec.decode_state(frame).reason, CanCodecRejectReason::kInvalidMode);
			frame = valid;
			frame.data[0U] = 0xFFU;
			frame.data[1U] = 0x7FU;
			refresh_crc(frame, *signal_map);
			EXPECT_EQ(codec.decode_state(frame).reason, CanCodecRejectReason::kOutOfRange);
		}

		TEST(CanCodecTest, RejectsSignalMapsOutsideTheFixedV1CodecContract)
		{
			auto signal_map = *load_signal_map();
			signal_map.crc.reflect_input = true;
			EXPECT_THROW(CanCodec{signal_map}, std::invalid_argument);

			signal_map = *load_signal_map();
			signal_map.crc.protected_payload_bytes = {0U, 1U};
			EXPECT_THROW(CanCodec{signal_map}, std::invalid_argument);

			signal_map = *load_signal_map();
			signal_map.state_frame.measured_speed_mps.scale = 0.02;
			EXPECT_THROW(CanCodec{signal_map}, std::invalid_argument);

			signal_map = *load_signal_map();
			signal_map.control_frame.target_speed_mps.offset = 1.0;
			EXPECT_THROW(CanCodec{signal_map}, std::invalid_argument);
		}

		TEST(RollingCounterCheckerTest, DistinguishesFirstExpectedDuplicateJumpAndInvalid)
		{
			EXPECT_THROW(RollingCounterChecker{1U}, std::invalid_argument);
			EXPECT_THROW(RollingCounterChecker{17U}, std::invalid_argument);

			RollingCounterChecker checker{16U};
			EXPECT_EQ(checker.observe(7U), CounterObservation::kFirstValue);
			EXPECT_EQ(checker.observe(8U), CounterObservation::kExpected);
			EXPECT_EQ(checker.observe(8U), CounterObservation::kDuplicate);
			EXPECT_EQ(checker.observe(10U), CounterObservation::kJump);
			EXPECT_EQ(checker.observe(10U), CounterObservation::kFirstValue);
			EXPECT_EQ(checker.observe(11U), CounterObservation::kExpected);
			EXPECT_EQ(checker.observe(16U), CounterObservation::kInvalid);
			EXPECT_EQ(checker.observe(12U), CounterObservation::kExpected);

			checker.reset();
			EXPECT_EQ(checker.observe(15U), CounterObservation::kFirstValue);
			EXPECT_EQ(checker.observe(0U), CounterObservation::kExpected);
		}

		TEST(CanCodecTest, ExposesStableRejectReasonNames)
		{
			const std::vector<std::pair<CanCodecRejectReason, std::string>> expected{
				{CanCodecRejectReason::kNone, "none"},
				{CanCodecRejectReason::kWrongCanId, "wrong_can_id"},
				{CanCodecRejectReason::kWrongDlc, "wrong_dlc"},
				{CanCodecRejectReason::kExtendedFrame, "extended_frame"},
				{CanCodecRejectReason::kRemoteFrame, "remote_frame"},
				{CanCodecRejectReason::kCrcMismatch, "crc_mismatch"},
				{CanCodecRejectReason::kReservedBitsNonzero, "reserved_bits_nonzero"},
				{CanCodecRejectReason::kInvalidMode, "invalid_mode"},
				{CanCodecRejectReason::kInvalidCommandValid, "invalid_command_valid"},
				{CanCodecRejectReason::kHoldNonzero, "hold_nonzero"},
				{CanCodecRejectReason::kNonFinite, "non_finite"},
				{CanCodecRejectReason::kOutOfRange, "out_of_range"},
				{CanCodecRejectReason::kCounterOutOfRange, "counter_out_of_range"},
			};
			for (const auto &[reason, name] : expected)
			{
				EXPECT_EQ(std::string{can_codec_reject_reason_name(reason)}, name);
			}
			EXPECT_STREQ(
				can_codec_reject_reason_name(static_cast<CanCodecRejectReason>(255U)),
				"unknown");
		}
	}  // namespace
}  // namespace control_link_adapters
