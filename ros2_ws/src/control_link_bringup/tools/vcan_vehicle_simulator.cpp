#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <filesystem>
#include <memory>
#include <limits>
#include <mutex>
#include <optional>
#include <stdexcept>
#include <string>
#include <utility>

#include "control_link_adapters/can_codec.hpp"
#include "control_link_adapters/local_watchdog.hpp"
#include "control_link_adapters/socketcan_transport.hpp"
#include "control_link_contract/contract_bundle.hpp"
#include "control_link_contract/fastdds_environment.hpp"
#include "control_link_contract/parser.hpp"
#include "rclcpp/rclcpp.hpp"

namespace
{
	using namespace std::chrono_literals;
	using control_link_adapters::CanCodec;
	using control_link_adapters::CanCodecRejectReason;
	using control_link_adapters::CanControlMode;
	using control_link_adapters::CanControlSignals;
	using control_link_adapters::CanFrame;
	using control_link_adapters::CanVehicleMode;
	using control_link_adapters::CanVehicleStateSignals;
	using control_link_adapters::CounterObservation;
	using control_link_adapters::LocalWatchdog;
	using control_link_adapters::RollingCounterChecker;
	using control_link_adapters::SocketCanIoError;
	using control_link_adapters::SocketCanTransport;
	using control_link_adapters::SocketCanTransportConfig;

	constexpr char kProfilePathParameter[] = "profile_path";
	constexpr char kConfigRootParameter[] = "config_root";
	constexpr char kDropStateParameter[] = "drop_state";
	constexpr char kCorruptCrcParameter[] = "corrupt_crc";
	constexpr char kFreezeCounterParameter[] = "freeze_counter";
	constexpr char kFaultCodeParameter[] = "fault_code";

	std::chrono::milliseconds checked_milliseconds(
		std::uint64_t value,
		const char *field_name)
	{
		const auto maximum = static_cast<std::uint64_t>(
			std::numeric_limits<std::chrono::milliseconds::rep>::max());
		if (value == 0U || value > maximum)
		{
			throw std::invalid_argument(
				std::string(field_name) +
				" must fit a positive milliseconds duration");
		}
		return std::chrono::milliseconds{
			static_cast<std::chrono::milliseconds::rep>(value)};
	}

	class VcanVehicleSimulator final : public rclcpp::Node
	{
	public:
		VcanVehicleSimulator()
			: rclcpp::Node("vehicle_simulator"),
			  control_counter_checker_(2U)
		{
			declare_parameter<std::string>(kProfilePathParameter, "");
			declare_parameter<std::string>(kConfigRootParameter, "");
			declare_parameter<bool>(kDropStateParameter, false);
			declare_parameter<bool>(kCorruptCrcParameter, false);
			declare_parameter<bool>(kFreezeCounterParameter, false);
			declare_parameter<std::int64_t>(kFaultCodeParameter, 0);

			const auto profile_path = std::filesystem::path{
				get_parameter(kProfilePathParameter).as_string()};
			const auto config_root = std::filesystem::path{
				get_parameter(kConfigRootParameter).as_string()};
			bundle_ = control_link_contract::load_contract_bundle(
				profile_path,
				config_root);
			adas_profile_ = std::get_if<control_link_contract::AdasProfile>(
				bundle_->profile.get());
			if (adas_profile_ == nullptr)
			{
				throw std::invalid_argument(
					"VehicleSimulator requires profile_id=adas");
			}
			(void)control_link_contract::validate_fastdds_process_environment(
				*bundle_->profile,
				"VehicleSimulator");

			signal_map_ = bundle_->can_signal_map;
			if (!signal_map_)
			{
				throw std::logic_error("ADAS ContractBundle is missing its CAN signal map");
			}
			codec_ = std::make_unique<CanCodec>(*signal_map_);
			control_counter_checker_ = RollingCounterChecker(
				codec_->config().rolling_counter_modulo);
			control_watchdog_ = std::make_unique<LocalWatchdog>(
				adas_profile_->vehicle_simulator.local_watchdog_timeout_ms);

			const auto state_period = checked_milliseconds(
				adas_profile_->vehicle_simulator.state_period_ms,
				"vehicle_simulator.state_period_ms");
			transport_ = std::make_unique<SocketCanTransport>(
				SocketCanTransportConfig{
					adas_profile_->adapter.interface,
					codec_->config().control_can_id,
					state_period,
					state_period});

			{
				std::lock_guard<std::mutex> lock{simulation_mutex_};
				refresh_parameters_locked();
				last_model_update_ = std::chrono::steady_clock::now();
			}
			transport_->start(
				[this](
					const CanFrame &frame,
					std::chrono::steady_clock::time_point received_at)
				{
					handle_control_frame(frame, received_at);
				},
				[this](const SocketCanIoError &error)
				{
					handle_can_error(error);
				},
				[this](std::chrono::steady_clock::time_point now)
				{
					return make_periodic_state_frame(now);
				},
				[this](const CanFrame &frame)
				{
					confirm_state_frame_transmitted(frame);
				});

			parameter_timer_ = create_wall_timer(
				100ms,
				[this]()
				{
					refresh_parameters();
				});

			RCLCPP_INFO(
				get_logger(),
				"VehicleSimulator ready: interface=%s, control_id=0x%03X, state_id=0x%03X, state_period_ms=%llu",
				adas_profile_->adapter.interface.c_str(),
				codec_->config().control_can_id,
				codec_->config().state_can_id,
				static_cast<unsigned long long>(
					adas_profile_->vehicle_simulator.state_period_ms));
		}

		~VcanVehicleSimulator() override
		{
			if (transport_)
			{
				transport_->stop();
			}
		}

	private:
		void handle_control_frame(
			const CanFrame &frame,
			std::chrono::steady_clock::time_point received_at) noexcept
		{
			const auto decoded = codec_->decode_control(frame);
			std::lock_guard<std::mutex> lock{simulation_mutex_};
			if (!decoded.accepted())
			{
				last_control_reject_reason_ = decoded.reason;
				++rejected_control_frames_;
				return;
			}

			const auto observation = control_counter_checker_.observe(
				decoded.signals->rolling_counter);
			last_control_counter_observation_ = observation;
			if (observation != CounterObservation::kFirstValue &&
				observation != CounterObservation::kExpected)
			{
				++rejected_control_frames_;
				return;
			}

			control_watchdog_->observe_valid_command(received_at);
			last_control_counter_ = decoded.signals->rolling_counter;
			last_control_reject_reason_ = CanCodecRejectReason::kNone;
			++accepted_control_frames_;
			if (decoded.signals->mode == CanControlMode::kHold)
			{
				target_speed_mps_ = 0.0;
				target_yaw_rate_radps_ = 0.0;
				vehicle_mode_ = CanVehicleMode::kStandby;
			}
			else
			{
				target_speed_mps_ = decoded.signals->target_speed_mps;
				target_yaw_rate_radps_ = decoded.signals->target_yaw_rate_radps;
				vehicle_mode_ = injected_fault_code_ == 0U ?
					CanVehicleMode::kRunning : CanVehicleMode::kFault;
			}
		}

		void handle_can_error(const SocketCanIoError &error) noexcept
		{
			std::lock_guard<std::mutex> lock{simulation_mutex_};
			last_can_error_ = error;
			if (error.fatal)
			{
				fatal_can_io_ = true;
			}
		}

		std::optional<CanFrame> make_periodic_state_frame(
			std::chrono::steady_clock::time_point now)
		{
			std::lock_guard<std::mutex> lock{simulation_mutex_};
			update_model_locked(now);
			if (drop_state_)
			{
				return std::nullopt;
			}

			const auto encoded = codec_->encode_state(
				CanVehicleStateSignals{
					vehicle_mode_,
					measured_speed_mps_,
					measured_yaw_rate_radps_,
					vehicle_mode_ == CanVehicleMode::kFault ?
						injected_fault_code_ : static_cast<std::uint8_t>(0U),
					next_state_counter_,
					last_control_counter_});
			if (!encoded.accepted())
			{
				throw std::logic_error(
					std::string("simulator state encoding failed: ") +
					control_link_adapters::can_codec_reject_reason_name(
						encoded.reason));
			}

			CanFrame frame = encoded.frame.value();
			if (corrupt_crc_)
			{
				frame.data[7U] ^= 0xFFU;
			}
			return frame;
		}

		void confirm_state_frame_transmitted(const CanFrame &frame)
		{
			std::lock_guard<std::mutex> lock{simulation_mutex_};
			if (freeze_counter_)
			{
				return;
			}
			const auto transmitted_counter = static_cast<std::uint8_t>(
				frame.data[6U] & 0x0FU);
			if (transmitted_counter != next_state_counter_)
			{
				throw std::logic_error(
					"simulator transmitted state counter does not match pending counter");
			}
			next_state_counter_ = static_cast<std::uint8_t>(
				(next_state_counter_ + 1U) %
				codec_->config().rolling_counter_modulo);
		}

		void update_model_locked(std::chrono::steady_clock::time_point now)
		{
			if (now < last_model_update_)
			{
				throw std::logic_error(
					"VehicleSimulator steady clock moved backwards");
			}
			const auto elapsed = std::chrono::duration<double>(
				now - last_model_update_).count();
			last_model_update_ = now;

			const auto watchdog_state = control_watchdog_->evaluate(now);
			if (watchdog_state != control_link_adapters::LocalWatchdogState::kHealthy ||
				fatal_can_io_)
			{
				target_speed_mps_ = 0.0;
				target_yaw_rate_radps_ = 0.0;
				vehicle_mode_ = CanVehicleMode::kSafeStop;
			}
			else if (injected_fault_code_ != 0U)
			{
				target_speed_mps_ = 0.0;
				target_yaw_rate_radps_ = 0.0;
				vehicle_mode_ = CanVehicleMode::kFault;
			}

			const double tau_seconds =
				static_cast<double>(adas_profile_->vehicle_simulator.first_order_time_constant_ms) /
				1000.0;
			const double alpha = tau_seconds <= 0.0 ? 1.0 :
				1.0 - std::exp(-elapsed / tau_seconds);
			measured_speed_mps_ += alpha * (target_speed_mps_ - measured_speed_mps_);
			measured_yaw_rate_radps_ +=
				alpha * (target_yaw_rate_radps_ - measured_yaw_rate_radps_);
		}

		void refresh_parameters_locked()
		{
			drop_state_ = get_parameter(kDropStateParameter).as_bool();
			corrupt_crc_ = get_parameter(kCorruptCrcParameter).as_bool();
			freeze_counter_ = get_parameter(kFreezeCounterParameter).as_bool();
			const auto raw_fault_code = get_parameter(kFaultCodeParameter).as_int();
			if (raw_fault_code < 0 || raw_fault_code > 255)
			{
				throw std::out_of_range(
					"fault_code parameter must be within [0,255]");
			}
			injected_fault_code_ = static_cast<std::uint8_t>(raw_fault_code);
		}

		void refresh_parameters()
		{
			try
			{
				std::lock_guard<std::mutex> lock{simulation_mutex_};
				refresh_parameters_locked();
			}
			catch (const std::exception &exception)
			{
				RCLCPP_WARN_THROTTLE(
					get_logger(),
					*get_clock(),
					2000,
					"VehicleSimulator parameter update rejected: %s",
					exception.what());
			}
		}

		control_link_contract::ContractBundlePtr bundle_;
		const control_link_contract::AdasProfile *adas_profile_{nullptr};
		control_link_contract::CanSignalMapPtr signal_map_;
		std::unique_ptr<CanCodec> codec_;
		std::unique_ptr<LocalWatchdog> control_watchdog_;
		std::unique_ptr<SocketCanTransport> transport_;

		rclcpp::TimerBase::SharedPtr parameter_timer_;
		std::mutex simulation_mutex_;
		RollingCounterChecker control_counter_checker_;
		std::chrono::steady_clock::time_point last_model_update_{};
		CanVehicleMode vehicle_mode_{CanVehicleMode::kSafeStop};
		double target_speed_mps_{0.0};
		double target_yaw_rate_radps_{0.0};
		double measured_speed_mps_{0.0};
		double measured_yaw_rate_radps_{0.0};
		std::uint8_t injected_fault_code_{0U};
		std::uint8_t last_control_counter_{0U};
		std::uint8_t next_state_counter_{0U};
		bool drop_state_{false};
		bool corrupt_crc_{false};
		bool freeze_counter_{false};
		bool fatal_can_io_{false};
		CanCodecRejectReason last_control_reject_reason_{
			CanCodecRejectReason::kNone};
		CounterObservation last_control_counter_observation_{
			CounterObservation::kFirstValue};
		std::uint64_t accepted_control_frames_{0U};
		std::uint64_t rejected_control_frames_{0U};
		std::optional<SocketCanIoError> last_can_error_;
	};
}

int main(int argc, char **argv)
{
	rclcpp::init(argc, argv);
	try
	{
		auto node = std::make_shared<VcanVehicleSimulator>();
		rclcpp::spin(node);
	}
	catch (const std::exception &exception)
	{
		RCLCPP_FATAL(
			rclcpp::get_logger("vcan_vehicle_simulator"),
			"VehicleSimulator failed: %s",
			exception.what());
		rclcpp::shutdown();
		return 1;
	}
	rclcpp::shutdown();
	return 0;
}
