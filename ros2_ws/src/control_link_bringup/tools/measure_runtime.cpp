#include <algorithm>
#include <array>
#include <atomic>
#include <cerrno>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <functional>
#include <iomanip>
#include <limits>
#include <map>
#include <memory>
#include <mutex>
#include <optional>
#include <sstream>
#include <stdexcept>
#include <string>
#include <system_error>
#include <thread>
#include <utility>
#include <variant>
#include <vector>

#include <fcntl.h>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <net/if.h>
#include <poll.h>
#include <sys/eventfd.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <unistd.h>

#include <nlohmann/json.hpp>

#include "control_link_adapters/can_codec.hpp"
#include "control_link_contract/contract_bundle.hpp"
#include "control_link_contract/fastdds_environment.hpp"
#include "control_link_contract/parser.hpp"
#include "control_link_contract/qos_factory.hpp"
#include "control_link_interfaces/msg/control_command.hpp"
#include "control_link_interfaces/msg/gateway_state.hpp"
#include "rclcpp/rclcpp.hpp"

namespace
{
	using Clock = std::chrono::steady_clock;
	using ControlCommand = control_link_interfaces::msg::ControlCommand;
	using GatewayState = control_link_interfaces::msg::GatewayState;
	using Json = nlohmann::json;
	using control_link_adapters::CanCodec;
	using control_link_adapters::CanFrame;

	constexpr std::size_t kCounterDomainSize = 16U;
	constexpr std::size_t kMaximumPendingInputs = 4096U;

	std::int64_t steady_nanoseconds(Clock::time_point time) noexcept
	{
		return std::chrono::duration_cast<std::chrono::nanoseconds>(
			time.time_since_epoch()).count();
	}

	std::chrono::seconds checked_seconds(
		std::int64_t value,
		const char *field,
		std::int64_t minimum)
	{
		if (value < minimum || value > 86'400)
		{
			throw std::out_of_range(
				std::string(field) + " must be in [" + std::to_string(minimum) +
				",86400]");
		}
		return std::chrono::seconds{value};
	}

	std::ofstream open_csv(const std::filesystem::path &path)
	{
		std::ofstream stream{path, std::ios::out | std::ios::trunc};
		if (!stream.is_open())
		{
			throw std::runtime_error("cannot create performance file: " + path.string());
		}
		stream << std::setprecision(17);
		return stream;
	}

	std::string csv_string(const std::string &value)
	{
		std::string escaped{"\""};
		for (const char character : value)
		{
			if (character == '"')
			{
				escaped += "\"\"";
			}
			else
			{
				escaped += character;
			}
		}
		escaped += '"';
		return escaped;
	}

	double percentile(const std::vector<double> &sorted, double quantile)
	{
		if (sorted.empty())
		{
			throw std::invalid_argument("percentile requires at least one sample");
		}
		const double position = quantile * static_cast<double>(sorted.size() - 1U);
		const auto lower = static_cast<std::size_t>(std::floor(position));
		const auto upper = static_cast<std::size_t>(std::ceil(position));
		const double fraction = position - static_cast<double>(lower);
		return sorted[lower] + (sorted[upper] - sorted[lower]) * fraction;
	}

	Json summarize_samples(const std::vector<double> &samples)
	{
		Json result;
		result["count"] = samples.size();
		if (samples.empty())
		{
			result["min"] = nullptr;
			result["mean"] = nullptr;
			result["stddev"] = nullptr;
			result["p50"] = nullptr;
			result["p95"] = nullptr;
			result["p99"] = nullptr;
			result["max"] = nullptr;
			return result;
		}

		std::vector<double> sorted = samples;
		std::sort(sorted.begin(), sorted.end());
		double sum = 0.0;
		for (const double sample : samples)
		{
			sum += sample;
		}
		const double mean = sum / static_cast<double>(samples.size());
		double squared_error = 0.0;
		for (const double sample : samples)
		{
			const double error = sample - mean;
			squared_error += error * error;
		}

		result["min"] = sorted.front();
		result["mean"] = mean;
		result["stddev"] = std::sqrt(
			squared_error / static_cast<double>(samples.size()));
		result["p50"] = percentile(sorted, 0.50);
		result["p95"] = percentile(sorted, 0.95);
		result["p99"] = percentile(sorted, 0.99);
		result["max"] = sorted.back();
		return result;
	}

	void close_descriptor(int descriptor) noexcept
	{
		if (descriptor >= 0)
		{
			(void)::close(descriptor);
		}
	}

	int open_event_descriptor()
	{
		const int descriptor = ::eventfd(0U, EFD_NONBLOCK | EFD_CLOEXEC);
		if (descriptor == -1)
		{
			throw std::system_error(
				errno,
				std::generic_category(),
				"measure_runtime eventfd() failed");
		}
		return descriptor;
	}

	int open_can_observer(
		const std::string &interface,
		std::uint32_t control_can_id,
		std::uint32_t state_can_id)
	{
		if (interface.empty() || interface.size() >= IFNAMSIZ)
		{
			throw std::invalid_argument(
				"measure_runtime CAN interface must fit Linux IFNAMSIZ");
		}

		const int descriptor = ::socket(
			PF_CAN,
			SOCK_RAW | SOCK_NONBLOCK | SOCK_CLOEXEC,
			CAN_RAW);
		if (descriptor == -1)
		{
			throw std::system_error(
				errno,
				std::generic_category(),
				"measure_runtime socket(PF_CAN) failed");
		}

		try
		{
			ifreq request{};
			std::copy(interface.begin(), interface.end(), request.ifr_name);
			request.ifr_name[interface.size()] = '\0';
			if (::ioctl(descriptor, SIOCGIFINDEX, &request) == -1)
			{
				throw std::system_error(
					errno,
					std::generic_category(),
					"measure_runtime ioctl(SIOCGIFINDEX) failed");
			}

			const can_filter filters[2]{
				can_filter{control_can_id, CAN_SFF_MASK | CAN_EFF_FLAG | CAN_RTR_FLAG},
				can_filter{state_can_id, CAN_SFF_MASK | CAN_EFF_FLAG | CAN_RTR_FLAG}};
			if (::setsockopt(
					descriptor,
					SOL_CAN_RAW,
					CAN_RAW_FILTER,
					filters,
					sizeof(filters)) == -1)
			{
				throw std::system_error(
					errno,
					std::generic_category(),
					"measure_runtime setsockopt(CAN_RAW_FILTER) failed");
			}

			sockaddr_can address{};
			address.can_family = AF_CAN;
			address.can_ifindex = request.ifr_ifindex;
			if (::bind(
					descriptor,
					reinterpret_cast<const sockaddr *>(&address),
					sizeof(address)) == -1)
			{
				throw std::system_error(
					errno,
					std::generic_category(),
					"measure_runtime bind(AF_CAN) failed");
			}
		}
		catch (...)
		{
			close_descriptor(descriptor);
			throw;
		}
		return descriptor;
	}

	CanFrame from_linux_frame(const can_frame &source) noexcept
	{
		const bool extended = (source.can_id & CAN_EFF_FLAG) != 0U;
		CanFrame frame{
			source.can_id & (extended ? CAN_EFF_MASK : CAN_SFF_MASK),
			source.can_dlc,
			extended,
			(source.can_id & CAN_RTR_FLAG) != 0U,
			{0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U}};
		std::copy(std::begin(source.data), std::end(source.data), frame.data.begin());
		return frame;
	}

	struct ProcessSample final
	{
		std::uint64_t cpu_ticks;
		std::uint64_t start_ticks;
		std::uint64_t rss_kib;
		std::uint64_t threads;
	};

	std::optional<ProcessSample> read_process_sample(std::int64_t pid)
	{
		const std::filesystem::path proc_root =
			std::filesystem::path{"/proc"} / std::to_string(pid);
		std::ifstream stat_stream{proc_root / "stat"};
		std::string stat_line;
		if (!stat_stream.is_open() || !std::getline(stat_stream, stat_line))
		{
			return std::nullopt;
		}

		const auto command_end = stat_line.rfind(')');
		if (command_end == std::string::npos || command_end + 2U >= stat_line.size())
		{
			return std::nullopt;
		}
		std::istringstream stat_fields{stat_line.substr(command_end + 2U)};
		std::vector<std::string> fields;
		std::string field;
		while (stat_fields >> field)
		{
			fields.push_back(field);
		}
		// fields[0] 是 /proc stat 的 field 3，utime/stime/starttime 分别是 14/15/22
		if (fields.size() <= 19U)
		{
			return std::nullopt;
		}

		std::uint64_t rss_kib = 0U;
		std::uint64_t threads = 0U;
		std::ifstream status_stream{proc_root / "status"};
		std::string status_line;
		while (std::getline(status_stream, status_line))
		{
			std::istringstream line{status_line};
			std::string key;
			line >> key;
			if (key == "VmRSS:")
			{
				line >> rss_kib;
			}
			else if (key == "Threads:")
			{
				line >> threads;
			}
		}

		try
		{
			return ProcessSample{
				std::stoull(fields[11U]) + std::stoull(fields[12U]),
				std::stoull(fields[19U]),
				rss_kib,
				threads};
		}
		catch (const std::exception &)
		{
			return std::nullopt;
		}
	}

	struct TargetProcess final
	{
		std::string label;
		std::int64_t pid;
		std::optional<ProcessSample> previous;
		std::optional<Clock::time_point> previous_at;
		std::uint64_t missing_samples{0U};
		std::vector<double> cpu_percent_samples;
		std::vector<double> rss_kib_samples;
	};

	struct PendingSourceSwitch final
	{
		std::string from_source_id;
		std::string to_source_id;
		Clock::time_point first_target_input_at;
	};

	class RuntimeMeasurement final : public rclcpp::Node
	{
	public:
		RuntimeMeasurement()
			: rclcpp::Node("measure_runtime")
		{
			declare_parameter<std::string>("profile_path", "");
			declare_parameter<std::string>("config_root", "");
			declare_parameter<std::string>("output_dir", "");
			declare_parameter<std::string>("can_interface", "");
			declare_parameter<std::int64_t>("warmup_seconds", 30);
			declare_parameter<std::int64_t>("measurement_seconds", 300);
			declare_parameter<std::int64_t>("min_output_ticks", 10'000);
			declare_parameter<std::int64_t>("max_extra_wait_seconds", 60);
			declare_parameter<bool>("require_source_switch", false);
			declare_parameter<std::vector<std::int64_t>>(
				"target_pids", std::vector<std::int64_t>{});
			declare_parameter<std::vector<std::string>>(
				"target_labels", std::vector<std::string>{});

			const std::filesystem::path profile_path{
				get_parameter("profile_path").as_string()};
			const std::filesystem::path config_root{
				get_parameter("config_root").as_string()};
			const std::filesystem::path output_dir{
				get_parameter("output_dir").as_string()};
			if (output_dir.empty())
			{
				throw std::invalid_argument("measure_runtime requires output_dir");
			}
			std::filesystem::create_directories(output_dir);
			for (const auto *artifact : {
					"output_ticks.csv",
					"callback_to_output.csv",
					"can_round_trip.csv",
					"source_switch.csv",
					"resources.csv",
					"summary.json"})
			{
				if (std::filesystem::exists(output_dir / artifact))
				{
					throw std::runtime_error(
						"measure_runtime refuses to overwrite existing artifact: " +
						(output_dir / artifact).string());
				}
			}

			bundle_ = control_link_contract::load_contract_bundle(
				profile_path,
				config_root);
			adas_profile_ = std::get_if<control_link_contract::AdasProfile>(
				bundle_->profile.get());
			if (adas_profile_ == nullptr)
			{
				throw std::invalid_argument(
					"measure_runtime first implementation requires profile_id=adas");
			}
			(void)control_link_contract::validate_fastdds_process_environment(
				*bundle_->profile,
				"RuntimeMeasurement");

			const std::string requested_interface =
				get_parameter("can_interface").as_string();
			can_interface_ = requested_interface.empty() ?
				adas_profile_->adapter.interface : requested_interface;
			if (can_interface_ != adas_profile_->adapter.interface)
			{
				throw std::invalid_argument(
					"measure_runtime can_interface must match ADAS Profile");
			}

			warmup_ = checked_seconds(
				get_parameter("warmup_seconds").as_int(),
				"warmup_seconds",
				0);
			measurement_duration_ = checked_seconds(
				get_parameter("measurement_seconds").as_int(),
				"measurement_seconds",
				1);
			max_extra_wait_ = checked_seconds(
				get_parameter("max_extra_wait_seconds").as_int(),
				"max_extra_wait_seconds",
				1);
			const auto minimum_ticks = get_parameter("min_output_ticks").as_int();
			if (minimum_ticks <= 0)
			{
				throw std::out_of_range("min_output_ticks must be positive");
			}
			minimum_output_ticks_ = static_cast<std::uint64_t>(minimum_ticks);
			require_source_switch_ = get_parameter("require_source_switch").as_bool();
			// 覆盖阈值只验证关键旁路确实产生了可统计样本，不作为产品性能门限
			minimum_callback_samples_ = std::max<std::uint64_t>(
				1U,
				minimum_output_ticks_ / 4U);
			minimum_can_round_trip_samples_ = std::max<std::uint64_t>(
				1U,
				minimum_output_ticks_ / 2U);
			minimum_resource_samples_ = std::max<std::uint64_t>(
				1U,
				static_cast<std::uint64_t>(measurement_duration_.count()) / 2U);
			nominal_output_period_ms_ =
				1000.0 / bundle_->gateway_contract->gateway.output_rate_hz;

			configure_target_processes();
			target_processes_.push_back(TargetProcess{
				"observer",
				static_cast<std::int64_t>(::getpid()),
				std::nullopt,
				std::nullopt,
				0U,
				{},
				{}});
			output_ticks_csv_ = open_csv(output_dir / "output_ticks.csv");
			callback_to_output_csv_ = open_csv(output_dir / "callback_to_output.csv");
			can_round_trip_csv_ = open_csv(output_dir / "can_round_trip.csv");
			source_switch_csv_ = open_csv(output_dir / "source_switch.csv");
			resources_csv_ = open_csv(output_dir / "resources.csv");
			output_ticks_csv_ <<
				"arrival_steady_ns,interval_ns,interval_ms,signed_jitter_ms,"
				"abs_jitter_ms,source_id,source_sequence,mode\n";
			callback_to_output_csv_ <<
				"input_arrival_steady_ns,output_arrival_steady_ns,latency_ns,"
				"latency_ms,source_id,source_sequence\n";
			can_round_trip_csv_ <<
				"control_send_observed_steady_ns,state_echo_observed_steady_ns,"
				"latency_ns,latency_ms,control_counter\n";
			source_switch_csv_ <<
				"first_target_input_steady_ns,switch_state_observed_steady_ns,"
				"latency_ns,latency_ms,from_source_id,to_source_id,"
				"transition_sequence,reason_code\n";
			resources_csv_ <<
				"sample_steady_ns,label,pid,cpu_percent,rss_kib,threads,alive\n";

			control_link_contract::QosFactory qos_factory{
				bundle_->gateway_contract};
			const auto input_qos = qos_factory.make(
				bundle_->gateway_contract->input.qos_profile);
			for (const auto &source_id : adas_profile_->common.enabled_sources)
			{
				const auto source = bundle_->source_policy->sources.find(source_id);
				if (source == bundle_->source_policy->sources.end())
				{
					throw std::logic_error(
						"enabled measurement source is absent from SourcePolicy: " +
						source_id);
				}
				input_subscriptions_.push_back(
					create_subscription<ControlCommand>(
						source->second.topic,
						input_qos,
						[this, source_id](ControlCommand::ConstSharedPtr command)
						{
							record_input(source_id, *command);
						}));
			}

			const auto output_qos = qos_factory.make(
				bundle_->gateway_contract->output.qos_profile);
			output_subscription_ = create_subscription<ControlCommand>(
				bundle_->gateway_contract->output.topic,
				output_qos,
				[this](ControlCommand::ConstSharedPtr command)
				{
					record_output(*command);
				});

			const auto state_contract =
				bundle_->gateway_contract->state_topics.find("gateway_state");
			if (state_contract == bundle_->gateway_contract->state_topics.end() ||
				!state_contract->second.qos_profile.has_value())
			{
				throw std::logic_error(
					"gateway_state Topic requires a Contract QoS profile");
			}
			const auto state_qos = qos_factory.make(
				state_contract->second.qos_profile.value());
			gateway_state_subscription_ = create_subscription<GatewayState>(
				state_contract->second.topic,
				state_qos,
				[this](GatewayState::ConstSharedPtr state)
				{
					record_gateway_state(*state);
				});

			signal_map_ = control_link_contract::load_can_signal_map(
				adas_profile_->adapter.config_path);
			can_codec_ = std::make_unique<CanCodec>(*signal_map_);
			can_fd_ = open_can_observer(
				can_interface_,
				can_codec_->config().control_can_id,
				can_codec_->config().state_can_id);
			try
			{
				event_fd_ = open_event_descriptor();
			}
			catch (...)
			{
				close_descriptor(can_fd_);
				can_fd_ = -1;
				throw;
			}

			const auto now = Clock::now();
			measurement_start_ = now + warmup_;
			measurement_deadline_ = measurement_start_ + measurement_duration_;
			absolute_deadline_ = measurement_deadline_ + max_extra_wait_;
			can_thread_ = std::thread{&RuntimeMeasurement::run_can_observer, this};

			resource_timer_ = create_wall_timer(
				std::chrono::seconds{1},
				[this]()
				{
					sample_resources();
				});
			completion_timer_ = create_wall_timer(
				std::chrono::milliseconds{100},
				[this]()
				{
					check_completion();
				});

			RCLCPP_INFO(
				get_logger(),
				"ADAS runtime measurement ready: warmup=%lds, duration=%lds, min_ticks=%lu, output=%s",
				static_cast<long>(warmup_.count()),
				static_cast<long>(measurement_duration_.count()),
				static_cast<unsigned long>(minimum_output_ticks_),
				output_dir.string().c_str());
		}

		~RuntimeMeasurement() override
		{
			stop_can_observer();
		}

		[[nodiscard]] int exit_code() const noexcept
		{
			return exit_code_.load(std::memory_order_acquire);
		}

	private:
		using InputKey = std::pair<std::string, std::uint64_t>;

		struct PendingCanFrame final
		{
			Clock::time_point observed_at;
		};

		void configure_target_processes()
		{
			const auto pids = get_parameter("target_pids").as_integer_array();
			const auto labels = get_parameter("target_labels").as_string_array();
			if (pids.empty() || pids.size() != labels.size())
			{
				throw std::invalid_argument(
					"target_pids and target_labels must be non-empty and have equal size");
			}

			std::map<std::string, bool> seen_labels;
			for (std::size_t index = 0U; index < pids.size(); ++index)
			{
				if (pids[index] <= 0 || labels[index].empty() ||
					seen_labels.count(labels[index]) != 0U)
				{
					throw std::invalid_argument(
						"target process labels must be unique and PIDs must be positive");
				}
				seen_labels.emplace(labels[index], true);
				target_processes_.push_back(TargetProcess{
					labels[index],
					pids[index],
					std::nullopt,
					std::nullopt,
					0U,
					{},
					{}});
			}
		}

		bool measurement_started(Clock::time_point now) const noexcept
		{
			return now >= measurement_start_ && !finished_.load(std::memory_order_acquire);
		}

		void record_input(
			const std::string &expected_source_id,
			const ControlCommand &command)
		{
			const auto now = Clock::now();
			if (!measurement_started(now) || command.source_id != expected_source_id)
			{
				return;
			}

			const InputKey key{expected_source_id, command.source_sequence};
			pending_inputs_.try_emplace(key, now);
			update_source_switch_candidate(expected_source_id, now);
			while (pending_inputs_.size() > kMaximumPendingInputs)
			{
				pending_inputs_.erase(pending_inputs_.begin());
				++expired_input_samples_;
			}
		}

		void record_output(const ControlCommand &command)
		{
			const auto now = Clock::now();
			if (!measurement_started(now))
			{
				return;
			}

			++output_tick_count_;
			if (last_output_arrival_.has_value())
			{
				const auto interval = now - last_output_arrival_.value();
				const auto interval_ns = std::chrono::duration_cast<
					std::chrono::nanoseconds>(interval).count();
				const double interval_ms =
					static_cast<double>(interval_ns) / 1'000'000.0;
				const double signed_jitter_ms = interval_ms - nominal_output_period_ms_;
				output_interval_ms_.push_back(interval_ms);
				output_signed_jitter_ms_.push_back(signed_jitter_ms);
				output_abs_jitter_ms_.push_back(std::abs(signed_jitter_ms));
				output_ticks_csv_ << steady_nanoseconds(now) << ',' << interval_ns << ','
					<< interval_ms << ',' << signed_jitter_ms << ','
					<< std::abs(signed_jitter_ms) << ','
					<< csv_string(command.source_id) << ','
					<< command.source_sequence << ','
					<< static_cast<unsigned int>(command.mode) << '\n';
			}
			last_output_arrival_ = now;

			const InputKey key{command.source_id, command.source_sequence};
			if (last_canonical_key_.has_value() && last_canonical_key_.value() == key)
			{
				return;
			}
			last_canonical_key_ = key;
			const auto input = pending_inputs_.find(key);
			if (input == pending_inputs_.end())
			{
				++unmatched_output_samples_;
				return;
			}
			if (now < input->second)
			{
				++negative_callback_latency_samples_;
				pending_inputs_.erase(input);
				return;
			}

			const auto latency_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
				now - input->second).count();
			const double latency_ms = static_cast<double>(latency_ns) / 1'000'000.0;
			callback_to_output_ms_.push_back(latency_ms);
			callback_to_output_csv_ << steady_nanoseconds(input->second) << ','
				<< steady_nanoseconds(now) << ',' << latency_ns << ',' << latency_ms << ','
				<< csv_string(command.source_id) << ',' << command.source_sequence << '\n';
			pending_inputs_.erase(input);
		}

		void record_gateway_state(const GatewayState &state)
		{
			const auto now = Clock::now();
			if (!measurement_started(now) || state.active_source_id.empty())
			{
				return;
			}

			const bool source_changed = last_gateway_source_id_.has_value() &&
				last_gateway_source_id_.value() != state.active_source_id;
			if (source_changed &&
				state.reason_code == GatewayState::REASON_SOURCE_SWITCH &&
				(!last_switch_transition_sequence_.has_value() ||
					last_switch_transition_sequence_.value() != state.transition_sequence))
			{
				if (pending_source_switch_.has_value() &&
					pending_source_switch_->to_source_id == state.active_source_id)
				{
					const auto &pending = pending_source_switch_.value();
					if (now >= pending.first_target_input_at)
					{
						const auto latency_ns = std::chrono::duration_cast<
							std::chrono::nanoseconds>(now - pending.first_target_input_at).count();
						const double latency_ms =
							static_cast<double>(latency_ns) / 1'000'000.0;
						source_switch_latency_ms_.push_back(latency_ms);
						source_switch_csv_ << steady_nanoseconds(
							pending.first_target_input_at) << ','
							<< steady_nanoseconds(now) << ',' << latency_ns << ',' << latency_ms << ','
							<< csv_string(pending.from_source_id) << ','
							<< csv_string(pending.to_source_id) << ','
							<< state.transition_sequence << ',' << state.reason_code << '\n';
					}
				}
				last_switch_transition_sequence_ = state.transition_sequence;
				pending_source_switch_.reset();
			}
			else if (source_changed)
			{
				// fallback/first-selection/recovery 不是 switch 延迟，清掉旧候选
				pending_source_switch_.reset();
			}
			last_gateway_source_id_ = state.active_source_id;
		}

		void update_source_switch_candidate(
			const std::string &source_id,
			Clock::time_point now)
		{
			if (!last_gateway_source_id_.has_value() ||
				source_id == last_gateway_source_id_.value())
			{
				return;
			}
			if (!pending_source_switch_.has_value() ||
				pending_source_switch_->to_source_id != source_id)
			{
				pending_source_switch_ = PendingSourceSwitch{
					last_gateway_source_id_.value(),
					source_id,
					now};
			}
		}

		void run_can_observer() noexcept
		{
			while (!can_stop_requested_.load(std::memory_order_acquire))
			{
				pollfd descriptors[2]{};
				descriptors[0].fd = can_fd_;
				descriptors[0].events = POLLIN;
				descriptors[1].fd = event_fd_;
				descriptors[1].events = POLLIN;
				const int result = ::poll(descriptors, 2, -1);
				if (result == -1)
				{
					if (errno == EINTR)
					{
						continue;
					}
					set_can_failure("CAN observer poll failed: errno=" + std::to_string(errno));
					return;
				}
				if ((descriptors[1].revents & POLLIN) != 0)
				{
					std::uint64_t value = 0U;
					const auto wake_result = ::read(event_fd_, &value, sizeof(value));
					if (wake_result != static_cast<ssize_t>(sizeof(value)) &&
						!(wake_result == -1 &&
							(errno == EAGAIN || errno == EWOULDBLOCK)))
					{
						set_can_failure("CAN observer eventfd wake read failed");
					}
					return;
				}
				if ((descriptors[0].revents & (POLLERR | POLLHUP | POLLNVAL)) != 0)
				{
					set_can_failure("CAN observer pollfd reported an invalid state");
					return;
				}
				if ((descriptors[0].revents & POLLIN) != 0)
				{
					drain_can_frames();
				}
			}
		}

		void drain_can_frames() noexcept
		{
			while (true)
			{
				can_frame linux_frame{};
				const auto result = ::read(can_fd_, &linux_frame, sizeof(linux_frame));
				if (result == static_cast<ssize_t>(sizeof(linux_frame)))
				{
					record_can_frame(from_linux_frame(linux_frame), Clock::now());
					continue;
				}
				if (result == -1 && errno == EINTR)
				{
					continue;
				}
				if (result == -1 && (errno == EAGAIN || errno == EWOULDBLOCK))
				{
					return;
				}
				set_can_failure(
					result == -1 ?
						"CAN observer read failed: errno=" + std::to_string(errno) :
						"CAN observer received a short frame");
				return;
			}
		}

		void record_can_frame(const CanFrame &frame, Clock::time_point now) noexcept
		{
			if (!measurement_started(now))
			{
				return;
			}

			std::lock_guard<std::mutex> lock{can_mutex_};
			if (frame.can_id == can_codec_->config().control_can_id)
			{
				const auto decoded = can_codec_->decode_control(frame);
				if (!decoded.accepted())
				{
					++invalid_control_frames_;
					return;
				}
				const auto counter = decoded.signals->rolling_counter;
				if (counter >= pending_can_frames_.size())
				{
					++invalid_control_frames_;
					return;
				}
				if (pending_can_frames_[counter].has_value())
				{
					// 4-bit counter 回绕前仍未配对，新旧 echo 已无法唯一归属
					pending_can_frames_[counter].reset();
					quarantined_can_counters_[counter] = true;
					++counter_reuse_collisions_;
					return;
				}
				pending_can_frames_[counter] = PendingCanFrame{now};
				return;
			}

			if (frame.can_id != can_codec_->config().state_can_id)
			{
				return;
			}
			const auto decoded = can_codec_->decode_state(frame);
			if (!decoded.accepted())
			{
				++invalid_state_frames_;
				return;
			}
			const auto counter = decoded.signals->echoed_control_counter;
			if (counter < quarantined_can_counters_.size() &&
				quarantined_can_counters_[counter])
			{
				// 丢弃碰撞后的首个 echo，以解除新旧 counter generation 歧义
				quarantined_can_counters_[counter] = false;
				++ambiguous_can_samples_;
				return;
			}
			if (counter >= pending_can_frames_.size() ||
				!pending_can_frames_[counter].has_value())
			{
				++unmatched_state_frames_;
				return;
			}

			const auto sent_at = pending_can_frames_[counter]->observed_at;
			pending_can_frames_[counter].reset();
			if (now < sent_at)
			{
				++negative_can_latency_samples_;
				return;
			}
			const auto maximum_unambiguous_age =
				std::chrono::milliseconds{adas_profile_->adapter.tx_period_ms} *
				static_cast<int>(can_codec_->config().rolling_counter_modulo - 1U);
			if (now - sent_at > maximum_unambiguous_age)
			{
				++ambiguous_can_samples_;
				return;
			}

			const auto latency_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
				now - sent_at).count();
			const double latency_ms = static_cast<double>(latency_ns) / 1'000'000.0;
			can_round_trip_ms_.push_back(latency_ms);
			can_round_trip_csv_ << steady_nanoseconds(sent_at) << ','
				<< steady_nanoseconds(now) << ',' << latency_ns << ',' << latency_ms << ','
				<< static_cast<unsigned int>(counter) << '\n';
		}

		void set_can_failure(std::string reason) noexcept
		{
			try
			{
				std::lock_guard<std::mutex> lock{can_mutex_};
				if (can_failure_reason_.empty())
				{
					can_failure_reason_ = std::move(reason);
				}
			}
			catch (...)
			{
				can_failure_without_detail_.store(true, std::memory_order_release);
			}
		}

		void stop_can_observer() noexcept
		{
			can_stop_requested_.store(true, std::memory_order_release);
			if (event_fd_ >= 0)
			{
				const std::uint64_t increment = 1U;
				const auto wake_result = ::write(
					event_fd_,
					&increment,
					sizeof(increment));
				if (wake_result != static_cast<ssize_t>(sizeof(increment)) &&
					!(wake_result == -1 &&
						(errno == EAGAIN || errno == EWOULDBLOCK)))
				{
					can_failure_without_detail_.store(true, std::memory_order_release);
				}
			}
			if (can_thread_.joinable())
			{
				can_thread_.join();
			}
			close_descriptor(event_fd_);
			event_fd_ = -1;
			close_descriptor(can_fd_);
			can_fd_ = -1;
		}

		void sample_resources()
		{
			const auto now = Clock::now();
			if (!measurement_started(now))
			{
				return;
			}
			const long ticks_per_second = ::sysconf(_SC_CLK_TCK);
			if (ticks_per_second <= 0)
			{
				finish(false, "sysconf(_SC_CLK_TCK) failed");
				return;
			}

			for (auto &target : target_processes_)
			{
				const auto sample = read_process_sample(target.pid);
				if (!sample.has_value())
				{
					++target.missing_samples;
					resources_csv_ << steady_nanoseconds(now) << ','
						<< csv_string(target.label) << ',' << target.pid
						<< ",,,,false\n";
					continue;
				}
				if (target.previous.has_value() &&
					target.previous->start_ticks != sample->start_ticks)
				{
					finish(false, "target process generation changed: " + target.label);
					return;
				}

				double cpu_percent = std::numeric_limits<double>::quiet_NaN();
				if (target.previous.has_value() && target.previous_at.has_value() &&
					target.previous->start_ticks == sample->start_ticks &&
					sample->cpu_ticks >= target.previous->cpu_ticks)
				{
					const double elapsed_seconds = std::chrono::duration<double>(
						now - target.previous_at.value()).count();
					if (elapsed_seconds > 0.0)
					{
						cpu_percent =
							static_cast<double>(sample->cpu_ticks - target.previous->cpu_ticks) /
							static_cast<double>(ticks_per_second) / elapsed_seconds * 100.0;
						target.cpu_percent_samples.push_back(cpu_percent);
					}
				}
				target.rss_kib_samples.push_back(static_cast<double>(sample->rss_kib));
				resources_csv_ << steady_nanoseconds(now) << ','
					<< csv_string(target.label) << ',' << target.pid << ',';
				if (std::isfinite(cpu_percent))
				{
					resources_csv_ << cpu_percent;
				}
				resources_csv_ << ',' << sample->rss_kib << ',' << sample->threads
					<< ",true\n";
				target.previous = sample;
				target.previous_at = now;
				target.missing_samples = 0U;
			}
		}

		void check_completion()
		{
			if (finished_.load(std::memory_order_acquire))
			{
				return;
			}
			const auto now = Clock::now();
			if (now < measurement_start_)
			{
				return;
			}

			std::string can_failure;
			{
				std::lock_guard<std::mutex> lock{can_mutex_};
				can_failure = can_failure_reason_;
			}
			if (!can_failure.empty())
			{
				finish(false, can_failure);
				return;
			}
			if (can_failure_without_detail_.load(std::memory_order_acquire))
			{
				finish(false, "CAN observer failed without an allocatable detail");
				return;
			}
			for (const auto &target : target_processes_)
			{
				if (target.missing_samples >= 3U)
				{
					finish(false, "target process disappeared: " + target.label);
					return;
				}
			}

			if (now >= measurement_deadline_ && output_tick_count_ >= minimum_output_ticks_)
			{
				if (require_source_switch_ && source_switch_latency_ms_.empty())
				{
					finish(false, "required source switch was not observed");
					return;
				}
				if (callback_to_output_ms_.size() < minimum_callback_samples_)
				{
					finish(false, "insufficient callback-to-output sample coverage");
					return;
				}
				std::size_t can_round_trip_sample_count = 0U;
				{
					std::lock_guard<std::mutex> lock{can_mutex_};
					can_round_trip_sample_count = can_round_trip_ms_.size();
				}
				if (can_round_trip_sample_count < minimum_can_round_trip_samples_)
				{
					finish(false, "insufficient CAN round-trip sample coverage");
					return;
				}
				for (const auto &target : target_processes_)
				{
					if (target.cpu_percent_samples.size() < minimum_resource_samples_ ||
						target.rss_kib_samples.size() < minimum_resource_samples_)
					{
						finish(
							false,
							"insufficient resource sample coverage: " + target.label);
						return;
					}
				}
				finish(true, "measurement protocol completed");
				return;
			}
			if (now >= absolute_deadline_)
			{
				finish(
					false,
					"measurement deadline reached before minimum output tick count");
			}
		}

		void finish(bool completed, const std::string &reason)
		{
			bool expected = false;
			if (!finished_.compare_exchange_strong(
					expected,
					true,
					std::memory_order_acq_rel,
					std::memory_order_acquire))
			{
				return;
			}
			const auto finished_at = Clock::now();
			stop_can_observer();

			output_ticks_csv_.flush();
			callback_to_output_csv_.flush();
			can_round_trip_csv_.flush();
			source_switch_csv_.flush();
			resources_csv_.flush();
			const bool raw_files_ok = output_ticks_csv_.good() &&
				callback_to_output_csv_.good() && can_round_trip_csv_.good() &&
				source_switch_csv_.good() &&
				resources_csv_.good();

			Json summary;
			summary["schema_version"] = 1;
			summary["completed"] = completed && raw_files_ok;
			summary["reason"] = raw_files_ok ? reason : "raw performance file write failed";
			summary["scope"] = "VM_ONLY_ADAS_EXTERNAL_OBSERVER";
			summary["clock"] = "std::chrono::steady_clock";
			summary["percentile_method"] = "linear_interpolation_q_times_n_minus_one";
			summary["cpu_percent_basis"] = "one_logical_cpu_equals_100_percent";
			summary["measurement_start_steady_ns"] = steady_nanoseconds(measurement_start_);
			summary["measurement_end_steady_ns"] = steady_nanoseconds(finished_at);
			summary["protocol"] = {
				{"warmup_seconds", warmup_.count()},
				{"minimum_duration_seconds", measurement_duration_.count()},
				{"minimum_output_ticks", minimum_output_ticks_},
				{"observed_output_ticks", output_tick_count_},
				{"minimum_callback_samples", minimum_callback_samples_},
				{"observed_callback_samples", callback_to_output_ms_.size()},
				{"minimum_can_round_trip_samples", minimum_can_round_trip_samples_},
				{"observed_can_round_trip_samples", can_round_trip_ms_.size()},
				{"minimum_resource_samples_per_process", minimum_resource_samples_},
				{"require_source_switch", require_source_switch_},
				{"observed_source_switch_samples", source_switch_latency_ms_.size()},
				{"protocol_conformant_parameters",
					warmup_ >= std::chrono::seconds{30} &&
					measurement_duration_ >= std::chrono::seconds{300} &&
					minimum_output_ticks_ >= 10'000U}};
			summary["metrics"]["canonical_delivery_interval_ms"] =
				summarize_samples(output_interval_ms_);
			summary["metrics"]["canonical_delivery_signed_jitter_ms"] =
				summarize_samples(output_signed_jitter_ms_);
			summary["metrics"]["canonical_delivery_abs_jitter_ms"] =
				summarize_samples(output_abs_jitter_ms_);
			summary["metrics"]["observer_callback_to_output_ms"] =
				summarize_samples(callback_to_output_ms_);
			summary["metrics"]["observer_source_switch_ms"] =
				summarize_samples(source_switch_latency_ms_);
		{
			std::lock_guard<std::mutex> lock{can_mutex_};
			summary["metrics"]["can_observer_round_trip_ms"] =
				summarize_samples(can_round_trip_ms_);
			summary["excluded"] = {
				{"expired_input_samples", expired_input_samples_},
				{"unmatched_output_samples", unmatched_output_samples_},
				{"negative_callback_latency_samples", negative_callback_latency_samples_},
				{"invalid_control_frames", invalid_control_frames_},
				{"invalid_state_frames", invalid_state_frames_},
				{"unmatched_state_frames", unmatched_state_frames_},
				{"counter_reuse_collisions", counter_reuse_collisions_},
				{"ambiguous_can_samples", ambiguous_can_samples_},
				{"negative_can_latency_samples", negative_can_latency_samples_}};
		}

			for (const auto &target : target_processes_)
			{
				summary["resources"][target.label] = {
					{"pid", target.pid},
					{"cpu_percent", summarize_samples(target.cpu_percent_samples)},
					{"rss_kib", summarize_samples(target.rss_kib_samples)}};
			}

			const std::filesystem::path output_dir{
				get_parameter("output_dir").as_string()};
			const auto temporary_path = output_dir / "summary.json.tmp";
			const auto summary_path = output_dir / "summary.json";
			{
				std::ofstream summary_stream{temporary_path, std::ios::out | std::ios::trunc};
				if (!summary_stream.is_open())
				{
					completed = false;
				}
				else
				{
					summary_stream << std::setw(2) << summary << '\n';
					if (!summary_stream.good())
					{
						completed = false;
					}
				}
			}
			if (completed && raw_files_ok)
			{
				std::filesystem::rename(temporary_path, summary_path);
			}
			else
			{
				std::error_code ignored;
				std::filesystem::rename(temporary_path, summary_path, ignored);
			}

			exit_code_.store(completed && raw_files_ok ? 0 : 2, std::memory_order_release);
			if (completed && raw_files_ok)
			{
				RCLCPP_INFO(
					get_logger(),
					"Runtime measurement completed: ticks=%lu, summary=%s",
					static_cast<unsigned long>(output_tick_count_),
					summary_path.string().c_str());
			}
			else
			{
				RCLCPP_ERROR(
					get_logger(),
					"Runtime measurement incomplete: %s",
					summary["reason"].get<std::string>().c_str());
			}
			rclcpp::shutdown();
		}

		control_link_contract::ContractBundlePtr bundle_;
		const control_link_contract::AdasProfile *adas_profile_{nullptr};
		control_link_contract::CanSignalMapPtr signal_map_;
		std::unique_ptr<CanCodec> can_codec_;
		std::string can_interface_;

		std::vector<rclcpp::Subscription<ControlCommand>::SharedPtr>
			input_subscriptions_;
		rclcpp::Subscription<ControlCommand>::SharedPtr output_subscription_;
		rclcpp::Subscription<GatewayState>::SharedPtr gateway_state_subscription_;
		rclcpp::TimerBase::SharedPtr resource_timer_;
		rclcpp::TimerBase::SharedPtr completion_timer_;

		std::chrono::seconds warmup_{0};
		std::chrono::seconds measurement_duration_{1};
		std::chrono::seconds max_extra_wait_{1};
		Clock::time_point measurement_start_{};
		Clock::time_point measurement_deadline_{};
		Clock::time_point absolute_deadline_{};
		std::uint64_t minimum_output_ticks_{1U};
		std::uint64_t minimum_callback_samples_{1U};
		std::uint64_t minimum_can_round_trip_samples_{1U};
		std::uint64_t minimum_resource_samples_{1U};
		bool require_source_switch_{false};
		double nominal_output_period_ms_{0.0};
		std::atomic<bool> finished_{false};
		std::atomic<int> exit_code_{2};

		std::ofstream output_ticks_csv_;
		std::ofstream callback_to_output_csv_;
		std::ofstream can_round_trip_csv_;
		std::ofstream source_switch_csv_;
		std::ofstream resources_csv_;
		std::optional<Clock::time_point> last_output_arrival_;
		std::optional<InputKey> last_canonical_key_;
		std::map<InputKey, Clock::time_point> pending_inputs_;
		std::uint64_t output_tick_count_{0U};
		std::uint64_t expired_input_samples_{0U};
		std::uint64_t unmatched_output_samples_{0U};
		std::uint64_t negative_callback_latency_samples_{0U};
		std::vector<double> output_interval_ms_;
		std::vector<double> output_signed_jitter_ms_;
		std::vector<double> output_abs_jitter_ms_;
		std::vector<double> callback_to_output_ms_;
		std::vector<double> source_switch_latency_ms_;
		std::optional<std::string> last_gateway_source_id_;
		std::optional<std::uint64_t> last_switch_transition_sequence_;
		std::optional<PendingSourceSwitch> pending_source_switch_;

		std::vector<TargetProcess> target_processes_;

		int can_fd_{-1};
		int event_fd_{-1};
		std::thread can_thread_;
		std::atomic<bool> can_stop_requested_{false};
		std::mutex can_mutex_;
		std::array<std::optional<PendingCanFrame>, kCounterDomainSize>
			pending_can_frames_{};
		std::array<bool, kCounterDomainSize> quarantined_can_counters_{};
		std::vector<double> can_round_trip_ms_;
		std::string can_failure_reason_;
		std::atomic<bool> can_failure_without_detail_{false};
		std::uint64_t invalid_control_frames_{0U};
		std::uint64_t invalid_state_frames_{0U};
		std::uint64_t unmatched_state_frames_{0U};
		std::uint64_t counter_reuse_collisions_{0U};
		std::uint64_t ambiguous_can_samples_{0U};
		std::uint64_t negative_can_latency_samples_{0U};
	};
}  // namespace

int main(int argc, char **argv)
{
	rclcpp::init(argc, argv);
	try
	{
		auto node = std::make_shared<RuntimeMeasurement>();
		rclcpp::executors::SingleThreadedExecutor executor;
		executor.add_node(node);
		executor.spin();
		const int result = node->exit_code();
		if (rclcpp::ok())
		{
			rclcpp::shutdown();
		}
		return result;
	}
	catch (const std::exception &exception)
	{
		RCLCPP_FATAL(
			rclcpp::get_logger("measure_runtime"),
			"Runtime measurement failed: %s",
			exception.what());
		rclcpp::shutdown();
		return 1;
	}
}
