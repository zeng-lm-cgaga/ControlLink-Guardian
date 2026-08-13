#include "control_link_adapters/socketcan_transport.hpp"

#include <algorithm>
#include <cerrno>
#include <cstring>
#include <exception>
#include <limits>
#include <stdexcept>
#include <system_error>
#include <utility>

#include <fcntl.h>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <net/if.h>
#include <poll.h>
#include <sys/eventfd.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <unistd.h>

namespace control_link_adapters
{
	namespace
	{
		constexpr std::uint32_t kMaxStandardCanId = CAN_SFF_MASK;

		void close_descriptor(int descriptor) noexcept
		{
			if (descriptor >= 0)
			{
				// Linux 可能已释放返回 EINTR 的 fd，重试存在误关复用 fd 的风险
				(void)::close(descriptor);
			}
		}

		void validate_config(const SocketCanTransportConfig &config)
		{
			if (config.interface.empty() || config.interface.size() >= IFNAMSIZ)
			{
				throw std::invalid_argument(
					"SocketCAN interface must fit Linux IFNAMSIZ");
			}
			if (config.receive_can_id > kMaxStandardCanId)
			{
				throw std::invalid_argument(
					"SocketCAN receive filter requires a standard 11-bit CAN ID");
			}
			if (config.poll_timeout <= std::chrono::milliseconds::zero() ||
				config.transmit_period <= std::chrono::milliseconds::zero() ||
				config.poll_timeout > config.transmit_period)
			{
				throw std::invalid_argument(
					"SocketCAN poll timeout must be positive and not exceed the transmit period");
			}
		}

		int open_can_socket(const SocketCanTransportConfig &config)
		{
			const int descriptor = ::socket(
				PF_CAN,
				SOCK_RAW | SOCK_NONBLOCK | SOCK_CLOEXEC,
				CAN_RAW);
			if (descriptor == -1)
			{
				throw std::system_error(
					errno,
					std::generic_category(),
					"socket(PF_CAN) failed");
			}

			try
			{
				ifreq request{};
				std::memcpy(
					request.ifr_name,
					config.interface.data(),
					config.interface.size());
				request.ifr_name[config.interface.size()] = '\0';
				if (::ioctl(descriptor, SIOCGIFINDEX, &request) == -1)
				{
					throw std::system_error(
						errno,
						std::generic_category(),
						"ioctl(SIOCGIFINDEX) failed for " + config.interface);
				}

				can_filter filter{};
				filter.can_id = config.receive_can_id;
				// EFF/RTR flag 进入 mask 后，只接收同 ID 的标准数据帧
				filter.can_mask = CAN_SFF_MASK | CAN_EFF_FLAG | CAN_RTR_FLAG;
				if (::setsockopt(
						descriptor,
						SOL_CAN_RAW,
						CAN_RAW_FILTER,
						&filter,
						sizeof(filter)) == -1)
				{
					throw std::system_error(
						errno,
						std::generic_category(),
						"setsockopt(CAN_RAW_FILTER) failed");
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
						"bind(AF_CAN) failed for " + config.interface);
				}
			}
			catch (...)
			{
				close_descriptor(descriptor);
				throw;
			}

			return descriptor;
		}

		int open_eventfd()
		{
			const int descriptor = ::eventfd(
				0U,
				EFD_NONBLOCK | EFD_CLOEXEC);
			if (descriptor == -1)
			{
				throw std::system_error(
					errno,
					std::generic_category(),
					"eventfd() failed");
			}
			return descriptor;
		}

		can_frame to_linux_frame(const CanFrame &source) noexcept
		{
			can_frame destination{};
			destination.can_id = source.can_id;
			if (source.is_extended)
			{
				destination.can_id |= CAN_EFF_FLAG;
			}
			if (source.is_remote)
			{
				destination.can_id |= CAN_RTR_FLAG;
			}
			destination.can_dlc = source.dlc;
			std::copy(source.data.begin(), source.data.end(), destination.data);
			return destination;
		}

		CanFrame from_linux_frame(const can_frame &source) noexcept
		{
			const bool is_extended = (source.can_id & CAN_EFF_FLAG) != 0U;
			CanFrame destination{
				source.can_id & (is_extended ? CAN_EFF_MASK : CAN_SFF_MASK),
				source.can_dlc,
				is_extended,
				(source.can_id & CAN_RTR_FLAG) != 0U,
				{0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U}};
			std::copy(std::begin(source.data), std::end(source.data), destination.data.begin());
			return destination;
		}

		std::string exception_detail(std::exception_ptr exception) noexcept
		{
			try
			{
				if (exception)
				{
					std::rethrow_exception(exception);
				}
				return "unknown exception";
			}
			catch (const std::exception &exception)
			{
				try
				{
					return exception.what();
				}
				catch (...)
				{
					return {};
				}
			}
			catch (...)
			{
				try
				{
					return "non-standard exception";
				}
				catch (...)
				{
					return {};
				}
			}
		}

		int poll_timeout_until(
			std::chrono::steady_clock::time_point now,
			std::chrono::steady_clock::time_point next_transmit,
			std::chrono::milliseconds configured_timeout) noexcept
		{
			if (now >= next_transmit)
			{
				return 0;
			}

			const auto remaining = next_transmit - now;
			const auto remaining_ms = std::chrono::ceil<
				std::chrono::milliseconds>(remaining);
			const auto bounded = std::min(configured_timeout, remaining_ms);
			const auto count = bounded.count();
			return static_cast<int>(std::min<std::int64_t>(
				count,
				std::numeric_limits<int>::max()));
		}
	} // namespace

	SocketCanTransport::SocketCanTransport(SocketCanTransportConfig config)
		: config_(std::move(config))
	{
		validate_config(config_);
		can_fd_ = open_can_socket(config_);
		try
		{
			event_fd_ = open_eventfd();
		}
		catch (...)
		{
			close_descriptor(can_fd_);
			can_fd_ = -1;
			throw;
		}
	}

	SocketCanTransport::~SocketCanTransport()
	{
		stop();
	}

	void SocketCanTransport::start(
		ReceiveHandler receive_handler,
		ErrorHandler error_handler,
		TransmitHandler transmit_handler,
		TransmitSuccessHandler transmit_success_handler)
	{
		if (!receive_handler)
		{
			throw std::invalid_argument(
				"SocketCanTransport requires a receive handler");
		}

		std::lock_guard<std::mutex> lock{lifecycle_mutex_};
		if (started_ || stopped_)
		{
			throw std::logic_error(
				"SocketCanTransport can be started exactly once");
		}

		receive_handler_ = std::move(receive_handler);
		error_handler_ = std::move(error_handler);
		transmit_handler_ = std::move(transmit_handler);
		transmit_success_handler_ = std::move(transmit_success_handler);
		stop_requested_.store(false, std::memory_order_release);
		try
		{
			worker_ = std::thread{&SocketCanTransport::run, this};
			started_ = true;
		}
		catch (...)
		{
			receive_handler_ = {};
			error_handler_ = {};
			transmit_handler_ = {};
			transmit_success_handler_ = {};
			throw;
		}
	}

	void SocketCanTransport::notify() noexcept
	{
		std::lock_guard<std::mutex> lock{lifecycle_mutex_};
		if (!started_ || stopped_)
		{
			return;
		}
		wake_worker_locked();
	}

	void SocketCanTransport::request_immediate_transmit() noexcept
	{
		immediate_transmit_requested_.store(true, std::memory_order_release);
		notify();
	}

	void SocketCanTransport::stop() noexcept
	{
		std::lock_guard<std::mutex> stop_lock{stop_mutex_};
		{
			std::lock_guard<std::mutex> lock{lifecycle_mutex_};
			if (stopped_)
			{
				return;
			}
			stop_requested_.store(true, std::memory_order_release);
			wake_worker_locked();
		}

		if (worker_.joinable())
		{
			if (worker_.get_id() == std::this_thread::get_id())
			{
				// I/O callback 误调用 stop 时只请求退出，必须由 owner 线程随后 join
				return;
			}
			worker_.join();
		}

		std::lock_guard<std::mutex> lock{lifecycle_mutex_};
		close_descriptors_locked();
		stopped_ = true;
	}

	bool SocketCanTransport::running() const noexcept
	{
		return running_.load(std::memory_order_acquire);
	}

	SocketCanTransportStats SocketCanTransport::stats() const noexcept
	{
		return SocketCanTransportStats{
			transmitted_frames_.load(std::memory_order_relaxed),
			received_frames_.load(std::memory_order_relaxed),
			transmit_would_block_.load(std::memory_order_relaxed),
			poll_errors_.load(std::memory_order_relaxed),
			read_errors_.load(std::memory_order_relaxed),
			write_errors_.load(std::memory_order_relaxed),
			wake_errors_.load(std::memory_order_relaxed),
			receive_handler_errors_.load(std::memory_order_relaxed)};
	}

	void SocketCanTransport::run() noexcept
	{
		running_.store(true, std::memory_order_release);
		std::optional<CanFrame> retry_frame;
		auto next_transmit = std::chrono::steady_clock::now() +
			config_.transmit_period;

		while (!stop_requested_.load(std::memory_order_acquire))
		{
			pollfd descriptors[2]{};
			descriptors[0].fd = can_fd_;
			descriptors[0].events = POLLIN;
			descriptors[1].fd = event_fd_;
			descriptors[1].events = POLLIN;

			const auto before_poll = std::chrono::steady_clock::now();
			const int result = ::poll(
				descriptors,
				2,
				poll_timeout_until(
					before_poll,
					next_transmit,
					config_.poll_timeout));
			if (result == -1)
			{
				if (errno == EINTR)
				{
					continue;
				}
				poll_errors_.fetch_add(1U, std::memory_order_relaxed);
				report_error(SocketCanIoError{
					SocketCanIoErrorKind::kPoll,
					errno,
					true});
				break;
			}

			if ((descriptors[1].revents & POLLIN) != 0)
			{
				if (!drain_eventfd())
				{
					break;
				}
			}
			if (stop_requested_.load(std::memory_order_acquire))
			{
				break;
			}
			if ((descriptors[0].revents & POLLIN) != 0)
			{
				if (!receive_available_frames())
				{
					break;
				}
			}
			if ((descriptors[0].revents & (POLLERR | POLLHUP | POLLNVAL)) != 0)
			{
				poll_errors_.fetch_add(1U, std::memory_order_relaxed);
				report_error(SocketCanIoError{
					SocketCanIoErrorKind::kPoll,
					0,
					true,
					"CAN pollfd reported error, hangup, or invalid descriptor"});
				break;
			}
			if ((descriptors[1].revents & (POLLERR | POLLHUP | POLLNVAL)) != 0)
			{
				poll_errors_.fetch_add(1U, std::memory_order_relaxed);
				report_error(SocketCanIoError{
					SocketCanIoErrorKind::kEventRead,
					0,
					true,
					"eventfd pollfd reported error, hangup, or invalid descriptor"});
				break;
			}

			auto now = std::chrono::steady_clock::now();
			const bool immediate = immediate_transmit_requested_.exchange(
				false,
				std::memory_order_acq_rel);
			if (immediate && transmit_handler_)
			{
				std::optional<CanFrame> immediate_frame;
				try
				{
					immediate_frame = transmit_handler_(now);
				}
				catch (...)
				{
					report_error(SocketCanIoError{
						SocketCanIoErrorKind::kTransmitHandler,
						0,
						true,
						exception_detail(std::current_exception())});
					break;
				}
				if (immediate_frame.has_value())
				{
					// 安全请求覆盖尚未发送的普通 retry frame
					retry_frame = immediate_frame;
					const auto transmit_result = transmit_frame(retry_frame.value());
					if (transmit_result == TransmitResult::kFatal)
					{
						break;
					}
					if (transmit_result == TransmitResult::kSent)
					{
						retry_frame.reset();
					}
				}
				now = std::chrono::steady_clock::now();
				next_transmit = now + config_.transmit_period;
			}
			else if (now >= next_transmit)
			{
				std::optional<CanFrame> periodic_frame = retry_frame;
				if (!periodic_frame.has_value() && transmit_handler_)
				{
					try
					{
						periodic_frame = transmit_handler_(now);
					}
					catch (...)
					{
						report_error(SocketCanIoError{
							SocketCanIoErrorKind::kTransmitHandler,
							0,
							true,
							exception_detail(std::current_exception())});
						break;
					}
				}
				if (periodic_frame.has_value())
				{
					const auto transmit_result = transmit_frame(periodic_frame.value());
					if (transmit_result == TransmitResult::kFatal)
					{
						break;
					}
					if (transmit_result == TransmitResult::kRetry)
					{
						retry_frame = periodic_frame;
					}
					else
					{
						retry_frame.reset();
					}
				}
				now = std::chrono::steady_clock::now();
				next_transmit = now + config_.transmit_period;
			}
		}

		// 无论是正常 stop 还是致命 I/O 错误，worker 退出后都不再接受新帧
		stop_requested_.store(true, std::memory_order_release);
		running_.store(false, std::memory_order_release);
	}

	bool SocketCanTransport::drain_eventfd() noexcept
	{
		std::uint64_t value = 0U;
		while (true)
		{
			const auto result = ::read(event_fd_, &value, sizeof(value));
			if (result == static_cast<ssize_t>(sizeof(value)))
			{
				continue;
			}
			if (result == -1 && errno == EINTR)
			{
				continue;
			}
			if (result == -1 && (errno == EAGAIN || errno == EWOULDBLOCK))
			{
				return true;
			}

			read_errors_.fetch_add(1U, std::memory_order_relaxed);
			report_error(SocketCanIoError{
				SocketCanIoErrorKind::kEventRead,
				result == -1 ? errno : 0,
				true,
				result == -1 ? std::string{} : "short eventfd read"});
			return false;
		}
	}

	bool SocketCanTransport::receive_available_frames() noexcept
	{
		while (true)
		{
			can_frame frame{};
			const auto result = ::read(can_fd_, &frame, sizeof(frame));
			if (result == static_cast<ssize_t>(sizeof(frame)))
			{
				received_frames_.fetch_add(1U, std::memory_order_relaxed);
				try
				{
					receive_handler_(
						from_linux_frame(frame),
						std::chrono::steady_clock::now());
				}
				catch (...)
				{
					receive_handler_errors_.fetch_add(
						1U,
						std::memory_order_relaxed);
					report_error(SocketCanIoError{
						SocketCanIoErrorKind::kReceiveHandler,
						0,
						true,
						exception_detail(std::current_exception())});
					return false;
				}
				continue;
			}
			if (result == -1 && errno == EINTR)
			{
				continue;
			}
			if (result == -1 && (errno == EAGAIN || errno == EWOULDBLOCK))
			{
				return true;
			}

			read_errors_.fetch_add(1U, std::memory_order_relaxed);
			report_error(SocketCanIoError{
				result == -1 ? SocketCanIoErrorKind::kCanRead :
					SocketCanIoErrorKind::kShortCanRead,
				result == -1 ? errno : 0,
				true,
				result == -1 ? std::string{} : "short CAN frame read"});
			return false;
		}
	}

	SocketCanTransport::TransmitResult SocketCanTransport::transmit_frame(
		const CanFrame &frame) noexcept
	{
		const auto maximum_id = frame.is_extended ? CAN_EFF_MASK : CAN_SFF_MASK;
		if (frame.can_id > maximum_id || frame.dlc > CAN_MAX_DLEN)
		{
			report_error(SocketCanIoError{
				SocketCanIoErrorKind::kTransmitHandler,
				0,
				true,
				"transmit handler produced an invalid CAN ID or DLC"});
			return TransmitResult::kFatal;
		}
		const auto linux_frame = to_linux_frame(frame);
		while (true)
		{
			const auto result = ::write(can_fd_, &linux_frame, sizeof(linux_frame));
			if (result == static_cast<ssize_t>(sizeof(linux_frame)))
			{
				transmitted_frames_.fetch_add(1U, std::memory_order_relaxed);
				if (transmit_success_handler_)
				{
					try
					{
						transmit_success_handler_(frame);
					}
					catch (...)
					{
						report_error(SocketCanIoError{
							SocketCanIoErrorKind::kTransmitSuccessHandler,
							0,
							true,
							exception_detail(std::current_exception())});
						return TransmitResult::kFatal;
					}
				}
				return TransmitResult::kSent;
			}
			if (result == -1 && errno == EINTR)
			{
				continue;
			}
			if (result == -1 &&
				(errno == EAGAIN || errno == EWOULDBLOCK || errno == ENOBUFS))
			{
				transmit_would_block_.fetch_add(1U, std::memory_order_relaxed);
				report_error(SocketCanIoError{
					SocketCanIoErrorKind::kCanWrite,
					errno,
					false});
				return TransmitResult::kRetry;
			}

			write_errors_.fetch_add(1U, std::memory_order_relaxed);
			report_error(SocketCanIoError{
				result == -1 ? SocketCanIoErrorKind::kCanWrite :
					SocketCanIoErrorKind::kShortCanWrite,
				result == -1 ? errno : 0,
				true,
				result == -1 ? std::string{} : "short CAN frame write"});
			return TransmitResult::kFatal;
		}
	}

	void SocketCanTransport::report_error(SocketCanIoError error) noexcept
	{
		if (!error_handler_)
		{
			return;
		}
		try
		{
			error_handler_(error);
		}
		catch (...)
		{
			// 错误上报不能让 I/O thread 通过异常越过受控清理路径
		}
	}

	void SocketCanTransport::wake_worker_locked() noexcept
	{
		if (event_fd_ < 0)
		{
			return;
		}

		const std::uint64_t increment = 1U;
		while (true)
		{
			const auto result = ::write(
				event_fd_,
				&increment,
				sizeof(increment));
			if (result == static_cast<ssize_t>(sizeof(increment)))
			{
				return;
			}
			if (result == -1 && errno == EINTR)
			{
				continue;
			}
			if (result == -1 && (errno == EAGAIN || errno == EWOULDBLOCK))
			{
				// eventfd 已可读，已有 wake 足以使 worker 观察最新 mailbox 或 stop flag
				return;
			}

			wake_errors_.fetch_add(1U, std::memory_order_relaxed);
			return;
		}
	}

	void SocketCanTransport::close_descriptors_locked() noexcept
	{
		close_descriptor(event_fd_);
		event_fd_ = -1;
		close_descriptor(can_fd_);
		can_fd_ = -1;
	}
} // namespace control_link_adapters
