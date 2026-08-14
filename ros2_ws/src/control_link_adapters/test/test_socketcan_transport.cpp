#include <algorithm>
#include <array>
#include <atomic>
#include <cerrno>
#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <filesystem>
#include <functional>
#include <mutex>
#include <optional>
#include <stdexcept>
#include <string>
#include <system_error>
#include <thread>
#include <vector>

#include <fcntl.h>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <net/if.h>
#include <poll.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <unistd.h>

#include <gtest/gtest.h>

#include "control_link_adapters/socketcan_transport.hpp"

namespace control_link_adapters
{
	namespace
	{
		using namespace std::chrono_literals;

		constexpr std::uint32_t kControlCanId = 0x180U;
		constexpr std::uint32_t kStateCanId = 0x280U;

		std::string vcan_interface()
		{
			const char *configured = std::getenv("CONTROL_LINK_TEST_VCAN_INTERFACE");
			return configured == nullptr ? "vcan0" : std::string{configured};
		}

		SocketCanTransportConfig transport_config(
			std::chrono::milliseconds transmit_period = 20ms)
		{
			return SocketCanTransportConfig{
				vcan_interface(),
				kStateCanId,
				std::min(10ms, transmit_period),
				transmit_period};
		}

		CanFrame test_frame(std::uint32_t can_id, std::uint8_t marker = 0U)
		{
			return CanFrame{
				can_id,
				8U,
				false,
				false,
				{marker, 1U, 2U, 3U, 4U, 5U, 6U, 7U}};
		}

		template<typename Predicate>
		bool wait_until(Predicate predicate, std::chrono::milliseconds timeout)
		{
			const auto deadline = std::chrono::steady_clock::now() + timeout;
			while (std::chrono::steady_clock::now() < deadline)
			{
				if (predicate())
				{
					return true;
				}
				std::this_thread::sleep_for(1ms);
			}
			return predicate();
		}

		std::size_t open_descriptor_count()
		{
			std::size_t count = 0U;
			for (const auto &entry : std::filesystem::directory_iterator{"/proc/self/fd"})
			{
				(void)entry;
				++count;
			}
			return count;
		}

		class RawCanPeer final
		{
		public:
			explicit RawCanPeer(const std::string &interface)
			{
				descriptor_ = ::socket(
					PF_CAN,
					SOCK_RAW | SOCK_NONBLOCK | SOCK_CLOEXEC,
					CAN_RAW);
				if (descriptor_ == -1)
				{
					throw std::system_error(
						errno,
						std::generic_category(),
						"test socket(PF_CAN) failed");
				}

				try
				{
					ifreq request{};
					if (interface.empty() || interface.size() >= IFNAMSIZ)
					{
						throw std::invalid_argument("invalid test CAN interface");
					}
					std::memcpy(
						request.ifr_name,
						interface.data(),
						interface.size());
					request.ifr_name[interface.size()] = '\0';
					if (::ioctl(descriptor_, SIOCGIFINDEX, &request) == -1)
					{
						throw std::system_error(
							errno,
							std::generic_category(),
							"test ioctl(SIOCGIFINDEX) failed");
					}

					sockaddr_can address{};
					address.can_family = AF_CAN;
					address.can_ifindex = request.ifr_ifindex;
					if (::bind(
							descriptor_,
							reinterpret_cast<const sockaddr *>(&address),
							sizeof(address)) == -1)
					{
						throw std::system_error(
							errno,
							std::generic_category(),
							"test bind(AF_CAN) failed");
					}
				}
				catch (...)
				{
					(void)::close(descriptor_);
					descriptor_ = -1;
					throw;
				}
			}

			~RawCanPeer()
			{
				if (descriptor_ >= 0)
				{
					(void)::close(descriptor_);
				}
			}

			RawCanPeer(const RawCanPeer &) = delete;
			RawCanPeer &operator=(const RawCanPeer &) = delete;

			void send(
				std::uint32_t can_id,
				std::uint8_t marker = 0U,
				std::uint8_t dlc = 8U)
			{
				can_frame frame{};
				frame.can_id = can_id;
				frame.can_dlc = dlc;
				frame.data[0U] = marker;

				const auto result = ::write(descriptor_, &frame, sizeof(frame));
				if (result != static_cast<ssize_t>(sizeof(frame)))
				{
					throw std::system_error(
						result == -1 ? errno : EIO,
						std::generic_category(),
						"test CAN frame write failed");
				}
			}

			std::optional<can_frame> receive(std::chrono::milliseconds timeout)
			{
				pollfd descriptor{};
				descriptor.fd = descriptor_;
				descriptor.events = POLLIN;
				const auto result = ::poll(
					&descriptor,
					1,
					static_cast<int>(timeout.count()));
				if (result == 0)
				{
					return std::nullopt;
				}
				if (result == -1)
				{
					throw std::system_error(
						errno,
						std::generic_category(),
						"test CAN poll failed");
				}
				if ((descriptor.revents & POLLIN) == 0)
				{
					throw std::runtime_error("test CAN poll returned without POLLIN");
				}

				can_frame frame{};
				const auto read_result = ::read(descriptor_, &frame, sizeof(frame));
				if (read_result != static_cast<ssize_t>(sizeof(frame)))
				{
					throw std::system_error(
						read_result == -1 ? errno : EIO,
						std::generic_category(),
						"test CAN frame read failed");
				}
				return frame;
			}

		private:
			int descriptor_{-1};
		};

		class ErrorSink final
		{
		public:
			void observe(const SocketCanIoError &error)
			{
				{
					std::lock_guard<std::mutex> lock{mutex_};
					errors_.push_back(error);
				}
				condition_.notify_all();
			}

			std::optional<SocketCanIoError> wait_for_error(
				std::chrono::milliseconds timeout)
			{
				std::unique_lock<std::mutex> lock{mutex_};
				condition_.wait_for(lock, timeout, [this]()
				{
					return !errors_.empty();
				});
				return errors_.empty() ? std::nullopt :
					std::optional<SocketCanIoError>{errors_.back()};
			}

		private:
			std::mutex mutex_;
			std::condition_variable condition_;
			std::vector<SocketCanIoError> errors_;
		};

		TEST(SocketCanTransportTest, RejectsInvalidConfigAndMissingInterface)
		{
			auto config = transport_config();
			config.interface.clear();
			EXPECT_THROW(SocketCanTransport{config}, std::invalid_argument);

			config = transport_config();
			config.interface = std::string(IFNAMSIZ, 'x');
			EXPECT_THROW(SocketCanTransport{config}, std::invalid_argument);

			config = transport_config();
			config.receive_can_id = CAN_SFF_MASK + 1U;
			EXPECT_THROW(SocketCanTransport{config}, std::invalid_argument);

			config = transport_config();
			config.poll_timeout = 0ms;
			EXPECT_THROW(SocketCanTransport{config}, std::invalid_argument);

			config = transport_config();
			config.poll_timeout = config.transmit_period + 1ms;
			EXPECT_THROW(SocketCanTransport{config}, std::invalid_argument);

			config = transport_config();
			config.interface = "missing_vcan0";
			EXPECT_THROW(SocketCanTransport{config}, std::system_error);
		}

		TEST(SocketCanTransportTest, OwnsSingleUseLifecycleAndIdempotentStop)
		{
			SocketCanTransport stopped_before_start{transport_config()};
			stopped_before_start.stop();
			stopped_before_start.stop();
			EXPECT_FALSE(stopped_before_start.running());
			EXPECT_THROW(
				stopped_before_start.start([](const CanFrame &, auto) {}, {}),
				std::logic_error);

			SocketCanTransport transport{transport_config()};
			transport.start([](const CanFrame &, auto) {}, {});
			ASSERT_TRUE(wait_until([&transport]()
			{
				return transport.running();
			}, 250ms));
			EXPECT_THROW(
				transport.start([](const CanFrame &, auto) {}, {}),
				std::logic_error);
			transport.stop();
			transport.stop();
			EXPECT_FALSE(transport.running());
		}

		TEST(SocketCanTransportTest, TransmitsPeriodicallyAndFiltersReceivedFrames)
		{
			RawCanPeer peer{vcan_interface()};
			SocketCanTransport transport{transport_config()};
			ErrorSink errors;
			std::mutex received_mutex;
			std::vector<CanFrame> received;
			std::atomic<std::uint64_t> success_count{0U};

			transport.start(
				[&received_mutex, &received](const CanFrame &frame, auto)
				{
					std::lock_guard<std::mutex> lock{received_mutex};
					received.push_back(frame);
				},
				[&errors](const SocketCanIoError &error)
				{
					errors.observe(error);
				},
				[]([[maybe_unused]] auto now)
				{
					return std::optional<CanFrame>{test_frame(kControlCanId, 0x2AU)};
				},
				[&success_count](const CanFrame &)
				{
					success_count.fetch_add(1U, std::memory_order_relaxed);
				});

			const auto transmitted = peer.receive(500ms);
			ASSERT_TRUE(transmitted.has_value());
			EXPECT_EQ(transmitted->can_id, kControlCanId);
			EXPECT_EQ(transmitted->can_dlc, 8U);
			EXPECT_EQ(transmitted->data[0U], 0x2AU);

			peer.send(kStateCanId + 1U, 1U);
			peer.send(CAN_EFF_FLAG | kStateCanId, 2U);
			peer.send(CAN_RTR_FLAG | kStateCanId, 3U, 0U);
			peer.send(kStateCanId, 4U);
			ASSERT_TRUE(wait_until([&received_mutex, &received]()
			{
				std::lock_guard<std::mutex> lock{received_mutex};
				return !received.empty();
			}, 500ms));
			std::this_thread::sleep_for(50ms);
			transport.stop();

			{
				std::lock_guard<std::mutex> lock{received_mutex};
				ASSERT_EQ(received.size(), 1U);
				EXPECT_EQ(received.front().can_id, kStateCanId);
				EXPECT_FALSE(received.front().is_extended);
				EXPECT_FALSE(received.front().is_remote);
				EXPECT_EQ(received.front().data[0U], 4U);
			}
			const auto stats = transport.stats();
			EXPECT_GE(stats.transmitted_frames, 1U);
			EXPECT_EQ(stats.received_frames, 1U);
			EXPECT_EQ(stats.poll_errors, 0U);
			EXPECT_EQ(stats.read_errors, 0U);
			EXPECT_EQ(stats.write_errors, 0U);
			EXPECT_GE(success_count.load(std::memory_order_relaxed), 1U);
			EXPECT_FALSE(errors.wait_for_error(10ms).has_value());
		}

		TEST(SocketCanTransportTest, NotifyDoesNotConsumeTransmitAndImmediateRequestWakesPoll)
		{
			RawCanPeer peer{vcan_interface()};
			SocketCanTransport transport{transport_config(2s)};
			std::atomic<std::uint64_t> generated_frames{0U};
			std::atomic<std::uint64_t> committed_frames{0U};
			transport.start(
				[](const CanFrame &, auto) {},
				{},
				[&generated_frames]([[maybe_unused]] auto now)
				{
					const auto generation = generated_frames.fetch_add(
						1U,
						std::memory_order_relaxed);
					return std::optional<CanFrame>{test_frame(
						kControlCanId,
						static_cast<std::uint8_t>(generation))};
				},
				[&committed_frames](const CanFrame &)
				{
					committed_frames.fetch_add(1U, std::memory_order_relaxed);
				});
			ASSERT_TRUE(wait_until([&transport]()
			{
				return transport.running();
			}, 250ms));

			transport.notify();
			transport.notify();
			std::this_thread::sleep_for(100ms);
			EXPECT_EQ(generated_frames.load(std::memory_order_relaxed), 0U);
			EXPECT_EQ(committed_frames.load(std::memory_order_relaxed), 0U);

			const auto requested_at = std::chrono::steady_clock::now();
			transport.request_immediate_transmit();
			const auto frame = peer.receive(500ms);
			const auto received_at = std::chrono::steady_clock::now();
			transport.stop();

			ASSERT_TRUE(frame.has_value());
			EXPECT_EQ(frame->can_id, kControlCanId);
			EXPECT_EQ(generated_frames.load(std::memory_order_relaxed), 1U);
			EXPECT_EQ(committed_frames.load(std::memory_order_relaxed), 1U);
			EXPECT_LT(received_at - requested_at, 500ms);
			EXPECT_EQ(transport.stats().transmitted_frames, 1U);
		}

		TEST(SocketCanTransportTest, ClassifiesTransmitHandlerExceptionWithoutErrno)
		{
			SocketCanTransport transport{transport_config(2s)};
			ErrorSink errors;
			transport.start(
				[](const CanFrame &, auto) {},
				[&errors](const SocketCanIoError &error)
				{
					errors.observe(error);
				},
				[]([[maybe_unused]] auto now) -> std::optional<CanFrame>
				{
					throw std::runtime_error("generated transmit failure");
				});
			transport.request_immediate_transmit();
			const auto error = errors.wait_for_error(500ms);
			ASSERT_TRUE(error.has_value());
			EXPECT_EQ(error->kind, SocketCanIoErrorKind::kTransmitHandler);
			EXPECT_EQ(error->error_number, 0);
			EXPECT_TRUE(error->fatal);
			EXPECT_EQ(error->detail, "generated transmit failure");
			ASSERT_TRUE(wait_until([&transport]()
			{
				return !transport.running();
			}, 250ms));
			transport.stop();
			EXPECT_EQ(transport.stats().write_errors, 0U);
		}

		TEST(SocketCanTransportTest, ClassifiesInvalidGeneratedFrameAsHandlerFailure)
		{
			SocketCanTransport transport{transport_config(2s)};
			ErrorSink errors;
			transport.start(
				[](const CanFrame &, auto) {},
				[&errors](const SocketCanIoError &error)
				{
					errors.observe(error);
				},
				[]([[maybe_unused]] auto now)
				{
					return std::optional<CanFrame>{test_frame(CAN_SFF_MASK + 1U)};
				});
			transport.request_immediate_transmit();
			const auto error = errors.wait_for_error(500ms);
			ASSERT_TRUE(error.has_value());
			EXPECT_EQ(error->kind, SocketCanIoErrorKind::kTransmitHandler);
			EXPECT_EQ(error->error_number, 0);
			EXPECT_TRUE(error->fatal);
			EXPECT_NE(error->detail.find("invalid CAN ID or DLC"), std::string::npos);
			transport.stop();
			EXPECT_EQ(transport.stats().transmitted_frames, 0U);
			EXPECT_EQ(transport.stats().write_errors, 0U);
		}

		TEST(SocketCanTransportTest, ClassifiesTransmitSuccessHandlerException)
		{
			RawCanPeer peer{vcan_interface()};
			SocketCanTransport transport{transport_config(2s)};
			ErrorSink errors;
			transport.start(
				[](const CanFrame &, auto) {},
				[&errors](const SocketCanIoError &error)
				{
					errors.observe(error);
				},
				[]([[maybe_unused]] auto now)
				{
					return std::optional<CanFrame>{test_frame(kControlCanId)};
				},
				[](const CanFrame &)
				{
					throw std::runtime_error("commit failure");
				});
			transport.request_immediate_transmit();
			ASSERT_TRUE(peer.receive(500ms).has_value());
			const auto error = errors.wait_for_error(500ms);
			ASSERT_TRUE(error.has_value());
			EXPECT_EQ(error->kind, SocketCanIoErrorKind::kTransmitSuccessHandler);
			EXPECT_EQ(error->error_number, 0);
			EXPECT_TRUE(error->fatal);
			EXPECT_EQ(error->detail, "commit failure");
			transport.stop();
			EXPECT_EQ(transport.stats().transmitted_frames, 1U);
			EXPECT_EQ(transport.stats().write_errors, 0U);
		}

		TEST(SocketCanTransportTest, ClassifiesReceiveHandlerExceptionWithoutErrno)
		{
			RawCanPeer peer{vcan_interface()};
			SocketCanTransport transport{transport_config()};
			ErrorSink errors;
			transport.start(
				[](const CanFrame &, auto)
				{
					throw std::runtime_error("receive failure");
				},
				[&errors](const SocketCanIoError &error)
				{
					errors.observe(error);
				});
			peer.send(kStateCanId);
			const auto error = errors.wait_for_error(500ms);
			ASSERT_TRUE(error.has_value());
			EXPECT_EQ(error->kind, SocketCanIoErrorKind::kReceiveHandler);
			EXPECT_EQ(error->error_number, 0);
			EXPECT_TRUE(error->fatal);
			EXPECT_EQ(error->detail, "receive failure");
			transport.stop();
			const auto stats = transport.stats();
			EXPECT_EQ(stats.received_frames, 1U);
			EXPECT_EQ(stats.receive_handler_errors, 1U);
			EXPECT_EQ(stats.read_errors, 0U);
		}

		TEST(SocketCanTransportTest, RepeatedStopWakesPollWithoutFdLeak)
		{
			const auto descriptors_before = open_descriptor_count();
			for (std::size_t iteration = 0U; iteration < 20U; ++iteration)
			{
				SocketCanTransport transport{transport_config(2s)};
				transport.start([](const CanFrame &, auto) {}, {});
				ASSERT_TRUE(wait_until([&transport]()
				{
					return transport.running();
				}, 250ms));
				const auto stop_started_at = std::chrono::steady_clock::now();
				transport.stop();
				EXPECT_LT(std::chrono::steady_clock::now() - stop_started_at, 250ms);
				transport.stop();
				EXPECT_FALSE(transport.running());
			}
			EXPECT_EQ(open_descriptor_count(), descriptors_before);
		}
	}  // namespace
}  // namespace control_link_adapters
