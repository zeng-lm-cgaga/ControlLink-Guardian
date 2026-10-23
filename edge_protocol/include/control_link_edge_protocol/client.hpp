#pragma once

#include <array>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <optional>
#include <string>

#include "control_link_edge_protocol/protocol.hpp"
#include "control_link_edge_protocol/channel_wait.hpp"
#include "edge_runtime/channel/sample.hpp"
#include "edge_runtime/common/result.hpp"

namespace control_link_edge_client
{
	using control_link_edge_protocol::ActiveSource;
	using control_link_edge_protocol::ClockDomain;
	using control_link_edge_protocol::CommandMode;
	using control_link_edge_protocol::FeedbackFrame;
	using control_link_edge_protocol::SessionToken;

	struct ClientOptions
	{
		std::string command_channel;
		std::string feedback_channel;
		std::chrono::milliseconds feedback_open_timeout{30'000};
		std::chrono::milliseconds reconnect_period{100};
		std::chrono::milliseconds feedback_freshness{500};
		WaitMode wait_mode{WaitMode::kFutex};
	};

	enum class ClientState : std::uint8_t
	{
		kOpening = 0U,
		kWaitingForSession = 1U,
		kReady = 2U,
		kFeedbackStale = 3U,
		kTransportLost = 4U,
		kProtocolError = 5U,
		kClosed = 6U,
		kClosing = 7U,
	};

	// SessionId 同时携带 token 和本地观察代次；重连即使暂时看到旧 token，也会使旧目标失效。
	struct SessionId
	{
		SessionToken token{};
		std::uint64_t epoch{0U};

		bool operator==(const SessionId &other) const noexcept
		{
			return epoch == other.epoch && token == other.token;
		}
		bool operator!=(const SessionId &other) const noexcept
		{
			return !(*this == other);
		}
	};

	struct SessionSnapshot
	{
		SessionId id{};
		std::uint64_t transport_generation{0U};
		std::array<std::byte, 16U> transport_instance_nonce{};
		bool has_token{false};
		bool ready{false};
		bool transport_allowed{false};
		bool feedback_fresh{false};
	};

	struct FeedbackSnapshot
	{
		FeedbackFrame frame{};
		// publish_boot_ns 是 EdgeRuntime 样本发布时刻，唯一用于 transport freshness。
		std::uint64_t publish_boot_ns{0U};
		// receive_boot_ns 只表示本地读取观察时刻，不得续期样本。
		std::uint64_t receive_boot_ns{0U};
		std::uint64_t missed_samples{0U};
		bool fresh{false};
		bool gateway_fresh{false};
		bool planning_fresh{false};
		bool vehicle_fresh{false};

		bool gateway_state_valid() const noexcept
		{
			return (frame.validity_flags & control_link_edge_protocol::kGatewayStateValid) != 0U;
		}
		bool planning_status_valid() const noexcept
		{
			return (frame.validity_flags & control_link_edge_protocol::kPlanningStatusValid) != 0U;
		}
		bool vehicle_state_valid() const noexcept
		{
			return (frame.validity_flags & control_link_edge_protocol::kVehicleStateValid) != 0U;
		}
		bool gateway_state_current() const noexcept
		{
			return gateway_state_valid() && gateway_fresh;
		}
		bool planning_status_current() const noexcept
		{
			return planning_status_valid() && planning_fresh;
		}
		bool vehicle_state_current() const noexcept
		{
			return vehicle_state_valid() && vehicle_fresh;
		}
	};

	// 业务目标必须由调用者绑定当前 SessionId；SDK 不替换旧 token，也不重盖 source_stamp。
	struct CommandTarget
	{
		SessionId session{};
		std::uint64_t command_sequence{0U};
		std::int64_t source_stamp_ns{0};
		ClockDomain clock_domain{ClockDomain::kSystem};
		CommandMode mode{CommandMode::kNormal};
		double linear_velocity_mps{0.0};
		double angular_velocity_radps{0.0};
	};

	struct SubmissionReceipt
	{
		std::uint64_t business_sequence{0U};
		edge_runtime::PublishInfo transport{};
	};

	struct ClientStatus
	{
		ClientState state{ClientState::kClosed};
		SessionSnapshot session{};
		bool feedback_seen{false};
		std::uint64_t last_feedback_sequence{0U};
		std::uint64_t last_feedback_publish_boot_ns{0U};
		std::uint64_t last_feedback_receive_boot_ns{0U};
		std::uint64_t last_submitted_sequence{0U};
		std::optional<FeedbackSnapshot> latest_feedback{};
		std::uint64_t notification_registrations{0U};
		std::uint64_t notification_registration_failures{0U};
		int command_notification_errno{0};
		bool cleanup_pending{false};
		bool last_control_committed{false};
	};

	// Client 只有一个 owner；同一实例的 open/read/submit/heartbeat/close 不能并发调用。
	class Client final
	{
	public:
		static edge_runtime::Result<Client> open(const ClientOptions &options);
#if CONTROL_LINK_EDGE_HAS_CONTROL_CONTEXT
		// prepare不接触SDK资源；connect失败时本对象仍持有receipt/handle，须fresh-context close。
		// pending时必须保有Client并显式重试；不满足析构前置条件会fail-fast，不隐式无限等待。
		static edge_runtime::Result<Client> prepare(const ClientOptions &options);
		edge_runtime::Result<void> connect(const ControlContext &context);
		edge_runtime::Result<CloseStatus> close(const ControlContext &context) noexcept;
#endif

		Client(const Client &) = delete;
		Client &operator=(const Client &) = delete;
		Client(Client &&other) noexcept;
		Client &operator=(Client &&other) noexcept;
		~Client();

		edge_runtime::Result<FeedbackSnapshot> wait_feedback(
			std::chrono::nanoseconds timeout);
		edge_runtime::Result<FeedbackSnapshot> try_feedback();
		edge_runtime::Result<SubmissionReceipt> submit(const CommandTarget &target);
		edge_runtime::Result<void> heartbeat() noexcept;

		SessionSnapshot current_session() const noexcept;
		ClientStatus status() const noexcept;
		edge_runtime::Result<void> close() noexcept;

	private:
		class Impl;
		explicit Client(std::unique_ptr<Impl> implementation) noexcept;
		std::unique_ptr<Impl> impl_;
	};
} // namespace control_link_edge_client
