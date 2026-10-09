#include "control_link_edge_protocol/client.hpp"

#include <chrono>
#include <cstdint>
#include <iostream>
#include <memory>
#include <string>
#include <thread>
#include <unistd.h>

#include "control_link_edge_protocol/edge_payload.hpp"
#include "edge_runtime/channel/channel_options.hpp"
#include "edge_runtime/channel/consumer.hpp"
#include "edge_runtime/channel/producer.hpp"

namespace
{
	using control_link_edge_client::Client;
	using control_link_edge_client::ClientOptions;
	using control_link_edge_client::ClientState;
	using control_link_edge_client::CommandTarget;
	using control_link_edge_client::SessionId;
	using control_link_edge_protocol::ActiveSource;
	using control_link_edge_protocol::ClockDomain;
	using control_link_edge_protocol::CommandFrame;
	using control_link_edge_protocol::CommandMode;
	using control_link_edge_protocol::FeedbackFrame;
	using control_link_edge_protocol::SessionToken;
	using control_link_edge_protocol::TransportState;
	using CommandConsumer = edge_runtime::Consumer<CommandFrame>;
	using FeedbackProducer = edge_runtime::Producer<FeedbackFrame>;

	SessionToken make_token(std::uint8_t seed)
	{
		SessionToken token;
		for (std::size_t index = 0U; index < token.bytes.size(); ++index)
		{
			token.bytes[index] = static_cast<std::byte>(seed + index);
		}
		return token;
	}

	FeedbackFrame make_feedback(
		const SessionToken &token,
		std::uint64_t sequence,
		std::uint64_t receive_boot_ns)
	{
		FeedbackFrame frame;
		frame.session_token = token;
		frame.feedback_sequence = sequence;
		frame.bridge_publish_boot_ns = receive_boot_ns;
		frame.transport_state = TransportState::kWaitingForCommand;
		frame.gateway_state = 2U;
		frame.validity_flags = control_link_edge_protocol::kGatewayStateValid |
			control_link_edge_protocol::kPlanningStatusValid |
			control_link_edge_protocol::kVehicleStateValid |
			control_link_edge_protocol::kSessionReady;
		frame.active_source = ActiveSource::kPlanning;
		frame.vehicle_state = 1U;
		frame.gateway_observed_ros_ns =
			static_cast<std::uint64_t>(control_link_edge_protocol::system_time_now_ns());
		frame.gateway_receive_boot_ns = receive_boot_ns;
		frame.planning_observed_ros_ns = frame.gateway_observed_ros_ns;
		frame.planning_receive_boot_ns = receive_boot_ns;
		frame.vehicle_observed_ros_ns = frame.gateway_observed_ros_ns;
		frame.vehicle_receive_boot_ns = receive_boot_ns;
		frame.vehicle_linear_velocity_mps = 0.1;
		frame.vehicle_angular_velocity_radps = 0.01;
		return frame;
	}

	bool check(bool condition, const char *message)
	{
		if (condition) return true;
		std::cerr << "test_client: " << message << '\n';
		return false;
	}

	CommandTarget make_target(const SessionId &session, std::uint64_t sequence)
	{
		CommandTarget target;
		target.session = session;
		target.command_sequence = sequence;
		target.source_stamp_ns = control_link_edge_protocol::system_time_now_ns();
		target.clock_domain = ClockDomain::kSystem;
		target.mode = CommandMode::kNormal;
		target.linear_velocity_mps = 0.2;
		target.angular_velocity_radps = 0.05;
		return target;
	}
}

int main()
{
	const auto suffix = std::to_string(static_cast<unsigned long long>(::getpid()));
	const std::string command_name = "control_link_a5_client_command_" + suffix;
	const std::string feedback_name = "control_link_a5_client_feedback_" + suffix;

	edge_runtime::ChannelOptions feedback_options;
	feedback_options.name = feedback_name;
	feedback_options.stale_timeout = std::chrono::milliseconds(500);
	auto feedback_created = FeedbackProducer::create(
		feedback_options, edge_runtime::feedback_schema());
	if (!feedback_created)
	{
		std::cerr << "feedback producer create failed: "
			<< edge_runtime::to_string(feedback_created.error().code)
			<< " errno=" << feedback_created.error().errno_value
			<< " context=" << feedback_created.error().context << '\n';
		return 1;
	}
	auto feedback_producer = std::make_unique<FeedbackProducer>(
		std::move(feedback_created.value()));

	ClientOptions options;
	options.command_channel = command_name;
	options.feedback_channel = feedback_name;
	options.feedback_open_timeout = std::chrono::seconds(2);
	options.reconnect_period = std::chrono::milliseconds(20);
	options.feedback_freshness = std::chrono::milliseconds(100);
	auto opened = Client::open(options);
	if (!check(static_cast<bool>(opened), "client open failed"))
	{
		(void)feedback_producer->remove_if_owner();
		return 1;
	}
	Client client = std::move(opened.value());

	edge_runtime::ChannelOptions command_options;
	command_options.name = command_name;
	command_options.stale_timeout = std::chrono::milliseconds(500);
	auto command_opened = CommandConsumer::open(
		command_options, edge_runtime::command_schema());
	if (!check(static_cast<bool>(command_opened), "command consumer open failed"))
	{
		(void)client.close();
		(void)feedback_producer->remove_if_owner();
		return 1;
	}
	auto command_consumer = std::make_unique<CommandConsumer>(
		std::move(command_opened.value()));

	const auto first_token = make_token(0x21U);
	const auto first_boot_ns = control_link_edge_protocol::boottime_now_ns();
	if (!check(first_boot_ns != 0U, "BOOTTIME unavailable") ||
		!check(static_cast<bool>(feedback_producer->publish(
			make_feedback(first_token, 1U, first_boot_ns))),
			"first feedback publish failed"))
	{
		(void)client.close();
		command_consumer.reset();
		(void)feedback_producer->remove_if_owner();
		return 1;
	}

	std::this_thread::sleep_for(std::chrono::milliseconds(150));
	if (!check(static_cast<bool>(feedback_producer->heartbeat()),
			"feedback heartbeat failed before delayed read"))
	{
		(void)client.close();
		command_consumer.reset();
		(void)feedback_producer->remove_if_owner();
		return 1;
	}
	auto first_feedback = client.wait_feedback(std::chrono::seconds(1));
	if (!check(static_cast<bool>(first_feedback), "first feedback read failed"))
	{
		(void)client.close();
		command_consumer.reset();
		(void)feedback_producer->remove_if_owner();
		return 1;
	}
	if (!check(!first_feedback.value().fresh,
			"delayed first feedback was incorrectly fresh"))
	{
		(void)client.close();
		command_consumer.reset();
		(void)feedback_producer->remove_if_owner();
		return 1;
	}
	const auto first_session = client.current_session();
	if (!check(first_session.ready && first_session.has_token, "first session not ready"))
	{
		(void)client.close();
		command_consumer.reset();
		(void)feedback_producer->remove_if_owner();
		return 1;
	}
	const auto first_target = make_target(first_session.id, 1U);
	const auto delayed_submit = client.submit(first_target);
	if (!check(!delayed_submit &&
			delayed_submit.error().code == edge_runtime::ErrorCode::kDataStale,
			"delayed target was not rejected as stale"))
	{
		(void)client.close();
		command_consumer.reset();
		(void)feedback_producer->remove_if_owner();
		return 1;
	}

	const auto fresh_boot_ns = control_link_edge_protocol::boottime_now_ns();
	if (!check(static_cast<bool>(feedback_producer->publish(
			make_feedback(first_token, 2U, fresh_boot_ns))),
			"fresh feedback publish failed"))
	{
		(void)client.close();
		command_consumer.reset();
		(void)feedback_producer->remove_if_owner();
		return 1;
	}
	auto fresh_feedback = client.wait_feedback(std::chrono::seconds(1));
	if (!check(static_cast<bool>(fresh_feedback), "fresh feedback read failed") ||
		!check(fresh_feedback.value().fresh, "fresh feedback remained stale") ||
		!check(static_cast<bool>(client.submit(first_target)), "fresh target rejected"))
	{
		(void)client.close();
		command_consumer.reset();
		(void)feedback_producer->remove_if_owner();
		return 1;
	}
	auto first_command = command_consumer->wait_latest(std::chrono::seconds(1));
	if (!check(static_cast<bool>(first_command), "first command was not transported") ||
		!check(first_command.value().value.command_sequence == 1U,
			"first command sequence mismatch"))
	{
		(void)client.close();
		command_consumer.reset();
		(void)feedback_producer->remove_if_owner();
		return 1;
	}

	std::this_thread::sleep_for(std::chrono::milliseconds(150));
	if (!check(static_cast<bool>(feedback_producer->heartbeat()),
			"feedback heartbeat failed during sample stall"))
	{
		(void)client.close();
		command_consumer.reset();
		(void)feedback_producer->remove_if_owner();
		return 1;
	}
	const auto stalled_status = client.status();
	if (!check(stalled_status.state == ClientState::kFeedbackStale,
			"heartbeat-only stall did not become stale") ||
		!check(stalled_status.latest_feedback.has_value() &&
			!stalled_status.latest_feedback->fresh,
			"heartbeat-only stall retained fresh sample"))
	{
		(void)client.close();
		command_consumer.reset();
		(void)feedback_producer->remove_if_owner();
		return 1;
	}
	const auto stalled_target = make_target(first_session.id, 2U);
	const auto stalled_submit = client.submit(stalled_target);
	if (!check(!stalled_submit &&
			stalled_submit.error().code == edge_runtime::ErrorCode::kDataStale,
			"heartbeat-only stale feedback allowed submit"))
	{
		(void)client.close();
		command_consumer.reset();
		(void)feedback_producer->remove_if_owner();
		return 1;
	}

	const auto resumed_boot_ns = control_link_edge_protocol::boottime_now_ns();
	if (!check(static_cast<bool>(feedback_producer->publish(
			make_feedback(first_token, 3U, resumed_boot_ns))),
			"resumed feedback publish failed"))
	{
		(void)client.close();
		command_consumer.reset();
		(void)feedback_producer->remove_if_owner();
		return 1;
	}
	auto resumed_feedback = client.wait_feedback(std::chrono::seconds(1));
	if (!check(static_cast<bool>(resumed_feedback), "resumed feedback read failed") ||
		!check(resumed_feedback.value().fresh, "resumed feedback remained stale") ||
		!check(static_cast<bool>(client.submit(stalled_target)),
			"target after fresh feedback was rejected"))
	{
		(void)client.close();
		command_consumer.reset();
		(void)feedback_producer->remove_if_owner();
		return 1;
	}
	const auto resumed_command = command_consumer->wait_latest(std::chrono::seconds(1));
	if (!check(static_cast<bool>(resumed_command), "resumed command was not transported") ||
		!check(resumed_command.value().value.command_sequence == 2U,
			"resumed command sequence mismatch"))
	{
		(void)client.close();
		command_consumer.reset();
		(void)feedback_producer->remove_if_owner();
		return 1;
	}

	const auto stale_boot_ns = control_link_edge_protocol::boottime_now_ns();
	auto stale_feedback = make_feedback(first_token, 4U, stale_boot_ns);
	stale_feedback.gateway_receive_boot_ns = stale_boot_ns - 1'000'000'000U;
	stale_feedback.planning_receive_boot_ns = stale_feedback.gateway_receive_boot_ns;
	stale_feedback.vehicle_receive_boot_ns = stale_feedback.gateway_receive_boot_ns;
	if (!check(static_cast<bool>(feedback_producer->publish(stale_feedback)),
			"stale feedback publish failed"))
	{
		(void)client.close();
		command_consumer.reset();
		(void)feedback_producer->remove_if_owner();
		return 1;
	}
	auto stale_snapshot = client.wait_feedback(std::chrono::seconds(1));
	if (!check(static_cast<bool>(stale_snapshot), "stale feedback read failed") ||
			!check(stale_snapshot.value().fresh, "transport feedback unexpectedly stale") ||
		!check(!stale_snapshot.value().gateway_state_current(),
			"stale gateway observation remained current") ||
		!check(!stale_snapshot.value().planning_status_current(),
			"stale planning observation remained current") ||
		!check(!stale_snapshot.value().vehicle_state_current(),
			"stale vehicle observation remained current"))
	{
		(void)client.close();
		command_consumer.reset();
		(void)feedback_producer->remove_if_owner();
		return 1;
	}
	std::this_thread::sleep_for(std::chrono::milliseconds(150));
	const auto stale_status = client.status();
	if (!check(stale_status.state == ClientState::kFeedbackStale,
			"status did not report stale feedback") ||
		!check(stale_status.latest_feedback.has_value() &&
			!stale_status.latest_feedback->fresh,
			"status retained fresh feedback after transport timeout"))
	{
		(void)client.close();
		command_consumer.reset();
		(void)feedback_producer->remove_if_owner();
		return 1;
	}

	const auto second_token = make_token(0x61U);
	const auto second_boot_ns = control_link_edge_protocol::boottime_now_ns();
	if (!check(static_cast<bool>(feedback_producer->publish(
			make_feedback(second_token, 5U, second_boot_ns))),
			"second feedback publish failed"))
	{
		(void)client.close();
		command_consumer.reset();
		(void)feedback_producer->remove_if_owner();
		return 1;
	}
	auto second_feedback = client.wait_feedback(std::chrono::seconds(1));
	const auto second_session = client.current_session();
	if (!check(static_cast<bool>(second_feedback), "second feedback read failed") ||
		!check(second_session.id != first_session.id, "session epoch did not change") ||
		!check(second_session.id.token == second_token, "session token did not change"))
	{
		(void)client.close();
		command_consumer.reset();
		(void)feedback_producer->remove_if_owner();
		return 1;
	}

	auto old_target_result = client.submit(first_target);
	if (!check(!old_target_result &&
			old_target_result.error().code == edge_runtime::ErrorCode::kStaleHandle,
			"old target was not rejected after session change"))
	{
		(void)client.close();
		command_consumer.reset();
		(void)feedback_producer->remove_if_owner();
		return 1;
	}
	if (!check(static_cast<bool>(client.submit(make_target(second_session.id, 1U))),
			"new target was rejected after session change"))
	{
		(void)client.close();
		command_consumer.reset();
		(void)feedback_producer->remove_if_owner();
		return 1;
	}
	const auto second_command = command_consumer->wait_latest(std::chrono::seconds(1));
	if (!check(static_cast<bool>(second_command), "new command was not transported") ||
		!check(second_command.value().value.session_token == second_token,
			"new command used the old session token"))
	{
		(void)client.close();
		command_consumer.reset();
		(void)feedback_producer->remove_if_owner();
		return 1;
	}

	const auto removed_feedback = feedback_producer->remove_if_owner();
	feedback_producer.reset();
	if (!check(static_cast<bool>(removed_feedback), "feedback producer replacement cleanup failed"))
	{
		(void)client.close();
		command_consumer.reset();
		return 1;
	}
	auto replacement_created = FeedbackProducer::create(
		feedback_options, edge_runtime::feedback_schema());
	if (!check(static_cast<bool>(replacement_created), "replacement feedback producer create failed"))
	{
		(void)client.close();
		command_consumer.reset();
		return 1;
	}
	feedback_producer = std::make_unique<FeedbackProducer>(
		std::move(replacement_created.value()));
	const auto third_token = make_token(0x91U);
	const auto third_boot_ns = control_link_edge_protocol::boottime_now_ns();
	if (!check(static_cast<bool>(feedback_producer->publish(
			make_feedback(third_token, 1U, third_boot_ns))),
		"replacement feedback publish failed"))
	{
		(void)client.close();
		command_consumer.reset();
		(void)feedback_producer->remove_if_owner();
		return 1;
	}
	bool replacement_observed = false;
	for (int attempt = 0; attempt < 80 && !replacement_observed; ++attempt)
	{
		auto replacement_feedback = client.wait_feedback(std::chrono::milliseconds(50));
		if (replacement_feedback && client.current_session().id.token == third_token)
		{
			replacement_observed = true;
		}
	}
	if (!check(replacement_observed, "feedback producer replacement was not observed"))
	{
		(void)client.close();
		command_consumer.reset();
		(void)feedback_producer->remove_if_owner();
		return 1;
	}

	const auto closed = client.close();
	command_consumer.reset();
	const auto removed = feedback_producer->remove_if_owner();
	if (!check(static_cast<bool>(closed), "client close failed") ||
		!check(static_cast<bool>(removed), "feedback channel cleanup failed"))
	{
		return 1;
	}
	std::cout << "A5_CLIENT_SESSION_SEMANTICS_PASSED\n";
	return 0;
}
