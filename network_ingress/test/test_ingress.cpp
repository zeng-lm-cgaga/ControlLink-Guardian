#include "control_link_network_ingress/ingress.hpp"

#include <cstddef>
#include <cstdint>
#include <cstring>
#include <limits>
#include <stdexcept>
#include <string>

#include <gtest/gtest.h>

namespace control_link::network {
namespace {

namespace codec = control_link_edge_protocol;

wire::OpenSessionRequest open_request(char identity) {
  wire::OpenSessionRequest request;
  request.set_schema_version(1);
  request.set_holder_nonce(std::string(16, identity));
  request.set_open_request_id(std::string(16, identity));
  return request;
}

wire::KeepSessionRequest keep_request(const wire::OpenSessionRequest &open,
                                     std::uint64_t sequence) {
  wire::KeepSessionRequest request;
  request.set_schema_version(1);
  request.set_holder_nonce(open.holder_nonce());
  request.set_open_request_id(open.open_request_id());
  request.set_keepalive_sequence(sequence);
  return request;
}

codec::CommandFrame command(const std::string &token) {
  codec::CommandFrame frame;
  if (token.size() != frame.session_token.bytes.size()) throw std::invalid_argument("invalid fixture token");
  std::memcpy(frame.session_token.bytes.data(), token.data(), token.size());
  frame.command_sequence = 9;
  frame.source_stamp_ns = 1700000000123456789;
  frame.linear_velocity_mps = 0.5;
  return frame;
}

TEST(NetworkIngress, GrpcHolderUsesWideLiveRpcIdentityAndKeepDoesNotExpireIt) {
  const IngressResources resources{4096, 64, 0};
  PlanningIngress ingress(resources, {}, wire::PLANNING_TRANSPORT_GRPC);
  const auto request = open_request('a');
  constexpr std::uint64_t holder_rpc = (std::uint64_t{1} << 40) + 7;
  EXPECT_EQ(ingress.open(0, request, 1).result(), wire::SESSION_OPERATION_RESULT_REJECTED);
  const auto first = ingress.open(holder_rpc, request, 1);
  ASSERT_EQ(first.result(), wire::SESSION_OPERATION_RESULT_OPENING);
  EXPECT_EQ(first.session().planning_transport(), wire::PLANNING_TRANSPORT_GRPC);
  EXPECT_EQ(first.capabilities().planning_transport(), wire::PLANNING_TRANSPORT_GRPC);
  EXPECT_EQ(first.capabilities().schema_fingerprint(), schema_fingerprint());
  const auto epoch = ingress.snapshot().epoch;
  const auto retry = ingress.open(holder_rpc, request, 6000000001);
  EXPECT_EQ(retry.result(), wire::SESSION_OPERATION_RESULT_IDEMPOTENT);
  EXPECT_EQ(retry.session().session_token(), first.session().session_token());
  EXPECT_EQ(ingress.snapshot().epoch, epoch);
  const auto competitor = ingress.open(holder_rpc + 1, request, 6000000002);
  EXPECT_EQ(competitor.result(), wire::SESSION_OPERATION_RESULT_BUSY);
  EXPECT_FALSE(competitor.has_session());
  EXPECT_EQ(ingress.keep(holder_rpc, keep_request(request, 1), 6000000003).result(),
            wire::SESSION_OPERATION_RESULT_OPENING);
  EXPECT_EQ(ingress.keep(holder_rpc, keep_request(request, 1), 6000000004).result(),
            wire::SESSION_OPERATION_RESULT_REJECTED);
  EXPECT_EQ(ingress.keep(holder_rpc + 1, keep_request(request, 2), 6000000005).result(),
            wire::SESSION_OPERATION_RESULT_REJECTED);
  ingress.expire(std::numeric_limits<std::uint64_t>::max());
  EXPECT_EQ(ingress.snapshot().epoch, epoch);
  EXPECT_EQ(ingress.snapshot().state, wire::CONTROL_SESSION_STATE_OPENING);
  EXPECT_TRUE(ingress.readonly_status().session().session_token().empty());
}

TEST(NetworkIngress, GrpcUnaryTokenQualificationPreservesCommandAndEpochRevocation) {
  PlanningIngress ingress(IngressResources{4096, 64, 0}, {}, wire::PLANNING_TRANSPORT_GRPC);
  const auto request = open_request('a');
  const auto opened = ingress.open(UINT16_MAX, request, 1);
  ASSERT_EQ(opened.result(), wire::SESSION_OPERATION_RESULT_OPENING);
  const auto epoch = ingress.snapshot().epoch;
  ASSERT_TRUE(ingress.bind_publisher(epoch, std::string(24, 'g'), "rmw_fastrtps_cpp"));
  ASSERT_TRUE(ingress.set_admitted(epoch, true));
  const auto frame = command(opened.session().session_token());
  auto wrong_token = frame;
  wrong_token.session_token.bytes[0] ^= std::byte{1};
  EXPECT_EQ(ingress.submit(0, wrong_token, 2).forwarding_state(), wire::FORWARDING_STATE_REJECTED);
  EXPECT_EQ(ingress.submit(0, frame, 3).forwarding_state(), wire::FORWARDING_STATE_PENDING);
  codec::CommandFrame forwarded;
  ASSERT_TRUE(ingress.publish_pending(epoch, 4, [&](const auto &value) { forwarded = value; }));
  EXPECT_EQ(forwarded.command_sequence, frame.command_sequence);
  EXPECT_EQ(forwarded.source_stamp_ns, frame.source_stamp_ns);
  EXPECT_EQ(forwarded.session_token.bytes, frame.session_token.bytes);
  EXPECT_EQ(forwarded.linear_velocity_mps, frame.linear_velocity_mps);
  EXPECT_TRUE(ingress.readonly_status().latest_forwarded().key().session_token().empty());
  ASSERT_EQ(ingress.submit(123, frame, 5).forwarding_state(), wire::FORWARDING_STATE_PENDING);
  ASSERT_TRUE(ingress.revoke_if_current(epoch, "RPC_EOF"));
  const auto revoked_epoch = ingress.snapshot().epoch;
  EXPECT_FALSE(ingress.publish_pending(epoch, 6, [](const auto &) { FAIL() << "revoked epoch published"; }));
  ASSERT_TRUE(ingress.finish_release(revoked_epoch));
  const auto next = ingress.open(UINT16_MAX + std::uint64_t{1}, open_request('b'), 7);
  ASSERT_EQ(next.result(), wire::SESSION_OPERATION_RESULT_OPENING);
  EXPECT_NE(next.session().session_token(), opened.session().session_token());
  EXPECT_FALSE(ingress.revoke_if_current(epoch, "LATE_RPC_DONE"));
  EXPECT_EQ(ingress.snapshot().state, wire::CONTROL_SESSION_STATE_OPENING);
}

TEST(NetworkIngress, TransportReplyDropOnlyUpdatesObservedCounters) {
  PlanningIngress ingress(IngressResources{4096, 64, 0}, {}, wire::PLANNING_TRANSPORT_GRPC);
  const auto request = open_request('a');
  const auto opened = ingress.open(1, request, 1);
  ASSERT_EQ(opened.result(), wire::SESSION_OPERATION_RESULT_OPENING);
  const auto epoch = ingress.snapshot().epoch;
  ASSERT_TRUE(ingress.bind_publisher(epoch, std::string(24, 'g'), "rmw_fastrtps_cpp"));
  ASSERT_TRUE(ingress.set_admitted(epoch, true));
  const auto frame = command(opened.session().session_token());
  ASSERT_EQ(ingress.submit(0, frame, 2).forwarding_state(), wire::FORWARDING_STATE_PENDING);
  const auto before = ingress.readonly_status();
  ingress.record_transport_drop();
  const auto after = ingress.readonly_status();
  EXPECT_EQ(ingress.snapshot().epoch, epoch);
  EXPECT_EQ(after.session().SerializeAsString(), before.session().SerializeAsString());
  EXPECT_EQ(after.transport().missed_count(), before.transport().missed_count() + 1);
  EXPECT_EQ(after.transport().dropped_count(), before.transport().dropped_count() + 1);
  EXPECT_EQ(after.transport().rpc_received_count(), before.transport().rpc_received_count());
  EXPECT_EQ(after.transport().forwarded_count(), before.transport().forwarded_count());
  ASSERT_TRUE(ingress.publish_pending(epoch, 3, [&](const auto &value) {
    EXPECT_EQ(value.source_stamp_ns, frame.source_stamp_ns);
    EXPECT_EQ(value.command_sequence, frame.command_sequence);
  }));
}

TEST(NetworkIngress, LegacySomeipPeerRestrictionsIdleAndCapacityRemain) {
  const auto config = ::control_link::someip::load_config(CONTROL_LINK_PROTOCOL_CONFIG_PATH);
  PlanningIngress ingress(config, {});
  const auto request = open_request('a');
  EXPECT_EQ(ingress.open(0, request, 1).result(), wire::SESSION_OPERATION_RESULT_REJECTED);
  EXPECT_EQ(ingress.open(UINT16_MAX, request, 1).result(), wire::SESSION_OPERATION_RESULT_REJECTED);
  EXPECT_EQ(ingress.open(std::uint64_t{1} << 40, request, 1).result(), wire::SESSION_OPERATION_RESULT_REJECTED);
  const auto opened = ingress.open(0x3101, request, 1);
  ASSERT_EQ(opened.result(), wire::SESSION_OPERATION_RESULT_OPENING);
  EXPECT_EQ(opened.session().planning_transport(), wire::PLANNING_TRANSPORT_SOMEIP);
  const auto epoch = ingress.snapshot().epoch;
  ASSERT_TRUE(ingress.bind_publisher(epoch, std::string(24, 'g'), "rmw_fastrtps_cpp"));
  ASSERT_TRUE(ingress.set_admitted(epoch, true));
  EXPECT_EQ(ingress.submit(0, command(opened.session().session_token()), 2).forwarding_state(),
            wire::FORWARDING_STATE_REJECTED);
  ingress.expire(std::uint64_t{config.idle_timeout_ms} * 1000000 + 1);
  EXPECT_EQ(ingress.snapshot().state, wire::CONTROL_SESSION_STATE_REVOKING);
  EXPECT_EQ(ingress.snapshot().failure, "HOLDER_IDLE_TIMEOUT");
  auto too_small = config;
  too_small.max_payload_bytes = 64;
  EXPECT_THROW(PlanningIngress(too_small, {}), std::invalid_argument);
  EXPECT_THROW(PlanningIngress(IngressResources{64, 64, 0}, {}, wire::PLANNING_TRANSPORT_GRPC),
               std::invalid_argument);
}

TEST(NetworkIngress, UnaryCancellationMatchesSlotOwnerNotBusinessKey) {
  PlanningIngress ingress(IngressResources{4096, 64, 0}, {}, wire::PLANNING_TRANSPORT_GRPC);
  const auto opened = ingress.open(1, open_request('a'), 1);
  ASSERT_EQ(opened.result(), wire::SESSION_OPERATION_RESULT_OPENING);
  const auto epoch = ingress.snapshot().epoch;
  ASSERT_TRUE(ingress.bind_publisher(epoch, std::string(24, 'g'), "rmw_fastrtps_cpp"));
  ASSERT_TRUE(ingress.set_admitted(epoch, true));
  const auto frame = command(opened.session().session_token());
  ASSERT_EQ(ingress.submit(0, frame, 2, 101).forwarding_state(), wire::FORWARDING_STATE_PENDING);
  ASSERT_EQ(ingress.submit(0, frame, 3, 102).forwarding_state(), wire::FORWARDING_STATE_PENDING);
  const auto after_overwrite = ingress.readonly_status().transport();
  EXPECT_FALSE(ingress.cancel_pending_submission(epoch, 101));
  EXPECT_FALSE(ingress.cancel_pending_submission(epoch, 0));
  EXPECT_FALSE(ingress.cancel_pending_submission(epoch + 1, 102));
  EXPECT_EQ(ingress.readonly_status().transport().SerializeAsString(), after_overwrite.SerializeAsString());
  ASSERT_TRUE(ingress.cancel_pending_submission(epoch, 102));
  EXPECT_EQ(ingress.readonly_status().transport().dropped_count(), after_overwrite.dropped_count() + 1);
  EXPECT_FALSE(ingress.cancel_pending_submission(epoch, 102));
  EXPECT_FALSE(ingress.publish_pending(epoch, 4, [](const auto &) { FAIL() << "cancelled slot published"; }));
  ASSERT_EQ(ingress.submit(0, frame, 5, 103).forwarding_state(), wire::FORWARDING_STATE_PENDING);
  ASSERT_TRUE(ingress.publish_pending(epoch, 6, [](const auto &) {}));
  EXPECT_FALSE(ingress.cancel_pending_submission(epoch, 103));
  ASSERT_EQ(ingress.submit(0, frame, 7, 104).forwarding_state(), wire::FORWARDING_STATE_PENDING);
  ASSERT_TRUE(ingress.revoke_if_current(epoch, "RPC_DONE"));
  EXPECT_FALSE(ingress.cancel_pending_submission(epoch, 104));
  ASSERT_TRUE(ingress.finish_release(ingress.snapshot().epoch));
  const auto next = ingress.open(2, open_request('b'), 8);
  ASSERT_EQ(next.result(), wire::SESSION_OPERATION_RESULT_OPENING);
  const auto next_epoch = ingress.snapshot().epoch;
  ASSERT_TRUE(ingress.bind_publisher(next_epoch, std::string(24, 'n'), "rmw_fastrtps_cpp"));
  ASSERT_TRUE(ingress.set_admitted(next_epoch, true));
  ASSERT_EQ(ingress.submit(0, command(next.session().session_token()), 9, 104).forwarding_state(),
            wire::FORWARDING_STATE_PENDING);
  EXPECT_FALSE(ingress.cancel_pending_submission(epoch, 104));
  ASSERT_TRUE(ingress.publish_pending(next_epoch, 10, [](const auto &) {}));
}

}
}
