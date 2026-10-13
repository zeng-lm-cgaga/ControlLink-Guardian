#include "control_link_network_ingress/ingress.hpp"

#include <cstddef>
#include <cstdint>
#include <cstring>
#include <functional>
#include <future>
#include <limits>
#include <stdexcept>
#include <string>
#include <vector>

#include <gtest/gtest.h>

namespace control_link::network {
namespace {

namespace codec = control_link_edge_protocol;

wire::OpenSessionRequest open_request(const PlanningIngress &ingress, char identity) {
  wire::OpenSessionRequest request;
  request.set_schema_version(1);
  request.set_holder_nonce(std::string(16, identity));
  request.set_open_request_id(std::string(16, identity));
  *request.mutable_opening_domain() = ingress.opening_domain();
  return request;
}

wire::KeepSessionRequest keep_request(const wire::OpenSessionRequest &open,
                                     std::uint64_t sequence) {
  wire::KeepSessionRequest request;
  request.set_schema_version(1);
  request.set_holder_nonce(open.holder_nonce());
  request.set_open_request_id(open.open_request_id());
  request.set_keepalive_sequence(sequence);
  *request.mutable_opening_domain() = open.opening_domain();
  return request;
}

wire::CloseSessionRequest close_request(const wire::OpenSessionRequest &open) {
  wire::CloseSessionRequest request;
  request.set_schema_version(1);
  request.set_holder_nonce(open.holder_nonce());
  request.set_open_request_id(open.open_request_id());
  *request.mutable_opening_domain() = open.opening_domain();
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
  const auto request = open_request(ingress, 'a');
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
  const auto request = open_request(ingress, 'a');
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
  const auto next = ingress.open(UINT16_MAX + std::uint64_t{1}, open_request(ingress, 'b'), 7);
  ASSERT_EQ(next.result(), wire::SESSION_OPERATION_RESULT_OPENING);
  EXPECT_NE(next.session().session_token(), opened.session().session_token());
  EXPECT_FALSE(ingress.revoke_if_current(epoch, "LATE_RPC_DONE"));
  EXPECT_EQ(ingress.snapshot().state, wire::CONTROL_SESSION_STATE_OPENING);
}

TEST(NetworkIngress, TransportReplyDropOnlyUpdatesObservedCounters) {
  PlanningIngress ingress(IngressResources{4096, 64, 0}, {}, wire::PLANNING_TRANSPORT_GRPC);
  const auto request = open_request(ingress, 'a');
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
  const auto request = open_request(ingress, 'a');
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
  const auto opened = ingress.open(1, open_request(ingress, 'a'), 1);
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
  const auto next = ingress.open(2, open_request(ingress, 'b'), 8);
  ASSERT_EQ(next.result(), wire::SESSION_OPERATION_RESULT_OPENING);
  const auto next_epoch = ingress.snapshot().epoch;
  ASSERT_TRUE(ingress.bind_publisher(next_epoch, std::string(24, 'n'), "rmw_fastrtps_cpp"));
  ASSERT_TRUE(ingress.set_admitted(next_epoch, true));
  ASSERT_EQ(ingress.submit(0, command(next.session().session_token()), 9, 104).forwarding_state(),
            wire::FORWARDING_STATE_PENDING);
  EXPECT_FALSE(ingress.cancel_pending_submission(epoch, 104));
  ASSERT_TRUE(ingress.publish_pending(next_epoch, 10, [](const auto &) {}));
}

TEST(NetworkIngress, AdmissionDomainAllows128RotationsWithBoundedRetiredKeysForBothTransports) {
  for (const auto transport : {wire::PLANNING_TRANSPORT_SOMEIP, wire::PLANNING_TRANSPORT_GRPC}) {
    SCOPED_TRACE(transport);
    const IngressResources resources{4096, 2, 5000};
    std::uint64_t identity_sequence = 1;
    PlanningIngress ingress(resources, {}, transport, [&] {
      std::string identity(16, 'r');
      std::memcpy(identity.data(), &identity_sequence, sizeof(identity_sequence));
      ++identity_sequence;
      return identity;
    });
    const auto initial_domain = ingress.opening_domain();
    ASSERT_EQ(initial_domain.service_instance_id().size(), 16U);
    ASSERT_EQ(initial_domain.generation_presence_case(), wire::OpenAdmissionDomain::kGeneration);
    EXPECT_EQ(initial_domain.generation(), 1U);
    EXPECT_EQ(initial_domain.domain_version(), 1U);
    std::vector<std::string> retired_requests;
    std::string previous_token;
    for (std::uint64_t rotation = 0; rotation < 128; ++rotation) {
      SCOPED_TRACE(rotation);
      const auto now = 1000000000 + rotation * 100;
      const auto request = open_request(ingress, 'a');
      EXPECT_EQ(request.opening_domain().generation(), rotation + 1);
      EXPECT_EQ(request.opening_domain().service_instance_id(), initial_domain.service_instance_id());
      const auto opened = ingress.open(0x3101, request, now);
      ASSERT_EQ(opened.result(), wire::SESSION_OPERATION_RESULT_OPENING);
      ASSERT_EQ(opened.session().session_token().size(), 16U);
      EXPECT_NE(opened.session().session_token(), previous_token);
      previous_token = opened.session().session_token();
      const auto epoch = ingress.snapshot().epoch;
      const auto retry = ingress.open(0x3101, request, now + 1);
      EXPECT_EQ(retry.result(), wire::SESSION_OPERATION_RESULT_IDEMPOTENT);
      EXPECT_EQ(retry.session().session_token(), opened.session().session_token());
      EXPECT_EQ(ingress.snapshot().epoch, epoch);
      const auto competitor = ingress.open(0x3102, request, now + 2);
      EXPECT_EQ(competitor.result(), wire::SESSION_OPERATION_RESULT_BUSY);
      EXPECT_FALSE(competitor.has_session());
      EXPECT_EQ(competitor.SerializeAsString().find(previous_token), std::string::npos);
      std::string gid(24, 'g');
      std::memcpy(gid.data(), &rotation, sizeof(rotation));
      ASSERT_TRUE(ingress.bind_publisher(epoch, gid, "rmw_fastrtps_cpp"));
      ASSERT_TRUE(ingress.set_admitted(epoch, true));
      const auto admitted_retry = ingress.open(0x3101, request, now + 3);
      EXPECT_EQ(admitted_retry.result(), wire::SESSION_OPERATION_RESULT_IDEMPOTENT);
      EXPECT_EQ(admitted_retry.session().ingress_publisher().gid(), gid);
      EXPECT_EQ(admitted_retry.session().session_token(), previous_token);
      EXPECT_EQ(ingress.snapshot().epoch, epoch);
      const auto before_readonly = ingress.snapshot();
      EXPECT_EQ(ingress.capabilities().opening_domain().SerializeAsString(),
                request.opening_domain().SerializeAsString());
      const auto status = ingress.readonly_status();
      EXPECT_TRUE(status.session().session_token().empty());
      EXPECT_EQ(status.opening_domain().SerializeAsString(), request.opening_domain().SerializeAsString());
      EXPECT_EQ(ingress.snapshot().opening_domain.SerializeAsString(), before_readonly.opening_domain.SerializeAsString());
      EXPECT_EQ(ingress.snapshot().epoch, before_readonly.epoch);
      EXPECT_EQ(ingress.snapshot().retired_open_count, 0U);
      const auto frame = command(previous_token);
      ASSERT_EQ(ingress.submit(0x3101, frame, now + 4, rotation + 1).forwarding_state(),
                wire::FORWARDING_STATE_PENDING);
      ASSERT_EQ(ingress.keep(0x3101, keep_request(request, 1), now + 5).result(),
                wire::SESSION_OPERATION_RESULT_ADMITTED);
      ASSERT_TRUE(ingress.publish_pending(epoch, now + 6, [&](const auto &forwarded) {
        EXPECT_EQ(forwarded.source_stamp_ns, frame.source_stamp_ns);
        EXPECT_EQ(forwarded.command_sequence, frame.command_sequence);
        EXPECT_EQ(forwarded.session_token.bytes, frame.session_token.bytes);
      }));
      ASSERT_EQ(ingress.submit(0x3101, frame, now + 7, rotation + 1).forwarding_state(),
                wire::FORWARDING_STATE_PENDING);
      ASSERT_EQ(ingress.close(0x3101, close_request(request)).result(), wire::SESSION_OPERATION_RESULT_REVOKED);
      const auto revoked_epoch = ingress.snapshot().epoch;
      const auto busy = ingress.open(0x3102, request, now + 8);
      EXPECT_EQ(busy.result(), wire::SESSION_OPERATION_RESULT_BUSY);
      EXPECT_FALSE(busy.has_session());
      EXPECT_FALSE(ingress.publish_pending(epoch, now + 9, [](const auto &) { FAIL() << "revoked target published"; }));
      EXPECT_FALSE(ingress.cancel_pending_submission(epoch, rotation + 1));
      EXPECT_FALSE(ingress.finish_release(epoch));
      EXPECT_LE(ingress.snapshot().retired_open_count, resources.retired_open_capacity);
      ASSERT_TRUE(ingress.finish_release(revoked_epoch));
      const auto released = ingress.snapshot();
      EXPECT_EQ(released.state, wire::CONTROL_SESSION_STATE_IDLE);
      EXPECT_EQ(released.opening_domain.generation(), rotation + 2);
      EXPECT_EQ(released.retired_open_count, 0U);
      EXPECT_LE(released.retired_open_count, resources.retired_open_capacity);
      EXPECT_FALSE(ingress.bind_publisher(epoch, gid, "rmw_fastrtps_cpp"));
      EXPECT_FALSE(ingress.set_admitted(epoch, true));
      EXPECT_FALSE(ingress.revoke_if_current(epoch, "late completion"));
      EXPECT_FALSE(ingress.publish_pending(epoch, now + 10, [](const auto &) { FAIL() << "retired target published"; }));
      EXPECT_FALSE(ingress.readonly_status().has_latest_forwarded());
      retired_requests.push_back(request.SerializeAsString());
      const auto rejected = ingress.open(0x3101, request, now + 11);
      EXPECT_EQ(rejected.result(), wire::SESSION_OPERATION_RESULT_REJECTED);
      EXPECT_EQ(rejected.failure_reason(), "OPEN_ADMISSION_DOMAIN_REQUIRED_OR_STALE");
      EXPECT_FALSE(rejected.has_session());
      EXPECT_EQ(request.SerializeAsString(), retired_requests.back());
    }
    EXPECT_EQ(ingress.opening_domain().generation(), 129U);
    const auto final_epoch = ingress.snapshot().epoch;
    for (const auto &bytes : retired_requests) {
      wire::OpenSessionRequest delayed;
      ASSERT_TRUE(delayed.ParseFromString(bytes));
      for (const auto peer : {0x3101, 0x3102}) {
        const auto rejected = ingress.open(peer, delayed, 1000000000 + 128 * 100);
        EXPECT_EQ(rejected.result(), wire::SESSION_OPERATION_RESULT_REJECTED);
        EXPECT_EQ(rejected.failure_reason(), "OPEN_ADMISSION_DOMAIN_REQUIRED_OR_STALE");
        EXPECT_FALSE(rejected.has_session());
      }
      EXPECT_EQ(delayed.SerializeAsString(), bytes);
      EXPECT_EQ(ingress.snapshot().epoch, final_epoch);
      EXPECT_EQ(ingress.snapshot().retired_open_count, 0U);
    }
    EXPECT_EQ(ingress.open(0x3101, open_request(ingress, 'a'), 1000000000 + 129 * 100).result(),
              wire::SESSION_OPERATION_RESULT_OPENING);
  }
}

TEST(NetworkIngress, MissingMalformedForeignAndFutureAdmissionDomainsNeverClaimHolder) {
  PlanningIngress ingress(IngressResources{4096, 2, 0}, {}, wire::PLANNING_TRANSPORT_GRPC,
                           [] { return std::string(16, 'i'); });
  const auto current = open_request(ingress, 'a');
  std::vector<wire::OpenSessionRequest> invalid_requests;
  auto invalid = current;
  invalid.clear_opening_domain();
  invalid_requests.push_back(invalid);
  invalid = current;
  invalid.mutable_opening_domain()->clear_generation();
  invalid_requests.push_back(invalid);
  for (const auto generation : {std::uint64_t{0}, std::uint64_t{2}, std::numeric_limits<std::uint64_t>::max()}) {
    invalid = current;
    invalid.mutable_opening_domain()->set_generation(generation);
    invalid_requests.push_back(invalid);
  }
  for (const auto &identity : {std::string{}, std::string("short"), std::string(16, 'x')}) {
    invalid = current;
    invalid.mutable_opening_domain()->set_service_instance_id(identity);
    invalid_requests.push_back(invalid);
  }
  for (const auto version : {0U, 2U}) {
    invalid = current;
    invalid.mutable_opening_domain()->set_domain_version(version);
    invalid_requests.push_back(invalid);
  }
  for (const auto &request : invalid_requests) {
    SCOPED_TRACE(request.DebugString());
    const auto rejected = ingress.open(1, request, 1);
    EXPECT_EQ(rejected.result(), wire::SESSION_OPERATION_RESULT_REJECTED);
    EXPECT_EQ(rejected.failure_reason(), "OPEN_ADMISSION_DOMAIN_REQUIRED_OR_STALE");
    EXPECT_FALSE(rejected.has_session());
    EXPECT_EQ(ingress.snapshot().state, wire::CONTROL_SESSION_STATE_IDLE);
    EXPECT_EQ(ingress.snapshot().epoch, 0U);
    EXPECT_EQ(ingress.snapshot().retired_open_count, 0U);
    EXPECT_EQ(ingress.opening_domain().SerializeAsString(), current.opening_domain().SerializeAsString());
  }
  EXPECT_EQ(ingress.open(1, current, 2).result(), wire::SESSION_OPERATION_RESULT_OPENING);
}

TEST(NetworkIngress, NewServiceInstanceRejectsOldHandshakeBytesEvenWhenGenerationAndKeysMatch) {
  std::uint64_t old_entropy_calls = 0;
  PlanningIngress previous(IngressResources{4096, 2, 0}, {}, wire::PLANNING_TRANSPORT_GRPC, [&] {
    return std::string(16, ++old_entropy_calls == 1 ? 'i' : 't');
  });
  const auto old_request = open_request(previous, 'a');
  const auto old_bytes = old_request.SerializeAsString();
  const auto opened = previous.open(1, old_request, 1);
  ASSERT_EQ(opened.result(), wire::SESSION_OPERATION_RESULT_OPENING);
  previous.stop();
  std::uint64_t new_entropy_calls = 0;
  PlanningIngress replacement(IngressResources{4096, 2, 0}, {}, wire::PLANNING_TRANSPORT_GRPC, [&] {
    return std::string(16, ++new_entropy_calls == 1 ? 'j' : 'u');
  });
  const auto current_request = open_request(replacement, 'a');
  EXPECT_EQ(current_request.opening_domain().generation(), old_request.opening_domain().generation());
  EXPECT_EQ(current_request.holder_nonce(), old_request.holder_nonce());
  EXPECT_EQ(current_request.open_request_id(), old_request.open_request_id());
  EXPECT_NE(current_request.opening_domain().service_instance_id(), old_request.opening_domain().service_instance_id());
  wire::OpenSessionRequest replay;
  ASSERT_TRUE(replay.ParseFromString(old_bytes));
  const auto rejected = replacement.open(1, replay, 2);
  EXPECT_EQ(rejected.result(), wire::SESSION_OPERATION_RESULT_REJECTED);
  EXPECT_EQ(rejected.failure_reason(), "OPEN_ADMISSION_DOMAIN_REQUIRED_OR_STALE");
  EXPECT_FALSE(rejected.has_session());
  EXPECT_EQ(replacement.snapshot().epoch, 0U);
  const auto fresh = replacement.open(1, current_request, 3);
  ASSERT_EQ(fresh.result(), wire::SESSION_OPERATION_RESULT_OPENING);
  const auto current_epoch = replacement.snapshot().epoch;
  EXPECT_EQ(replacement.keep(1, keep_request(replay, 1), 4).result(), wire::SESSION_OPERATION_RESULT_REJECTED);
  EXPECT_EQ(replacement.close(1, close_request(replay)).result(), wire::SESSION_OPERATION_RESULT_REJECTED);
  EXPECT_EQ(replacement.snapshot().epoch, current_epoch);
  EXPECT_EQ(replacement.snapshot().session.session_token(), fresh.session().session_token());
  EXPECT_EQ(replay.SerializeAsString(), old_bytes);
}

TEST(NetworkIngress, ConcurrentRequestsDuringRevocationCannotClaimOrPublishAndLateCompletionCannotChangeReplacement) {
  PlanningIngress ingress(IngressResources{4096, 2, 0}, {}, wire::PLANNING_TRANSPORT_GRPC);
  const auto request = open_request(ingress, 'a');
  const auto opened = ingress.open(1, request, 1);
  ASSERT_EQ(opened.result(), wire::SESSION_OPERATION_RESULT_OPENING);
  const auto epoch = ingress.snapshot().epoch;
  ASSERT_TRUE(ingress.bind_publisher(epoch, std::string(24, 'g'), "rmw_fastrtps_cpp"));
  ASSERT_TRUE(ingress.set_admitted(epoch, true));
  ASSERT_EQ(ingress.submit(0, command(opened.session().session_token()), 2, 123).forwarding_state(),
            wire::FORWARDING_STATE_PENDING);
  ASSERT_TRUE(ingress.revoke_if_current(epoch, "RPC_DONE"));
  const auto revoked_epoch = ingress.snapshot().epoch;
  std::promise<void> start;
  auto started = start.get_future().share();
  auto retry = std::async(std::launch::async, [&] {
    started.wait();
    return ingress.open(1, request, 3);
  });
  auto competitor = std::async(std::launch::async, [&] {
    started.wait();
    return ingress.open(2, request, 3);
  });
  auto completion = std::async(std::launch::async, [&] {
    started.wait();
    return ingress.publish_pending(epoch, 3, [](const auto &) { FAIL() << "revoked epoch published"; });
  });
  start.set_value();
  for (const auto &response : {retry.get(), competitor.get()}) {
    EXPECT_EQ(response.result(), wire::SESSION_OPERATION_RESULT_BUSY);
    EXPECT_FALSE(response.has_session());
  }
  EXPECT_FALSE(completion.get());
  EXPECT_EQ(ingress.snapshot().epoch, revoked_epoch);
  EXPECT_FALSE(ingress.cancel_pending_submission(epoch, 123));
  ASSERT_TRUE(ingress.finish_release(revoked_epoch));
  const auto replacement = ingress.open(2, open_request(ingress, 'a'), 4);
  ASSERT_EQ(replacement.result(), wire::SESSION_OPERATION_RESULT_OPENING);
  const auto replacement_epoch = ingress.snapshot().epoch;
  ASSERT_TRUE(ingress.bind_publisher(replacement_epoch, std::string(24, 'n'), "rmw_fastrtps_cpp"));
  ASSERT_TRUE(ingress.set_admitted(replacement_epoch, true));
  auto old_target = command(opened.session().session_token());
  old_target.source_stamp_ns += 1000000000;
  old_target.command_sequence = 100;
  EXPECT_EQ(ingress.submit(0, old_target, 5, 123).forwarding_state(), wire::FORWARDING_STATE_REJECTED);
  auto fresh = command(replacement.session().session_token());
  fresh.command_sequence = 1;
  ASSERT_EQ(ingress.submit(0, fresh, 6, 123).forwarding_state(), wire::FORWARDING_STATE_PENDING);
  EXPECT_FALSE(ingress.revoke_if_current(epoch, "LATE_DONE"));
  EXPECT_FALSE(ingress.finish_release(revoked_epoch));
  EXPECT_FALSE(ingress.cancel_pending_submission(epoch, 123));
  EXPECT_FALSE(ingress.bind_publisher(epoch, std::string(24, 'g'), "rmw_fastrtps_cpp"));
  EXPECT_FALSE(ingress.set_admitted(epoch, true));
  EXPECT_FALSE(ingress.publish_pending(epoch, 7, [](const auto &) { FAIL() << "late epoch published"; }));
  ASSERT_TRUE(ingress.publish_pending(replacement_epoch, 8, [&](const auto &forwarded) {
    EXPECT_EQ(forwarded.command_sequence, 1U);
    EXPECT_EQ(forwarded.source_stamp_ns, fresh.source_stamp_ns);
    EXPECT_EQ(forwarded.session_token.bytes, fresh.session_token.bytes);
  }));
  EXPECT_EQ(ingress.snapshot().state, wire::CONTROL_SESSION_STATE_ADMITTED);
  EXPECT_EQ(ingress.snapshot().epoch, replacement_epoch);
}

TEST(NetworkIngress, StopPermanentlyRejectsOpeningEvenAfterResourceReleaseForBothTransports) {
  for (const auto transport : {wire::PLANNING_TRANSPORT_SOMEIP, wire::PLANNING_TRANSPORT_GRPC}) {
    SCOPED_TRACE(transport);
    PlanningIngress ingress(IngressResources{4096, 2, 5000}, {}, transport);
    const auto request = open_request(ingress, 'a');
    const auto opened = ingress.open(0x3101, request, 1);
    ASSERT_EQ(opened.result(), wire::SESSION_OPERATION_RESULT_OPENING);
    const auto epoch = ingress.snapshot().epoch;
    ASSERT_TRUE(ingress.bind_publisher(epoch, std::string(24, 'g'), "rmw_fastrtps_cpp"));
    ASSERT_TRUE(ingress.set_admitted(epoch, true));
    ASSERT_EQ(ingress.submit(0x3101, command(opened.session().session_token()), 2, 1).forwarding_state(),
              wire::FORWARDING_STATE_PENDING);
    ingress.stop();
    const auto stopped = ingress.snapshot();
    EXPECT_EQ(stopped.state, wire::CONTROL_SESSION_STATE_REVOKING);
    EXPECT_EQ(stopped.failure, "STOPPED");
    EXPECT_FALSE(stopped.session.submission_ready());
    ingress.stop();
    EXPECT_EQ(ingress.snapshot().epoch, stopped.epoch);
    EXPECT_FALSE(ingress.set_admitted(epoch, true));
    EXPECT_FALSE(ingress.publish_pending(epoch, 3, [](const auto &) { FAIL() << "stopped epoch published"; }));
    EXPECT_FALSE(ingress.cancel_pending_submission(epoch, 1));
    EXPECT_EQ(ingress.submit(0x3101, command(opened.session().session_token()), 4).forwarding_state(),
              wire::FORWARDING_STATE_REJECTED);
    ASSERT_TRUE(ingress.finish_release(stopped.epoch));
    const auto fresh = open_request(ingress, 'a');
    for (const auto &candidate : {request, fresh}) {
      const auto rejected = ingress.open(0x3101, candidate, 5);
      EXPECT_EQ(rejected.result(), wire::SESSION_OPERATION_RESULT_REJECTED);
      EXPECT_EQ(rejected.failure_reason(), "STOPPED");
      EXPECT_FALSE(rejected.has_session());
    }
    PlanningIngress idle(IngressResources{4096, 2, 5000}, {}, transport);
    idle.stop();
    const auto rejected = idle.open(0x3101, open_request(idle, 'a'), 1);
    EXPECT_EQ(rejected.result(), wire::SESSION_OPERATION_RESULT_REJECTED);
    EXPECT_EQ(rejected.failure_reason(), "STOPPED");
    EXPECT_FALSE(rejected.has_session());
  }
}

TEST(NetworkIngress, AdmissionGenerationBoundaryDoesNotWrapOrReopenAndRetirementStaysBounded) {
  const IngressResources resources{4096, 1, 0, 2};
  PlanningIngress ingress(resources, {}, wire::PLANNING_TRANSPORT_GRPC);
  for (std::uint64_t generation = 1; generation <= resources.admission_generation_limit; ++generation) {
    SCOPED_TRACE(generation);
    const auto request = open_request(ingress, 'a');
    ASSERT_EQ(request.opening_domain().generation(), generation);
    const auto opened = ingress.open(1, request, generation);
    ASSERT_EQ(opened.result(), wire::SESSION_OPERATION_RESULT_OPENING);
    const auto epoch = ingress.snapshot().epoch;
    ASSERT_TRUE(ingress.bind_publisher(epoch, std::string(24, 'g'), "rmw_fastrtps_cpp"));
    ASSERT_TRUE(ingress.set_admitted(epoch, true));
    ASSERT_EQ(ingress.submit(0, command(opened.session().session_token()), generation, generation).forwarding_state(),
              wire::FORWARDING_STATE_PENDING);
    ASSERT_EQ(ingress.close(1, close_request(request)).result(), wire::SESSION_OPERATION_RESULT_REVOKED);
    const auto revoked_epoch = ingress.snapshot().epoch;
    if (generation < resources.admission_generation_limit) {
      ASSERT_TRUE(ingress.finish_release(revoked_epoch));
      EXPECT_EQ(ingress.snapshot().retired_open_count, 0U);
      continue;
    }
    EXPECT_FALSE(ingress.finish_release(revoked_epoch));
    const auto exhausted = ingress.snapshot();
    EXPECT_EQ(exhausted.state, wire::CONTROL_SESSION_STATE_REVOKING);
    EXPECT_EQ(exhausted.failure, "ADMISSION_GENERATION_EXHAUSTED");
    EXPECT_FALSE(exhausted.session.submission_ready());
    EXPECT_EQ(exhausted.opening_domain.generation(), generation);
    EXPECT_EQ(exhausted.retired_open_count, 1U);
    EXPECT_LE(exhausted.retired_open_count, resources.retired_open_capacity);
    EXPECT_FALSE(ingress.finish_release(revoked_epoch));
    EXPECT_EQ(ingress.snapshot().retired_open_count, exhausted.retired_open_count);
    EXPECT_FALSE(ingress.bind_publisher(epoch, std::string(24, 'g'), "rmw_fastrtps_cpp"));
    EXPECT_FALSE(ingress.set_admitted(epoch, true));
    EXPECT_FALSE(ingress.set_admitted(revoked_epoch, true));
    EXPECT_FALSE(ingress.cancel_pending_submission(epoch, generation));
    EXPECT_FALSE(ingress.publish_pending(epoch, generation + 1, [](const auto &) { FAIL() << "exhausted epoch published"; }));
    EXPECT_FALSE(ingress.publish_pending(revoked_epoch, generation + 1, [](const auto &) { FAIL() << "revoked epoch published"; }));
    const auto rejected = ingress.open(2, open_request(ingress, 'b'), generation + 1);
    EXPECT_EQ(rejected.result(), wire::SESSION_OPERATION_RESULT_REJECTED);
    EXPECT_FALSE(rejected.has_session());
    EXPECT_EQ(ingress.snapshot().failure, "ADMISSION_GENERATION_EXHAUSTED");
    EXPECT_EQ(ingress.snapshot().opening_domain.SerializeAsString(), exhausted.opening_domain.SerializeAsString());
  }
  EXPECT_THROW((PlanningIngress(IngressResources{4096, 0, 0}, {}, wire::PLANNING_TRANSPORT_GRPC)),
               std::invalid_argument);
  EXPECT_THROW((PlanningIngress(IngressResources{4096, 1, 0, 0}, {}, wire::PLANNING_TRANSPORT_GRPC)),
               std::invalid_argument);
}

TEST(NetworkIngress, InvalidStartupEntropyFailsBeforeAnAdmissionDomainCanBePublished) {
  for (const auto &identity : {std::string{}, std::string("short"), std::string(16, '\0')}) {
    EXPECT_THROW((PlanningIngress(IngressResources{4096, 2, 0}, {}, wire::PLANNING_TRANSPORT_GRPC,
                                  [identity] { return identity; })), std::runtime_error);
  }
  const std::function<std::string()> failing_source = []() -> std::string {
    throw std::runtime_error("fixture entropy unavailable");
  };
  EXPECT_THROW((PlanningIngress(IngressResources{4096, 2, 0}, {}, wire::PLANNING_TRANSPORT_GRPC,
                                failing_source)), std::runtime_error);
}

TEST(NetworkIngress, FailedTokenEntropyCannotClaimHolderOrAdvanceAdmissionDomainAndExplicitRetryCanRecover) {
  for (const auto failure : {0, 1, 2}) {
    SCOPED_TRACE(failure);
    std::uint64_t calls = 0;
    PlanningIngress ingress(IngressResources{4096, 2, 0}, {}, wire::PLANNING_TRANSPORT_GRPC, [&]() -> std::string {
      ++calls;
      if (calls == 1) return std::string(16, 'i');
      if (calls == 2) {
        if (failure == 0) throw std::runtime_error("fixture entropy unavailable");
        if (failure == 1) return "short";
        return std::string(16, '\0');
      }
      return std::string(16, 't');
    });
    const auto request = open_request(ingress, 'a');
    const auto before = ingress.snapshot();
    const auto rejected = ingress.open(1, request, 1);
    EXPECT_EQ(rejected.result(), wire::SESSION_OPERATION_RESULT_REJECTED);
    EXPECT_EQ(rejected.failure_reason(), "RANDOM_SOURCE_FAILED");
    EXPECT_FALSE(rejected.has_session());
    const auto failed = ingress.snapshot();
    EXPECT_EQ(failed.failure, "RANDOM_SOURCE_FAILED");
    EXPECT_EQ(failed.state, wire::CONTROL_SESSION_STATE_IDLE);
    EXPECT_TRUE(failed.session.session_token().empty());
    EXPECT_EQ(failed.epoch, before.epoch);
    EXPECT_EQ(failed.opening_domain.SerializeAsString(), before.opening_domain.SerializeAsString());
    EXPECT_EQ(failed.retired_open_count, 0U);
    EXPECT_FALSE(ingress.bind_publisher(failed.epoch, std::string(24, 'g'), "rmw_fastrtps_cpp"));
    EXPECT_FALSE(ingress.set_admitted(failed.epoch, true));
    EXPECT_EQ(ingress.submit(0, command(std::string(16, 't')), 2).forwarding_state(), wire::FORWARDING_STATE_REJECTED);
    const auto retry = ingress.open(1, request, 3);
    EXPECT_EQ(retry.result(), wire::SESSION_OPERATION_RESULT_OPENING);
    EXPECT_EQ(retry.session().session_token(), std::string(16, 't'));
    EXPECT_EQ(ingress.snapshot().epoch, before.epoch + 1);
    EXPECT_EQ(ingress.opening_domain().SerializeAsString(), request.opening_domain().SerializeAsString());
    EXPECT_EQ(calls, 3U);
  }
}

}
}
