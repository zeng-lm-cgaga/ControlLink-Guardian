#include "control_link_someip_transport/ingress.hpp"

#include <linux/filter.h>
#include <linux/seccomp.h>
#include <sys/prctl.h>
#include <sys/syscall.h>
#include <sys/wait.h>
#include <unistd.h>

#include <cerrno>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <filesystem>
#include <future>
#include <limits>
#include <stdexcept>
#include <string>
#include <thread>

#include <gtest/gtest.h>

namespace control_link::someip {
namespace {

namespace codec = control_link_edge_protocol;

Config fixture_config() {
#ifdef CONTROL_LINK_PROTOCOL_CONFIG_PATH
  return load_config(CONTROL_LINK_PROTOCOL_CONFIG_PATH);
#else
  return load_config((std::filesystem::path(__FILE__).parent_path().parent_path().parent_path() /
                      "config/adas/protocol_transports.yaml").string());
#endif
}

wire::Capabilities fixture_capabilities() {
  wire::Capabilities capabilities;
  capabilities.set_schema_version(1);
  capabilities.set_contract_id("control-link-fixture");
  capabilities.set_planning_transport(wire::PLANNING_TRANSPORT_SOMEIP);
  return capabilities;
}

wire::OpenSessionRequest open_request(char identity = 'a') {
  wire::OpenSessionRequest request;
  request.set_schema_version(1);
  request.set_holder_nonce(std::string(16, identity));
  request.set_open_request_id(std::string(16, identity + 1));
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

wire::CloseSessionRequest close_request(const wire::OpenSessionRequest &open) {
  wire::CloseSessionRequest request;
  request.set_schema_version(1);
  request.set_holder_nonce(open.holder_nonce());
  request.set_open_request_id(open.open_request_id());
  return request;
}

codec::CommandFrame command(const std::string &token, std::uint64_t sequence = 7) {
  codec::CommandFrame frame;
  if (token.size() != frame.session_token.bytes.size()) throw std::invalid_argument("invalid fixture token");
  std::memcpy(frame.session_token.bytes.data(), token.data(), token.size());
  frame.command_sequence = sequence;
  frame.source_stamp_ns = 1700000000123456789;
  frame.linear_velocity_mps = 0.5;
  return frame;
}

class SomeipIngress : public ::testing::Test {
 protected:
  SomeipIngress() : config(fixture_config()), ingress(config, fixture_capabilities()) {}

  void admit() {
    const auto opened = ingress.open(0x3101, request, initial_ns);
    ASSERT_EQ(opened.result(), wire::SESSION_OPERATION_RESULT_OPENING);
    epoch = ingress.snapshot().epoch;
    token = opened.session().session_token();
    ASSERT_TRUE(ingress.bind_publisher(epoch, gid, "rmw_fastrtps_cpp"));
    ASSERT_TRUE(ingress.set_admitted(epoch, true));
  }

  Config config;
  PlanningIngress ingress;
  wire::OpenSessionRequest request = open_request();
  const std::string gid = std::string(24, 'g');
  const std::uint64_t initial_ns = 1000000000;
  std::uint64_t epoch{0};
  std::string token;
};

TEST_F(SomeipIngress, SameHolderRetriesAreIdempotentWithoutRotatingOrExtendingIdle) {
  const auto first = ingress.open(0x3101, request, initial_ns);
  ASSERT_EQ(first.result(), wire::SESSION_OPERATION_RESULT_OPENING);
  const auto first_epoch = ingress.snapshot().epoch;
  const auto retry = ingress.open(0x3101, request, initial_ns + 4000000000);
  EXPECT_EQ(retry.result(), wire::SESSION_OPERATION_RESULT_IDEMPOTENT);
  EXPECT_EQ(retry.session().session_token(), first.session().session_token());
  EXPECT_FALSE(retry.session().submission_ready());
  EXPECT_EQ(ingress.snapshot().epoch, first_epoch);
  ASSERT_TRUE(ingress.bind_publisher(first_epoch, gid, "rmw_fastrtps_cpp"));
  ASSERT_TRUE(ingress.set_admitted(first_epoch, true));
  const auto admitted_retry = ingress.open(0x3101, request, initial_ns + 4999000000);
  EXPECT_EQ(admitted_retry.result(), wire::SESSION_OPERATION_RESULT_IDEMPOTENT);
  EXPECT_EQ(admitted_retry.session().ingress_publisher().gid(), gid);
  EXPECT_EQ(admitted_retry.session().session_token(), first.session().session_token());
  ingress.expire(initial_ns + 5000000000);
  EXPECT_EQ(ingress.snapshot().state, wire::CONTROL_SESSION_STATE_REVOKING);
  EXPECT_EQ(ingress.snapshot().epoch, first_epoch + 1);
}

TEST_F(SomeipIngress, CompetitorOrChangedKeyIsBusyWithoutLeakingOrChangingSession) {
  admit();
  const auto competitor = ingress.open(0x3102, request, initial_ns + 1);
  EXPECT_EQ(competitor.result(), wire::SESSION_OPERATION_RESULT_BUSY);
  EXPECT_FALSE(competitor.has_session());
  EXPECT_EQ(competitor.open_request_id(), request.open_request_id());
  const auto changed = ingress.open(0x3101, open_request('c'), initial_ns + 2);
  EXPECT_EQ(changed.result(), wire::SESSION_OPERATION_RESULT_BUSY);
  EXPECT_FALSE(changed.has_session());
  EXPECT_EQ(ingress.snapshot().epoch, epoch);
  EXPECT_EQ(ingress.snapshot().session.session_token(), token);
}

TEST_F(SomeipIngress, InvalidOpenPresenceLengthsAndPeerNeverClaimsHolder) {
  auto invalid_request = request;
  invalid_request.clear_schema_version();
  EXPECT_EQ(ingress.open(0x3101, invalid_request, initial_ns).result(), wire::SESSION_OPERATION_RESULT_REJECTED);
  invalid_request = request;
  invalid_request.set_holder_nonce("short");
  EXPECT_EQ(ingress.open(0x3101, invalid_request, initial_ns).result(), wire::SESSION_OPERATION_RESULT_REJECTED);
  invalid_request = request;
  invalid_request.set_schema_version(2);
  EXPECT_EQ(ingress.open(0x3101, invalid_request, initial_ns).result(), wire::SESSION_OPERATION_RESULT_REJECTED);
  EXPECT_EQ(ingress.open(0, request, initial_ns).result(), wire::SESSION_OPERATION_RESULT_REJECTED);
  EXPECT_EQ(ingress.snapshot().state, wire::CONTROL_SESSION_STATE_IDLE);
  EXPECT_EQ(ingress.snapshot().epoch, 0);
}

TEST_F(SomeipIngress, OpeningKeepIsIndependentStrictlyIncreasingAndDoesNotAdmit) {
  ASSERT_EQ(ingress.open(0x3101, request, initial_ns).result(), wire::SESSION_OPERATION_RESULT_OPENING);
  const auto first_epoch = ingress.snapshot().epoch;
  const auto first_token = ingress.snapshot().session.session_token();
  const auto keep = keep_request(request, 1);
  EXPECT_EQ(ingress.keep(0x3101, keep, initial_ns + 4000000000).result(), wire::SESSION_OPERATION_RESULT_OPENING);
  EXPECT_EQ(ingress.keep(0x3101, keep, initial_ns + 8000000000).result(), wire::SESSION_OPERATION_RESULT_REJECTED);
  EXPECT_EQ(ingress.keep(0x3102, keep_request(request, 2), initial_ns + 8000000000).result(), wire::SESSION_OPERATION_RESULT_REJECTED);
  ingress.expire(initial_ns + 8999000000);
  EXPECT_EQ(ingress.snapshot().state, wire::CONTROL_SESSION_STATE_OPENING);
  EXPECT_FALSE(ingress.snapshot().session.submission_ready());
  EXPECT_EQ(ingress.snapshot().epoch, first_epoch);
  EXPECT_EQ(ingress.snapshot().session.session_token(), first_token);
  ingress.expire(initial_ns + 9000000000);
  EXPECT_EQ(ingress.snapshot().state, wire::CONTROL_SESSION_STATE_REVOKING);
}

TEST_F(SomeipIngress, LateKeepCannotReviveExpiredHolderAndTimeRegressionFailsClosed) {
  admit();
  const auto late = ingress.keep(0x3101, keep_request(request, 1), initial_ns + 5000000000);
  EXPECT_EQ(late.result(), wire::SESSION_OPERATION_RESULT_REJECTED);
  EXPECT_FALSE(late.has_session());
  EXPECT_EQ(ingress.snapshot().state, wire::CONTROL_SESSION_STATE_REVOKING);
  ASSERT_TRUE(ingress.finish_release(ingress.snapshot().epoch));
  ASSERT_EQ(ingress.open(0x3101, open_request('c'), initial_ns + 6000000000).result(), wire::SESSION_OPERATION_RESULT_OPENING);
  ingress.expire(initial_ns + 5999999999);
  EXPECT_EQ(ingress.snapshot().state, wire::CONTROL_SESSION_STATE_REVOKING);
  EXPECT_EQ(ingress.snapshot().failure, "STEADY_CLOCK_REGRESSION");
}

TEST_F(SomeipIngress, PublisherBindingCannotChangeGenerationWithinEpoch) {
  const auto opened = ingress.open(0x3101, request, initial_ns);
  ASSERT_EQ(opened.result(), wire::SESSION_OPERATION_RESULT_OPENING);
  const auto current = ingress.snapshot().epoch;
  EXPECT_FALSE(ingress.set_admitted(current, true));
  EXPECT_FALSE(ingress.bind_publisher(current, "short", "rmw_fastrtps_cpp"));
  EXPECT_FALSE(ingress.bind_publisher(current, gid, ""));
  ASSERT_TRUE(ingress.bind_publisher(current, gid, "rmw_fastrtps_cpp"));
  EXPECT_TRUE(ingress.bind_publisher(current, gid, "rmw_fastrtps_cpp"));
  EXPECT_FALSE(ingress.bind_publisher(current, std::string(24, 'h'), "rmw_fastrtps_cpp"));
  EXPECT_FALSE(ingress.bind_publisher(current, gid, "other_rmw"));
  EXPECT_EQ(ingress.snapshot().session.ingress_publisher().gid(), gid);
}

TEST_F(SomeipIngress, LatestSlotForwardsOriginalValuesOnceAndReportsOverwrite) {
  admit();
  const auto old = command(token, 7);
  auto latest = command(token, 8);
  latest.source_stamp_ns = old.source_stamp_ns - 1;
  EXPECT_EQ(ingress.submit(0x3101, old, 10).forwarding_state(), wire::FORWARDING_STATE_PENDING);
  const auto receipt = ingress.submit(0x3101, latest, 11);
  EXPECT_EQ(receipt.forwarding_state(), wire::FORWARDING_STATE_PENDING);
  EXPECT_EQ(receipt.gateway_acceptance().state(), wire::GATEWAY_ACCEPTANCE_STATE_UNKNOWN);
  EXPECT_TRUE(receipt.rpc_received());
  EXPECT_EQ(receipt.rpc_received_boottime_ns(), 11);
  std::size_t calls = 0;
  EXPECT_TRUE(ingress.publish_pending(epoch, 12, [&](const auto &forwarded) {
    ++calls;
    EXPECT_EQ(forwarded.source_stamp_ns, latest.source_stamp_ns);
    EXPECT_EQ(forwarded.command_sequence, 8);
    EXPECT_EQ(forwarded.session_token, latest.session_token);
  }));
  EXPECT_FALSE(ingress.publish_pending(epoch, 13, [&](const auto &) { ++calls; }));
  EXPECT_EQ(calls, 1);
  const auto status = ingress.readonly_status();
  EXPECT_EQ(status.transport().forwarded_count(), 1);
  EXPECT_EQ(status.transport().dropped_count(), 1);
  EXPECT_EQ(status.transport().missed_count(), 1);
  EXPECT_EQ(status.latest_forwarded().key().source_sequence(), 8);
  EXPECT_EQ(status.latest_forwarded().forwarded_boottime_ns(), 12);
}

TEST_F(SomeipIngress, BusinessModeFiniteSequenceAndAgeArePreservedForGatewayOwner) {
  admit();
  auto frame = command(token, 0);
  frame.source_stamp_ns = 0;
  frame.mode = static_cast<codec::CommandMode>(255);
  frame.linear_velocity_mps = std::numeric_limits<double>::quiet_NaN();
  frame.angular_velocity_radps = std::numeric_limits<double>::infinity();
  EXPECT_EQ(ingress.submit(0x3101, frame, 10).forwarding_state(), wire::FORWARDING_STATE_PENDING);
  ASSERT_TRUE(ingress.publish_pending(epoch, 11, [&](const auto &forwarded) {
    EXPECT_EQ(forwarded.command_sequence, 0);
    EXPECT_EQ(forwarded.source_stamp_ns, 0);
    EXPECT_EQ(static_cast<std::uint8_t>(forwarded.mode), 255);
    EXPECT_TRUE(std::isnan(forwarded.linear_velocity_mps));
    EXPECT_TRUE(std::isinf(forwarded.angular_velocity_radps));
  }));
  frame = command(token, 7);
  ASSERT_EQ(ingress.submit(0x3101, frame, 12).forwarding_state(), wire::FORWARDING_STATE_PENDING);
  ASSERT_TRUE(ingress.publish_pending(epoch, 13, [](const auto &) {}));
  EXPECT_EQ(ingress.submit(0x3101, frame, 14).forwarding_state(), wire::FORWARDING_STATE_PENDING);
  frame.command_sequence = 6;
  EXPECT_EQ(ingress.submit(0x3101, frame, 15).forwarding_state(), wire::FORWARDING_STATE_PENDING);
}

TEST_F(SomeipIngress, WrongPeerTokenClockAndUnrepresentableStampDoNotOverwritePending) {
  admit();
  const auto valid = command(token, 7);
  ASSERT_EQ(ingress.submit(0x3101, valid, 10).forwarding_state(), wire::FORWARDING_STATE_PENDING);
  auto frame = command(std::string(16, 'x'), 8);
  frame.source_stamp_ns += 1000000000;
  const auto wrong = ingress.submit(0x3101, frame, 11);
  EXPECT_EQ(wrong.forwarding_state(), wire::FORWARDING_STATE_REJECTED);
  EXPECT_EQ(wrong.key().session_token(), std::string(16, 'x'));
  EXPECT_EQ(wrong.gateway_acceptance().state(), wire::GATEWAY_ACCEPTANCE_STATE_UNKNOWN);
  EXPECT_EQ(ingress.submit(0x3102, valid, 12).forwarding_state(), wire::FORWARDING_STATE_REJECTED);
  frame = valid;
  frame.clock_domain = static_cast<codec::ClockDomain>(1);
  EXPECT_EQ(ingress.submit(0x3101, frame, 13).forwarding_state(), wire::FORWARDING_STATE_REJECTED);
  frame = valid;
  frame.source_stamp_ns = std::numeric_limits<std::int64_t>::max();
  EXPECT_EQ(ingress.submit(0x3101, frame, 14).forwarding_state(), wire::FORWARDING_STATE_REJECTED);
  ASSERT_TRUE(ingress.publish_pending(epoch, 15, [&](const auto &forwarded) {
    EXPECT_EQ(forwarded.command_sequence, 7);
    EXPECT_EQ(forwarded.source_stamp_ns, valid.source_stamp_ns);
  }));
}

TEST_F(SomeipIngress, SubmitReadonlyAndRejectedKeepDoNotExtendNetworkIdle) {
  admit();
  ASSERT_EQ(ingress.submit(0x3101, command(token), 10).forwarding_state(), wire::FORWARDING_STATE_PENDING);
  ingress.readonly_status();
  const auto invalid = ingress.keep(0x3102, keep_request(request, 1), initial_ns + 4000000000);
  EXPECT_EQ(invalid.result(), wire::SESSION_OPERATION_RESULT_REJECTED);
  ingress.expire(initial_ns + 5000000000);
  EXPECT_EQ(ingress.snapshot().state, wire::CONTROL_SESSION_STATE_REVOKING);
  EXPECT_FALSE(ingress.publish_pending(epoch, 11, [](const auto &) {}));
}

TEST_F(SomeipIngress, KeepNeverRestampsOrRequeuesPreviouslyForwardedBusinessTarget) {
  admit();
  const auto original = command(token, 7);
  ASSERT_EQ(ingress.submit(0x3101, original, 10).forwarding_state(), wire::FORWARDING_STATE_PENDING);
  ASSERT_TRUE(ingress.publish_pending(epoch, 11, [&](const auto &forwarded) {
    EXPECT_EQ(forwarded.source_stamp_ns, original.source_stamp_ns);
  }));
  ASSERT_EQ(ingress.keep(0x3101, keep_request(request, 1), initial_ns + 4000000000).result(), wire::SESSION_OPERATION_RESULT_ADMITTED);
  EXPECT_FALSE(ingress.publish_pending(epoch, 12, [](const auto &) { FAIL() << "Keep requeued command"; }));
  EXPECT_EQ(ingress.readonly_status().latest_forwarded().forwarded_boottime_ns(), 11);
  EXPECT_EQ(ingress.readonly_status().latest_forwarded().key().source_sequence(), 7);
  EXPECT_EQ(ingress.snapshot().session.session_token(), token);
}

TEST_F(SomeipIngress, ReadinessLossClearsPendingButPreservesHolderTokenAndGeneration) {
  admit();
  ASSERT_EQ(ingress.submit(0x3101, command(token), 10).forwarding_state(), wire::FORWARDING_STATE_PENDING);
  EXPECT_TRUE(ingress.set_admitted(epoch, false));
  EXPECT_FALSE(ingress.publish_pending(epoch, 11, [](const auto &) {}));
  const auto snapshot = ingress.snapshot();
  EXPECT_EQ(snapshot.state, wire::CONTROL_SESSION_STATE_OPENING);
  EXPECT_EQ(snapshot.epoch, epoch);
  EXPECT_EQ(snapshot.session.session_token(), token);
  EXPECT_EQ(snapshot.session.ingress_publisher().gid(), gid);
  EXPECT_EQ(ingress.submit(0x3101, command(token, 8), 12).forwarding_state(), wire::FORWARDING_STATE_REJECTED);
  ASSERT_TRUE(ingress.set_admitted(epoch, true));
  EXPECT_EQ(ingress.submit(0x3101, command(token, 8), 13).forwarding_state(), wire::FORWARDING_STATE_PENDING);
}

TEST_F(SomeipIngress, MatchingCloseRevokesOnceAndReleaseRequiresCurrentRevokedEpoch) {
  admit();
  ASSERT_EQ(ingress.submit(0x3101, command(token), 10).forwarding_state(), wire::FORWARDING_STATE_PENDING);
  EXPECT_EQ(ingress.close(0x3102, close_request(request)).result(), wire::SESSION_OPERATION_RESULT_REJECTED);
  const auto closed = ingress.close(0x3101, close_request(request));
  EXPECT_EQ(closed.result(), wire::SESSION_OPERATION_RESULT_REVOKED);
  EXPECT_FALSE(closed.session().submission_ready());
  const auto revoked_epoch = ingress.snapshot().epoch;
  EXPECT_EQ(revoked_epoch, epoch + 1);
  EXPECT_EQ(ingress.close(0x3101, close_request(request)).result(), wire::SESSION_OPERATION_RESULT_REVOKED);
  EXPECT_EQ(ingress.snapshot().epoch, revoked_epoch);
  EXPECT_FALSE(ingress.publish_pending(epoch, 11, [](const auto &) {}));
  EXPECT_FALSE(ingress.finish_release(epoch));
  EXPECT_EQ(ingress.open(0x3102, open_request('c'), initial_ns + 1).result(), wire::SESSION_OPERATION_RESULT_BUSY);
  ASSERT_TRUE(ingress.finish_release(revoked_epoch));
  EXPECT_EQ(ingress.snapshot().state, wire::CONTROL_SESSION_STATE_IDLE);
  EXPECT_TRUE(ingress.snapshot().session.session_token().empty());
  EXPECT_FALSE(ingress.finish_release(revoked_epoch));
}

TEST_F(SomeipIngress, RetiredKeysCannotReviveWithReusedPeerAndCapacityNeverEvicts) {
  auto small = config;
  small.retired_open_capacity = 2;
  PlanningIngress limited(small, fixture_capabilities());
  const auto first = open_request('a');
  const auto second = open_request('c');
  for (const auto &key : {first, second}) {
    ASSERT_EQ(limited.open(0x3101, key, initial_ns).result(), wire::SESSION_OPERATION_RESULT_OPENING);
    limited.revoke("fixture close");
    ASSERT_TRUE(limited.finish_release(limited.snapshot().epoch));
  }
  EXPECT_EQ(limited.open(0x3102, first, initial_ns).result(), wire::SESSION_OPERATION_RESULT_REJECTED);
  const auto rejected = limited.open(0x3101, open_request('e'), initial_ns);
  EXPECT_EQ(rejected.result(), wire::SESSION_OPERATION_RESULT_RESOURCE_EXHAUSTED);
  EXPECT_FALSE(rejected.has_session());
  EXPECT_NE(limited.snapshot().failure.find("RESOURCE_EXHAUSTED"), std::string::npos);
  EXPECT_EQ(limited.snapshot().state, wire::CONTROL_SESSION_STATE_IDLE);
  EXPECT_EQ(limited.open(0x3101, second, initial_ns).result(), wire::SESSION_OPERATION_RESULT_REJECTED);
}

TEST_F(SomeipIngress, OldEpochTimeoutCompletionAndOldTokenCannotKillOrUseNewSession) {
  admit();
  const auto old_epoch = epoch;
  const auto old_token = token;
  ASSERT_TRUE(ingress.revoke_if_current(old_epoch, "graph timeout"));
  const auto revoked = ingress.snapshot().epoch;
  ASSERT_TRUE(ingress.finish_release(revoked));
  const auto next = ingress.open(0x3101, open_request('c'), initial_ns + 1);
  ASSERT_EQ(next.result(), wire::SESSION_OPERATION_RESULT_OPENING);
  const auto next_epoch = ingress.snapshot().epoch;
  ASSERT_TRUE(ingress.bind_publisher(next_epoch, std::string(24, 'h'), "rmw_fastrtps_cpp"));
  ASSERT_TRUE(ingress.set_admitted(next_epoch, true));
  EXPECT_NE(next.session().session_token(), old_token);
  EXPECT_FALSE(ingress.revoke_if_current(old_epoch, "late timeout commit"));
  EXPECT_FALSE(ingress.finish_release(revoked));
  EXPECT_FALSE(ingress.bind_publisher(old_epoch, gid, "rmw_fastrtps_cpp"));
  EXPECT_FALSE(ingress.set_admitted(old_epoch, false));
  auto obsolete = command(old_token, 100);
  obsolete.source_stamp_ns += 1000000000;
  EXPECT_EQ(ingress.submit(0x3101, obsolete, 10).forwarding_state(), wire::FORWARDING_STATE_REJECTED);
  EXPECT_EQ(ingress.snapshot().state, wire::CONTROL_SESSION_STATE_ADMITTED);
  EXPECT_EQ(ingress.snapshot().epoch, next_epoch);
}

TEST_F(SomeipIngress, FinalPublishAndRevocationShareOneSynchronizedCommitBoundary) {
  admit();
  ASSERT_EQ(ingress.submit(0x3101, command(token), 10).forwarding_state(), wire::FORWARDING_STATE_PENDING);
  std::promise<void> publish_started;
  std::promise<void> release_publish;
  auto release = release_publish.get_future().share();
  auto publishing = std::async(std::launch::async, [&] {
    return ingress.publish_pending(epoch, 11, [&](const auto &) {
      publish_started.set_value();
      release.wait();
    });
  });
  publish_started.get_future().wait();
  std::promise<void> revoke_attempted;
  auto revocation = std::async(std::launch::async, [&] {
    revoke_attempted.set_value();
    return ingress.revoke_if_current(epoch, "concurrent stop");
  });
  revoke_attempted.get_future().wait();
  release_publish.set_value();
  EXPECT_TRUE(publishing.get());
  EXPECT_TRUE(revocation.get());
  EXPECT_EQ(ingress.snapshot().state, wire::CONTROL_SESSION_STATE_REVOKING);
  EXPECT_EQ(ingress.readonly_status().transport().forwarded_count(), 1);
  EXPECT_FALSE(ingress.publish_pending(epoch, 12, [](const auto &) { FAIL() << "old epoch published"; }));
  ASSERT_TRUE(ingress.finish_release(ingress.snapshot().epoch));
  EXPECT_EQ(ingress.open(0x3101, open_request('c'), initial_ns + 1).result(), wire::SESSION_OPERATION_RESULT_OPENING);
  EXPECT_FALSE(ingress.publish_pending(epoch, 13, [](const auto &) { FAIL() << "retired epoch published"; }));
}

TEST_F(SomeipIngress, PublisherExceptionRevokesAndDropsPendingFailClosed) {
  admit();
  ASSERT_EQ(ingress.submit(0x3101, command(token), 10).forwarding_state(), wire::FORWARDING_STATE_PENDING);
  EXPECT_FALSE(ingress.publish_pending(epoch, 11, [](const auto &) { throw std::runtime_error("publisher unavailable"); }));
  EXPECT_EQ(ingress.snapshot().state, wire::CONTROL_SESSION_STATE_REVOKING);
  EXPECT_EQ(ingress.snapshot().failure, "PUBLISH_FAILED");
  EXPECT_EQ(ingress.readonly_status().transport().forwarded_count(), 0);
  EXPECT_EQ(ingress.readonly_status().transport().dropped_count(), 1);
  EXPECT_EQ(ingress.submit(0x3101, command(token, 8), 12).forwarding_state(), wire::FORWARDING_STATE_REJECTED);
}

TEST_F(SomeipIngress, FinalReadinessGuardSeesEndpointUpdateAfterEarlierCheck) {
  admit();
  ingress.update_observations([](auto &status) { status.add_sources()->set_endpoint_state(5); });
  const auto usable = [](const auto &status) {
    return status.sources_size() == 1 && status.sources(0).endpoint_state() == 5;
  };
  ASSERT_TRUE(usable(ingress.readonly_status()));
  ASSERT_EQ(ingress.submit(0x3101, command(token), 10).forwarding_state(), wire::FORWARDING_STATE_PENDING);
  ingress.update_observations([](auto &status) { status.mutable_sources(0)->set_endpoint_state(1); });
  bool published = false;
  EXPECT_FALSE(ingress.publish_pending(epoch, 11, [&](const auto &) { published = true; }, usable));
  EXPECT_FALSE(published);
  EXPECT_FALSE(ingress.snapshot().session.submission_ready());
  EXPECT_EQ(ingress.snapshot().epoch, epoch);
  EXPECT_EQ(ingress.readonly_status().transport().forwarded_count(), 0);
  EXPECT_EQ(ingress.readonly_status().transport().dropped_count(), 1);
}

TEST_F(SomeipIngress, FinalAdmissionGuardChecksDeadlineAndStopAtCommit) {
  const auto opened = ingress.open(0x3101, request, initial_ns);
  epoch = ingress.snapshot().epoch;
  ASSERT_TRUE(ingress.bind_publisher(epoch, gid, "rmw_fastrtps_cpp"));
  std::uint64_t injected_now = 10;
  const std::uint64_t deadline = 20;
  bool stopping = false;
  const auto guard = [&](const auto &) { return !stopping && injected_now < deadline; };
  ASSERT_TRUE(guard(ingress.readonly_status()));
  injected_now = deadline;
  EXPECT_FALSE(ingress.set_admitted(epoch, true, guard));
  EXPECT_FALSE(ingress.snapshot().session.submission_ready());
  EXPECT_EQ(ingress.snapshot().epoch, epoch);
  injected_now = 10;
  stopping = true;
  EXPECT_FALSE(ingress.set_admitted(epoch, true, guard));
  EXPECT_FALSE(ingress.snapshot().session.submission_ready());
  stopping = false;
  EXPECT_TRUE(ingress.set_admitted(epoch, true, guard));
  EXPECT_EQ(ingress.snapshot().session.session_token(), opened.session().session_token());
}

TEST_F(SomeipIngress, ReadonlyRedactsEveryNestedControlTokenAndKeepsOriginalObservations) {
  admit();
  ASSERT_EQ(ingress.submit(0x3101, command(token), 10).forwarding_state(), wire::FORWARDING_STATE_PENDING);
  ASSERT_TRUE(ingress.publish_pending(epoch, 11, [](const auto &) {}));
  ingress.update_observations([&](auto &status) {
    auto *source = status.add_sources();
    source->set_source_id("planning");
    source->set_endpoint_stable(true);
    source->set_endpoint_state(5);
    source->set_endpoint_publisher_count(1);
    source->set_endpoint_generation_valid(true);
    source->mutable_endpoint_publisher()->set_gid(gid);
    source->mutable_endpoint_publisher()->set_rmw_implementation("rmw_fastrtps_cpp");
    source->set_last_source_sequence(7);
    source->set_accepted_count(9);
    source->mutable_observation()->set_validity(wire::OBSERVATION_VALIDITY_VALID);
    source->mutable_observation()->set_observed_system_ns(123);
    source->mutable_observation()->set_received_boottime_ns(456);
  });
  const auto status = ingress.readonly_status();
  EXPECT_TRUE(status.session().session_token().empty());
  EXPECT_TRUE(status.latest_forwarded().key().session_token().empty());
  EXPECT_TRUE(status.latest_gateway_acceptance().key().session_token().empty());
  EXPECT_EQ(status.latest_gateway_acceptance().state(), wire::GATEWAY_ACCEPTANCE_STATE_LATEST_KEY_OBSERVED);
  EXPECT_EQ(status.latest_gateway_acceptance().accepted_count(), 9);
  EXPECT_EQ(status.sources(0).observation().observed_system_ns(), 123);
  EXPECT_EQ(status.sources(0).observation().received_boottime_ns(), 456);
  EXPECT_FALSE(status.has_gateway());
  EXPECT_FALSE(status.has_vehicle());
  EXPECT_EQ(ingress.snapshot().session.session_token(), token);
  const auto again = ingress.readonly_status();
  EXPECT_EQ(again.SerializeAsString(), status.SerializeAsString());
}

TEST_F(SomeipIngress, AcceptanceRequiresExactKeyStableSingleMatchingGidAndActualAcceptedCount) {
  admit();
  ASSERT_EQ(ingress.submit(0x3101, command(token), 10).forwarding_state(), wire::FORWARDING_STATE_PENDING);
  ASSERT_TRUE(ingress.publish_pending(epoch, 11, [](const auto &) {}));
  ingress.update_observations([&](auto &status) {
    auto *source = status.add_sources();
    source->set_source_id("planning");
    source->set_endpoint_stable(true);
    source->set_endpoint_state(5);
    source->set_endpoint_publisher_count(1);
    source->set_endpoint_generation_valid(true);
    source->mutable_endpoint_publisher()->set_gid(gid);
    source->mutable_endpoint_publisher()->set_rmw_implementation("rmw_fastrtps_cpp");
    source->set_last_source_sequence(6);
    source->set_accepted_count(100);
    source->mutable_observation()->set_validity(wire::OBSERVATION_VALIDITY_VALID);
  });
  EXPECT_EQ(ingress.readonly_status().latest_gateway_acceptance().state(), wire::GATEWAY_ACCEPTANCE_STATE_UNKNOWN);
  ingress.update_observations([](auto &status) { status.mutable_sources(0)->set_last_source_sequence(7); });
  EXPECT_EQ(ingress.readonly_status().latest_gateway_acceptance().state(), wire::GATEWAY_ACCEPTANCE_STATE_LATEST_KEY_OBSERVED);
  ingress.update_observations([](auto &status) { status.mutable_sources(0)->set_accepted_count(0); });
  EXPECT_EQ(ingress.readonly_status().latest_gateway_acceptance().state(), wire::GATEWAY_ACCEPTANCE_STATE_UNKNOWN);
  ingress.update_observations([](auto &status) {
    status.mutable_sources(0)->set_accepted_count(100);
    status.mutable_sources(0)->set_endpoint_publisher_count(2);
  });
  EXPECT_EQ(ingress.readonly_status().latest_gateway_acceptance().state(), wire::GATEWAY_ACCEPTANCE_STATE_UNKNOWN);
  ingress.update_observations([](auto &status) {
    status.mutable_sources(0)->set_endpoint_publisher_count(1);
    status.mutable_sources(0)->mutable_endpoint_publisher()->set_gid(std::string(24, 'h'));
  });
  EXPECT_EQ(ingress.readonly_status().latest_gateway_acceptance().state(), wire::GATEWAY_ACCEPTANCE_STATE_UNKNOWN);
}

TEST_F(SomeipIngress, ObservationMapperCannotRewriteHolderCountersOrInventForwarding) {
  admit();
  ingress.update_observations([](auto &status) {
    status.mutable_session()->set_session_token("injected");
    status.mutable_transport()->set_forwarded_count(999);
    status.mutable_latest_forwarded()->mutable_key()->set_session_token("injected");
    status.mutable_latest_gateway_acceptance()->set_state(wire::GATEWAY_ACCEPTANCE_STATE_LATEST_KEY_OBSERVED);
  });
  const auto status = ingress.readonly_status();
  EXPECT_EQ(ingress.snapshot().session.session_token(), token);
  EXPECT_EQ(status.transport().forwarded_count(), 0);
  EXPECT_FALSE(status.has_latest_forwarded());
  EXPECT_EQ(status.latest_gateway_acceptance().state(), wire::GATEWAY_ACCEPTANCE_STATE_UNKNOWN);
}

TEST_F(SomeipIngress, CapabilitiesAreCopiedForEveryReplyAndDoNotOwnBusinessThresholds) {
  auto advertised = fixture_capabilities();
  PlanningIngress local(config, advertised);
  advertised.set_contract_id("changed outside owner");
  const auto opened = local.open(0x3101, request, initial_ns);
  EXPECT_EQ(opened.capabilities().contract_id(), "control-link-fixture");
  EXPECT_EQ(opened.capabilities().schema_version(), 1);
  EXPECT_EQ(opened.capabilities().schema_fingerprint().size(), 32);
  const auto busy = local.open(0x3102, request, initial_ns + 1);
  EXPECT_EQ(busy.capabilities().contract_id(), "control-link-fixture");
  EXPECT_FALSE(busy.has_session());
  EXPECT_EQ(busy.capabilities().schema_fingerprint(), opened.capabilities().schema_fingerprint());
}

TEST_F(SomeipIngress, RawProtocolRejectionIsCountedWithoutObservationsOrSessionMutation) {
  admit();
  const auto before = ingress.readonly_status();
  ingress.record_protocol_rejection();
  const auto after = ingress.readonly_status();
  EXPECT_EQ(after.transport().rpc_received_count(), before.transport().rpc_received_count() + 1);
  EXPECT_EQ(after.transport().protocol_rejected_count(), before.transport().protocol_rejected_count() + 1);
  EXPECT_EQ(ingress.snapshot().epoch, epoch);
  EXPECT_EQ(ingress.snapshot().session.session_token(), token);
  EXPECT_FALSE(after.has_gateway());
  EXPECT_FALSE(after.has_vehicle());
}

TEST_F(SomeipIngress, RandomSourceFailureCannotAllocateSession) {
  const auto child = ::fork();
  ASSERT_GE(child, 0);
  if (child == 0) {
    sock_filter filters[] = {
        BPF_STMT(BPF_LD | BPF_W | BPF_ABS, offsetof(seccomp_data, nr)),
        BPF_JUMP(BPF_JMP | BPF_JEQ | BPF_K, __NR_getrandom, 0, 1),
        BPF_STMT(BPF_RET | BPF_K, SECCOMP_RET_ERRNO | EPERM),
        BPF_STMT(BPF_RET | BPF_K, SECCOMP_RET_ALLOW)};
    sock_fprog program{static_cast<unsigned short>(std::size(filters)), filters};
    if (::prctl(PR_SET_NO_NEW_PRIVS, 1, 0, 0, 0) != 0 ||
        ::prctl(PR_SET_SECCOMP, SECCOMP_MODE_FILTER, &program) != 0) ::_exit(2);
    const auto opened = ingress.open(0x3101, request, initial_ns);
    const auto state = ingress.snapshot();
    ::_exit(opened.result() == wire::SESSION_OPERATION_RESULT_REJECTED && !opened.has_session() &&
            state.state == wire::CONTROL_SESSION_STATE_IDLE && state.epoch == 0 &&
            state.failure == "RANDOM_SOURCE_FAILED" ? 0 : 1);
  }
  int status = 0;
  ASSERT_EQ(::waitpid(child, &status, 0), child);
  ASSERT_TRUE(WIFEXITED(status));
  EXPECT_EQ(WEXITSTATUS(status), 0) << "2 means fault-injection setup failed, not a passing random-source test";
}

TEST_F(SomeipIngress, ClocksAndRandomIdentityUseDistinctDeclaredDomains) {
  EXPECT_GT(steady_now_ns(), 0);
  EXPECT_GT(boottime_now_ns(), 0);
  EXPECT_GT(system_now_ns(), 0);
  EXPECT_EQ(random_identity().size(), 16);
}

TEST_F(SomeipIngress, RejectedHolderOperationsKeepPendingAndNeverDiscloseHolderToken) {
  admit();
  const auto original = command(token);
  ASSERT_EQ(ingress.submit(0x3101, original, 10).forwarding_state(), wire::FORWARDING_STATE_PENDING);
  const auto changed_key = open_request('k');
  const auto wrong_peer_keep = ingress.keep(0x3102, keep_request(request, 1), initial_ns + 1);
  const auto changed_key_keep = ingress.keep(0x3101, keep_request(changed_key, 1), initial_ns + 2);
  const auto wrong_peer_close = ingress.close(0x3102, close_request(request));
  const auto changed_key_close = ingress.close(0x3101, close_request(changed_key));
  const auto busy = ingress.open(0x3102, changed_key, initial_ns + 3);
  for (const auto &response : {wrong_peer_keep, changed_key_keep, wrong_peer_close, changed_key_close}) {
    EXPECT_EQ(response.result(), wire::SESSION_OPERATION_RESULT_REJECTED);
    EXPECT_FALSE(response.has_session());
    EXPECT_EQ(response.SerializeAsString().find(token), std::string::npos);
  }
  EXPECT_EQ(busy.result(), wire::SESSION_OPERATION_RESULT_BUSY);
  EXPECT_FALSE(busy.has_session());
  EXPECT_EQ(busy.SerializeAsString().find(token), std::string::npos);
  EXPECT_EQ(ingress.snapshot().epoch, epoch);
  EXPECT_EQ(ingress.snapshot().session.session_token(), token);
  ASSERT_TRUE(ingress.publish_pending(epoch, 11, [&](const auto &forwarded) {
    EXPECT_EQ(forwarded.command_sequence, original.command_sequence);
    EXPECT_EQ(forwarded.source_stamp_ns, original.source_stamp_ns);
    EXPECT_EQ(forwarded.session_token, original.session_token);
  }));
  EXPECT_EQ(ingress.readonly_status().transport().dropped_count(), 0);
}

TEST_F(SomeipIngress, RevocationBeforeCommitCannotPublishOrDropReplacementEpochTarget) {
  admit();
  const auto old_epoch = epoch;
  const auto old_token = token;
  ASSERT_EQ(ingress.submit(0x3101, command(old_token), 10).forwarding_state(), wire::FORWARDING_STATE_PENDING);
  ASSERT_TRUE(ingress.revoke_if_current(old_epoch, "stop before commit"));
  bool old_published = false;
  EXPECT_FALSE(ingress.publish_pending(old_epoch, 11, [&](const auto &) { old_published = true; }));
  EXPECT_FALSE(old_published);
  ASSERT_TRUE(ingress.finish_release(ingress.snapshot().epoch));
  const auto replacement = ingress.open(0x3101, open_request('k'), initial_ns + 1);
  ASSERT_EQ(replacement.result(), wire::SESSION_OPERATION_RESULT_OPENING);
  const auto replacement_epoch = ingress.snapshot().epoch;
  const auto replacement_gid = std::string(24, 'n');
  ASSERT_TRUE(ingress.bind_publisher(replacement_epoch, replacement_gid, "rmw_fastrtps_cpp"));
  ASSERT_TRUE(ingress.set_admitted(replacement_epoch, true));
  const auto next = command(replacement.session().session_token(), 1);
  ASSERT_EQ(ingress.submit(0x3101, next, 12).forwarding_state(), wire::FORWARDING_STATE_PENDING);
  EXPECT_FALSE(ingress.publish_pending(old_epoch, 13, [&](const auto &) { old_published = true; }));
  EXPECT_FALSE(ingress.revoke_if_current(old_epoch, "late stop completion"));
  EXPECT_FALSE(ingress.set_admitted(old_epoch, false));
  EXPECT_FALSE(old_published);
  ASSERT_TRUE(ingress.publish_pending(replacement_epoch, 14, [&](const auto &forwarded) {
    EXPECT_EQ(forwarded.command_sequence, 1);
    EXPECT_EQ(forwarded.session_token, next.session_token);
    EXPECT_EQ(forwarded.source_stamp_ns, next.source_stamp_ns);
  }));
  const auto status = ingress.readonly_status();
  EXPECT_EQ(status.transport().dropped_count(), 1);
  EXPECT_EQ(status.transport().forwarded_count(), 1);
  EXPECT_EQ(status.latest_forwarded().publisher().gid(), replacement_gid);
}

TEST_F(SomeipIngress, InvalidPublishCommitRevokesInsteadOfInventingForwardedObservation) {
  for (const bool missing_callback : {true, false}) {
    PlanningIngress local(config, fixture_capabilities());
    const auto opened = local.open(0x3101, request, initial_ns);
    ASSERT_EQ(opened.result(), wire::SESSION_OPERATION_RESULT_OPENING);
    const auto current_epoch = local.snapshot().epoch;
    ASSERT_TRUE(local.bind_publisher(current_epoch, gid, "rmw_fastrtps_cpp"));
    ASSERT_TRUE(local.set_admitted(current_epoch, true));
    ASSERT_EQ(local.submit(0x3101, command(opened.session().session_token()), 10).forwarding_state(),
              wire::FORWARDING_STATE_PENDING);
    bool published = false;
    std::function<void(const codec::CommandFrame &)> callback;
    if (!missing_callback) callback = [&](const auto &) { published = true; };
    EXPECT_FALSE(local.publish_pending(current_epoch, missing_callback ? 11 : 0, callback));
    EXPECT_FALSE(published);
    EXPECT_EQ(local.snapshot().epoch, current_epoch + 1);
    EXPECT_EQ(local.snapshot().state, wire::CONTROL_SESSION_STATE_REVOKING);
    EXPECT_EQ(local.snapshot().failure, "INVALID_PUBLISH_COMMIT");
    const auto status = local.readonly_status();
    EXPECT_EQ(status.transport().dropped_count(), 1);
    EXPECT_EQ(status.transport().forwarded_count(), 0);
    EXPECT_FALSE(status.has_latest_forwarded());
    EXPECT_EQ(status.latest_gateway_acceptance().state(), wire::GATEWAY_ACCEPTANCE_STATE_UNKNOWN);
  }
}

TEST_F(SomeipIngress, GuardRecoveryRequiresNewSubmissionAndCannotReplayDroppedTarget) {
  admit();
  ingress.update_observations([](auto &status) {
    status.add_sources()->mutable_observation()->set_validity(wire::OBSERVATION_VALIDITY_VALID);
  });
  const auto observed_valid = [](const auto &status) {
    return status.sources_size() == 1 &&
           status.sources(0).observation().validity() == wire::OBSERVATION_VALIDITY_VALID;
  };
  ASSERT_EQ(ingress.submit(0x3101, command(token), 10).forwarding_state(), wire::FORWARDING_STATE_PENDING);
  ingress.update_observations([](auto &status) {
    status.mutable_sources(0)->mutable_observation()->set_validity(wire::OBSERVATION_VALIDITY_STALE);
  });
  std::size_t publish_calls = 0;
  EXPECT_FALSE(ingress.publish_pending(epoch, 11, [&](const auto &) { ++publish_calls; }, observed_valid));
  EXPECT_FALSE(ingress.snapshot().session.submission_ready());
  ingress.update_observations([](auto &status) {
    status.mutable_sources(0)->mutable_observation()->set_validity(wire::OBSERVATION_VALIDITY_VALID);
  });
  ASSERT_TRUE(ingress.set_admitted(epoch, true, observed_valid));
  EXPECT_FALSE(ingress.publish_pending(epoch, 12, [&](const auto &) { ++publish_calls; }, observed_valid));
  const auto fresh = command(token, 8);
  ASSERT_EQ(ingress.submit(0x3101, fresh, 13).forwarding_state(), wire::FORWARDING_STATE_PENDING);
  ASSERT_TRUE(ingress.publish_pending(epoch, 14, [&](const auto &forwarded) {
    ++publish_calls;
    EXPECT_EQ(forwarded.command_sequence, 8);
    EXPECT_EQ(forwarded.source_stamp_ns, fresh.source_stamp_ns);
  }, observed_valid));
  EXPECT_EQ(publish_calls, 1);
  EXPECT_EQ(ingress.snapshot().epoch, epoch);
  EXPECT_EQ(ingress.snapshot().session.session_token(), token);
  EXPECT_EQ(ingress.snapshot().session.ingress_publisher().gid(), gid);
  EXPECT_EQ(ingress.readonly_status().transport().dropped_count(), 1);
}

TEST_F(SomeipIngress, KeepAndReadonlyCannotRefreshBusinessFactsOrIndependentTopicTimes) {
  admit();
  const auto original = command(token);
  ASSERT_EQ(ingress.submit(0x3101, original, 10).forwarding_state(), wire::FORWARDING_STATE_PENDING);
  ASSERT_TRUE(ingress.publish_pending(epoch, 11, [](const auto &) {}));
  ingress.update_observations([](auto &status) {
    auto *gateway = status.mutable_gateway();
    gateway->mutable_observation()->set_observed_system_ns(101);
    gateway->mutable_observation()->set_received_boottime_ns(201);
    gateway->mutable_observation()->set_validity(wire::OBSERVATION_VALIDITY_STALE);
    gateway->set_state(3);
    auto *source = status.add_sources();
    source->set_source_id("planning");
    source->mutable_observation()->set_observed_system_ns(102);
    source->mutable_observation()->set_received_boottime_ns(202);
    source->mutable_observation()->set_validity(wire::OBSERVATION_VALIDITY_STALE);
    source->set_lease_valid(false);
    source->set_command_valid(false);
    source->set_command_age_ms(900.0);
    source->set_last_source_sequence(7);
    source->set_accepted_count(1);
    auto *vehicle = status.mutable_vehicle();
    vehicle->mutable_observation()->set_observed_system_ns(103);
    vehicle->mutable_observation()->set_received_boottime_ns(203);
    vehicle->mutable_observation()->set_validity(wire::OBSERVATION_VALIDITY_INVALID);
    vehicle->set_fault_code(42);
  });
  const auto before = ingress.readonly_status();
  ASSERT_EQ(ingress.keep(0x3101, keep_request(request, 1), initial_ns + 4000000000).result(),
            wire::SESSION_OPERATION_RESULT_ADMITTED);
  ingress.readonly_status();
  ASSERT_EQ(ingress.keep(0x3101, keep_request(request, 2), initial_ns + 8000000000).result(),
            wire::SESSION_OPERATION_RESULT_ADMITTED);
  ingress.expire(initial_ns + 12000000000ULL);
  EXPECT_EQ(ingress.snapshot().state, wire::CONTROL_SESSION_STATE_ADMITTED);
  EXPECT_FALSE(ingress.publish_pending(epoch, 12, [](const auto &) { FAIL() << "Keep queued old business data"; }));
  const auto after = ingress.readonly_status();
  EXPECT_EQ(after.gateway().SerializeAsString(), before.gateway().SerializeAsString());
  EXPECT_EQ(after.sources(0).SerializeAsString(), before.sources(0).SerializeAsString());
  EXPECT_EQ(after.vehicle().SerializeAsString(), before.vehicle().SerializeAsString());
  EXPECT_EQ(after.latest_forwarded().SerializeAsString(), before.latest_forwarded().SerializeAsString());
  EXPECT_EQ(after.transport().forwarded_count(), before.transport().forwarded_count());
  EXPECT_EQ(after.transport().dropped_count(), before.transport().dropped_count());
  EXPECT_EQ(after.latest_gateway_acceptance().state(), wire::GATEWAY_ACCEPTANCE_STATE_UNKNOWN);
}

TEST_F(SomeipIngress, ReadonlyWireDuringRevocationRedactsRetainedForwardedToken) {
  admit();
  ASSERT_EQ(ingress.submit(0x3101, command(token), 10).forwarding_state(), wire::FORWARDING_STATE_PENDING);
  ASSERT_TRUE(ingress.publish_pending(epoch, 11, [](const auto &) {}));
  ingress.revoke("stop while observers are subscribed");
  const auto status = ingress.readonly_status();
  ASSERT_TRUE(status.has_latest_forwarded());
  EXPECT_EQ(status.session().control_session_state(), wire::CONTROL_SESSION_STATE_REVOKING);
  EXPECT_TRUE(status.session().session_token().empty());
  EXPECT_TRUE(status.latest_forwarded().key().session_token().empty());
  EXPECT_EQ(status.latest_forwarded().key().source_sequence(), 7);
  EXPECT_EQ(status.latest_gateway_acceptance().state(), wire::GATEWAY_ACCEPTANCE_STATE_UNKNOWN);
  const auto bytes = status.SerializeAsString();
  EXPECT_EQ(bytes.find(token), std::string::npos);
  wire::StatusSnapshot decoded;
  ASSERT_TRUE(decoded.ParseFromString(bytes));
  EXPECT_EQ(decoded.SerializeAsString(), bytes);
  EXPECT_EQ(ingress.readonly_status().SerializeAsString(), bytes);
  EXPECT_EQ(ingress.snapshot().session.session_token(), token);
  ASSERT_TRUE(ingress.finish_release(ingress.snapshot().epoch));
  EXPECT_FALSE(ingress.readonly_status().has_latest_forwarded());
}

TEST_F(SomeipIngress, RosStampBoundaryPassesOriginalNanosecondsAndRejectsUnrepresentableNeighbor) {
  admit();
  constexpr std::int64_t nanoseconds_per_second = 1000000000;
  const auto minimum = static_cast<std::int64_t>(std::numeric_limits<std::int32_t>::min()) * nanoseconds_per_second;
  const auto maximum = static_cast<std::int64_t>(std::numeric_limits<std::int32_t>::max()) * nanoseconds_per_second + 999999999;
  std::uint64_t next_sequence = 1;
  for (const auto stamp : {minimum, std::int64_t{-1}, std::int64_t{0}, maximum}) {
    auto valid = command(token, next_sequence++);
    valid.source_stamp_ns = stamp;
    ASSERT_EQ(ingress.submit(0x3101, valid, 10).forwarding_state(), wire::FORWARDING_STATE_PENDING);
    for (const auto outside : {minimum - 1, maximum + 1}) {
      auto invalid = valid;
      invalid.source_stamp_ns = outside;
      EXPECT_EQ(ingress.submit(0x3101, invalid, 11).forwarding_state(), wire::FORWARDING_STATE_REJECTED);
    }
    ASSERT_TRUE(ingress.publish_pending(epoch, 12, [&](const auto &forwarded) {
      EXPECT_EQ(forwarded.command_sequence, valid.command_sequence);
      EXPECT_EQ(forwarded.source_stamp_ns, stamp);
    }));
  }
  EXPECT_EQ(ingress.readonly_status().transport().forwarded_count(), 4);
  EXPECT_EQ(ingress.readonly_status().transport().dropped_count(), 0);
  EXPECT_EQ(ingress.readonly_status().transport().protocol_rejected_count(), 8);
}

}
}
