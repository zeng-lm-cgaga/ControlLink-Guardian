#include "../src/session_reply_order.hpp"

#include <string>

#include <gtest/gtest.h>

namespace control_link::someip {
namespace {

namespace wire = ::control_link::network::v1;

wire::SessionInfo admitted_session() {
  wire::SessionInfo session;
  session.set_session_token(std::string(16, 't'));
  session.mutable_ingress_publisher()->set_gid(std::string(24, 'g'));
  session.mutable_ingress_publisher()->set_rmw_implementation("rmw_fastrtps_cpp");
  session.set_planning_transport(wire::PLANNING_TRANSPORT_SOMEIP);
  session.set_clock_domain(wire::CLOCK_DOMAIN_SYSTEM);
  session.set_control_session_state(wire::CONTROL_SESSION_STATE_ADMITTED);
  session.set_connection_state(wire::CONNECTION_STATE_CONNECTED);
  session.set_submission_ready(true);
  return session;
}

TEST(SomeipSessionReplyOrder, ReplyWithoutLaterRevocationCommitsOriginalIdentityAndReady) {
  detail::SessionReplyOrder order;
  auto reply = admitted_session();
  wire::SessionInfo cached;
  const auto received_revision = order.capture_reply();
  const auto original_reply = reply.SerializeAsString();

  EXPECT_TRUE(order.commit_reply(received_revision, reply, cached));
  EXPECT_TRUE(reply.submission_ready());
  EXPECT_TRUE(cached.submission_ready());
  EXPECT_EQ(reply.SerializeAsString(), original_reply);
  EXPECT_EQ(cached.SerializeAsString(), original_reply);
}

TEST(SomeipSessionReplyOrder, LaterRevocationProtectsCacheAndPublicOpenResponse) {
  detail::SessionReplyOrder order;
  wire::OpenSessionResponse response;
  response.set_result(wire::SESSION_OPERATION_RESULT_ADMITTED);
  *response.mutable_session() = admitted_session();
  auto cached = admitted_session();
  const auto received_revision = order.capture_reply();

  // 确定性模拟 callback 已收到 reply，owner 解码前较晚 event 已撤准入。
  order.observe_revocation();
  cached.set_submission_ready(false);
  cached.set_control_session_state(wire::CONTROL_SESSION_STATE_OPENING);
  cached.set_connection_state(wire::CONNECTION_STATE_CONNECTING);
  const auto newer_cache = cached.SerializeAsString();

  EXPECT_FALSE(order.commit_reply(received_revision, *response.mutable_session(), cached));
  EXPECT_EQ(cached.SerializeAsString(), newer_cache);
  EXPECT_FALSE(response.session().submission_ready());
  EXPECT_EQ(response.session().control_session_state(), wire::CONTROL_SESSION_STATE_UNKNOWN);
  EXPECT_EQ(response.session().connection_state(), wire::CONNECTION_STATE_UNKNOWN);
  EXPECT_EQ(response.session().session_token(), std::string(16, 't'));
  EXPECT_EQ(response.session().ingress_publisher().gid(), std::string(24, 'g'));
  EXPECT_EQ(response.result(), wire::SESSION_OPERATION_RESULT_ADMITTED);
}

TEST(SomeipSessionReplyOrder, GenuinelyLaterReplyRestoresReadyWithoutRotatingIdentity) {
  detail::SessionReplyOrder order;
  auto old_reply = admitted_session();
  auto cached = admitted_session();
  const auto old_revision = order.capture_reply();
  order.observe_revocation();
  cached.set_submission_ready(false);
  EXPECT_FALSE(order.commit_reply(old_revision, old_reply, cached));

  auto fresh_reply = admitted_session();
  const auto fresh_revision = order.capture_reply();
  EXPECT_TRUE(order.commit_reply(fresh_revision, fresh_reply, cached));
  EXPECT_TRUE(fresh_reply.submission_ready());
  EXPECT_TRUE(cached.submission_ready());
  EXPECT_EQ(cached.session_token(), std::string(16, 't'));
  EXPECT_EQ(cached.ingress_publisher().gid(), std::string(24, 'g'));
}

TEST(SomeipSessionReplyOrder, RevocationBeforeFirstOpenCommitCannotCreateReadyCache) {
  detail::SessionReplyOrder order;
  wire::SessionInfo cached;
  auto reply = admitted_session();
  const auto received_revision = order.capture_reply();
  const auto empty_cache = cached.SerializeAsString();
  order.observe_revocation();

  EXPECT_FALSE(order.commit_reply(received_revision, reply, cached));
  EXPECT_FALSE(reply.submission_ready());
  EXPECT_EQ(cached.SerializeAsString(), empty_cache);
  EXPECT_TRUE(cached.session_token().empty());
}

TEST(SomeipSessionReplyOrder, SecondRevocationSupersedesReplyReceivedAfterFirstRevocation) {
  detail::SessionReplyOrder order;
  order.observe_revocation();
  const auto middle_revision = order.capture_reply();
  auto reply = admitted_session();
  auto cached = admitted_session();
  order.observe_revocation();
  cached.set_submission_ready(false);
  cached.set_control_session_state(wire::CONTROL_SESSION_STATE_REVOKING);
  cached.set_connection_state(wire::CONNECTION_STATE_CLOSING);
  const auto revoked_cache = cached.SerializeAsString();

  EXPECT_FALSE(order.commit_reply(middle_revision, reply, cached));
  EXPECT_FALSE(reply.submission_ready());
  EXPECT_EQ(cached.SerializeAsString(), revoked_cache);

  auto fresh_reply = admitted_session();
  EXPECT_TRUE(order.commit_reply(order.capture_reply(), fresh_reply, cached));
  EXPECT_TRUE(cached.submission_ready());
}

TEST(SomeipSessionReplyOrder, RejectedOldCompletionsDoNotMoveRevisionOrDropNewerCache) {
  detail::SessionReplyOrder order;
  const auto old_revision = order.capture_reply();
  order.observe_revocation();
  const auto new_revision = order.capture_reply();
  auto fresh_reply = admitted_session();
  wire::SessionInfo cached;
  ASSERT_TRUE(order.commit_reply(new_revision, fresh_reply, cached));
  const auto newer_cache = cached.SerializeAsString();

  for (int completion = 0; completion < 2; ++completion) {
    auto old_reply = admitted_session();
    old_reply.set_submission_ready(completion == 0);
    EXPECT_FALSE(order.commit_reply(old_revision, old_reply, cached));
    EXPECT_FALSE(old_reply.submission_ready());
    EXPECT_EQ(cached.SerializeAsString(), newer_cache);
    EXPECT_EQ(order.capture_reply(), new_revision);
  }
}

TEST(SomeipSessionReplyOrder, GuardDoesNotRewritePublicTokenGidTransportOrClock) {
  detail::SessionReplyOrder order;
  auto reply = admitted_session();
  auto cached = admitted_session();
  const auto received_revision = order.capture_reply();
  order.observe_revocation();
  cached.set_submission_ready(false);

  EXPECT_FALSE(order.commit_reply(received_revision, reply, cached));
  EXPECT_EQ(reply.session_token(), cached.session_token());
  EXPECT_EQ(reply.ingress_publisher().SerializeAsString(), cached.ingress_publisher().SerializeAsString());
  EXPECT_EQ(reply.planning_transport(), wire::PLANNING_TRANSPORT_SOMEIP);
  EXPECT_EQ(reply.clock_domain(), wire::CLOCK_DOMAIN_SYSTEM);
}

}
}
