#include "../src/subscription_state.hpp"
#include <gtest/gtest.h>

namespace control_link::someip {
namespace {
using State = SubscriptionState;

TEST(SomeipSubscription, SubscribeIsPendingNotAccepted) {
  detail::SubscriptionObservationState state(1, 2, 3, 4);
  state.begin(10, 100, 50);
  EXPECT_EQ(state.snapshot(149).state, State::Pending);
  EXPECT_FALSE(state.snapshot(149).callback_observed);
  EXPECT_EQ(state.snapshot(150).state, State::TimedOut);
  EXPECT_FALSE(state.snapshot(150).callback_observed);
}

TEST(SomeipSubscription, ExactPublicAckAndNackAreDifferentObservations) {
  detail::SubscriptionObservationState accepted(1, 2, 3, 4), rejected(1, 2, 5, 6);
  auto generation = accepted.begin(9, 100, 50);
  accepted.observe(generation, 1, 2, 3, 4, 0, 110);
  generation = rejected.begin(9, 100, 50);
  rejected.observe(generation, 1, 2, 5, 6, 7, 111);
  EXPECT_EQ(accepted.snapshot(120).state, State::Accepted);
  EXPECT_EQ(rejected.snapshot(120).state, State::Rejected);
  EXPECT_TRUE(rejected.snapshot(120).callback_observed);
  EXPECT_EQ(rejected.snapshot(120).error, 7);
}

TEST(SomeipSubscription, WrongIdentityAndRetiredHandlerCannotRestoreState) {
  detail::SubscriptionObservationState state(1, 2, 3, 4);
  const auto old_generation = state.begin(9, 100, 50);
  state.observe(old_generation, 1, 99, 3, 4, 0, 110);
  EXPECT_EQ(state.snapshot(110).state, State::Pending);
  state.reset(State::Disabled);
  state.observe(old_generation, 1, 2, 3, 4, 0, 120);
  EXPECT_EQ(state.snapshot(120).state, State::Disabled);
  state.begin(10, 130, 50);
  state.observe(old_generation, 1, 2, 3, 4, 0, 140);
  EXPECT_EQ(state.snapshot(140).state, State::Pending);
  EXPECT_EQ(state.snapshot(140).ignored_callbacks, 3);
}

TEST(SomeipSubscription, ReRegisteredTupleDoesNotProveNewAttemptAcceptance) {
  detail::SubscriptionObservationState state(1, 2, 3, 4);
  state.begin(9, 100, 50);
  state.reset(State::Unknown);
  const auto generation = state.begin(10, 200, 50);
  state.observe(generation, 1, 2, 3, 4, 0, 210);
  const auto result = state.snapshot(210);
  EXPECT_EQ(result.state, State::Unknown);
  EXPECT_TRUE(result.callback_observed);
  EXPECT_FALSE(result.callback_attempt_unambiguous);
  EXPECT_EQ(result.availability_generation, 10);
}

TEST(SomeipSubscription, LateCallbackRetainsTimeoutRatherThanInventingSuccess) {
  detail::SubscriptionObservationState state(1, 2, 3, 4);
  const auto generation = state.begin(9, 100, 50);
  state.observe(generation, 1, 2, 3, 4, 0, 150);
  EXPECT_EQ(state.snapshot(160).state, State::TimedOut);
  EXPECT_TRUE(state.snapshot(160).callback_observed);
  EXPECT_EQ(state.snapshot(160).error, 0);
}
}
}
