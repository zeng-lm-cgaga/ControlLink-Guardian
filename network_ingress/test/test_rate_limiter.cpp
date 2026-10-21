#include "control_link_network_ingress/rate_limiter.hpp"
#include <gtest/gtest.h>

using namespace control_link::network;
namespace {
RateLimitPolicy policy() { return {40, 2, 200, 4, 2, 1000}; }
TEST(IngressRateLimiter, BurstAndSteadyRefillDoNotShareMethodBudgets) {
  IngressRateLimiter limiter(policy());
  EXPECT_TRUE(limiter.admit("peer", IngressMethod::Submit, 1000000000));
  EXPECT_TRUE(limiter.admit("peer", IngressMethod::Submit, 1000000000));
  EXPECT_FALSE(limiter.admit("peer", IngressMethod::Submit, 1000000000));
  EXPECT_TRUE(limiter.admit("peer", IngressMethod::Status, 1000000000));
  EXPECT_TRUE(limiter.admit("peer", IngressMethod::Keep, 1000000000));
  EXPECT_TRUE(limiter.admit("peer", IngressMethod::Submit, 1025000000));
}
TEST(IngressRateLimiter, OneCallerCannotConsumeAnotherCallerAllocation) {
  IngressRateLimiter limiter(policy());
  for (int attempt = 0; attempt < 20; ++attempt) limiter.admit("flood", IngressMethod::Status, 1000000000);
  EXPECT_TRUE(limiter.admit("other", IngressMethod::Status, 1000000000));
  EXPECT_EQ(limiter.snapshot()[static_cast<std::size_t>(IngressMethod::Status)].caller_limited, 18U);
}
TEST(IngressRateLimiter, ChurnCannotBypassGlobalBudgetAndKeysAreBounded) {
  auto config = policy(); config.max_keys_per_method = 8;
  IngressRateLimiter limiter(config);
  for (int index = 0; index < 4; ++index) EXPECT_TRUE(limiter.admit(std::to_string(index), IngressMethod::Open, 1000000000));
  EXPECT_FALSE(limiter.admit("fifth", IngressMethod::Open, 1000000000));
  EXPECT_EQ(limiter.snapshot()[0].global_limited, 1U);
}
TEST(IngressRateLimiter, IdleReclaimDoesNotRefillGlobalAndCloseIsReserved) {
  IngressRateLimiter limiter(policy());
  EXPECT_TRUE(limiter.admit("first", IngressMethod::Submit, 1000000000));
  EXPECT_TRUE(limiter.admit("second", IngressMethod::Submit, 1000000000));
  EXPECT_FALSE(limiter.admit("third", IngressMethod::Submit, 1000000000));
  EXPECT_TRUE(limiter.admit("third", IngressMethod::Submit, 2000000000));
  EXPECT_TRUE(limiter.admit("third", IngressMethod::Close, 2000000000));
  EXPECT_FALSE(limiter.admit("third", IngressMethod::Submit, 1999999999));
}
TEST(IngressRateLimiter, InvalidPolicyAndCallerAreRejected) {
  auto invalid = policy(); invalid.max_keys_per_method = 0;
  EXPECT_THROW(IngressRateLimiter{invalid}, std::invalid_argument);
  IngressRateLimiter limiter(policy());
  EXPECT_FALSE(limiter.admit(std::string(129, 'x'), IngressMethod::Status, 1000000000));
  EXPECT_FALSE(limiter.admit("peer", IngressMethod::Status, 0));
}
}
