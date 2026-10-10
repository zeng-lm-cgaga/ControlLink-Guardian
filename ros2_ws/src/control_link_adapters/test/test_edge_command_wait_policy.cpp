#include <gtest/gtest.h>

#include "control_link_adapters/edge_command_wait_policy.hpp"

namespace
{

using control_link_adapters::is_command_wait_without_new_sample;
using edge_runtime::ErrorCode;

TEST(EdgeCommandWaitPolicyTest, DataStaleDoesNotStartRecoveryBackoff)
{
	EXPECT_TRUE(is_command_wait_without_new_sample(ErrorCode::kNoNewSample));
	EXPECT_TRUE(is_command_wait_without_new_sample(ErrorCode::kTimeout));
	EXPECT_TRUE(is_command_wait_without_new_sample(ErrorCode::kDataStale));
}

TEST(EdgeCommandWaitPolicyTest, TransportFailuresRemainRecoveryInputs)
{
	EXPECT_FALSE(is_command_wait_without_new_sample(ErrorCode::kProducerOffline));
	EXPECT_FALSE(is_command_wait_without_new_sample(ErrorCode::kRecoveryBlocked));
	EXPECT_FALSE(is_command_wait_without_new_sample(ErrorCode::kStaleHandle));
	EXPECT_FALSE(is_command_wait_without_new_sample(ErrorCode::kTransportFailed));
	EXPECT_FALSE(is_command_wait_without_new_sample(ErrorCode::kPayloadCorrupt));
}

}  // namespace
