#pragma once

#include <array>
#include <cstdint>
#include <map>
#include <mutex>
#include <string>

namespace YAML { class Node; }

namespace control_link::network {

enum class IngressMethod : std::size_t { Open, Submit, Status, Capabilities, Subscribe, Keep, Close, Count };
struct RateLimitPolicy {
  std::uint32_t per_caller_per_second{0}, per_caller_burst{0};
  std::uint32_t global_per_second{0}, global_burst{0};
  std::uint32_t max_keys_per_method{0}, idle_ttl_ms{0};
};
struct RateLimitCounters {
  std::uint64_t allowed{0}, caller_limited{0}, global_limited{0}, capacity_limited{0}, invalid{0};
};
using RateLimitSnapshot = std::array<RateLimitCounters, static_cast<std::size_t>(IngressMethod::Count)>;
RateLimitPolicy parse_rate_limit_policy(const YAML::Node &node);
void validate_rate_limit_policy(const RateLimitPolicy &policy);

// peer/client只是本地资源分组，不是认证。每方法独立预算保留查询/Keep机会，换键仍受全局桶约束。
// Close独享方法预算，本地cancel/stop不经网络限流；不刷新业务stamp、lease或holder资格。
class IngressRateLimiter final {
 public:
  explicit IngressRateLimiter(RateLimitPolicy policy);
  bool admit(const std::string &caller, IngressMethod method, std::uint64_t steady_ns);
  RateLimitSnapshot snapshot() const;
 private:
  struct Bucket { double tokens{0}; std::uint64_t refill_ns{0}, seen_ns{0}; };
  void refill(Bucket &bucket, std::uint64_t now, std::uint32_t rate, std::uint32_t burst);
  RateLimitPolicy policy_;
  mutable std::mutex mutex_;
  std::array<std::map<std::string, Bucket>, static_cast<std::size_t>(IngressMethod::Count)> callers_;
  std::array<Bucket, static_cast<std::size_t>(IngressMethod::Count)> global_;
  RateLimitSnapshot counters_{};
};
}
