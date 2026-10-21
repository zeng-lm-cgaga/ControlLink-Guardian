#include "control_link_network_ingress/rate_limiter.hpp"

#include <algorithm>
#include <limits>
#include <set>
#include <stdexcept>
#include <yaml-cpp/yaml.h>

namespace control_link::network {
namespace {
void increment(std::uint64_t &counter) {
  if (counter != std::numeric_limits<std::uint64_t>::max()) ++counter;
}
}
RateLimitPolicy parse_rate_limit_policy(const YAML::Node &node) {
  const std::set<std::string> fields{"per_caller_per_second", "per_caller_burst", "global_per_second",
      "global_burst", "max_keys_per_method", "idle_ttl_ms"};
  if (!node.IsMap() || node.size() != fields.size()) throw std::invalid_argument("rate_limits: exact mapping required");
  std::set<std::string> seen;
  for (const auto &entry : node) {
    const auto key = entry.first.as<std::string>();
    if (!fields.count(key) || !seen.insert(key).second)
      throw std::invalid_argument("rate_limits: unknown/duplicate field");
  }
  const auto read = [&](const char *field) {
    if (!node[field].IsScalar()) throw std::invalid_argument("rate_limits: integer scalar required");
    const auto text = node[field].Scalar();
    if (text.empty() || text.find_first_not_of("0123456789") != std::string::npos || text.size() > 9)
      throw std::invalid_argument("rate_limits: bounded positive decimal required");
    return static_cast<std::uint32_t>(std::stoul(text));
  };
  RateLimitPolicy policy{read("per_caller_per_second"), read("per_caller_burst"),
      read("global_per_second"), read("global_burst"), read("max_keys_per_method"), read("idle_ttl_ms")};
  validate_rate_limit_policy(policy);
  return policy;
}
void validate_rate_limit_policy(const RateLimitPolicy &policy) {
  if (!policy.per_caller_per_second || !policy.per_caller_burst ||
      policy.global_per_second < policy.per_caller_per_second || policy.global_burst < policy.per_caller_burst ||
      policy.global_per_second > 100000 || policy.global_burst > 100000 ||
      !policy.max_keys_per_method || policy.max_keys_per_method > 4096 ||
      !policy.idle_ttl_ms || policy.idle_ttl_ms > 3600000)
    throw std::invalid_argument("rate_limits: invalid finite rate/burst/key/TTL budget");
}
IngressRateLimiter::IngressRateLimiter(RateLimitPolicy policy) : policy_(policy) {
  validate_rate_limit_policy(policy_);
  for (auto &bucket : global_) bucket.tokens = policy_.global_burst;
}
void IngressRateLimiter::refill(Bucket &bucket, std::uint64_t now, std::uint32_t rate, std::uint32_t burst) {
  bucket.tokens = std::min<double>(burst, bucket.tokens + (now - bucket.refill_ns) / 1e9 * rate);
  bucket.refill_ns = now;
}
bool IngressRateLimiter::admit(const std::string &caller, IngressMethod method, std::uint64_t now) {
  std::lock_guard<std::mutex> lock(mutex_);
  const auto index = static_cast<std::size_t>(method);
  if (index >= counters_.size()) return false;
  auto &counts = counters_[index];
  auto &global = global_[index];
  if (caller.empty() || caller.size() > 128 || !now || now < global.refill_ns) {
    increment(counts.invalid); return false;
  }
  refill(global, now, policy_.global_per_second, policy_.global_burst);
  auto &keys = callers_[index];
  for (auto entry = keys.begin(); entry != keys.end();) {
    if (now - entry->second.seen_ns >= static_cast<std::uint64_t>(policy_.idle_ttl_ms) * 1000000)
      entry = keys.erase(entry);
    else ++entry;
  }
  auto found = keys.find(caller);
  if (found == keys.end()) {
    if (keys.size() >= policy_.max_keys_per_method) { increment(counts.capacity_limited); return false; }
    found = keys.emplace(caller, Bucket{static_cast<double>(policy_.per_caller_burst), now, now}).first;
  }
  auto &bucket = found->second;
  refill(bucket, now, policy_.per_caller_per_second, policy_.per_caller_burst);
  bucket.seen_ns = now;
  if (bucket.tokens < 1) { increment(counts.caller_limited); return false; }
  if (global.tokens < 1) { increment(counts.global_limited); return false; }
  --bucket.tokens; --global.tokens; increment(counts.allowed);
  return true;
}
RateLimitSnapshot IngressRateLimiter::snapshot() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return counters_;
}
}
