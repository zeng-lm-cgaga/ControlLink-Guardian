#pragma once

#include "control_link_someip_transport/client.hpp"

namespace control_link::someip::detail {

// 由Client互斥锁保护；固定两组实例，不建队列/线程，不触碰控制session。
class SubscriptionObservationState final {
 public:
  SubscriptionObservationState(std::uint16_t service, std::uint16_t instance,
                               std::uint16_t group, std::uint16_t event) {
    current_.service = service;
    current_.instance = instance;
    current_.eventgroup = group;
    current_.event = event;
  }

  std::uint64_t begin(std::uint64_t availability_generation, std::uint64_t now,
                      std::uint64_t budget_ns) {
    ++current_.request_generation;
    current_.availability_generation = availability_generation;
    current_.state = SubscriptionState::Pending;
    current_.requested_steady_ns = now;
    current_.deadline_steady_ns = now + budget_ns;
    current_.callback_steady_ns = 0;
    current_.callback_observed = false;
    current_.error = 0;
    // vsomeip回调无请求代次或远端实例token。重新订阅后的同tuple回调不能证明属于新请求。
    current_.callback_attempt_unambiguous = current_.request_generation == 1;
    active_ = true;
    return current_.request_generation;
  }

  void observe(std::uint64_t generation, std::uint16_t service, std::uint16_t instance,
               std::uint16_t group, std::uint16_t event, std::uint16_t error,
               std::uint64_t now) {
    if (!active_ || generation != current_.request_generation || service != current_.service ||
        instance != current_.instance || group != current_.eventgroup || event != current_.event ||
        now < current_.requested_steady_ns) {
      ++current_.ignored_callbacks;
      return;
    }
    const bool expired = current_.state == SubscriptionState::TimedOut ||
        (current_.state == SubscriptionState::Pending && now >= current_.deadline_steady_ns);
    current_.callback_observed = true;
    current_.callback_steady_ns = now;
    current_.error = error;
    if (expired) current_.state = SubscriptionState::TimedOut;
    else if (!current_.callback_attempt_unambiguous) current_.state = SubscriptionState::Unknown;
    else current_.state = error == 0 ? SubscriptionState::Accepted : SubscriptionState::Rejected;
  }

  void reset(SubscriptionState state) {
    active_ = false;
    current_.state = state;
  }

  SubscriptionObservation snapshot(std::uint64_t now) const {
    auto result = current_;
    if (result.state == SubscriptionState::Pending && now >= result.deadline_steady_ns)
      result.state = SubscriptionState::TimedOut;
    return result;
  }

 private:
  SubscriptionObservation current_;
  bool active_{false};
};

}
