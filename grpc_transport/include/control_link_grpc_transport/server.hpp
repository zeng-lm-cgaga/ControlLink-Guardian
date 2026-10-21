#pragma once

#include <cstdint>
#include <memory>
#include <string>

#include "control_link_grpc_transport/config.hpp"
#include "control_link_network_ingress/ingress.hpp"

namespace control_link::grpc_transport {

// 仅描述RPC资源事实，不代表Gateway接受或车辆执行。
struct ServerRuntime {
  std::uint64_t active_calls{0};
  std::uint64_t peak_calls{0};
  std::uint64_t completed_tags{0};
  std::uint64_t dropped_updates{0};
  std::uint64_t unstarted_requests{0};
  std::uint64_t unresolved_calls{0};
  bool cq_drained{false};
  bool processing_failed{false};
  network::RateLimitSnapshot rate_limits{};
};

// CQ独占call/tag推进；ROS owner只操作同一个PlanningIngress。start/stop由装配owner串行调用。
// stop在非CQ线程撤资格、取消、Shutdown、排空并join，不在网络回调中等待ROS或执行业务判断。
class GrpcServer final {
 public:
  GrpcServer(Config config, std::shared_ptr<network::PlanningIngress> ingress,
             network::v1::Capabilities capabilities);
  ~GrpcServer();
  GrpcServer(const GrpcServer &) = delete;
  GrpcServer &operator=(const GrpcServer &) = delete;
  void start();
  void stop() noexcept;
  ServerRuntime runtime() const;

 private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};

}
