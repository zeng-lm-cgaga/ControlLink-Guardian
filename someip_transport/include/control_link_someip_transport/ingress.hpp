#pragma once

#include <cstdint>
#include <functional>
#include <memory>
#include <string>

#include "control_link_edge_protocol/protocol.hpp"
#include "control_link_network_v1.pb.h"
#include "control_link_someip_transport/config.hpp"

namespace control_link::someip {

namespace wire = ::control_link::network::v1;

// 公共入口只需传输资源预算；gRPC 不必构造 SOME/IP 地址、method 或 SD 配置。
// idle_timeout_ms 仅 SOME/IP 使用，gRPC holder 存活由 control RPC owner 撤销。
struct IngressResources {
  std::uint32_t max_payload_bytes{0};
  std::uint32_t retired_open_capacity{0};
  std::uint32_t idle_timeout_ms{0};
};

struct IngressSnapshot {
  std::uint64_t epoch{0};
  wire::ControlSessionState state{wire::CONTROL_SESSION_STATE_IDLE};
  wire::SessionInfo session;
  std::string failure;
};

// 只拥有网络holder、epoch和容量1转发资格；业务合法性、lease和仲裁始终归Gateway。
// ready/publish的最终复核与撤销共用同一mutex；guard/publish不得重入或执行网络等待。
class PlanningIngress final {
 public:
  explicit PlanningIngress(Config config, wire::Capabilities capabilities,
                           wire::PlanningTransport transport = wire::PLANNING_TRANSPORT_SOMEIP);
  explicit PlanningIngress(IngressResources resources, wire::Capabilities capabilities,
                           wire::PlanningTransport transport = wire::PLANNING_TRANSPORT_SOMEIP);
  ~PlanningIngress();
  PlanningIngress(const PlanningIngress &) = delete;
  PlanningIngress &operator=(const PlanningIngress &) = delete;

  wire::OpenSessionResponse open(std::uint64_t peer,
                                const wire::OpenSessionRequest &request,
                                std::uint64_t steady_ns);
  wire::OpenSessionResponse keep(std::uint64_t peer,
                                const wire::KeepSessionRequest &request,
                                std::uint64_t steady_ns);
  wire::OpenSessionResponse close(std::uint64_t peer,
                                 const wire::CloseSessionRequest &request);
  // gRPC 短 unary 仅匹配 live session token，peer 可为0；SOME/IP 仍须匹配 holder peer。
  wire::SubmitReceipt submit(std::uint64_t peer,
                            const control_link_edge_protocol::CommandFrame &frame,
                            std::uint64_t received_boot_ns,
                            std::uint64_t submission_id = 0);
  // 仅撤销当前 epoch 内同 RPC 所有的未转发 slot；业务 key 重复不能代替 RPC 身份。
  bool cancel_pending_submission(std::uint64_t epoch, std::uint64_t submission_id);
  void expire(std::uint64_t steady_ns);
  void revoke(const std::string &reason);
  bool revoke_if_current(std::uint64_t epoch, const std::string &reason);
  bool finish_release(std::uint64_t revoked_epoch);
  IngressSnapshot snapshot() const;
  bool bind_publisher(std::uint64_t epoch, const std::string &gid,
                      const std::string &rmw);
  bool set_admitted(std::uint64_t epoch, bool ready,
      const std::function<bool(const wire::StatusSnapshot &)> &ready_guard = {});
  bool publish_pending(
      std::uint64_t epoch, std::uint64_t forwarded_boot_ns,
      const std::function<void(const control_link_edge_protocol::CommandFrame &)> &publish,
      const std::function<bool(const wire::StatusSnapshot &)> &ready_guard = {});
  void update_observations(const std::function<void(wire::StatusSnapshot &)> &update);
  void record_protocol_rejection();
  // 网络回复/只读 latest 覆盖事实；不得续命、换代或改变业务恢复判断。
  void record_transport_drop();
  wire::StatusSnapshot readonly_status() const;

 private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};

std::uint64_t steady_now_ns();
std::uint64_t boottime_now_ns();
std::int64_t system_now_ns();
std::string random_identity();

}
