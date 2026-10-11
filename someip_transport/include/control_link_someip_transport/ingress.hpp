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
  explicit PlanningIngress(Config config, wire::Capabilities capabilities);
  ~PlanningIngress();
  PlanningIngress(const PlanningIngress &) = delete;
  PlanningIngress &operator=(const PlanningIngress &) = delete;

  wire::OpenSessionResponse open(std::uint16_t peer,
                                const wire::OpenSessionRequest &request,
                                std::uint64_t steady_ns);
  wire::OpenSessionResponse keep(std::uint16_t peer,
                                const wire::KeepSessionRequest &request,
                                std::uint64_t steady_ns);
  wire::OpenSessionResponse close(std::uint16_t peer,
                                 const wire::CloseSessionRequest &request);
  wire::SubmitReceipt submit(std::uint16_t peer,
                            const control_link_edge_protocol::CommandFrame &frame,
                            std::uint64_t received_boot_ns);
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
