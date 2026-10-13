#pragma once

#include <chrono>
#include <cstdint>
#include <memory>
#include <string>

#include "control_link_grpc_transport/config.hpp"
#include "control_link_edge_protocol/protocol.hpp"
#include "control_link_network_v1.pb.h"

namespace control_link::grpc_transport {

namespace wire = ::control_link::network::v1;

struct ClientStatus {
  bool running{false};
  // availability 是网络连接提示，不代表 holder/ROS 准入、Gateway ACTIVE 或执行 ACK。
  bool service_available{false};
  bool control_rpc_live{false};
  bool control_active{false};
  bool readonly_subscribed{false};
  std::uint64_t availability_generation{0};
  std::uint64_t control_generation{0};
  std::uint64_t status_received_steady_ns{0};
  std::uint64_t readonly_overwritten_count{0};
  wire::SessionInfo session;
  wire::Capabilities capabilities;
  wire::StatusSnapshot observations;
  std::string failure;
};

// 所有公开调用由同一 owner 串行执行；CQ 后台仅推进操作并更新独立 latest 观察。
// RPC 回复不等于 Gateway 接受；任何目标均保留调用者原 stamp/sequence/token，不自动重试。
class GrpcClient final {
 public:
  explicit GrpcClient(Config config);
  ~GrpcClient();
  GrpcClient(const GrpcClient &) = delete;
  GrpcClient &operator=(const GrpcClient &) = delete;

  void start();
  bool wait_available(std::chrono::milliseconds timeout);
  // 新键对先GetCapabilities发现域并保存；Open/Keep/Close必须显式带同一域。
  // SDK不更新旧请求的域；readonly发现及透明重连不能使旧开立或旧目标复活。
  wire::OpenSessionResponse open(const wire::OpenSessionRequest &request,
      std::chrono::milliseconds timeout = std::chrono::milliseconds(500));
  wire::OpenSessionResponse keep(const wire::KeepSessionRequest &request,
      std::chrono::milliseconds timeout = std::chrono::milliseconds(500));
  wire::OpenSessionResponse close(const wire::CloseSessionRequest &request,
      std::chrono::milliseconds timeout = std::chrono::milliseconds(500));
  wire::SubmitReceipt submit(const control_link_edge_protocol::CommandFrame &target,
      std::chrono::milliseconds timeout = std::chrono::milliseconds(500));
  wire::StatusEnvelope get_status(
      std::chrono::milliseconds timeout = std::chrono::milliseconds(500));
  wire::Capabilities get_capabilities(
      std::chrono::milliseconds timeout = std::chrono::milliseconds(500));
  void set_status_subscription(bool enabled);
  ClientStatus status() const;

  // WritesDone 半关闭按应用契约放弃控制；先撤本地资格，不等待网络 EOF 才禁止旧目标。
  void half_close();
  void cancel_control() noexcept;
  void stop() noexcept;

 private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};

// 普通 CMake consumer 与工作区 client 共用此循环；固定速度输入不是规划/控制算法。
int run_planning_example(int argc, char **argv);

}
