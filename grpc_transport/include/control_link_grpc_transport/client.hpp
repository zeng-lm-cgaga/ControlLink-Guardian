#pragma once

#include <chrono>
#include <cstdint>
#include <memory>
#include <functional>
#include <future>
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
  std::uint64_t callback_exceptions{0};
  std::uint64_t async_overloaded{0};
};

template <typename Response>
struct AsyncResult {
  Response response;
  std::string error;
  int rpc_code{0};
  bool ok() const noexcept { return error.empty(); }
};

// future 与取消句柄可比 client 活得更久；取消只影响该请求，已完成时无效果。
template <typename Response>
class RequestHandle {
 public:
  RequestHandle(std::shared_future<AsyncResult<Response>> result, std::function<void()> cancel)
      : result_(std::move(result)), cancel_(std::move(cancel)) {}
  const std::shared_future<AsyncResult<Response>> &future() const noexcept { return result_; }
  void cancel() const { cancel_(); }
 private:
  std::shared_future<AsyncResult<Response>> result_;
  std::function<void()> cancel_;
};

// 生命周期/控制流操作由单 owner 串行发起；submit/只读异步请求可并发在途。
// callback 在有界独立分发线程运行；不得在 callback 内 stop/析构或阻塞等待本client回调。
// stop 由外部 owner 调用，等待所有已接纳 callback 返回；用户回调不得永久阻塞。
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
  template <typename Response>
  using Callback = std::function<void(const AsyncResult<Response> &)>;
  RequestHandle<wire::OpenSessionResponse> open_async(const wire::OpenSessionRequest &request,
      std::chrono::milliseconds timeout = std::chrono::milliseconds(500), Callback<wire::OpenSessionResponse> callback = {});
  RequestHandle<wire::OpenSessionResponse> keep_async(const wire::KeepSessionRequest &request,
      std::chrono::milliseconds timeout = std::chrono::milliseconds(500), Callback<wire::OpenSessionResponse> callback = {});
  RequestHandle<wire::OpenSessionResponse> close_async(const wire::CloseSessionRequest &request,
      std::chrono::milliseconds timeout = std::chrono::milliseconds(500), Callback<wire::OpenSessionResponse> callback = {});
  RequestHandle<wire::SubmitReceipt> submit_async(const control_link_edge_protocol::CommandFrame &target,
      std::chrono::milliseconds timeout = std::chrono::milliseconds(500), Callback<wire::SubmitReceipt> callback = {});
  RequestHandle<wire::StatusEnvelope> get_status_async(
      std::chrono::milliseconds timeout = std::chrono::milliseconds(500), Callback<wire::StatusEnvelope> callback = {});
  RequestHandle<wire::Capabilities> get_capabilities_async(
      std::chrono::milliseconds timeout = std::chrono::milliseconds(500), Callback<wire::Capabilities> callback = {});
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
