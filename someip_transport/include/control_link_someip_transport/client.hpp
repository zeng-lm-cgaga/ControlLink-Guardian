#pragma once

#include <chrono>
#include <cstdint>
#include <memory>
#include <string>

#include "control_link_someip_transport/ingress.hpp"

namespace control_link::someip {

struct ClientStatus {
  bool running{false};
  bool registered{false};
  bool service_available{false};
  // 服务消失或进程重启会使调用者放弃旧目标；只读查询/订阅不会改变控制 session。
  std::uint64_t availability_generation{0};
  std::uint64_t gateway_status_received_steady_ns{0};
  std::uint64_t vehicle_status_received_steady_ns{0};
  wire::SessionInfo session;
  wire::StatusSnapshot observations;
  std::string failure;
};

// 同一个 client 的 start/open/keep/close/submit/get_status/stop 必须由单 owner 串行调用。
// 后台只接收响应及两组独立 latest 状态；最多一个在途同步请求，超时/stop 有界唤醒。
// RPC 成功只说明收到边界回复，不能冒充 Gateway 接受或执行完成；原目标不自动重发。
class SomeipClient final {
 public:
  SomeipClient(Config config, std::string vsomeip_config_path,
               std::string application_name);
  ~SomeipClient();
  SomeipClient(const SomeipClient &) = delete;
  SomeipClient &operator=(const SomeipClient &) = delete;

  void start();
  void stop() noexcept;
  bool wait_available(std::chrono::milliseconds timeout);
  // 只影响只读订阅；不会 Close、续期或重开控制 holder/token/publisher generation。
  void set_status_subscription(bool enabled);
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
  ClientStatus status() const;

 private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};

// 安装示例共用这一个调用链；固定速度仅是输入示例，不是规划或轨迹跟踪算法。
int run_planning_example(int argc, char **argv);

}
