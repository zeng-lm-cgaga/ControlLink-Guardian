#pragma once

#include <memory>
#include <string>

#include "control_link_someip_transport/ingress.hpp"

namespace control_link::someip {

// 网络 owner 只做协议边界、资源存活和只读编码；ROS 发布由 PlanningIngress 的独立 owner 提交。
// start/stop 由同一装配线程串行调用，stop 撤资格并唤醒、join 后才释放应用 handle。
class SomeipServer final {
 public:
  SomeipServer(Config config, std::shared_ptr<PlanningIngress> ingress,
               std::string vsomeip_config_path);
  ~SomeipServer();
  SomeipServer(const SomeipServer &) = delete;
  SomeipServer &operator=(const SomeipServer &) = delete;

  void start();
  void stop() noexcept;

 private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};

// 返回 CMake 声明式生成输入的原 .proto SHA256（32B），不是 descriptor 或 Contract hash。
std::string network_schema_fingerprint();

}
