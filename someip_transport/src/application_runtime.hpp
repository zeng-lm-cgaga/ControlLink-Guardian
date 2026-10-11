#pragma once

#include <cstdio>
#include <cstdlib>
#include <exception>
#include <filesystem>
#include <stdexcept>
#include <string>

namespace control_link::someip::detail {

// vsomeip优先读取按application命名的环境变量；两种入口都必须绑定已验证的同一文件。
inline void bind_application_configuration(const std::string &name, const std::string &path) {
  if (name.empty() || path.empty() || !std::filesystem::is_regular_file(path)) {
    throw std::invalid_argument("An explicit existing vsomeip JSON and application name are required");
  }
  const auto absolute = std::filesystem::canonical(path).string();
  for (const auto &variable : {std::string{"VSOMEIP_CONFIGURATION"},
                              std::string{"VSOMEIP_CONFIGURATION_"} + name}) {
    const auto *existing = std::getenv(variable.c_str());
    if (existing != nullptr && absolute != std::filesystem::weakly_canonical(existing).string()) {
      throw std::invalid_argument(variable + " conflicts with the explicit JSON path");
    }
  }
  if (::setenv("VSOMEIP_CONFIGURATION", absolute.c_str(), 1) != 0) {
    throw std::runtime_error("Could not set explicit VSOMEIP_CONFIGURATION");
  }
}

// 锁定库start抛出时可能已有部分内部线程，公共API不能保证回收；不能再等待已退出start的stop通知。
// 先撤本地资格，再诊断并进程级fail-closed。普通RPC/配置错误不走此不可恢复分支。
template <typename Start, typename Failure, typename Exited>
void run_application_loop(Start start, Failure failure, Exited exited) {
  const auto fatal = [&](const char *reason) {
    try {
      failure(reason);
    } catch (...) {
    }
    std::fprintf(stderr, "ControlLink SOME/IP application loop failed; process fail-closed: %s\n", reason);
    std::fflush(stderr);
    std::_Exit(EXIT_FAILURE);
  };
  try {
    start();
  } catch (const std::exception &error) {
    fatal(error.what());
  } catch (...) {
    fatal("Unknown exception in SOME/IP application loop");
  }
  exited();
}

}
