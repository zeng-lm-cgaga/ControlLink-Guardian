#pragma once

#include <cstdint>
#include <string>

#include "control_link_network_ingress/ingress.hpp"

namespace control_link::grpc_transport {

// 数值只来自网络配置 owner；零初始化不另设生产默认值，也不依赖 SOME/IP parser。
struct Config {
  std::string listen_address;
  ::control_link::network::IngressResources ingress;
  std::uint32_t max_message_bytes{0};
  std::uint32_t readonly_inflight_limit{0};
  std::uint32_t max_status_subscribers{0};
  std::uint32_t max_unary_deadline_ms{0};
  std::uint32_t control_open_timeout_ms{0};
  std::uint32_t max_control_stream_lifetime_ms{0};
  std::uint32_t max_readonly_stream_lifetime_ms{0};
  std::uint32_t shutdown_timeout_ms{0};
  std::uint32_t status_event_period_ms{0};
  std::uint32_t reconnect_initial_ms{0};
  std::uint32_t reconnect_max_ms{0};
  std::uint32_t graph_timeout_ms{0};
  std::uint32_t graph_quiet_ms{0};
  std::uint32_t graph_poll_ms{0};
};

Config load_config(const std::string &path);
void validate_config(const Config &config);

}
