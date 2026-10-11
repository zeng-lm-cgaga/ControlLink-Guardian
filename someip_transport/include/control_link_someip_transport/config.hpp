#pragma once

#include <cstdint>
#include <string>

namespace control_link::someip {

// 数值只从 protocol_transports.yaml 读取；零初始化不是另一套生产默认配置。
struct Config {
  std::string application_name;
  std::string bind_address;
  std::string sd_multicast_address;
  std::uint16_t service_id{0};
  std::uint16_t instance_id{0};
  std::uint16_t reliable_port{0};
  std::uint16_t sd_port{0};
  std::uint8_t major_version{0};
  std::uint32_t minor_version{0};
  std::uint16_t open_method{0};
  std::uint16_t submit_method{0};
  std::uint16_t get_status_method{0};
  std::uint16_t keep_method{0};
  std::uint16_t close_method{0};
  std::uint16_t gateway_event{0};
  std::uint16_t vehicle_event{0};
  std::uint16_t gateway_eventgroup{0};
  std::uint16_t vehicle_eventgroup{0};
  std::uint32_t max_payload_bytes{0};
  std::uint32_t readonly_inflight_limit{0};
  std::uint32_t status_event_period_ms{0};
  std::uint32_t keep_period_ms{0};
  std::uint32_t idle_timeout_ms{0};
  std::uint32_t retired_open_capacity{0};
  std::uint32_t reconnect_initial_ms{0};
  std::uint32_t reconnect_max_ms{0};
  std::uint32_t shutdown_timeout_ms{0};
  std::uint32_t graph_timeout_ms{0};
  std::uint32_t graph_quiet_ms{0};
  std::uint32_t graph_poll_ms{0};
};

// 缺失、重复、越界或非本版本字段抛出 std::invalid_argument；不探测网络依赖。
Config load_config(const std::string &path);
void validate_config(const Config &config);

// 启动前核对实际JSON与同一配置owner，禁止手写JSON静默覆盖SD、payload或队列预算。
// app ID、routing域和具体unicast是显式装配身份；其余字段须匹配本库生成契约。
void validate_vsomeip_configuration(const Config &config, const std::string &path,
                                    const std::string &application_name, bool server);

// 每个 app 自持 routing；不同 network_name 隔离本机 routing 域，TCP/SD 仍须实际取证。
std::string vsomeip_configuration_json(const Config &config,
                                      const std::string &application_name,
                                      std::uint16_t application_id,
                                      const std::string &network_name,
                                      bool server);

}
