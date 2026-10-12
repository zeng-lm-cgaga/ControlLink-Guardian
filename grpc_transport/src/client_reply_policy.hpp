#pragma once

#include <cstdint>

namespace control_link::grpc_transport {

// 请求发起后的只读撤准入不可被旧控制回复覆盖；revision只表达本机观察顺序。
inline bool reply_preserves_readiness(bool reply_ready, std::uint64_t request_revision,
                                     std::uint64_t observed_revision) {
  return reply_ready && request_revision == observed_revision;
}

}
