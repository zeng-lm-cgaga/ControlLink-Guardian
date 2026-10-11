#pragma once

#include <cstdint>
#include <limits>

#include "control_link_network_v1.pb.h"

namespace control_link::someip::detail {

// 只记录本地撤准入观察与 reply callback 的先后，不拥有业务状态、token/GID或命令 lease。
// 调用者用同一 client mutex 保护 observe/capture/commit；正常健康 event 不移动 revision。
class SessionReplyOrder final {
 public:
  using Revision = std::uint64_t;

  void observe_revocation() noexcept {
    if (revocation_revision_ != std::numeric_limits<Revision>::max()) ++revocation_revision_;
  }

  Revision capture_reply() const noexcept { return revocation_revision_; }

  // 旧 reply 既不能覆盖较新的缓存，也不能通过返回值给示例/调用者重新授予可提交提示。
  // revision 耗尽时永久拒绝升级，不回绕；真正更晚的 reply 在未耗尽时仍可恢复提示。
  bool commit_reply(Revision received_revision,
                    ::control_link::network::v1::SessionInfo &reply,
                    ::control_link::network::v1::SessionInfo &cached) const {
    if (revocation_revision_ == std::numeric_limits<Revision>::max() ||
        received_revision != revocation_revision_) {
      reply.set_submission_ready(false);
      reply.set_control_session_state(::control_link::network::v1::CONTROL_SESSION_STATE_UNKNOWN);
      reply.set_connection_state(::control_link::network::v1::CONNECTION_STATE_UNKNOWN);
      return false;
    }
    cached = reply;
    return true;
  }

 private:
  Revision revocation_revision_{0};
};

}
