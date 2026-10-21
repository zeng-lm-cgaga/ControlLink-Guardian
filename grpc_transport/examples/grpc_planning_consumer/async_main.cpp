#include "control_link_grpc_transport/client.hpp"

#include <cstring>
#include <cerrno>
#include <deque>
#include <iostream>
#include <optional>
#include <thread>
#include <csignal>
#include <pthread.h>

namespace transport = control_link::grpc_transport;
namespace wire = control_link::network::v1;
using namespace std::chrono_literals;

namespace {
class StopSignals final {
 public:
  StopSignals() {
    sigemptyset(&signals_);
    sigaddset(&signals_, SIGINT);
    sigaddset(&signals_, SIGTERM);
    if (pthread_sigmask(SIG_BLOCK, &signals_, &previous_) != 0)
      throw std::runtime_error("cannot establish signal owner");
  }
  ~StopSignals() { pthread_sigmask(SIG_SETMASK, &previous_, nullptr); }
  bool requested() const {
    const timespec immediate{0, 0};
    const int signal = sigtimedwait(&signals_, nullptr, &immediate);
    if (signal == SIGINT || signal == SIGTERM) return true;
    if (signal < 0 && errno != EAGAIN && errno != EINTR)
      throw std::runtime_error("signal observation failed");
    return false;
  }
 private:
  sigset_t signals_{};
  sigset_t previous_{};
};
}

// 单 owner 推进控制握手；每次只轮询 future，不等待RPC，Submit 与只读请求可同时在途。
int main(int argc, char **argv) {
  if (argc < 2 || argc > 3) { std::cerr << "usage: grpc_async_planning_client CONFIG.yaml [SECONDS:1..120]\n"; return 2; }
  try {
    const std::string duration = argc == 3 ? argv[2] : "10";
    if (duration.empty() || duration.size() > 3 || duration.find_first_not_of("0123456789") != std::string::npos)
      throw std::invalid_argument("bounded decimal seconds required");
    const int seconds = std::stoi(duration);
    if (seconds < 1 || seconds > 120) throw std::invalid_argument("seconds must be 1..120");
    StopSignals signals;
    transport::GrpcClient client(transport::load_config(argv[1]));
    client.start();
    auto capabilities = client.get_capabilities_async();
    std::optional<transport::RequestHandle<wire::OpenSessionResponse>> control;
    std::deque<transport::RequestHandle<wire::SubmitReceipt>> submits;
    wire::OpenSessionRequest opening;
    opening.set_schema_version(1);
    opening.set_holder_nonce(control_link::network::random_identity());
    opening.set_open_request_id(control_link::network::random_identity());
    std::uint64_t sequence = 0;
    std::uint64_t keep_sequence = 0;
    bool opened = false;
    bool failed = false;
    const auto end = std::chrono::steady_clock::now() + std::chrono::seconds(seconds);
    auto next = std::chrono::steady_clock::now();
    auto next_keep = next;
    while (std::chrono::steady_clock::now() < end && !failed && !signals.requested()) {
      const auto now = std::chrono::steady_clock::now();
      if (!opened && capabilities.future().wait_for(0ms) == std::future_status::ready) {
        const auto &result = capabilities.future().get();
        if (!result.ok()) throw std::runtime_error(result.error);
        *opening.mutable_opening_domain() = result.response.opening_domain();
        control.emplace(client.open_async(opening));
        opened = true;
      }
      if (control && control->future().wait_for(0ms) == std::future_status::ready) {
        const auto &result = control->future().get();
        if (!result.ok()) throw std::runtime_error(result.error);
        control.reset();
      }
      while (!submits.empty() && submits.front().future().wait_for(0ms) == std::future_status::ready) {
        const auto &result = submits.front().future().get();
        if (!result.ok()) { std::cerr << result.error << '\n'; failed = true; }
        submits.pop_front();
      }
      const auto status = client.status();
      if (opened && !control && now >= next_keep) {
        wire::KeepSessionRequest keep;
        keep.set_schema_version(1);
        keep.set_holder_nonce(opening.holder_nonce());
        keep.set_open_request_id(opening.open_request_id());
        *keep.mutable_opening_domain() = opening.opening_domain();
        keep.set_keepalive_sequence(++keep_sequence);
        control.emplace(client.keep_async(keep));
        next_keep = now + 1s;
      }
      if (now >= next) {
        next = now + 50ms;
        if (status.control_active && status.session.submission_ready() && submits.size() < 4) {
          control_link_edge_protocol::CommandFrame target;
          std::memcpy(target.session_token.bytes.data(), status.session.session_token().data(), target.session_token.bytes.size());
          target.command_sequence = ++sequence;
          target.source_stamp_ns = control_link::network::system_now_ns();
          target.clock_domain = control_link_edge_protocol::ClockDomain::kSystem;
          target.mode = control_link_edge_protocol::CommandMode::kNormal;
          target.linear_velocity_mps = 0.3;
          submits.emplace_back(client.submit_async(target));
        }
      }
      std::this_thread::sleep_for(1ms);
    }
    for (const auto &request : submits) request.cancel();
    client.cancel_control();
    client.stop();
    std::cout << "submitted=" << sequence << " receipt_is_not_gateway_acceptance=true\n";
    return failed ? 1 : sequence ? 0 : 2;
  } catch (const std::exception &failure) {
    std::cerr << failure.what() << '\n';
    return 1;
  }
}
