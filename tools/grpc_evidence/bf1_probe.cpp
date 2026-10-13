#include "control_link_grpc_transport/client.hpp"
#include "control_link_network_ingress/ingress.hpp"
#include "control_link_network_v1.grpc.pb.h"

#include <grpcpp/grpcpp.h>

#include <chrono>
#include <cstring>
#include <filesystem>
#include <iostream>
#include <stdexcept>
#include <string>
#include <thread>

namespace {
namespace transport = control_link::grpc_transport;
namespace network = control_link::network;
namespace wire = network::wire;
namespace codec = control_link_edge_protocol;
using Clock = std::chrono::steady_clock;
using Milliseconds = std::chrono::milliseconds;

void require(bool condition, const std::string &message) {
  if (!condition) throw std::runtime_error(message);
}

std::string hex(const std::string &bytes) {
  const std::string digits = "0123456789abcdef";
  std::string result;
  for (const auto value : bytes) {
    const auto byte = static_cast<unsigned char>(value);
    result.push_back(digits[byte >> 4]);
    result.push_back(digits[byte & 15]);
  }
  return result;
}

// 固定子窗口记录原墙钟与单调钟；CAN/Source/Gateway判据仅由独立观察者给出。
class Probe final {
 public:
  Probe(const std::string &configuration, std::filesystem::path gates)
      : config_(transport::load_config(configuration)), client_(config_),
        raw_(wire::PlanningIngress::NewStub(grpc::CreateChannel(
            config_.listen_address, grpc::InsecureChannelCredentials()))), gates_(std::move(gates)),
        deadline_(Clock::now() + std::chrono::seconds(100)) {}

  void run() {
    client_.start();
    require(client_.wait_available(Milliseconds{2000}), "gRPC channel unavailable");
    open_new();
    const auto original = request_;
    const auto original_token = session_.session_token();
    const auto original_gid = session_.ingress_publisher().gid();
    begin("before_rotation", true);
    send_until_gate("rotate");
    end();
    begin("revoked", false);
    client_.half_close();
    const auto wait_end = Clock::now() + Milliseconds{800};
    while (Clock::now() < wait_end) {
      budget();
      std::this_thread::sleep_for(Milliseconds{5});
    }
    end();
    open_new(original.opening_domain().generation());
    require(session_.session_token() != original_token, "Token did not change on rotation");
    require(session_.ingress_publisher().gid() != original_gid, "Publisher GID did not change on rotation");
    const auto after_epoch = client_.status().session.opening_domain().generation();
    require(after_epoch > original.opening_domain().generation(), "Opening domain did not advance");
    reject_retired_target(original_token);
    begin("after_rotation", true);
    send_until_gate("pause");
    end();
    begin("keep_only", false);
    next_keep_ = Clock::now();
    const auto pause_end = Clock::now() + Milliseconds{1500};
    while (Clock::now() < pause_end) {
      budget();
      keep_due();
      std::this_thread::sleep_for(Milliseconds{5});
    }
    end();
    begin("same_session_recovery", true);
    send_until_gate("finish");
    end();
    wire::CloseSessionRequest close;
    close.set_schema_version(1);
    close.set_holder_nonce(request_.holder_nonce());
    close.set_open_request_id(request_.open_request_id());
    *close.mutable_opening_domain() = request_.opening_domain();
    require(client_.close(close).result() == wire::SESSION_OPERATION_RESULT_REVOKED, "Final Close rejected");
    client_.stop();
    std::cout << "{\"event\":\"completed\",\"period_ms\":50,\"keep_period_ms\":1000}" << std::endl;
  }

 private:
  void budget() const {
    require(Clock::now() < deadline_, "BF1 bounded business probe timed out");
  }

  void open_new(std::uint64_t old_generation = 0) {
    request_.Clear();
    request_.set_schema_version(1);
    request_.set_holder_nonce(network::random_identity());
    request_.set_open_request_id(network::random_identity());
    keep_sequence_ = 0;
    sequence_ = 0;
    session_.Clear();
    const auto open_deadline = Clock::now() + std::chrono::seconds(40);
    while (!request_.has_opening_domain()) {
      budget();
      require(Clock::now() < open_deadline, "New opening domain not observed");
      const auto capabilities = client_.get_capabilities();
      if (capabilities.opening_domain().generation() > old_generation) {
        *request_.mutable_opening_domain() = capabilities.opening_domain();
      } else std::this_thread::sleep_for(Milliseconds{10});
    }
    auto next_open = Clock::now();
    while (true) {
      budget();
      require(Clock::now() < open_deadline, "Graph admission did not complete within fixture budget");
      if (Clock::now() >= next_open) {
        const auto response = client_.open(request_);
        require(response.has_session(), "Explicit-domain holder Open rejected");
        session_ = response.session();
        next_open = Clock::now() + Milliseconds{100};
      }
      if (session_.submission_ready()) break;
      keep_due();
      std::this_thread::sleep_for(Milliseconds{5});
    }
    next_command_ = Clock::now();
  }

  void keep_due() {
    if (Clock::now() < next_keep_ || session_.session_token().empty()) return;
    wire::KeepSessionRequest keep;
    keep.set_schema_version(1);
    keep.set_holder_nonce(request_.holder_nonce());
    keep.set_open_request_id(request_.open_request_id());
    keep.set_keepalive_sequence(++keep_sequence_);
    *keep.mutable_opening_domain() = request_.opening_domain();
    const auto response = client_.keep(keep);
    require(response.has_session(), "Keep lost the matching control holder");
    session_ = response.session();
    std::cout << "{\"event\":\"keep\",\"phase\":\"" << phase_
              << "\",\"keep_sequence\":" << keep_sequence_
              << ",\"domain\":" << request_.opening_domain().generation() << "}" << std::endl;
    next_keep_ = Clock::now() + Milliseconds{1000};
  }

  codec::CommandFrame frame(const std::string &token, std::uint64_t sequence) const {
    require(token.size() == 16, "Invalid fixture session token");
    codec::CommandFrame target;
    std::memcpy(target.session_token.bytes.data(), token.data(), token.size());
    target.command_sequence = sequence;
    target.source_stamp_ns = codec::system_time_now_ns();
    target.clock_domain = codec::ClockDomain::kSystem;
    target.linear_velocity_mps = 0.3;
    target.angular_velocity_radps = 0.1;
    return target;
  }

  void reject_retired_target(const std::string &token) const {
    wire::SubmitRequest request;
    auto *target = request.mutable_target();
    target->set_schema_version(1);
    target->mutable_key()->set_session_token(token);
    target->mutable_key()->set_source_sequence(7000);
    target->set_source_stamp_ns(codec::system_time_now_ns());
    target->set_clock_domain(wire::CLOCK_DOMAIN_SYSTEM);
    target->set_mode(wire::COMMAND_MODE_NORMAL);
    target->set_linear_velocity_mps(0.3);
    target->set_angular_velocity_radps(0.1);
    grpc::ClientContext context;
    context.set_deadline(std::chrono::system_clock::now() + Milliseconds{config_.max_unary_deadline_ms});
    wire::SubmitReceipt receipt;
    const auto result = raw_->Submit(&context, request, &receipt);
    require(result.ok() && receipt.rpc_received() &&
                receipt.forwarding_state() == wire::FORWARDING_STATE_REJECTED,
            "Production Submit did not reject retired token with a new timestamp");
    std::cout << "{\"event\":\"retired_target_rejected\",\"rpc_received\":true,\"sequence\":7000"
              << ",\"source_stamp_ns\":" << target->source_stamp_ns()
              << ",\"forwarding_state\":" << receipt.forwarding_state() << "}" << std::endl;
  }

  void send_until_gate(const std::string &name) {
    const auto started = Clock::now();
    const auto phase_deadline = started + std::chrono::seconds(12);
    while (true) {
      budget();
      require(Clock::now() < phase_deadline, "Independent ACTIVE/CAN precondition gate missing");
      keep_due();
      if (Clock::now() >= next_command_) {
        const auto target = frame(session_.session_token(), ++sequence_);
        const auto receipt = client_.submit(target);
        require(receipt.forwarding_state() == wire::FORWARDING_STATE_PENDING, "New target not queued by ingress");
        std::cout << "{\"event\":\"submitted\",\"phase\":\"" << phase_
                  << "\",\"sequence\":" << target.command_sequence
                  << ",\"stamp\":" << target.source_stamp_ns << "}" << std::endl;
        next_command_ += Milliseconds{50};
        if (next_command_ < Clock::now()) next_command_ = Clock::now() + Milliseconds{50};
      }
      if (Clock::now() - started > std::chrono::seconds(2) && std::filesystem::exists(gates_ / (name + ".ok"))) break;
      std::this_thread::sleep_for(Milliseconds{1});
    }
  }

  void begin(const std::string &name, bool commands) {
    phase_ = name;
    std::cout << "{\"event\":\"phase_begin\",\"phase\":\"" << phase_
              << "\",\"commands\":" << (commands ? "true" : "false")
              << ",\"duration_ms\":" << (commands ? 2000 : (name == "keep_only" ? 1500 : 800))
              << ",\"gid\":\"" << hex(session_.ingress_publisher().gid())
              << "\",\"session\":\"" << hex(session_.session_token())
              << "\",\"domain\":" << request_.opening_domain().generation()
              << ",\"wall_clock_ns\":" << codec::system_time_now_ns()
              << ",\"monotonic_ns\":" << network::steady_now_ns() << "}" << std::endl;
  }
  void end() {
    std::cout << "{\"event\":\"phase_end\",\"phase\":\"" << phase_
              << "\",\"wall_clock_ns\":" << codec::system_time_now_ns()
              << ",\"monotonic_ns\":" << network::steady_now_ns() << "}" << std::endl;
  }

  const transport::Config config_;
  transport::GrpcClient client_;
  std::unique_ptr<wire::PlanningIngress::Stub> raw_;
  const std::filesystem::path gates_;
  const Clock::time_point deadline_;
  wire::OpenSessionRequest request_;
  wire::SessionInfo session_;
  std::string phase_;
  std::uint64_t sequence_{0};
  std::uint64_t keep_sequence_{0};
  Clock::time_point next_keep_{};
  Clock::time_point next_command_{};
};
}

int main(int argc, char **argv) {
  std::cout.setf(std::ios::unitbuf);
  try {
    require(argc == 3, "Usage: grpc_bf1_probe NETWORK_YAML PRECONDITION_DIRECTORY");
    Probe(argv[1], argv[2]).run();
    std::cout.flush();
    std::cerr.flush();
    require(static_cast<bool>(std::cout) && static_cast<bool>(std::cerr), "Probe stdout/stderr flush failed");
    return 0;
  } catch (const std::exception &failure) {
    std::cerr << "BF1 business fixture: " << failure.what() << std::endl;
    return 1;
  }
}
