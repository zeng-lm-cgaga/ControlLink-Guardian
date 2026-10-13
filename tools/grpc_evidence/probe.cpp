#include "control_link_grpc_transport/client.hpp"
#include "control_link_network_v1.grpc.pb.h"

#include <grpcpp/grpcpp.h>
#include <google/protobuf/struct.pb.h>
#include <google/protobuf/util/json_util.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <string>
#include <thread>
#include <time.h>
#include <unistd.h>

namespace {

namespace transport = ::control_link::grpc_transport;
namespace network = ::control_link::network;
namespace wire = ::control_link::network::v1;
namespace codec = ::control_link_edge_protocol;
using Milliseconds = std::chrono::milliseconds;
using Steady = std::chrono::steady_clock;

std::uint64_t clock_ns(clockid_t domain) {
  timespec value{};
  if (::clock_gettime(domain, &value) != 0) throw std::runtime_error("clock_gettime failed");
  return static_cast<std::uint64_t>(value.tv_sec) * 1000000000ULL +
      static_cast<std::uint64_t>(value.tv_nsec);
}

std::string json_quote(const std::string &text) {
  static constexpr char digits[] = "0123456789abcdef";
  std::string result = "\"";
  for (const unsigned char character : text) {
    if (character == 34 || character == 92) {
      result += "\\";
      result += static_cast<char>(character);
    } else if (character < 32) {
      result += "\\u00";
      result += digits[character >> 4];
      result += digits[character & 15];
    } else result += static_cast<char>(character);
  }
  return result + "\"";
}

std::string hex(const std::string &bytes) {
  static constexpr char digits[] = "0123456789abcdef";
  std::string result;
  result.reserve(bytes.size() * 2);
  for (const unsigned char character : bytes) {
    result += digits[character >> 4];
    result += digits[character & 15];
  }
  return result;
}

const char *boolean(bool value) { return value ? "true" : "false"; }

void emit(const std::string &fields, std::uint64_t wall_ns = 0, std::uint64_t monotonic_ns = 0) {
  if (!wall_ns) wall_ns = clock_ns(CLOCK_REALTIME);
  if (!monotonic_ns) monotonic_ns = clock_ns(CLOCK_MONOTONIC);
  std::cout << "{" << fields << ",\"wall_clock_ns\":" << wall_ns
      << ",\"monotonic_ns\":" << monotonic_ns << "}" << std::endl;
}

std::uint32_t number(const std::string &value) {
  if (value.empty() || value.find_first_not_of("0123456789") != std::string::npos) {
    throw std::invalid_argument("Unsigned decimal argument required");
  }
  const auto parsed = std::stoull(value);
  if (parsed > std::numeric_limits<std::uint32_t>::max()) throw std::invalid_argument("Argument exceeds uint32");
  return static_cast<std::uint32_t>(parsed);
}

struct Options {
  std::string config;
  std::string address;
  std::string precondition_dir;
  std::uint32_t max_run_seconds{90};
  std::uint32_t keep_period_ms{1000};
  bool smoke_only{false};
  bool arbitration_only{false};
};

Options options(int argc, char **argv) {
  Options result;
  for (int index = 1; index < argc; ++index) {
    const std::string option = argv[index];
    if (option == "--smoke-only") { result.smoke_only = true; continue; }
    if (option == "--arbitration-only") { result.arbitration_only = true; continue; }
    if (++index == argc) throw std::invalid_argument("Missing value for " + option);
    const std::string value = argv[index];
    if (option == "--config") result.config = value;
    else if (option == "--address") result.address = value;
    else if (option == "--precondition-dir") result.precondition_dir = value;
    else if (option == "--max-run-seconds") result.max_run_seconds = number(value);
    else if (option == "--keep-period-ms") result.keep_period_ms = number(value);
    else if (option == "--period-ms") {
      if (number(value) != 50) throw std::invalid_argument("Formal probe command period is fixed at 50ms");
    } else throw std::invalid_argument("Unknown option " + option);
  }
  if (result.config.empty() || !result.keep_period_ms || !result.max_run_seconds || result.max_run_seconds > 90) {
    throw std::invalid_argument("--config required; positive Keep and max-run-seconds in [1,90]");
  }
  if (result.precondition_dir.empty()) {
    throw std::invalid_argument("--precondition-dir required: native runner must prove planning ACTIVE and CRC-valid nonzero CAN before faults");
  }
  std::error_code directory_error;
  if (!std::filesystem::is_directory(result.precondition_dir, directory_error) || directory_error) {
    throw std::invalid_argument("--precondition-dir must name an existing runner-owned directory");
  }
  return result;
}

transport::Config configuration(const Options &arguments) {
  auto result = transport::load_config(arguments.config);
  if (!arguments.address.empty()) result.listen_address = arguments.address;
  transport::validate_config(result);
  return result;
}

struct Phase {
  std::string name;
  bool commands{false};
  bool business_window{true};
  std::uint64_t wall_begin{0};
  std::uint64_t monotonic_begin{0};
  std::uint64_t sequence_begin{0};
  std::uint64_t submitted_valid{0};
  std::uint64_t submitted_fault{0};
  std::uint64_t accepted_begin{0};
  std::uint64_t rejected_begin{0};
  std::uint32_t duration_ms{0};
  std::string gid_begin;
};

// 只生成输入和记录网络/只读事实；不复制 Validator、Graph准入或控制状态机。
class Probe final {
 public:
  explicit Probe(Options arguments)
      : arguments_(std::move(arguments)), config_(configuration(arguments_)), client_(config_),
        run_id_(std::to_string(::getpid()) + "-" + std::to_string(clock_ns(CLOCK_MONOTONIC))),
        deadline_(Steady::now() + std::chrono::seconds(arguments_.max_run_seconds)) {
    grpc::ChannelArguments channel_arguments;
    channel_arguments.SetInt("grpc.enable_retries", 0);
    channel_arguments.SetMaxReceiveMessageSize(static_cast<int>(config_.max_message_bytes));
    channel_arguments.SetMaxSendMessageSize(static_cast<int>(config_.max_message_bytes));
    raw_channel_ = grpc::CreateCustomChannel(config_.listen_address,
        grpc::InsecureChannelCredentials(), channel_arguments);
    raw_ = wire::PlanningIngress::NewStub(raw_channel_);
  }

  ~Probe() {
    if (slow_context_) {
      slow_context_->TryCancel();
      if (slow_reader_) slow_reader_->Finish();
    }
    client_.stop();
  }

  void run() {
    emit("\"event\":\"probe_start\",\"run_id\":" + json_quote(run_id_) +
        ",\"pid\":" + std::to_string(::getpid()) + ",\"exe\":" +
        json_quote(std::filesystem::read_symlink("/proc/self/exe").string()) +
        ",\"address\":" + json_quote(config_.listen_address) +
        ",\"config\":" + json_quote(arguments_.config) + ",\"period_ms\":50,\"keep_period_ms\":" +
        std::to_string(arguments_.keep_period_ms) + ",\"max_run_seconds\":" +
        std::to_string(arguments_.max_run_seconds) + ",\"external_can_judge_required\":true,\"external_gate_enabled\":true,\"precondition_dir\":" +
        json_quote(arguments_.precondition_dir));
    client_.start();
    if (!client_.wait_available(timeout(500))) throw std::runtime_error("gRPC listener did not become available");
    client_.set_status_subscription(true);
    begin("initial_open", false, 35000, false);
    new_keys();
    wait_ready(std::min(deadline_, Steady::now() + std::chrono::seconds(35)));
    end();
    valid_phase("normal", 3000);
    if (arguments_.arbitration_only) {
      arbitration_and_close();
      return;
    }

    if (!arguments_.smoke_only) {
      fault_phase("duplicate");
      valid_phase("recover_duplicate", 1500);
      fault_phase("regressing");
      valid_phase("recover_regressing", 1500);
      fault_phase("stale");
      valid_phase("recover_stale", 1500);
      fault_phase("invalid");
      valid_phase("recover_invalid", 1500);
    }

    before_fault("command_pause");
    begin("command_pause", false, 600);
    interval(600, "pause");
    require_fail_closed();
    end();
    valid_phase("recover_pause", 2000);

    readonly_phase();
    before_fault("half_close");
    const auto retired_token = token_;
    const auto retired_gid = gid_;
    const auto retired_sequence = sequence_;
    const auto retired_generation = generation_;
    begin("half_close_reopen", false, 35000, false);
    const auto reopen_deadline = std::min(deadline_, Steady::now() + std::chrono::seconds(35));
    client_.half_close();
    while (client_.status().control_rpc_live) {
      budget(reopen_deadline);
      observe();
      std::this_thread::sleep_for(Milliseconds(2));
    }
    new_keys();
    wait_ready(reopen_deadline);
    if (token_ == retired_token || gid_ == retired_gid) {
      throw std::runtime_error("New control RPC did not produce a new token/GID");
    }
    emit("\"event\":\"new_session\",\"session_generation\":" + std::to_string(generation_) +
        ",\"old_gid\":" + json_quote(hex(retired_gid)) + ",\"gid\":" + json_quote(hex(gid_)) +
        ",\"retired_session_generation\":" + std::to_string(retired_generation) + ",\"token_changed\":true");
    end();

    // 新 holder ready 不是业务恢复；先发新目标并取得本轮新 GID 的外部 CAN 前置许可。
    before_fault("old_token_fresh_stamp");
    begin("old_token_fresh_stamp", false, 500);
    const auto old_token_end = std::min(deadline_, Steady::now() + Milliseconds(500));
    auto next = Steady::now();
    while (Steady::now() < old_token_end) {
      budget();
      keep_due();
      if (Steady::now() >= next) {
        const auto receipt = raw_submit(raw_target(retired_token,
            retired_sequence + phase_.submitted_fault + 1, network::system_now_ns()), "old_token");
        if (!receipt.rpc_received() || receipt.forwarding_state() != wire::FORWARDING_STATE_REJECTED) {
          throw std::runtime_error("Old token with fresh timestamp was not rejected at network boundary");
        }
        ++phase_.submitted_fault;
        next += Milliseconds(50);
      }
      observe();
      std::this_thread::sleep_for(Milliseconds(2));
    }
    end();
    valid_phase("new_session_normal", 3000);

    arbitration_and_close();
  }

 private:
  void arbitration_and_close() {

    // 仲裁专窗由 runner 划分 teleop/释放恢复子窗，不混入 planning-only 判据。
    begin("teleop_arbitration", true, 6000, false);
    interval(6000, "valid");
    const auto arbitration_state = client_.status();
    if (!arbitration_state.control_active || arbitration_state.session.session_token() != token_ ||
        arbitration_state.session.ingress_publisher().gid() != gid_) {
      throw std::runtime_error("Control holder changed during teleop arbitration fixture");
    }
    end();

    wire::CloseSessionRequest close;
    close.set_schema_version(1);
    close.set_holder_nonce(nonce_);
    close.set_open_request_id(open_key_);
    *close.mutable_opening_domain() = opening_domain_;
    const auto closed = client_.close(close, timeout(100));
    emit("\"event\":\"close\",\"result\":" + std::to_string(closed.result()));
    client_.stop();
    emit("\"event\":\"probe_complete\",\"run_id\":" + json_quote(run_id_) +
        ",\"rpc_fixture_complete\":true,\"business_verdict\":\"EXTERNAL_JUDGE_REQUIRED\"");
  }

  Options arguments_;
  transport::Config config_;
  transport::GrpcClient client_;
  std::shared_ptr<grpc::Channel> raw_channel_;
  std::unique_ptr<wire::PlanningIngress::Stub> raw_;
  std::unique_ptr<grpc::ClientContext> slow_context_;
  std::unique_ptr<grpc::ClientReader<wire::StatusEnvelope>> slow_reader_;
  std::string run_id_;
  Steady::time_point deadline_;
  std::string nonce_;
  std::string open_key_;
  wire::OpenAdmissionDomain opening_domain_;
  std::string token_;
  std::string gid_;
  std::uint64_t generation_{0};
  std::uint64_t sequence_{0};
  std::uint64_t keep_sequence_{0};
  std::uint64_t last_valid_sequence_{0};
  Steady::time_point next_keep_{};
  Steady::time_point next_observation_{};
  Phase phase_;
  bool phase_live_{false};
  bool fault_cutoff_seen_{false};
  std::uint64_t fault_accepted_cutoff_{0};
  std::uint64_t fault_reference_sequence_{0};

  void budget(Steady::time_point local_deadline = Steady::time_point::max()) const {
    if (Steady::now() >= std::min(deadline_, local_deadline)) throw std::runtime_error("Bounded probe budget exceeded");
  }

  Milliseconds timeout(std::uint32_t preferred = 20) const {
    budget();
    const auto remaining = std::chrono::duration_cast<Milliseconds>(deadline_ - Steady::now());
    if (remaining <= Milliseconds::zero()) throw std::runtime_error("No remaining RPC budget");
    return std::min({Milliseconds(preferred), Milliseconds(config_.max_unary_deadline_ms), remaining});
  }

  const wire::SourceStatusObservation *planning(const transport::ClientStatus &state) const {
    for (const auto &source : state.observations.sources()) {
      if (source.source_id() == "planning") return &source;
    }
    return nullptr;
  }

  std::uint64_t accepted() const {
    const auto state = client_.status();
    const auto *source = planning(state);
    return source ? source->accepted_count() : 0;
  }

  void begin(const std::string &name, bool commands, std::uint32_t duration_ms, bool business = true) {
    if (phase_live_) throw std::logic_error("Nested phase would hide evidence window");
    phase_ = Phase{};
    phase_.name = name;
    phase_.commands = commands;
    phase_.business_window = business;
    phase_.duration_ms = duration_ms;
    phase_.wall_begin = clock_ns(CLOCK_REALTIME);
    phase_.monotonic_begin = clock_ns(CLOCK_MONOTONIC);
    phase_.sequence_begin = sequence_;
    phase_.accepted_begin = accepted();
    const auto state = client_.status();
    const auto *source = planning(state);
    phase_.rejected_begin = source ? source->rejected_count() : 0;
    phase_.gid_begin = gid_;
    phase_live_ = true;
    emit("\"event\":\"phase_begin\",\"phase\":" + json_quote(name) +
        (business ? std::string(",\"commands\":") + boolean(commands) : std::string{}) +
        ",\"business_window\":" + boolean(business) +
        ",\"duration_ms\":" + std::to_string(duration_ms) + ",\"gid\":" + json_quote(hex(gid_)) +
        ",\"session_generation\":" + std::to_string(generation_) + ",\"sequence_start\":" +
        std::to_string(sequence_) + ",\"sequence_end\":" + std::to_string(sequence_) +
        ",\"accepted_start\":" + std::to_string(phase_.accepted_begin), phase_.wall_begin, phase_.monotonic_begin);
  }

  void end() {
    const auto wall_end = clock_ns(CLOCK_REALTIME);
    const auto monotonic_end = clock_ns(CLOCK_MONOTONIC);
    const auto elapsed = (monotonic_end - phase_.monotonic_begin) / 1000000ULL;
    emit("\"event\":\"phase_end\",\"phase\":" + json_quote(phase_.name) +
        (phase_.business_window ? std::string(",\"commands\":") + boolean(phase_.commands) : std::string{}) +
        ",\"business_window\":" + boolean(phase_.business_window) +
        ",\"duration_ms\":" + std::to_string(elapsed) + ",\"planned_duration_ms\":" +
        std::to_string(phase_.duration_ms) + ",\"gid\":" + json_quote(hex(gid_)) +
        ",\"gid_begin\":" + json_quote(hex(phase_.gid_begin)) + ",\"session_generation\":" +
        std::to_string(generation_) + ",\"sequence_start\":" + std::to_string(phase_.sequence_begin) +
        ",\"sequence_end\":" + std::to_string(sequence_) + ",\"submitted_valid\":" +
        std::to_string(phase_.submitted_valid) + ",\"submitted_fault\":" + std::to_string(phase_.submitted_fault) +
        ",\"accepted_start\":" + std::to_string(phase_.accepted_begin) + ",\"accepted_end\":" +
        std::to_string(accepted()) + ",\"begin_wall_clock_ns\":" + std::to_string(phase_.wall_begin) +
        ",\"begin_monotonic_ns\":" + std::to_string(phase_.monotonic_begin), wall_end, monotonic_end);
    phase_live_ = false;
  }

  void new_keys() {
    nonce_ = network::random_identity();
    open_key_ = network::random_identity();
    opening_domain_.Clear();
    token_.clear();
    gid_.clear();
    sequence_ = 0;
    keep_sequence_ = 0;
    last_valid_sequence_ = 0;
    ++generation_;
    next_keep_ = Steady::now();
  }

  void reply(const wire::OpenSessionResponse &response) {
    emit("\"event\":\"control_reply\",\"phase\":" + json_quote(phase_.name) +
        ",\"session_generation\":" + std::to_string(generation_) + ",\"result\":" +
        std::to_string(response.result()) + ",\"ready\":" + boolean(response.session().submission_ready()) +
        ",\"gid\":" + json_quote(hex(response.session().ingress_publisher().gid())));
    if (!response.has_session()) return;
    if (!token_.empty() && token_ != response.session().session_token()) {
      throw std::runtime_error("Token unexpectedly rotated within live RPC");
    }
    token_ = response.session().session_token();
    if (!response.session().ingress_publisher().gid().empty()) {
      if (!gid_.empty() && gid_ != response.session().ingress_publisher().gid()) {
        throw std::runtime_error("Publisher GID unexpectedly rotated within live RPC");
      }
      gid_ = response.session().ingress_publisher().gid();
    }
  }

  void keep_due() {
    if (token_.empty() || Steady::now() < next_keep_) return;
    wire::KeepSessionRequest request;
    request.set_schema_version(1);
    request.set_holder_nonce(nonce_);
    request.set_open_request_id(open_key_);
    request.set_keepalive_sequence(++keep_sequence_);
    *request.mutable_opening_domain() = opening_domain_;
    next_keep_ = Steady::now() + Milliseconds(arguments_.keep_period_ms);
    reply(client_.keep(request, timeout()));
  }

  void wait_ready(Steady::time_point local_deadline) {
    bool requested = false;
    auto next_open = Steady::now();
    while (true) {
      budget(local_deadline);
      const auto state = client_.status();
      if (requested && !state.control_rpc_live) {
        new_keys();
        requested = false;
      }
      if (!state.readonly_subscribed) client_.set_status_subscription(true);
      if (!token_.empty() && state.control_active && state.session.submission_ready() &&
          state.session.session_token() == token_ && state.session.ingress_publisher().gid().size() == 24) {
        gid_ = state.session.ingress_publisher().gid();
        return;
      }
      if (Steady::now() >= next_open) {
        wire::OpenSessionRequest request;
        request.set_schema_version(1);
        request.set_holder_nonce(nonce_);
        request.set_open_request_id(open_key_);
        if (!opening_domain_.has_generation()) opening_domain_ = client_.get_capabilities().opening_domain();
        *request.mutable_opening_domain() = opening_domain_;
        requested = true;
        next_open = Steady::now() + Milliseconds(config_.reconnect_initial_ms);
        try { reply(client_.open(request, timeout(100))); }
        catch (const std::exception &error) {
          emit("\"event\":\"open_wait\",\"error\":" + json_quote(error.what()));
        }
      }
      keep_due();
      observe();
      std::this_thread::sleep_for(Milliseconds(2));
    }
  }

  wire::SubmitRequest raw_target(const std::string &token, std::uint64_t sequence, std::int64_t stamp) const {
    wire::SubmitRequest request;
    auto *target = request.mutable_target();
    target->set_schema_version(1);
    target->mutable_key()->set_session_token(token);
    target->mutable_key()->set_source_sequence(sequence);
    target->set_source_stamp_ns(stamp);
    target->set_clock_domain(wire::CLOCK_DOMAIN_SYSTEM);
    target->set_mode(wire::COMMAND_MODE_NORMAL);
    target->set_linear_velocity_mps(0.3);
    target->set_angular_velocity_radps(0.0);
    return request;
  }

  void receipt(const wire::SubmitReceipt &value, const std::string &kind, std::uint64_t sequence,
               std::int64_t stamp, int status_code) const {
    emit("\"event\":\"rpc_boundary\",\"phase\":" + json_quote(phase_.name) + ",\"kind\":" + json_quote(kind) +
        ",\"gid\":" + json_quote(hex(gid_)) + ",\"session_generation\":" + std::to_string(generation_) +
        ",\"sequence\":" + std::to_string(sequence) + ",\"source_stamp_ns\":" + std::to_string(stamp) +
        ",\"grpc_code\":" + std::to_string(status_code) + ",\"rpc_received\":" + boolean(value.rpc_received()) +
        ",\"forwarding_state\":" + std::to_string(value.forwarding_state()) +
        ",\"gateway_acceptance\":" + std::to_string(value.gateway_acceptance().state()) +
        ",\"uses_retired_token\":" + boolean(kind == "old_token"));
  }

  wire::SubmitReceipt raw_submit(const wire::SubmitRequest &request, const std::string &kind) const {
    grpc::ClientContext context;
    context.set_deadline(std::chrono::system_clock::now() + timeout());
    context.set_wait_for_ready(false);
    wire::SubmitReceipt value;
    const auto status = raw_->Submit(&context, request, &value);
    receipt(value, kind, request.target().key().source_sequence(), request.target().source_stamp_ns(),
        static_cast<int>(status.error_code()));
    if (!status.ok()) throw std::runtime_error("Native fault RPC failed: " + status.error_message());
    return value;
  }

  void valid_command() {
    if (token_.size() != codec::kSessionTokenSize) throw std::runtime_error("No ready holder token");
    codec::CommandFrame frame;
    std::memcpy(frame.session_token.bytes.data(), token_.data(), token_.size());
    frame.command_sequence = ++sequence_;
    frame.source_stamp_ns = network::system_now_ns();
    frame.mode = codec::CommandMode::kNormal;
    frame.linear_velocity_mps = 0.3;
    const auto value = client_.submit(frame, timeout());
    receipt(value, "valid", frame.command_sequence, frame.source_stamp_ns, 0);
    if (!value.rpc_received() || value.forwarding_state() == wire::FORWARDING_STATE_REJECTED) {
      throw std::runtime_error("Valid command did not enter current ingress");
    }
    last_valid_sequence_ = frame.command_sequence;
    ++phase_.submitted_valid;
  }

  void interval(std::uint32_t duration_ms, const std::string &kind) {
    const auto until = Steady::now() + Milliseconds(duration_ms);
    fault_cutoff_seen_ = false;
    auto next_command = Steady::now();
    while (Steady::now() < until) {
      budget();
      keep_due();
      if (Steady::now() >= next_command) {
        if (kind == "valid") valid_command();
        else if (kind != "pause") {
          const auto stamp = network::system_now_ns() - (kind == "stale" ? 1000000000LL : 0);
          // 重复/倒退基于 Gateway 已观察的接受 sequence，不把 RPC收到误作业务已接受。
          const auto invalid_sequence = kind == "duplicate" ? fault_reference_sequence_ :
              (kind == "regressing" ? fault_reference_sequence_ - 1 : ++sequence_);
          auto request = raw_target(token_, invalid_sequence, stamp);
          if (kind == "invalid") request.mutable_target()->set_linear_velocity_mps(std::numeric_limits<double>::quiet_NaN());
          raw_submit(request, kind);
          ++phase_.submitted_fault;
        }
        next_command += Milliseconds(50);
        if (next_command <= Steady::now()) next_command = Steady::now() + Milliseconds(50);
      }
      if (kind != "valid" && !fault_cutoff_seen_ &&
          clock_ns(CLOCK_MONOTONIC) - phase_.monotonic_begin >= 200000000ULL) {
        fault_accepted_cutoff_ = accepted();
        fault_cutoff_seen_ = true;
      }
      observe();
      std::this_thread::sleep_for(Milliseconds(2));
    }
  }

  bool auxiliary_active(std::uint64_t wall_begin, std::uint64_t accepted_begin) const {
    const auto state = client_.status();
    const auto *source = planning(state);
    const auto &gateway = state.observations.gateway();
    const auto &vehicle = state.observations.vehicle();
    return source && state.control_active && state.session.submission_ready() &&
        state.session.session_token() == token_ && source->endpoint_generation_valid() &&
        source->endpoint_stable() && source->endpoint_publisher_count() == 1 &&
        source->endpoint_publisher().gid() == gid_ && source->accepted_count() > accepted_begin &&
        source->observation().validity() == wire::OBSERVATION_VALIDITY_VALID &&
        gateway.observation().validity() == wire::OBSERVATION_VALIDITY_VALID && gateway.state() == 1 &&
        gateway.active_source_id() == "planning" &&
        gateway.observation().observed_system_ns() >= static_cast<std::int64_t>(wall_begin) &&
        vehicle.observation().validity() == wire::OBSERVATION_VALIDITY_VALID &&
        vehicle.observation().observed_system_ns() >= static_cast<std::int64_t>(wall_begin) &&
        std::isfinite(vehicle.linear_velocity_mps()) && vehicle.linear_velocity_mps() != 0.0;
  }

  void valid_phase(const std::string &name, std::uint32_t duration_ms) {
    begin(name, true, duration_ms);
    interval(duration_ms, "valid");
    if (!auxiliary_active(phase_.wall_begin, phase_.accepted_begin)) {
      throw std::runtime_error("Valid phase did not establish current-GID planning ACTIVE/nonzero Vehicle auxiliary facts");
    }
    emit("\"event\":\"auxiliary_active\",\"phase\":" + json_quote(name) + ",\"gid\":" + json_quote(hex(gid_)) +
        ",\"external_can_proof_required\":true");
    end();
  }

  const google::protobuf::Value &gate_field(const google::protobuf::Struct &object,
                                          const std::string &name) const {
    const auto found = object.fields().find(name);
    if (found == object.fields().end()) throw std::runtime_error("Precondition receipt missing " + name);
    return found->second;
  }

  std::string gate_string(const google::protobuf::Struct &object, const std::string &name) const {
    const auto &value = gate_field(object, name);
    if (value.kind_case() != google::protobuf::Value::kStringValue) {
      throw std::runtime_error("Precondition receipt requires string " + name);
    }
    return value.string_value();
  }

  double gate_clock(const google::protobuf::Struct &object, const std::string &name) const {
    const auto &value = gate_field(object, name);
    if (value.kind_case() != google::protobuf::Value::kNumberValue ||
        !std::isfinite(value.number_value()) || value.number_value() <= 0.0) {
      throw std::runtime_error("Precondition receipt requires positive clock " + name);
    }
    return value.number_value();
  }

  bool gate_matches(const std::filesystem::path &path, const std::string &name,
                    const std::string &marker) const {
    std::error_code file_error;
    const auto file_status = std::filesystem::symlink_status(path, file_error);
    if (file_error == std::errc::no_such_file_or_directory ||
        file_status.type() == std::filesystem::file_type::not_found) return false;
    if (file_error || !std::filesystem::is_regular_file(file_status)) {
      throw std::runtime_error("Precondition gate must be a readable regular file: " + path.string());
    }
    const auto size = std::filesystem::file_size(path, file_error);
    if (file_error || size == 0 || size > 4096) {
      throw std::runtime_error("Precondition receipt exceeds bounded JSON capacity or is empty");
    }
    std::ifstream input(path, std::ios::binary);
    std::string content(static_cast<std::size_t>(size), '\0');
    input.read(content.data(), static_cast<std::streamsize>(size));
    if (!input || input.gcount() != static_cast<std::streamsize>(size) ||
        input.peek() != std::char_traits<char>::eof() || input.bad()) {
      throw std::runtime_error("Precondition receipt read failed or changed while reading");
    }
    google::protobuf::Struct proof;
    if (!google::protobuf::util::JsonStringToMessage(content, &proof).ok()) {
      throw std::runtime_error("Precondition receipt is not valid JSON");
    }
    const auto &request_value = gate_field(proof, "request");
    const auto &capture_value = gate_field(proof, "marker");
    if (request_value.kind_case() != google::protobuf::Value::kStructValue ||
        capture_value.kind_case() != google::protobuf::Value::kStructValue) {
      throw std::runtime_error("Precondition receipt must include original request and capture marker objects");
    }
    const auto &request = request_value.struct_value();
    if (gate_string(proof, "gid") != hex(gid_) ||
        gate_string(request, "event") != "fault_ready_request" ||
        gate_string(request, "phase") != name || gate_string(request, "gid") != hex(gid_) ||
        gate_string(request, "marker") != marker) return false;
    const auto &capture = capture_value.struct_value();
    // JSON number 精度只用于同域顺序/回显检查；唯一 marker 精确绑定请求，不跨时钟相减。
    if (gate_clock(request, "wall_clock_ns") != static_cast<double>(phase_.wall_begin) ||
        gate_clock(request, "monotonic_ns") != static_cast<double>(phase_.monotonic_begin) ||
        gate_clock(capture, "wall_ns") < static_cast<double>(phase_.wall_begin) ||
        gate_clock(capture, "wall_ns") > static_cast<double>(clock_ns(CLOCK_REALTIME)) ||
        gate_clock(capture, "monotonic_ns") < static_cast<double>(phase_.monotonic_begin) ||
        gate_clock(capture, "monotonic_ns") > static_cast<double>(clock_ns(CLOCK_MONOTONIC))) {
      throw std::runtime_error("Precondition receipt does not match the current same-domain capture window");
    }
    return true;
  }

  void before_fault(const std::string &name) {
    // 仅 runner 读取 CAN 并签发许可；等待期间仍发送新目标/Keep，缺许可绝不注入。
    begin("await_precondition_" + name, true, 5000, false);
    const auto marker = run_id_ + ":" + std::to_string(generation_) + ":" + name;
    const auto path = std::filesystem::path(arguments_.precondition_dir) / (name + ".ok");
    emit("\"event\":\"fault_ready_request\",\"phase\":" + json_quote(name) +
        ",\"gid\":" + json_quote(hex(gid_)) + ",\"marker\":" + json_quote(marker) +
        ",\"gate_path\":" + json_quote(path.string()) + ",\"duration_ms\":5000,\"period_ms\":50,\"keep_period_ms\":" +
        std::to_string(arguments_.keep_period_ms), phase_.wall_begin, phase_.monotonic_begin);
    const auto until = std::min(deadline_, Steady::now() + Milliseconds(5000));
    auto next_command = Steady::now();
    bool allowed = false;
    while (Steady::now() < until) {
      budget(until);
      keep_due();
      if (Steady::now() >= next_command) {
        valid_command();
        next_command += Milliseconds(50);
        if (next_command <= Steady::now()) next_command = Steady::now() + Milliseconds(50);
      }
      if (gate_matches(path, name, marker)) {
        budget(until);
        const auto state = client_.status();
        if (!state.control_active || state.session.session_token() != token_ ||
            state.session.ingress_publisher().gid() != gid_) {
          throw std::runtime_error("Control generation changed before external fault permit was consumed");
        }
        allowed = true;
        emit("\"event\":\"fault_ready_granted\",\"phase\":" + json_quote(name) +
            ",\"gid\":" + json_quote(hex(gid_)) + ",\"marker\":" + json_quote(marker) +
            ",\"gate_path\":" + json_quote(path.string()) + ",\"sdk_auxiliary_active\":" +
            boolean(auxiliary_active(phase_.wall_begin, phase_.accepted_begin)) +
            ",\"can_proof_owner\":\"native_runner\"");
        break;
      }
      observe();
      std::this_thread::sleep_for(Milliseconds(2));
    }
    end();
    if (!allowed) throw std::runtime_error("External current-GID ACTIVE/CRC CAN gate missing after 5s: " + path.string());
  }

  void fault_phase(const std::string &kind) {
    before_fault(kind);
    const auto state = client_.status();
    const auto *source = planning(state);
    if (!source || source->last_source_sequence() == 0) throw std::runtime_error("No observed accepted sequence for bad-input fixture");
    fault_reference_sequence_ = source->last_source_sequence();
    begin(kind, false, 500);
    interval(500, kind);
    require_fail_closed();
    end();
  }

  void require_fail_closed() const {
    const auto state = client_.status();
    const auto *source = planning(state);
    const auto &gateway = state.observations.gateway();
    if (!source || !fault_cutoff_seen_ || source->endpoint_publisher().gid() != gid_ ||
        source->accepted_count() != fault_accepted_cutoff_ || source->lease_valid() ||
        source->last_source_sequence() > last_valid_sequence_ ||
        (phase_.name != "command_pause" && source->rejected_count() <= phase_.rejected_begin) ||
        gateway.observation().validity() != wire::OBSERVATION_VALIDITY_VALID || gateway.state() != 3 ||
        gateway.observation().observed_system_ns() < static_cast<std::int64_t>(phase_.wall_begin)) {
      throw std::runtime_error("Stopped/rejected commands did not establish SAFE_STOP/unchanged accepted/expired lease auxiliary facts");
    }
    emit("\"event\":\"auxiliary_fail_closed\",\"phase\":" + json_quote(phase_.name) +
        ",\"gid\":" + json_quote(hex(gid_)) + ",\"external_hold_can_judge_required\":true");
  }

  void readonly_phase() {
    const auto original_token = token_;
    const auto original_gid = gid_;
    begin("readonly_cancel", true, 750);
    client_.set_status_subscription(false);
    interval(750, "valid");
    if (client_.status().session.session_token() != original_token) {
      throw std::runtime_error("Readonly cancel changed control holder");
    }
    end();
    begin("readonly_resubscribe_slow", true, 1500);
    client_.set_status_subscription(true);
    slow_context_ = std::make_unique<grpc::ClientContext>();
    const auto readonly_budget = std::min({Milliseconds(4000), Milliseconds(config_.max_readonly_stream_lifetime_ms),
        std::chrono::duration_cast<Milliseconds>(deadline_ - Steady::now())});
    if (readonly_budget <= Milliseconds::zero()) throw std::runtime_error("No remaining readonly budget");
    slow_context_->set_deadline(std::chrono::system_clock::now() + readonly_budget);
    slow_context_->set_wait_for_ready(false);
    wire::SubscribeStatusRequest request;
    request.set_schema_version(1);
    // 原生慢订阅故意从不 Read；只有此夹具对象拥有其取消和最终 Finish。
    slow_reader_ = raw_->SubscribeStatus(slow_context_.get(), request);
    interval(1500, "valid");
    slow_context_->TryCancel();
    const auto finished = slow_reader_->Finish();
    slow_reader_.reset();
    slow_context_.reset();
    emit("\"event\":\"slow_readonly_finish\",\"grpc_code\":" + std::to_string(static_cast<int>(finished.error_code())));
    if (client_.status().session.session_token() != original_token ||
        client_.status().session.ingress_publisher().gid() != original_gid ||
        !auxiliary_active(phase_.wall_begin, phase_.accepted_begin)) {
      throw std::runtime_error("Readonly resubscription/slow peer affected control identity or ACTIVE auxiliary facts");
    }
    end();
  }

  void observe() {
    if (Steady::now() < next_observation_) return;
    next_observation_ = Steady::now() + Milliseconds(100);
    const auto state = client_.status();
    const auto *source = planning(state);
    const auto &gateway = state.observations.gateway();
    const auto &vehicle = state.observations.vehicle();
    emit("\"event\":\"sdk_observation\",\"phase\":" + json_quote(phase_.name) +
        ",\"gid\":" + json_quote(hex(gid_)) + ",\"session_generation\":" + std::to_string(generation_) +
        ",\"ready\":" + boolean(state.session.submission_ready()) + ",\"gateway_state\":" +
        std::to_string(gateway.state()) + ",\"active_source\":" + json_quote(gateway.active_source_id()) +
        ",\"active_sequence\":" + std::to_string(gateway.active_source_sequence()) +
        ",\"gateway_validity\":" + std::to_string(gateway.observation().validity()) +
        ",\"gateway_observed_system_ns\":" + std::to_string(gateway.observation().observed_system_ns()) +
        ",\"planning_accepted\":" + std::to_string(source ? source->accepted_count() : 0) +
        ",\"planning_rejected\":" + std::to_string(source ? source->rejected_count() : 0) +
        ",\"planning_reject_reason\":" + std::to_string(source ? source->last_reject_reason() : 0) +
        ",\"planning_sequence\":" + std::to_string(source ? source->last_source_sequence() : 0) +
        ",\"source_gid\":" + json_quote(source ? hex(source->endpoint_publisher().gid()) : "") +
        ",\"publisher_count\":" + std::to_string(source ? source->endpoint_publisher_count() : 0) +
        ",\"vehicle_validity\":" + std::to_string(vehicle.observation().validity()) +
        ",\"vehicle_observed_system_ns\":" + std::to_string(vehicle.observation().observed_system_ns()) +
        ",\"vehicle_nonzero_finite\":" + boolean(std::isfinite(vehicle.linear_velocity_mps()) &&
          vehicle.linear_velocity_mps() != 0.0));
  }
};

}

int main(int argc, char **argv) {
  if (argc == 2 && std::string(argv[1]) == "--help") {
    std::cout << "--config YAML [--address 127.0.0.1:50051] [--period-ms 50 --keep-period-ms 1000] "
        "--precondition-dir DIR [--max-run-seconds 90] [--smoke-only | --arbitration-only]" << std::endl;
    return 0;
  }
  try {
    // 磁盘/管道写失败不得输出成功或被忽略；外部runner仍须封存observer退出/固定窗及hash。
    std::cout.exceptions(std::ios::badbit | std::ios::failbit);
    Probe probe(options(argc, argv));
    probe.run();
    return 0;
  } catch (const std::exception &error) {
    try { emit("\"event\":\"probe_failed\",\"error\":" + json_quote(error.what())); }
    catch (...) {}
    std::cerr << "grpc_protocol_probe: " << error.what() << std::endl;
    return 2;
  }
}
