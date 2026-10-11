#include "control_link_someip_transport/client.hpp"
#include "control_link_someip_transport/server.hpp"

#include <openssl/evp.h>
#include <vsomeip/vsomeip.hpp>

#include <array>
#include <algorithm>
#include <cerrno>
#include <chrono>
#include <condition_variable>
#include <cstdlib>
#include <cstring>
#include <filesystem>
#include <iomanip>
#include <iostream>
#include <limits>
#include <memory>
#include <mutex>
#include <optional>
#include <sstream>
#include <stdexcept>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include <unistd.h>

namespace {

namespace net = control_link::someip;
namespace wire = control_link::network::v1;
namespace codec = control_link_edge_protocol;
using Clock = std::chrono::steady_clock;
using Milliseconds = std::chrono::milliseconds;

void require(bool condition, const std::string &message) {
  if (!condition) throw std::runtime_error(message);
}

std::string quoted(const std::string &text) {
  std::ostringstream output;
  output << '"';
  for (unsigned char value : text) {
    switch (value) {
      case '"': output << "\\\""; break;
      case '\\': output << "\\\\"; break;
      case '\n': output << "\\n"; break;
      case '\r': output << "\\r"; break;
      case '\t': output << "\\t"; break;
      default:
        if (value < 0x20) output << "\\u" << std::hex << std::setw(4) << std::setfill('0') << unsigned(value);
        else output << char(value);
    }
  }
  output << '"';
  return output.str();
}

std::string hex(const std::string &bytes) {
  std::ostringstream output;
  output << std::hex << std::setfill('0');
  for (unsigned char value : bytes) output << std::setw(2) << unsigned(value);
  return output.str();
}

std::string digest(const std::string &bytes) {
  std::array<unsigned char, EVP_MAX_MD_SIZE> result{};
  unsigned int size = 0;
  require(EVP_Digest(bytes.data(), bytes.size(), result.data(), &size, EVP_sha256(), nullptr) == 1 && size == 32,
          "Token hash failed");
  return hex(std::string(reinterpret_cast<const char *>(result.data()), size));
}

// JSONL 单 owner；写盘/管道错误直接失败。vsomeip 的文本日志通过独立 RAII redirect 去 stderr。
class JsonLog final {
 public:
  void emit(const std::string &event,
            std::initializer_list<std::pair<std::string, std::string>> fields = {}) {
    std::ostringstream output;
    output << "{\"event\":" << quoted(event)
           << ",\"wall_clock_ns\":" << net::system_now_ns()
           << ",\"monotonic_ns\":" << net::steady_now_ns();
    for (const auto &field : fields) output << ',' << quoted(field.first) << ':' << field.second;
    output << "}\n";
    const auto record = output.str();
    std::size_t offset = 0;
    while (offset < record.size()) {
      const auto written = ::write(STDOUT_FILENO, record.data() + offset, record.size() - offset);
      if (written < 0 && errno == EINTR) continue;
      if (written <= 0) throw std::runtime_error("JSONL output write failed");
      offset += static_cast<std::size_t>(written);
    }
  }
};

class LibraryLogRedirect final {
 public:
  LibraryLogRedirect() : previous_(std::cout.rdbuf(std::cerr.rdbuf())) {}
  ~LibraryLogRedirect() { std::cout.rdbuf(previous_); }
 private:
  std::streambuf *previous_;
};

std::string serialize(const google::protobuf::MessageLite &value) {
  std::string bytes;
  require(value.SerializeToString(&bytes), "Protobuf serialization failed");
  return bytes;
}

template <typename Message>
Message parse(const std::string &bytes) {
  Message value;
  require(value.ParseFromArray(bytes.data(), static_cast<int>(bytes.size())), "Protobuf response parse failed");
  return value;
}

std::string command_bytes(const codec::CommandFrame &frame) {
  std::array<std::byte, codec::kCommandFrameSize> bytes{};
  require(codec::encode_command_layout(frame, bytes.data(), bytes.size()), "Command layout encode failed");
  return std::string(reinterpret_cast<const char *>(bytes.data()), bytes.size());
}

bool readonly_redacted(const wire::StatusSnapshot &status) {
  return status.session().session_token().empty() &&
         status.latest_forwarded().key().session_token().empty() &&
         status.latest_gateway_acceptance().key().session_token().empty();
}

struct RawReply {
  vsomeip::return_code_e code{vsomeip::return_code_e::E_UNKNOWN};
  vsomeip::message_type_e type{vsomeip::message_type_e::MT_UNKNOWN};
  std::string bytes;
};

struct DroppedOpenIdentity {
  std::string token_sha256;
  std::string gid;
};

// 仅为异常协议夹具：公有 vsomeip API、在途1、bounded wait，不复制任何生产 session/validator。
class RawPeer final {
 public:
  RawPeer(net::Config config, std::string json, std::string name)
      : config_(std::move(config)), path_(std::move(json)), name_(std::move(name)) {}
  ~RawPeer() { stop(); }

  void start() {
    require(!app_, "Raw peer already started");
    require(std::filesystem::is_regular_file(path_), "Explicit network JSON required");
    const auto path = std::filesystem::canonical(path_).string();
    const char *old = std::getenv("VSOMEIP_CONFIGURATION");
    require(old == nullptr || std::filesystem::weakly_canonical(old).string() == path,
            "VSOMEIP_CONFIGURATION conflicts with explicit JSON");
    require(::setenv("VSOMEIP_CONFIGURATION", path.c_str(), 1) == 0, "Cannot configure vsomeip JSON");
    app_ = vsomeip::runtime::get()->create_application(name_);
    require(app_ && app_->init(), "Raw application init failed");
    try {
      running_ = true;
      app_->register_state_handler([this](vsomeip::state_type_e state) {
        std::lock_guard<std::mutex> lock(mutex_);
        registered_ = state == vsomeip::state_type_e::ST_REGISTERED;
        if (registered_ && running_) {
          app_->request_service(config_.service_id, config_.instance_id, config_.major_version, config_.minor_version);
          app_->request_event(config_.service_id, config_.instance_id, config_.gateway_event,
              {config_.gateway_eventgroup}, vsomeip::event_type_e::ET_EVENT, vsomeip::reliability_type_e::RT_RELIABLE);
          app_->request_event(config_.service_id, config_.instance_id, config_.vehicle_event,
              {config_.vehicle_eventgroup}, vsomeip::event_type_e::ET_EVENT, vsomeip::reliability_type_e::RT_RELIABLE);
        }
        if (!registered_) available_ = false;
        wake_.notify_all();
      });
      app_->register_availability_handler(config_.service_id, config_.instance_id,
          [this](vsomeip::service_t, vsomeip::instance_t, bool available) {
            std::lock_guard<std::mutex> lock(mutex_);
            available_ = available;
            if (available && enabled_ && running_) subscribe_locked(true);
            wake_.notify_all();
          }, config_.major_version, config_.minor_version);
      app_->register_message_handler(config_.service_id, config_.instance_id, vsomeip::ANY_METHOD,
          [this](const std::shared_ptr<vsomeip::message> &message) { receive(message); });
      thread_ = std::thread([this] {
        try {
          app_->start();
        } catch (...) {
          std::lock_guard<std::mutex> lock(mutex_);
          loop_error_ = true;
        }
        std::lock_guard<std::mutex> lock(mutex_);
        running_ = false;
        available_ = false;
        wake_.notify_all();
      });
    } catch (...) {
      stop();
      throw;
    }
  }

  void stop() noexcept {
    if (!app_) return;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      running_ = false;
      available_ = false;
    }
    wake_.notify_all();
    try {
      if (thread_.joinable()) app_->stop();
    } catch (...) {
    }
    if (thread_.joinable()) thread_.join();
    try { app_->clear_all_handler(); } catch (...) {}
    app_.reset();
  }

  bool wait_available(Milliseconds timeout) {
    std::unique_lock<std::mutex> lock(mutex_);
    wake_.wait_for(lock, timeout, [this] { return !running_ || (registered_ && available_); });
    return running_ && registered_ && available_ && !loop_error_;
  }

  void subscription(bool enabled) {
    std::lock_guard<std::mutex> lock(mutex_);
    enabled_ = enabled;
    if (running_ && available_) subscribe_locked(enabled);
  }

  RawReply request(std::uint16_t method, const std::string &bytes, Milliseconds timeout = Milliseconds(100)) {
    require(bytes.size() <= config_.max_payload_bytes && timeout.count() > 0, "Raw request exceeds resource bound");
    std::unique_lock<std::mutex> lock(mutex_);
    require(running_ && registered_ && available_ && !pending_, "Raw peer unavailable or multiple inflight");
    auto request = vsomeip::runtime::get()->create_request(true);
    request->set_service(config_.service_id);
    request->set_instance(config_.instance_id);
    request->set_interface_version(config_.major_version);
    request->set_method(method);
    auto payload = vsomeip::runtime::get()->create_payload();
    payload->set_data(reinterpret_cast<const vsomeip::byte_t *>(bytes.data()),
                      static_cast<vsomeip::length_t>(bytes.size()));
    request->set_payload(payload);
    pending_ = request;
    reply_.reset();
    try {
      app_->send(request);
      const bool received = wake_.wait_for(lock, timeout, [this] { return reply_.has_value() || !running_ || !available_; });
      pending_.reset();
      require(received && reply_.has_value(), "Raw response timeout/offline");
      return *reply_;
    } catch (...) {
      pending_.reset();
      throw;
    }
  }

  wire::StatusSnapshot snapshot() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return snapshot_;
  }

  DroppedOpenIdentity discard_first_open(const wire::OpenSessionRequest &request) {
    {
      std::lock_guard<std::mutex> lock(mutex_);
      discard_next_open_ = true;
      discarded_open_.reset();
    }
    bool wait_failed = false;
    try {
      static_cast<void>(this->request(config_.open_method, serialize(request)));
    } catch (const std::exception &) {
      wait_failed = true;
    }
    std::lock_guard<std::mutex> lock(mutex_);
    discard_next_open_ = false;
    require(wait_failed && discarded_open_.has_value() && running_ && available_ && !loop_error_,
            "Expected first Open response was not captured/discarded before timeout");
    return *discarded_open_;
  }

  void merge_status(const wire::StatusSnapshot &status) {
    std::lock_guard<std::mutex> lock(mutex_);
    merge_locked(status);
  }

 private:
  void subscribe_locked(bool enabled) {
    for (auto group : {config_.gateway_eventgroup, config_.vehicle_eventgroup}) {
      if (enabled) app_->subscribe(config_.service_id, config_.instance_id, group, config_.major_version);
      else app_->unsubscribe(config_.service_id, config_.instance_id, group);
    }
  }

  void merge_locked(const wire::StatusSnapshot &status) {
    require(readonly_redacted(status), "Readonly control token leaked");
    if (status.has_session()) *snapshot_.mutable_session() = status.session();
    if (status.has_transport()) *snapshot_.mutable_transport() = status.transport();
    if (status.has_gateway()) *snapshot_.mutable_gateway() = status.gateway();
    if (status.has_vehicle()) *snapshot_.mutable_vehicle() = status.vehicle();
    if (status.sources_size() > 0) snapshot_.mutable_sources()->CopyFrom(status.sources());
  }

  void receive(const std::shared_ptr<vsomeip::message> &message) noexcept {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!running_ || !message) return;
    try {
      if (message->get_interface_version() != config_.major_version || !message->is_reliable()) return;
      const auto payload = message->get_payload();
      require(payload && payload->get_length() <= config_.max_payload_bytes, "Oversized raw response");
      std::string bytes(reinterpret_cast<const char *>(payload->get_data()), payload->get_length());
      const auto type = message->get_message_type();
      if (type == vsomeip::message_type_e::MT_NOTIFICATION && enabled_) {
        if (message->get_method() != config_.gateway_event && message->get_method() != config_.vehicle_event) return;
        const auto status = parse<wire::StatusEnvelope>(bytes);
        require(status.schema_version_presence_case() == wire::StatusEnvelope::kSchemaVersion &&
                status.schema_version() == 1 && status.schema_fingerprint() == net::network_schema_fingerprint() &&
                status.publisher_instance_token().size() == 16 && status.has_status(),
                "Raw status schema/fingerprint mismatch");
        merge_locked(status.status());
      } else if (pending_ && !reply_ && message->get_session() == pending_->get_session() &&
          message->get_client() == pending_->get_client() && message->get_method() == pending_->get_method() &&
          (type == vsomeip::message_type_e::MT_RESPONSE || type == vsomeip::message_type_e::MT_ERROR)) {
        if (discard_next_open_ && message->get_method() == config_.open_method &&
            type == vsomeip::message_type_e::MT_RESPONSE && message->get_return_code() == vsomeip::return_code_e::E_OK) {
          const auto first = parse<wire::OpenSessionResponse>(bytes);
          require(first.has_session() && first.session().session_token().size() == 16,
                  "First Open to discard was not a holder session");
          discarded_open_ = DroppedOpenIdentity{digest(first.session().session_token()),
                                                first.session().ingress_publisher().gid()};
          discard_next_open_ = false;
          // 不向等待 owner 交付响应/token；只保留hash/GID供随后同键重试比对，真实请求等待到期。
          return;
        }
        reply_ = RawReply{message->get_return_code(), type, std::move(bytes)};
        wake_.notify_all();
      }
    } catch (...) {
      loop_error_ = true;
      running_ = false;
      wake_.notify_all();
    }
  }

  net::Config config_;
  std::string path_;
  std::string name_;
  std::shared_ptr<vsomeip::application> app_;
  std::thread thread_;
  mutable std::mutex mutex_;
  std::condition_variable wake_;
  bool running_{false};
  bool registered_{false};
  bool available_{false};
  bool enabled_{true};
  bool loop_error_{false};
  bool discard_next_open_{false};
  std::optional<DroppedOpenIdentity> discarded_open_;
  std::shared_ptr<vsomeip::message> pending_;
  std::optional<RawReply> reply_;
  wire::StatusSnapshot snapshot_;
};

// 两个后端在独立进程运行：SDK证明安装消费，raw后端才发送非法布局，不覆盖SDK handlers。
class Peer final {
 public:
  Peer(net::Config config, const std::string &path, const std::string &name, bool raw)
      : config_(std::move(config)) {
    if (raw) raw_ = std::make_unique<RawPeer>(config_, path, name);
    else sdk_ = std::make_unique<net::SomeipClient>(config_, path, name);
  }
  void start() {
    if (sdk_) sdk_->start(); else raw_->start();
    require(sdk_ ? sdk_->wait_available(Milliseconds(10000)) : raw_->wait_available(Milliseconds(10000)),
            "Service discovery did not become available within 10s");
  }
  void stop() { if (sdk_) sdk_->stop(); else raw_->stop(); }
  bool is_raw() const { return raw_ != nullptr; }
  DroppedOpenIdentity discard_first_open(const wire::OpenSessionRequest &request) {
    require(raw_ != nullptr, "Callback-level response discard requires raw backend");
    return raw_->discard_first_open(request);
  }
  void subscription(bool enabled) {
    if (sdk_) sdk_->set_status_subscription(enabled); else raw_->subscription(enabled);
  }
  wire::OpenSessionResponse open(const wire::OpenSessionRequest &request) {
    return sdk_ ? sdk_->open(request, Milliseconds(100)) :
        response<wire::OpenSessionResponse>(config_.open_method, serialize(request));
  }
  wire::OpenSessionResponse keep(const wire::KeepSessionRequest &request) {
    return sdk_ ? sdk_->keep(request, Milliseconds(20)) :
        response<wire::OpenSessionResponse>(config_.keep_method, serialize(request), Milliseconds(20));
  }
  wire::OpenSessionResponse close(const wire::CloseSessionRequest &request) {
    return sdk_ ? sdk_->close(request, Milliseconds(100)) :
        response<wire::OpenSessionResponse>(config_.close_method, serialize(request));
  }
  wire::SubmitReceipt submit(const codec::CommandFrame &frame) {
    return sdk_ ? sdk_->submit(frame, Milliseconds(20)) :
        response<wire::SubmitReceipt>(config_.submit_method, command_bytes(frame), Milliseconds(20));
  }
  wire::StatusEnvelope get_status() {
    if (sdk_) return sdk_->get_status(Milliseconds(20));
    wire::GetStatusRequest request;
    request.set_schema_version(1);
    auto result = response<wire::StatusEnvelope>(config_.get_status_method, serialize(request), Milliseconds(20));
    require(result.schema_version_presence_case() == wire::StatusEnvelope::kSchemaVersion &&
            result.schema_version() == 1 && result.schema_fingerprint() == net::network_schema_fingerprint() &&
            result.publisher_instance_token().size() == 16 && result.has_status(), "Raw GetStatus envelope mismatch");
    require(readonly_redacted(result.status()), "GetStatus leaked control token");
    raw_->merge_status(result.status());
    return result;
  }
  wire::StatusSnapshot snapshot() const {
    return sdk_ ? sdk_->status().observations : raw_->snapshot();
  }
  RawReply raw_request(std::uint16_t method, const std::string &bytes) {
    require(raw_ != nullptr, "Malformed requests require --backend raw");
    return raw_->request(method, bytes);
  }
 private:
  template <typename Message>
  Message response(std::uint16_t method, const std::string &bytes, Milliseconds timeout = Milliseconds(100)) {
    const auto reply = raw_->request(method, bytes, timeout);
    require(reply.code == vsomeip::return_code_e::E_OK && reply.type == vsomeip::message_type_e::MT_RESPONSE,
            "Unexpected SOME/IP error response");
    return parse<Message>(reply.bytes);
  }
  net::Config config_;
  std::unique_ptr<net::SomeipClient> sdk_;
  std::unique_ptr<RawPeer> raw_;
};

struct Options {
  std::string config;
  std::string json;
  std::string app;
  std::string mode{"session"};
  std::string backend{"sdk"};
  std::string bad_case{"all"};
  std::uint32_t period_ms{50};
  std::uint32_t keep_ms{1000};
  std::uint32_t baseline_ms{1500};
  std::uint32_t pause_ms{400};
  std::uint32_t recover_ms{1500};
  std::uint32_t observe_ms{300};
  bool require_active{false};
};

std::uint32_t numeric(const std::string &text) {
  require(!text.empty() && text.front() != '-', "Expected nonnegative numeric option");
  std::size_t used = 0;
  const auto value = std::stoull(text, &used);
  require(used == text.size() && value <= 60000, "Option must fit bounded 0..60000ms range");
  return static_cast<std::uint32_t>(value);
}

Options options(int argc, char **argv) {
  Options value;
  for (int index = 1; index < argc; ++index) {
    const std::string name(argv[index]);
    if (name == "--require-active") { value.require_active = true; continue; }
    require(index + 1 < argc, "Missing argument for " + name);
    const std::string argument(argv[++index]);
    if (name == "--config") value.config = argument;
    else if (name == "--network-json" || name == "--vsomeip-config") value.json = argument;
    else if (name == "--app-name" || name == "--application") value.app = argument;
    else if (name == "--mode") value.mode = argument;
    else if (name == "--backend") value.backend = argument;
    else if (name == "--case") value.bad_case = argument;
    else if (name == "--period-ms" || name == "--run-period-ms") value.period_ms = numeric(argument);
    else if (name == "--keep-ms") value.keep_ms = numeric(argument);
    else if (name == "--baseline-ms") value.baseline_ms = numeric(argument);
    else if (name == "--pause-ms") value.pause_ms = numeric(argument);
    else if (name == "--recover-ms") value.recover_ms = numeric(argument);
    else if (name == "--observe-ms") value.observe_ms = numeric(argument);
    else throw std::invalid_argument("Unknown option " + name);
  }
  require(!value.config.empty() && !value.json.empty() && !value.app.empty(), "Explicit config/network JSON/app-name required");
  require(value.period_ms == 50 && value.keep_ms == 1000, "This fixture fixes period=50ms/Keep=1000ms");
  require(value.backend == "sdk" || value.backend == "raw", "Backend must be sdk or raw");
  require(value.mode == "session" || value.mode == "second-peer" || value.mode == "malformed" ||
          value.mode == "hold" || value.mode == "idle" || value.mode == "server-restart",
          "Mode must be session/second-peer/malformed/hold/idle/server-restart");
  if (value.mode == "malformed") value.backend = "raw";
  require(value.baseline_ms > 0 && value.recover_ms > 0 && value.pause_ms > 100,
          "Baseline/recovery positive, pause must exceed unmodified Contract100ms");
  return value;
}

// 以下只断言协议身份/receipt，不在夹具中复制 Validator、lease、arbiter 或 canonical规则。
class Probe final {
 public:
  Probe(const Options &options, const net::Config &config, Peer &peer, JsonLog &log)
      : options_(options), config_(config), peer_(peer), log_(log) { new_keys(); }

  void run() {
    if (options_.mode == "second-peer") { second_peer(); return; }
    establish(true);
    pump("baseline", options_.baseline_ms, true);
    if (options_.mode == "hold") {
      log_.emit("holder_running", {{"token_sha256", quoted(digest(session_.session_token()))},
          {"gid", quoted(hex(session_.ingress_publisher().gid()))}});
      pump("hold", options_.recover_ms, true);
      close();
      return;
    }
    if (options_.mode == "server-restart") server_restart_checks();
    else if (options_.mode == "idle") idle_checks();
    else if (options_.mode == "malformed") malformed();
    else session_checks();
    close();
    log_.emit("protocol_checks_complete", {{"business_acceptance_verified", "false"},
              {"can_verified", "false"}});
  }

 private:
  void new_keys() {
    request_.Clear();
    request_.set_schema_version(1);
    request_.set_holder_nonce(net::random_identity());
    request_.set_open_request_id(net::random_identity());
    keep_sequence_ = 0;
    command_sequence_ = 0;
    next_keep_ = Clock::now();
  }

  void log_session(const std::string &event, const wire::OpenSessionResponse &response) {
    log_.emit(event, {{"result", std::to_string(response.result())},
        {"has_session", response.has_session() ? "true" : "false"},
        {"ready", response.session().submission_ready() ? "true" : "false"},
        {"token_sha256", quoted(response.has_session() ? digest(response.session().session_token()) : "")},
        {"gid", quoted(hex(response.session().ingress_publisher().gid()))},
        {"open_key_sha256", quoted(digest(request_.open_request_id()))}});
  }

  void validate_reply(const wire::OpenSessionResponse &response) {
    require(response.open_request_id() == request_.open_request_id() && response.has_capabilities() &&
            response.capabilities().schema_version() == 1 &&
            response.capabilities().schema_fingerprint() == net::network_schema_fingerprint(),
            "Session response key/schema/fingerprint mismatch");
  }

  void same_session(const wire::OpenSessionResponse &response) {
    validate_reply(response);
    require(response.has_session() && response.session().session_token() == session_.session_token(),
            "Live idempotent/Keep operation rotated token");
    if (!session_.ingress_publisher().gid().empty()) {
      require(response.session().ingress_publisher().gid() == session_.ingress_publisher().gid(),
              "Live idempotent/Keep operation rotated publisher GID");
    }
    session_ = response.session();
  }

  void keep() {
    if (Clock::now() < next_keep_) return;
    wire::KeepSessionRequest request;
    request.set_schema_version(1);
    request.set_holder_nonce(request_.holder_nonce());
    request.set_open_request_id(request_.open_request_id());
    request.set_keepalive_sequence(++keep_sequence_);
    next_keep_ = Clock::now() + Milliseconds(options_.keep_ms);
    const auto response = peer_.keep(request);
    log_session("keep", response);
    same_session(response);
  }

  void establish(bool discard_first) {
    wire::OpenSessionResponse response;
    if (discard_first && peer_.is_raw()) {
      const auto first = peer_.discard_first_open(request_);
      log_.emit("first_open_reply_discarded_in_raw_callback", {{"token_sha256", quoted(first.token_sha256)},
          {"gid", quoted(hex(first.gid))}, {"bounded_request_timed_out", "true"}});
      response = peer_.open(request_);
      validate_reply(response);
      require(response.has_session() && response.result() == wire::SESSION_OPERATION_RESULT_IDEMPOTENT &&
              digest(response.session().session_token()) == first.token_sha256,
              "Callback-discarded first Open retry rotated token/not idempotent");
      if (!first.gid.empty()) require(response.session().ingress_publisher().gid() == first.gid,
                                     "Callback-discarded first Open retry rotated GID");
      session_ = response.session();
      log_session("same_key_retry", response);
    } else {
      // SDK不覆盖生产handler；调用方应用级忽略首个回复，不宣称SDK内部timeout/TCP丢包。
      response = peer_.open(request_);
      validate_reply(response);
      require(response.has_session() && response.session().session_token().size() == 16,
              "First holder Open was rejected/busy");
      session_ = response.session();
    }
    if (discard_first && !peer_.is_raw()) {
      log_session("first_open_reply_discarded_at_application", response);
      response = peer_.open(request_);
      log_session("same_key_retry", response);
      require(response.result() == wire::SESSION_OPERATION_RESULT_IDEMPOTENT,
              "Same-holder response-loss retry was not idempotent");
      same_session(response);
    }
    const auto deadline = Clock::now() + Milliseconds(config_.graph_timeout_ms);
    while (!session_.submission_ready()) {
      require(Clock::now() < deadline, "Graph/session admission deadline expired");
      keep();
      std::this_thread::sleep_for(Milliseconds(50));
      response = peer_.open(request_);
      same_session(response);
      log_session("admission_probe", response);
    }
    require(session_.ingress_publisher().gid().size() == 24 &&
            session_.planning_transport() == wire::PLANNING_TRANSPORT_SOMEIP &&
            session_.clock_domain() == wire::CLOCK_DOMAIN_SYSTEM, "Admitted identity/domain mismatch");
    log_session("admitted", response);
  }

  codec::CommandFrame target() {
    require(session_.session_token().size() == codec::kSessionTokenSize, "No usable holder token");
    require(command_sequence_ < std::numeric_limits<std::uint64_t>::max(), "Business sequence exhausted");
    codec::CommandFrame frame;
    std::memcpy(frame.session_token.bytes.data(), session_.session_token().data(), codec::kSessionTokenSize);
    frame.command_sequence = ++command_sequence_;
    frame.source_stamp_ns = net::system_now_ns();
    frame.linear_velocity_mps = 0.3;
    frame.angular_velocity_radps = 0.0;
    return frame;
  }

  void receipt(const std::string &kind, const codec::CommandFrame &frame,
               const wire::SubmitReceipt &response, bool rejected = false) {
    const std::string token(reinterpret_cast<const char *>(frame.session_token.bytes.data()), codec::kSessionTokenSize);
    require(response.schema_version_presence_case() == wire::SubmitReceipt::kSchemaVersion &&
            response.schema_version() == 1 && response.rpc_received() && response.has_key() &&
            response.rpc_received_boottime_presence_case() == wire::SubmitReceipt::kRpcReceivedBoottimeNs &&
            response.rpc_received_boottime_ns() > 0 && response.has_gateway_acceptance() &&
            response.key().source_sequence_presence_case() == wire::CommandKey::kSourceSequence &&
            response.key().source_sequence() == frame.command_sequence && response.key().session_token() == token,
            "Submit receipt does not match original target");
    require(response.gateway_acceptance().state() == wire::GATEWAY_ACCEPTANCE_STATE_UNKNOWN,
            "Immediate receipt falsely claimed Gateway acceptance");
    if (rejected) require(response.forwarding_state() == wire::FORWARDING_STATE_REJECTED,
                          "Expected protocol rejection was queued/forwarded");
    else require(response.forwarding_state() == wire::FORWARDING_STATE_PENDING ||
                 response.forwarding_state() == wire::FORWARDING_STATE_FORWARDED,
                 "Layout-representable business target was rejected before Gateway");
    log_.emit("submit", {{"kind", quoted(kind)}, {"source_sequence", std::to_string(frame.command_sequence)},
        {"source_stamp_ns", std::to_string(frame.source_stamp_ns)}, {"token_sha256", quoted(digest(token))},
        {"gid", quoted(hex(session_.ingress_publisher().gid()))}, {"rpc_received", "true"},
        {"forwarding_state", std::to_string(response.forwarding_state())}, {"gateway_acceptance", "0"}});
  }

  void observation(const std::string &phase) {
    const auto snapshot = peer_.snapshot();
    require(readonly_redacted(snapshot), "Readonly event leaked nested control token");
    std::uint64_t accepted = 0;
    std::uint64_t rejected = 0;
    std::uint64_t source_sequence = 0;
    std::int64_t source_observed_ns = 0;
    std::uint64_t source_received_boot_ns = 0;
    std::uint32_t source_validity = wire::OBSERVATION_VALIDITY_UNKNOWN;
    std::uint32_t endpoint_count = 0;
    bool source_present = false;
    bool endpoint_stable = false;
    std::string gid;
    for (const auto &source : snapshot.sources()) {
      if (source.source_id() == "planning") {
        source_present = true;
        accepted = source.accepted_count(); rejected = source.rejected_count();
        source_sequence = source.last_source_sequence(); gid = source.endpoint_publisher().gid();
        source_observed_ns = source.observation().observed_system_ns();
        source_received_boot_ns = source.observation().received_boottime_ns();
        source_validity = source.observation().validity();
        endpoint_count = source.endpoint_publisher_count();
        endpoint_stable = source.endpoint_stable();
      }
    }
    log_.emit("observation", {{"phase", quoted(phase)},
        {"gateway_state", std::to_string(snapshot.gateway().state())},
        {"gateway_validity", std::to_string(snapshot.gateway().observation().validity())},
        {"gateway_observed_system_ns", std::to_string(snapshot.gateway().observation().observed_system_ns())},
        {"gateway_received_boottime_ns", std::to_string(snapshot.gateway().observation().received_boottime_ns())},
        {"active_source", quoted(snapshot.gateway().active_source_id())},
        {"source_present", source_present ? "true" : "false"},
        {"source_validity", std::to_string(source_validity)},
        {"source_observed_system_ns", std::to_string(source_observed_ns)},
        {"source_received_boottime_ns", std::to_string(source_received_boot_ns)},
        {"endpoint_stable", endpoint_stable ? "true" : "false"},
        {"endpoint_publisher_count", std::to_string(endpoint_count)},
        {"source_gid", quoted(hex(gid))}, {"source_sequence", std::to_string(source_sequence)},
        {"accepted_count", std::to_string(accepted)}, {"rejected_count", std::to_string(rejected)},
        {"vehicle_validity", std::to_string(snapshot.vehicle().observation().validity())},
        {"vehicle_received_boottime_ns", std::to_string(snapshot.vehicle().observation().received_boottime_ns())},
        {"vehicle_observed_system_ns", std::to_string(snapshot.vehicle().observation().observed_system_ns())}});
  }

  void pump(const std::string &phase, std::uint32_t duration_ms, bool commands, bool keep_enabled = true) {
    log_.emit("phase_begin", {{"phase", quoted(phase)}, {"commands", commands ? "true" : "false"},
                              {"keep_enabled", keep_enabled ? "true" : "false"},
                              {"duration_ms", std::to_string(duration_ms)},
                              {"gid", quoted(hex(session_.ingress_publisher().gid()))}});
    const auto deadline = Clock::now() + Milliseconds(duration_ms);
    auto next_command = Clock::now();
    auto next_observation = Clock::now();
    while (Clock::now() < deadline) {
      if (keep_enabled) keep();
      const auto now = Clock::now();
      if (commands && now >= next_command) {
        last_command_ = target();
        receipt("valid", last_command_, peer_.submit(last_command_));
        next_command += Milliseconds(options_.period_ms);
        if (next_command <= now) next_command = now + Milliseconds(options_.period_ms);
      }
      if (Clock::now() >= next_observation) {
        observation(phase);
        next_observation = Clock::now() + Milliseconds(config_.status_event_period_ms);
      }
      std::this_thread::sleep_for(Milliseconds(1));
    }
    if (commands && (options_.require_active || options_.mode == "idle")) {
      const auto status = peer_.get_status().status();
      require(status.gateway().observation().validity() == wire::OBSERVATION_VALIDITY_VALID &&
              status.gateway().state() == 1 && status.gateway().active_source_id() == "planning",
              "Required planning ACTIVE precondition missing (CAN still checked externally)");
    }
    log_.emit("phase_end", {{"phase", quoted(phase)}, {"source_sequence", std::to_string(command_sequence_)},
                            {"gid", quoted(hex(session_.ingress_publisher().gid()))}});
  }

  void readonly_check(bool enabled) {
    peer_.subscription(enabled);
    const auto status = peer_.get_status();
    require(readonly_redacted(status.status()), "Readonly GetStatus leaked nested token");
    const auto response = peer_.open(request_);
    same_session(response);
    log_session(enabled ? "readonly_resubscribed_same_session" : "readonly_unsubscribed_same_session", response);
  }

  void close() {
    wire::CloseSessionRequest request;
    request.set_schema_version(1);
    request.set_holder_nonce(request_.holder_nonce());
    request.set_open_request_id(request_.open_request_id());
    const auto response = peer_.close(request);
    validate_reply(response);
    require(response.result() == wire::SESSION_OPERATION_RESULT_REVOKED &&
            !response.session().submission_ready(), "Close did not revoke holder");
    log_session("closed", response);
  }

  void await_retired_key() {
    const auto deadline = Clock::now() + Milliseconds(5000);
    while (true) {
      const auto response = peer_.open(request_);
      validate_reply(response);
      require(!response.has_session(), "Retired/revoking Open leaked or reopened old token");
      if (response.result() == wire::SESSION_OPERATION_RESULT_REJECTED) {
        log_session("retired_key_rejected", response);
        break;
      }
      require(response.result() == wire::SESSION_OPERATION_RESULT_BUSY && Clock::now() < deadline,
              "Retired Open did not reach bounded rejection");
      std::this_thread::sleep_for(Milliseconds(50));
    }
  }

  void session_checks() {
    readonly_check(false);
    pump("readonly_unsubscribed_control_continues", 500, true);
    readonly_check(true);
    pump("readonly_resubscribed_control_continues", 500, true);
    pump("command_pause_keep_alive", options_.pause_ms, false);
    pump("same_session_resume", options_.recover_ms, true);
    const auto old_token = session_.session_token();
    const auto old_gid = session_.ingress_publisher().gid();
    close();
    await_retired_key();
    new_keys();
    establish(false);
    require(session_.session_token() != old_token && session_.ingress_publisher().gid() != old_gid,
            "New admitted holder reused retired token/GID");
    auto old_target = target();
    std::memcpy(old_target.session_token.bytes.data(), old_token.data(), codec::kSessionTokenSize);
    receipt("old_token_new_timestamp", old_target, peer_.submit(old_target), true);
    pump("new_session_recovery", options_.recover_ms, true);
  }

  void idle_checks() {
    const auto old_token = session_.session_token();
    const auto old_gid = session_.ingress_publisher().gid();
    const auto margin_ms = std::max<std::uint64_t>(options_.observe_ms, config_.keep_period_ms);
    const auto idle_wait_ms = static_cast<std::uint64_t>(config_.idle_timeout_ms) + margin_ms;
    require(idle_wait_ms <= std::numeric_limits<std::uint32_t>::max(), "Idle fixture duration overflow");
    log_.emit("idle_budget", {{"idle_timeout_ms", std::to_string(config_.idle_timeout_ms)},
        {"observation_margin_ms", std::to_string(margin_ms)},
        {"old_gid", quoted(hex(old_gid))}, {"old_token_sha256", quoted(digest(old_token))}});
    pump("idle_no_commands_no_keep", static_cast<std::uint32_t>(idle_wait_ms), false, false);
    const auto status = peer_.get_status().status();
    require(readonly_redacted(status) && status.has_session() && !status.session().submission_ready() &&
            (status.session().control_session_state() == wire::CONTROL_SESSION_STATE_REVOKING ||
             status.session().control_session_state() == wire::CONTROL_SESSION_STATE_IDLE),
            "Idle timeout did not revoke holder (readonly must not renew it)");
    log_.emit("idle_revoked", {{"control_session_state", std::to_string(status.session().control_session_state())},
        {"old_gid", quoted(hex(old_gid))}, {"observed_gid", quoted(hex(status.session().ingress_publisher().gid()))},
        {"submission_ready", "false"}});
    observation("idle_revoked");
    await_retired_key();
    log_.emit("phase_begin", {{"phase", quoted("idle_reopen_same_client")}, {"old_gid", quoted(hex(old_gid))}});
    new_keys();
    establish(false);
    require(session_.session_token() != old_token && session_.ingress_publisher().gid() != old_gid,
            "Same-client idle recovery reused retired token/GID");
    log_.emit("phase_end", {{"phase", quoted("idle_reopen_same_client")},
        {"old_gid", quoted(hex(old_gid))}, {"new_gid", quoted(hex(session_.ingress_publisher().gid()))},
        {"new_token_sha256", quoted(digest(session_.session_token()))}});
    auto old_target = target();
    std::memcpy(old_target.session_token.bytes.data(), old_token.data(), codec::kSessionTokenSize);
    receipt("idle_old_token_new_timestamp", old_target, peer_.submit(old_target), true);
    pump("idle_new_session_recovery", options_.recover_ms, true);
  }

  void server_restart_checks() {
    const auto old_token = session_.session_token();
    const auto old_gid = session_.ingress_publisher().gid();
    const auto old_instance = peer_.get_status().publisher_instance_token();
    log_.emit("service_restart_requested", {{"old_gid", quoted(hex(old_gid))},
        {"old_token_sha256", quoted(digest(old_token))}});
    const auto deadline = Clock::now() + Milliseconds(config_.graph_timeout_ms);
    bool changed = false;
    while (Clock::now() < deadline) {
      try {
        if (peer_.get_status().publisher_instance_token() != old_instance) {
          changed = true;
          break;
        }
      } catch (const std::exception &) {
      }
      std::this_thread::sleep_for(Milliseconds(100));
    }
    require(changed, "A new production service instance was not observed");
    log_.emit("service_instance_changed", {{"old_gid", quoted(hex(old_gid))}});
    new_keys();
    establish(false);
    require(session_.session_token() != old_token && session_.ingress_publisher().gid() != old_gid,
            "Production service restart reused old token/GID");
    auto old_target = target();
    std::memcpy(old_target.session_token.bytes.data(), old_token.data(), codec::kSessionTokenSize);
    receipt("service_restart_old_token_new_timestamp", old_target, peer_.submit(old_target), true);
    pump("service_restart_new_session_recovery", options_.recover_ms, true);
  }

  void second_peer() {
    const auto response = peer_.open(request_);
    validate_reply(response);
    require(response.result() == wire::SESSION_OPERATION_RESULT_BUSY && !response.has_session(),
            "Second peer took holder or leaked control session");
    log_session("second_peer_busy_no_session", response);
    const auto readonly = peer_.get_status();
    require(readonly_redacted(readonly.status()), "Second peer readonly leaked token");
  }

  void error_reply(const std::string &kind, std::uint16_t method, const std::string &bytes) {
    const auto response = peer_.raw_request(method, bytes);
    require(response.type == vsomeip::message_type_e::MT_ERROR &&
            response.code == vsomeip::return_code_e::E_MALFORMED_MESSAGE,
            "Malformed/schema request was not rejected at protocol boundary");
    log_.emit("raw_protocol_error", {{"kind", quoted(kind)}, {"return_code", "9"},
                                    {"payload_bytes", std::to_string(bytes.size())}});
  }

  void malformed() {
    const std::vector<std::string> cases{"malformed", "unknown-schema", "bad-token", "old-stamp",
        "duplicate", "rollback", "nonfinite", "invalid-mode"};
    require(options_.bad_case == "all" ||
            std::find(cases.begin(), cases.end(), options_.bad_case) != cases.end(), "Unknown malformed --case");
    for (const auto &kind : cases) {
      if (options_.bad_case != "all" && options_.bad_case != kind) continue;
      pump("valid_before_" + kind, options_.baseline_ms, true);
      if (kind == "malformed") {
        auto bytes = command_bytes(last_command_);
        bytes.pop_back();
        error_reply(kind, config_.submit_method, bytes);
        error_reply("malformed_protobuf", config_.open_method, std::string(1, char(0xff)));
      } else if (kind == "unknown-schema") {
        auto invalid = request_;
        invalid.set_schema_version(2);
        error_reply(kind, config_.open_method, serialize(invalid));
        invalid.clear_schema_version();
        error_reply("missing_schema_presence", config_.open_method, serialize(invalid));
      } else {
        codec::CommandFrame frame;
        if (kind == "duplicate") frame = last_command_;
        else frame = target();
        if (kind == "bad-token") {
          const auto token = net::random_identity();
          std::memcpy(frame.session_token.bytes.data(), token.data(), codec::kSessionTokenSize);
        } else if (kind == "old-stamp") frame.source_stamp_ns -= 5000000000LL;
        else if (kind == "rollback") frame.command_sequence = last_command_.command_sequence - 1;
        else if (kind == "nonfinite") frame.linear_velocity_mps = std::numeric_limits<double>::quiet_NaN();
        else if (kind == "invalid-mode") frame.mode = static_cast<codec::CommandMode>(255);
        receipt(kind, frame, peer_.submit(frame), kind == "bad-token");
      }
      pump("observe_rejection_" + kind, options_.observe_ms, false);
      pump("recover_after_" + kind, options_.recover_ms, true);
    }
  }

  const Options &options_;
  const net::Config &config_;
  Peer &peer_;
  JsonLog &log_;
  wire::OpenSessionRequest request_;
  wire::SessionInfo session_;
  std::uint64_t keep_sequence_{0};
  std::uint64_t command_sequence_{0};
  Clock::time_point next_keep_{};
  codec::CommandFrame last_command_;
};

}

int main(int argc, char **argv) {
  JsonLog log;
  try {
    if (argc == 2 && std::string(argv[1]) == "--help") {
      log.emit("usage", {{"arguments", quoted(
          "--config YAML --network-json JSON --app-name NAME --mode session|second-peer|malformed|hold|idle|server-restart "
          "--backend sdk|raw --case all|malformed|unknown-schema|bad-token|old-stamp|duplicate|rollback|nonfinite|invalid-mode "
          "--period-ms 50 --keep-ms 1000 --baseline-ms 1500 --pause-ms 400 --recover-ms 1500 --observe-ms 300 --require-active")}});
      return 0;
    }
    const auto arguments = options(argc, argv);
    const auto config = net::load_config(arguments.config);
    require(config.keep_period_ms == arguments.keep_ms, "Fixture Keep must match configured 1000ms");
    LibraryLogRedirect redirect;
    Peer peer(config, arguments.json, arguments.app, arguments.backend == "raw");
    log.emit("process", {{"pid", std::to_string(::getpid())},
        {"exe", quoted(std::filesystem::read_symlink("/proc/self/exe").string())},
        {"mode", quoted(arguments.mode)}, {"backend", quoted(arguments.backend)},
        {"period_ms", "50"}, {"keep_ms", "1000"},
        {"config", quoted(std::filesystem::canonical(arguments.config).string())},
        {"network_json", quoted(std::filesystem::canonical(arguments.json).string())},
        {"app_name", quoted(arguments.app)},
        {"schema_sha256", quoted(hex(net::network_schema_fingerprint()))}});
    peer.start();
    log.emit("sd_available", {{"transport_only", "true"}});
    Probe probe(arguments, config, peer, log);
    probe.run();
    peer.stop();
    log.emit("exit", {{"exit_code", "0"}, {"scope", quoted("protocol_fixture_not_CAN_verdict")}});
    return 0;
  } catch (const std::exception &failure) {
    try { log.emit("failure", {{"message", quoted(failure.what())}, {"exit_code", "1"}}); }
    catch (...) { std::cerr << "JSONL evidence output failed\n"; }
    std::cerr << "network_session_probe: " << failure.what() << '\n';
    return 1;
  }
}
