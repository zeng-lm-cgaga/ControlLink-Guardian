#include "control_link_someip_transport/server.hpp"
#include <vsomeip/vsomeip.hpp>
#include <algorithm>
#include <array>
#include <chrono>
#include <condition_variable>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <mutex>
#include <map>
#include <sstream>
#include <thread>
#include <unistd.h>

namespace {
namespace net = control_link::someip;
namespace wire = control_link::network::v1;
namespace codec = control_link_edge_protocol;
using namespace std::chrono_literals;

void require(bool condition, const std::string &message) {
  if (!condition) throw std::runtime_error(message);
}

std::string hex(const std::string &bytes) {
  std::ostringstream result;
  for (unsigned char value : bytes) result << std::hex << std::setw(2) << std::setfill('0') << unsigned(value);
  return result.str();
}

class Log {
 public:
  explicit Log(const std::string &path) {
    require(!std::filesystem::exists(path), "refuse existing fixture log");
    output_.exceptions(std::ios::badbit | std::ios::failbit);
    output_.open(path);
  }
  void emit(const std::string &fields) {
    std::lock_guard<std::mutex> guard(mutex_);
    output_ << "{\"monotonic_ns\":" << net::steady_now_ns() << "," << fields << "}\n";
    output_.flush();
  }
 private:
  std::mutex mutex_;
  std::ofstream output_;
};

class Peer {
 public:
  Peer(net::Config config, const std::string &path, Log &log) : config_(std::move(config)), log_(log) {
    require(::setenv("VSOMEIP_CONFIGURATION", path.c_str(), 1) == 0, "set fixture JSON");
    app_ = vsomeip::runtime::get()->create_application("rate_limit_peer");
    require(app_ && app_->init(), "peer init");
    app_->register_state_handler([this](vsomeip::state_type_e state) {
      if (state == vsomeip::state_type_e::ST_REGISTERED)
        app_->request_service(config_.service_id, config_.instance_id, config_.major_version, config_.minor_version);
    });
    app_->register_availability_handler(config_.service_id, config_.instance_id,
      [this](vsomeip::service_t, vsomeip::instance_t, bool available) {
        std::lock_guard<std::mutex> guard(mutex_);
        available_ = available;
        wake_.notify_all();
      }, config_.major_version, config_.minor_version);
    app_->register_message_handler(config_.service_id, config_.instance_id, vsomeip::ANY_METHOD,
      [this](const std::shared_ptr<vsomeip::message> &message) {
        std::lock_guard<std::mutex> guard(mutex_);
        if (pending_ && message->get_session() == pending_->get_session() && message->get_method() == pending_->get_method()
            && message->get_client() == pending_->get_client() && message->get_message_type() != vsomeip::message_type_e::MT_NOTIFICATION) {
          reply_ = message;
          wake_.notify_all();
        }
      });
    for (auto pair : {std::make_pair(config_.gateway_eventgroup, config_.gateway_event),
                      std::make_pair(config_.vehicle_eventgroup, config_.vehicle_event)}) {
      app_->register_subscription_status_handler(config_.service_id, config_.instance_id, pair.first, pair.second,
        [this](vsomeip::service_t, vsomeip::instance_t, vsomeip::eventgroup_t group, vsomeip::event_t, std::uint16_t error) {
          std::lock_guard<std::mutex> guard(mutex_);
          subscription_status_[group] = error;
          log_.emit("\"event\":\"subscription_ack\",\"group\":" + std::to_string(group) + ",\"error\":" + std::to_string(error));
          wake_.notify_all();
        }, true);
      app_->request_event(config_.service_id, config_.instance_id, pair.second, {pair.first},
                          vsomeip::event_type_e::ET_EVENT, vsomeip::reliability_type_e::RT_RELIABLE);
    }
    thread_ = std::thread([this] { app_->start(); });
  }
  ~Peer() {
    app_->stop();
    if (thread_.joinable()) thread_.join();
    app_->clear_all_handler();
  }
  void ready() {
    std::unique_lock<std::mutex> guard(mutex_);
    require(wake_.wait_for(guard, 5s, [this] { return available_; }), "availability timeout");
  }
  std::shared_ptr<vsomeip::message> request(std::uint16_t method, const std::string &bytes) {
    std::unique_lock<std::mutex> guard(mutex_);
    auto request = vsomeip::runtime::get()->create_request(true);
    request->set_service(config_.service_id);
    request->set_instance(config_.instance_id);
    request->set_interface_version(config_.major_version);
    request->set_method(method);
    auto payload = vsomeip::runtime::get()->create_payload();
    payload->set_data(reinterpret_cast<const vsomeip::byte_t *>(bytes.data()), static_cast<vsomeip::length_t>(bytes.size()));
    request->set_payload(payload);
    pending_ = request;
    reply_.reset();
    app_->send(request);
    require(wake_.wait_for(guard, 2s, [this] { return static_cast<bool>(reply_); }), "reply timeout");
    auto result = reply_;
    pending_.reset();
    log_.emit("\"event\":\"wire_response\",\"method\":" + std::to_string(method)
      + ",\"client\":" + std::to_string(result->get_client()) + ",\"session\":" + std::to_string(result->get_session())
      + ",\"type\":" + std::to_string(static_cast<unsigned>(result->get_message_type()))
      + ",\"return_code\":" + std::to_string(static_cast<unsigned>(result->get_return_code()))
      + ",\"payload_hex\":\"" + hex(payload_bytes(result)) + "\"");
    return result;
  }
  bool subscriptions() {
    for (auto group : {config_.gateway_eventgroup, config_.vehicle_eventgroup})
      app_->subscribe(config_.service_id, config_.instance_id, group, config_.major_version);
    std::unique_lock<std::mutex> guard(mutex_);
    wake_.wait_for(guard, 2s, [this] { return subscription_status_.size() == 2; });
    unsigned accepted = 0, rejected = 0;
    for (const auto &item : subscription_status_) item.second == 0 ? ++accepted : ++rejected;
    log_.emit("\"event\":\"subscription_coverage\",\"accepted\":" + std::to_string(accepted)
              + ",\"rejected\":" + std::to_string(rejected));
    for (auto group : {config_.gateway_eventgroup, config_.vehicle_eventgroup})
      app_->unsubscribe(config_.service_id, config_.instance_id, group);
    return accepted == 1 && rejected == 1;
  }
  static std::string payload_bytes(const std::shared_ptr<vsomeip::message> &message) {
    const auto payload = message->get_payload();
    return std::string(reinterpret_cast<const char *>(payload->get_data()), payload->get_length());
  }
 private:
  net::Config config_;
  Log &log_;
  std::shared_ptr<vsomeip::application> app_;
  std::thread thread_;
  std::mutex mutex_;
  std::condition_variable wake_;
  bool available_{false};
  std::shared_ptr<vsomeip::message> pending_, reply_;
  std::map<std::uint16_t, std::uint16_t> subscription_status_;
};

template<typename Message>
Message decoded(const std::shared_ptr<vsomeip::message> &response) {
  require(response->get_message_type() == vsomeip::message_type_e::MT_RESPONSE &&
          response->get_return_code() == vsomeip::return_code_e::E_OK, "normal response required");
  Message message;
  require(message.ParseFromString(Peer::payload_bytes(response)), "response protobuf");
  return message;
}
}

int main(int argc, char **argv) {
  try {
    require(argc == 6, "usage: probe configure|server|client YAML NETWORK_JSON LOG DONE");
    const std::string mode = argv[1], yaml = argv[2], json = argv[3], output = argv[4], done = argv[5];
    auto config = net::load_config(yaml);
    if (mode == "configure") {
      for (bool server : {true, false}) {
        auto assembled = config;
        assembled.bind_address = server ? "127.0.0.1" : "127.0.0.2";
        const std::string path = json + (server ? "/server.json" : "/client.json");
        require(!std::filesystem::exists(path), "refuse existing JSON");
        std::ofstream file(path);
        file << net::vsomeip_configuration_json(assembled, server ? config.application_name : "rate_limit_peer",
            server ? 0x5101 : 0x5102, server ? "rl_server_5976af9" : "rl_peer_5976af9", server);
        require(static_cast<bool>(file), "JSON write failed");
      }
      return 0;
    }
    Log log(output);
    log.emit("\"event\":\"identity\",\"pid\":" + std::to_string(::getpid()));
    if (mode == "server") {
      wire::Capabilities capabilities;
      capabilities.set_schema_version(1);
      capabilities.set_contract_id("rate-limit-transport-fixture-not-gateway");
      capabilities.set_planning_transport(wire::PLANNING_TRANSPORT_SOMEIP);
      auto ingress = std::make_shared<net::PlanningIngress>(config, capabilities);
      net::SomeipServer server(config, ingress, json);
      server.start();
      const auto deadline = std::chrono::steady_clock::now() + 20s;
      unsigned forwarded = 0;
      while (!std::filesystem::exists(done) && std::chrono::steady_clock::now() < deadline) {
        const auto state = ingress->snapshot();
        if (state.state == wire::CONTROL_SESSION_STATE_OPENING) {
          require(ingress->bind_publisher(state.epoch, std::string(24, 'g'), "fixture-no-ROS"), "fixture bind");
          require(ingress->set_admitted(state.epoch, true), "fixture admit");
        }
        ingress->publish_pending(state.epoch, net::boottime_now_ns(), [&](const codec::CommandFrame &frame) {
          ++forwarded;
          log.emit("\"event\":\"forwarded_fixture\",\"sequence\":" + std::to_string(frame.command_sequence)
            + ",\"source_stamp_ns\":" + std::to_string(frame.source_stamp_ns));
        });
        std::this_thread::sleep_for(2ms);
      }
      require(std::filesystem::exists(done), "fixture deadline");
      const auto counters = server.rate_limits();
      server.stop();
      ingress->stop();
      const auto submit = counters[static_cast<std::size_t>(control_link::network::IngressMethod::Submit)];
      const auto subscribe = counters[static_cast<std::size_t>(control_link::network::IngressMethod::Subscribe)];
      log.emit("\"event\":\"server_final\",\"forwarded\":" + std::to_string(forwarded)
        + ",\"submit_caller_limited\":" + std::to_string(submit.caller_limited)
        + ",\"subscribe_caller_limited\":" + std::to_string(subscribe.caller_limited));
      require(forwarded == 1 && submit.caller_limited >= 1 && subscribe.caller_limited >= 1, "limit/forwarding counters");
    } else {
      require(mode == "client", "unknown fixture mode");
      Peer peer(config, json, log);
      peer.ready();
      wire::GetStatusRequest status;
      status.set_schema_version(1);
      const auto discovered = decoded<wire::StatusEnvelope>(peer.request(config.get_status_method, status.SerializeAsString()));
      wire::OpenSessionRequest open;
      open.set_schema_version(1);
      open.set_holder_nonce(std::string(16, 'h'));
      open.set_open_request_id(std::string(16, 'r'));
      *open.mutable_opening_domain() = discovered.status().opening_domain();
      const auto opened = decoded<wire::OpenSessionResponse>(peer.request(config.open_method, open.SerializeAsString()));
      require(opened.session().session_token().size() == 16, "fixture session token");
      std::this_thread::sleep_for(100ms);
      codec::CommandFrame frame;
      std::memcpy(frame.session_token.bytes.data(), opened.session().session_token().data(), frame.session_token.bytes.size());
      frame.command_sequence = 1;
      frame.source_stamp_ns = net::system_now_ns();
      frame.linear_velocity_mps = .3;
      std::array<std::byte, 64> bytes{};
      require(codec::encode_command(frame, bytes.data(), bytes.size()), "encode command");
      const std::string command(reinterpret_cast<const char *>(bytes.data()), bytes.size());
      decoded<wire::SubmitReceipt>(peer.request(config.submit_method, command));
      const auto rejected = peer.request(config.submit_method, command);
      require(rejected->get_message_type() == vsomeip::message_type_e::MT_ERROR &&
          rejected->get_return_code() == vsomeip::return_code_e::E_NOT_OK && Peer::payload_bytes(rejected) == "RATE_LIMITED",
          "exact E_NOT_OK + ASCII RATE_LIMITED required");
      wire::KeepSessionRequest keep;
      keep.set_schema_version(1);
      keep.set_holder_nonce(open.holder_nonce());
      keep.set_open_request_id(open.open_request_id());
      keep.set_keepalive_sequence(1);
      *keep.mutable_opening_domain() = open.opening_domain();
      const auto maintained = decoded<wire::OpenSessionResponse>(peer.request(config.keep_method, keep.SerializeAsString()));
      require(maintained.session().submission_ready(), "Keep after Submit exhaustion remains served");
      std::this_thread::sleep_for(1100ms);
      const auto observed = decoded<wire::StatusEnvelope>(peer.request(config.get_status_method, status.SerializeAsString()));
      require(observed.status().latest_forwarded().key().source_sequence() == 1 &&
              observed.status().transport().forwarded_count() == 1,
              "rejected Submit/Keep/query must not forward another target");
      wire::CloseSessionRequest close;
      close.set_schema_version(1);
      close.set_holder_nonce(open.holder_nonce());
      close.set_open_request_id(open.open_request_id());
      *close.mutable_opening_domain() = open.opening_domain();
      decoded<wire::OpenSessionResponse>(peer.request(config.close_method, close.SerializeAsString()));
      log.emit("\"event\":\"client_final\",\"original_sequence\":1,\"original_stamp_ns\":" + std::to_string(frame.source_stamp_ns));
      const bool subscription_complete = peer.subscriptions();
      std::ofstream(done) << "closed\n";
      require(subscription_complete, "missing explicit subscription NACK; retain raw capture");
    }
    return 0;
  } catch (const std::exception &error) {
    std::cerr << "rate-limit fixture: " << error.what() << '\n';
    return 1;
  }
}
