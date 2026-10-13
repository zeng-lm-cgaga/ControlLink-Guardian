#include "control_link_someip_transport/server.hpp"
#include "application_runtime.hpp"

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstdlib>
#include <filesystem>
#include <iostream>
#include <limits>
#include <map>
#include <mutex>
#include <stdexcept>
#include <thread>
#include <utility>

#include <vsomeip/vsomeip.hpp>

#ifndef CONTROL_LINK_NETWORK_SCHEMA_SHA256_HEX
#error "The network target must define CONTROL_LINK_NETWORK_SCHEMA_SHA256_HEX from its .proto input"
#endif

namespace control_link::someip {
namespace {

std::shared_ptr<vsomeip::application> initialize_application(
    const std::string &name, const std::string &path) {
  detail::bind_application_configuration(name, path);
  auto app = vsomeip::runtime::get()->create_application(name);
  if (!app || !app->init()) {
    throw std::runtime_error("vsomeip application initialization failed");
  }
  return app;
}

bool ros_time_representable(std::int64_t stamp) {
  auto seconds = stamp / 1000000000LL;
  if (stamp % 1000000000LL < 0) {
    --seconds;
  }
  return seconds >= std::numeric_limits<std::int32_t>::min() &&
         seconds <= std::numeric_limits<std::int32_t>::max();
}

template <typename Message>
bool parse_versioned(const std::shared_ptr<vsomeip::payload> &payload,
                     Message &message) {
  return payload && message.ParseFromArray(payload->get_data(),
      static_cast<int>(payload->get_length())) &&
      message.schema_version_presence_case() == Message::kSchemaVersion &&
      message.schema_version() == 1;
}

}

std::string network_schema_fingerprint() {
  const std::string hex = CONTROL_LINK_NETWORK_SCHEMA_SHA256_HEX;
  if (hex.size() != 64) {
    throw std::runtime_error("Build-time network schema SHA256 must contain 64 hex digits");
  }
  std::string bytes;
  bytes.reserve(32);
  const auto decode = [](char value) -> unsigned {
    if (value >= '0' && value <= '9') return static_cast<unsigned>(value - '0');
    if (value >= 'a' && value <= 'f') return static_cast<unsigned>(value - 'a' + 10);
    if (value >= 'A' && value <= 'F') return static_cast<unsigned>(value - 'A' + 10);
    throw std::runtime_error("Invalid build-time schema fingerprint");
  };
  for (std::size_t index = 0; index < hex.size(); index += 2) {
    bytes.push_back(static_cast<char>((decode(hex[index]) << 4U) | decode(hex[index + 1])));
  }
  return bytes;
}

struct SomeipServer::Impl {
  Config config;
  std::shared_ptr<PlanningIngress> ingress;
  std::string path;
  std::shared_ptr<vsomeip::application> app;
  std::thread app_thread;
  std::thread event_thread;
  std::mutex gate;
  std::condition_variable wake;
  bool stopping{true};
  bool registered{false};
  bool started{false};
  std::string instance_token;
  std::string fingerprint;
  std::uint64_t publication_sequence{0};
  std::map<vsomeip::client_t, std::uint8_t> readonly_peers;
  std::chrono::steady_clock::time_point next_error_log{};

  Impl(Config settings, std::shared_ptr<PlanningIngress> target, std::string json)
      : config(std::move(settings)), ingress(std::move(target)), path(std::move(json)) {
    if (!ingress) throw std::invalid_argument("PlanningIngress is required");
  }

  void log_error(const std::string &reason) {
    const auto now = std::chrono::steady_clock::now();
    if (now >= next_error_log) {
      next_error_log = now + std::chrono::seconds(1);
      std::cerr << "someip protocol error: " << reason << '\n';
    }
  }

  void protocol_rejection() {
    ingress->record_protocol_rejection();
  }

  bool subscription(vsomeip::client_t peer, std::uint8_t group, bool subscribe) {
    std::lock_guard<std::mutex> lock(gate);
    if (stopping || !registered) return false;
    auto existing = readonly_peers.find(peer);
    if (!subscribe) {
      if (existing != readonly_peers.end()) {
        existing->second &= static_cast<std::uint8_t>(~group);
        if (existing->second == 0) readonly_peers.erase(existing);
      }
      return true;
    }
    if (existing == readonly_peers.end()) {
      if (readonly_peers.size() >= config.readonly_inflight_limit) {
        log_error("readonly subscriber RESOURCE_EXHAUSTED");
        return false;
      }
      existing = readonly_peers.emplace(peer, 0).first;
    }
    existing->second |= group;
    return true;
  }

  void error_reply(const std::shared_ptr<vsomeip::message> &request,
                   vsomeip::return_code_e code, const std::string &reason) {
    protocol_rejection();
    log_error(reason);
    auto response = vsomeip::runtime::get()->create_response(request);
    response->set_message_type(vsomeip::message_type_e::MT_ERROR);
    response->set_return_code(code);
    app->send(response);
  }

  template <typename Message>
  void reply(const std::shared_ptr<vsomeip::message> &request, const Message &body) {
    if (body.ByteSizeLong() > config.max_payload_bytes) {
      error_reply(request, vsomeip::return_code_e::E_NOT_OK, "response exceeds configured payload limit");
      return;
    }
    std::string bytes;
    if (!body.SerializeToString(&bytes)) {
      throw std::runtime_error("Protobuf response serialization failed");
    }
    auto payload = vsomeip::runtime::get()->create_payload();
    payload->set_data(reinterpret_cast<const vsomeip::byte_t *>(bytes.data()),
                      static_cast<vsomeip::length_t>(bytes.size()));
    auto response = vsomeip::runtime::get()->create_response(request);
    response->set_payload(payload);
    app->send(response);
  }

  wire::StatusEnvelope envelope(wire::StatusSnapshot snapshot) {
    if (publication_sequence == std::numeric_limits<std::uint64_t>::max()) {
      throw std::runtime_error("Status publication sequence exhausted");
    }
    wire::StatusEnvelope output;
    output.set_schema_version(1);
    output.set_schema_fingerprint(fingerprint);
    output.set_publisher_instance_token(instance_token);
    output.set_publication_sequence(++publication_sequence);
    output.set_published_system_ns(system_now_ns());
    output.set_published_boottime_ns(boottime_now_ns());
    *output.mutable_status() = std::move(snapshot);
    return output;
  }

  void request(const std::shared_ptr<vsomeip::message> &message) noexcept {
    // 同步 gate 与 stop/state 回调共享，阻止 stop 撤 epoch 后迟到 Open 再建立 holder。
    std::lock_guard<std::mutex> lock(gate);
    if (stopping || !registered || !message) return;
    try {
      if (message->get_message_type() != vsomeip::message_type_e::MT_REQUEST ||
          !message->is_reliable() || message->get_service() != config.service_id ||
          message->get_instance() != config.instance_id ||
          message->get_interface_version() != config.major_version || message->get_client() == 0) {
        error_reply(message, vsomeip::return_code_e::E_MALFORMED_MESSAGE, "request header/reliability mismatch");
        return;
      }
      const auto payload = message->get_payload();
      if (!payload || payload->get_length() > config.max_payload_bytes) {
        error_reply(message, vsomeip::return_code_e::E_MALFORMED_MESSAGE, "payload exceeds configured size");
        return;
      }
      const auto method = message->get_method();
      if (method == config.open_method) {
        wire::OpenSessionRequest input;
        if (!parse_versioned(payload, input) || input.holder_nonce().size() != 16 ||
            input.open_request_id().size() != 16) {
          error_reply(message, vsomeip::return_code_e::E_MALFORMED_MESSAGE, "invalid OpenSession schema/identity");
          return;
        }
        const auto response = ingress->open(message->get_client(), input, steady_now_ns());
        if (response.result() == wire::SESSION_OPERATION_RESULT_REJECTED) {
          const auto snapshot = ingress->snapshot();
          if (!snapshot.failure.empty()) log_error(snapshot.failure);
        }
        reply(message, response);
      } else if (method == config.keep_method) {
        wire::KeepSessionRequest input;
        if (!parse_versioned(payload, input) || input.holder_nonce().size() != 16 ||
            input.open_request_id().size() != 16 ||
            input.keepalive_sequence_presence_case() != wire::KeepSessionRequest::kKeepaliveSequence) {
          error_reply(message, vsomeip::return_code_e::E_MALFORMED_MESSAGE, "invalid KeepSession schema/identity");
          return;
        }
        reply(message, ingress->keep(message->get_client(), input, steady_now_ns()));
      } else if (method == config.close_method) {
        wire::CloseSessionRequest input;
        if (!parse_versioned(payload, input) || input.holder_nonce().size() != 16 ||
            input.open_request_id().size() != 16) {
          error_reply(message, vsomeip::return_code_e::E_MALFORMED_MESSAGE, "invalid CloseSession schema/identity");
          return;
        }
        reply(message, ingress->close(message->get_client(), input));
      } else if (method == config.get_status_method) {
        wire::GetStatusRequest input;
        if (!parse_versioned(payload, input)) {
          error_reply(message, vsomeip::return_code_e::E_MALFORMED_MESSAGE, "invalid GetStatus schema");
          return;
        }
        reply(message, envelope(ingress->readonly_status()));
      } else if (method == config.submit_method) {
        control_link_edge_protocol::CommandFrame command;
        control_link_edge_protocol::DecodeError failure;
        if (!control_link_edge_protocol::decode_command_layout(
              reinterpret_cast<const std::byte *>(payload->get_data()), payload->get_length(),
              &command, &failure) || !ros_time_representable(command.source_stamp_ns)) {
          error_reply(message, vsomeip::return_code_e::E_MALFORMED_MESSAGE, "invalid command layout/domain/time representation");
          return;
        }
        reply(message, ingress->submit(message->get_client(), command, boottime_now_ns()));
      } else {
        error_reply(message, vsomeip::return_code_e::E_UNKNOWN_METHOD, "unknown method");
      }
    } catch (const std::exception &failure) {
      ingress->revoke("SOME/IP protocol operation failed");
      log_error(failure.what());
    }
  }

  void notify(vsomeip::event_t event, const wire::StatusEnvelope &output) {
    if (output.ByteSizeLong() > config.max_payload_bytes) {
      protocol_rejection();
      log_error("status event exceeds configured payload limit");
      return;
    }
    std::string bytes;
    if (!output.SerializeToString(&bytes)) throw std::runtime_error("Status serialization failed");
    auto payload = vsomeip::runtime::get()->create_payload();
    payload->set_data(reinterpret_cast<const vsomeip::byte_t *>(bytes.data()),
                      static_cast<vsomeip::length_t>(bytes.size()));
    app->notify(config.service_id, config.instance_id, event, payload);
  }

  void events() noexcept {
    std::unique_lock<std::mutex> lock(gate);
    while (!stopping) {
      wake.wait_for(lock, std::chrono::milliseconds(config.status_event_period_ms),
                    [this] { return stopping; });
      if (stopping) break;
      try {
        // 网络 idle 不属于业务 freshness；编码/notify 不在任何 ROS callback 内执行。
        ingress->expire(steady_now_ns());
        if (!registered) continue;
        auto snapshot = ingress->readonly_status();
        wire::StatusSnapshot vehicle;
        if (snapshot.has_vehicle()) *vehicle.mutable_vehicle() = snapshot.vehicle();
        snapshot.clear_vehicle();
        snapshot.clear_alerts();
        notify(config.gateway_event, envelope(std::move(snapshot)));
        notify(config.vehicle_event, envelope(std::move(vehicle)));
      } catch (const std::exception &failure) {
        ingress->revoke("SOME/IP status encoding failed");
        log_error(failure.what());
      }
    }
  }

  void state(vsomeip::state_type_e state) noexcept {
    std::lock_guard<std::mutex> lock(gate);
    if (stopping) return;
    try {
      registered = state == vsomeip::state_type_e::ST_REGISTERED;
      if (registered) {
        app->offer_event(config.service_id, config.instance_id, config.gateway_event,
            {config.gateway_eventgroup}, vsomeip::event_type_e::ET_EVENT,
            std::chrono::milliseconds::zero(), false, true, nullptr,
            vsomeip::reliability_type_e::RT_RELIABLE);
        app->offer_event(config.service_id, config.instance_id, config.vehicle_event,
            {config.vehicle_eventgroup}, vsomeip::event_type_e::ET_EVENT,
            std::chrono::milliseconds::zero(), false, true, nullptr,
            vsomeip::reliability_type_e::RT_RELIABLE);
        app->offer_service(config.service_id, config.instance_id,
                           config.major_version, config.minor_version);
      } else {
        ingress->revoke("SOME/IP application deregistered");
      }
      wake.notify_all();
    } catch (const std::exception &failure) {
      registered = false;
      ingress->revoke("SOME/IP service offer failed");
      log_error(failure.what());
    }
  }
};

SomeipServer::SomeipServer(Config config, std::shared_ptr<PlanningIngress> ingress,
                         std::string vsomeip_config_path)
    : impl_(std::make_unique<Impl>(std::move(config), std::move(ingress),
                                  std::move(vsomeip_config_path))) {}

SomeipServer::~SomeipServer() { stop(); }

void SomeipServer::start() {
  if (impl_->started) throw std::logic_error("SomeipServer cannot be started twice");
  try {
    validate_config(impl_->config);
    validate_vsomeip_configuration(impl_->config, impl_->path, impl_->config.application_name, true);
    impl_->fingerprint = network_schema_fingerprint();
    impl_->instance_token = random_identity();
    if (impl_->instance_token.size() != 16) throw std::runtime_error("Status publisher randomness failed");
    impl_->app = initialize_application(impl_->config.application_name, impl_->path);
    impl_->started = true;
    impl_->app->register_state_handler([this](vsomeip::state_type_e state) { impl_->state(state); });
    impl_->app->register_subscription_handler(impl_->config.service_id, impl_->config.instance_id,
        impl_->config.gateway_eventgroup, vsomeip::subscription_handler_sec_t{
          [this](vsomeip::client_t peer, const vsomeip_sec_client_t *, const std::string &, bool subscribe) {
            return impl_->subscription(peer, 1, subscribe);
          }});
    impl_->app->register_subscription_handler(impl_->config.service_id, impl_->config.instance_id,
        impl_->config.vehicle_eventgroup, vsomeip::subscription_handler_sec_t{
          [this](vsomeip::client_t peer, const vsomeip_sec_client_t *, const std::string &, bool subscribe) {
            return impl_->subscription(peer, 2, subscribe);
          }});
    impl_->app->register_message_handler(impl_->config.service_id, impl_->config.instance_id,
        vsomeip::ANY_METHOD, [this](const std::shared_ptr<vsomeip::message> &message) {
          impl_->request(message);
        });
    impl_->stopping = false;
    impl_->app_thread = std::thread([this] {
      detail::run_application_loop([this] { impl_->app->start(); },
          [this](const char *) {
            std::lock_guard<std::mutex> lock(impl_->gate);
            impl_->registered = false;
            impl_->ingress->revoke("SOME/IP application loop failed");
          }, [this] {
            std::lock_guard<std::mutex> lock(impl_->gate);
            impl_->registered = false;
            impl_->ingress->revoke("SOME/IP application loop exited");
          });
    });
    impl_->event_thread = std::thread([this] { impl_->events(); });
  } catch (...) {
    impl_->ingress->revoke("SOME/IP start failed");
    stop();
    throw;
  }
}

void SomeipServer::stop() noexcept {
  if (!impl_ || !impl_->started || !impl_->app) return;
  {
    std::lock_guard<std::mutex> lock(impl_->gate);
    impl_->stopping = true;
    impl_->registered = false;
    impl_->ingress->stop();
  }
  impl_->wake.notify_all();
  try {
    impl_->app->stop_offer_service(impl_->config.service_id, impl_->config.instance_id,
                                 impl_->config.major_version, impl_->config.minor_version);
    impl_->app->stop_offer_event(impl_->config.service_id, impl_->config.instance_id,
                               impl_->config.gateway_event);
    impl_->app->stop_offer_event(impl_->config.service_id, impl_->config.instance_id,
                               impl_->config.vehicle_event);
  } catch (...) {
  }
  try {
    if (impl_->app_thread.joinable()) impl_->app->stop();
  } catch (...) {
  }
  if (impl_->event_thread.joinable()) impl_->event_thread.join();
  if (impl_->app_thread.joinable()) impl_->app_thread.join();
  try {
    impl_->app->clear_all_handler();
  } catch (...) {
  }
  impl_->app.reset();
}

}
