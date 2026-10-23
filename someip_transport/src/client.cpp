#include "control_link_someip_transport/client.hpp"
#include "control_link_someip_transport/server.hpp"
#include "application_runtime.hpp"
#include "session_reply_order.hpp"
#include "subscription_state.hpp"

#include <array>
#include <condition_variable>
#include <csignal>
#include <cstdlib>
#include <filesystem>
#include <cstring>
#include <iomanip>
#include <iostream>
#include <limits>
#include <mutex>
#include <sstream>
#include <stdexcept>
#include <thread>
#include <utility>

#include <unistd.h>

#include <vsomeip/vsomeip.hpp>

namespace control_link::someip {
namespace {

volatile std::sig_atomic_t example_stop_signal = 0;

void request_example_stop(int received) { example_stop_signal = received; }

struct ExampleSignalScope {
  using Handler = void (*)(int);
  Handler interrupt;
  Handler terminate;
  ExampleSignalScope()
      : interrupt(std::signal(SIGINT, request_example_stop)),
        terminate(std::signal(SIGTERM, request_example_stop)) {
    example_stop_signal = 0;
  }
  ~ExampleSignalScope() {
    std::signal(SIGINT, interrupt);
    std::signal(SIGTERM, terminate);
  }
};

std::shared_ptr<vsomeip::application> initialize_client(
    const std::string &name, const std::string &path) {
  detail::bind_application_configuration(name, path);
  auto app = vsomeip::runtime::get()->create_application(name);
  if (!app || !app->init()) throw std::runtime_error("vsomeip client initialization failed");
  return app;
}

template <typename Message>
Message decode_response(const std::string &bytes) {
  Message result;
  if (!result.ParseFromArray(bytes.data(), static_cast<int>(bytes.size()))) {
    throw std::runtime_error("Malformed Protobuf response");
  }
  return result;
}

std::string serialize_request(const google::protobuf::MessageLite &message) {
  std::string bytes;
  if (!message.SerializeToString(&bytes)) throw std::runtime_error("Protobuf request serialization failed");
  return bytes;
}

bool has_control_token(const wire::StatusSnapshot &snapshot) {
  return (snapshot.has_session() && !snapshot.session().session_token().empty()) ||
         (snapshot.has_latest_forwarded() && !snapshot.latest_forwarded().key().session_token().empty()) ||
         (snapshot.has_latest_gateway_acceptance() &&
          !snapshot.latest_gateway_acceptance().key().session_token().empty());
}

template <typename Request>
void require_opening_domain(const Request &request) {
  if (!request.has_opening_domain() || !valid_opening_domain(request.opening_domain())) {
    throw std::invalid_argument("SOME/IP Open/Keep/Close requires an explicit V1 opening domain; retain it for retries");
  }
}

}

struct SomeipClient::Impl {
  struct Reply {
    std::string bytes;
    std::uint64_t generation{0};
    detail::SessionReplyOrder::Revision readiness_revision{0};
  };
  Config config;
  std::string path;
  std::string application_name;
  std::string fingerprint;
  std::shared_ptr<vsomeip::application> app;
  std::thread app_thread;
  mutable std::mutex mutex;
  std::condition_variable wake;
  ClientStatus current;
  bool started{false};
  bool subscription_requested{false};
  bool subscription_enabled{true};
  detail::SubscriptionObservationState gateway_subscription;
  detail::SubscriptionObservationState vehicle_subscription;
  std::shared_ptr<vsomeip::message> pending;
  bool response_ready{false};
  std::string response_bytes;
  std::string response_error;
  std::uint64_t pending_generation{0};
  detail::SessionReplyOrder session_reply_order;
  detail::SessionReplyOrder::Revision response_readiness_revision{0};
  std::string publisher_instance;
  std::uint64_t gateway_sequence{0};
  std::uint64_t vehicle_sequence{0};

  Impl(Config settings, std::string json, std::string name)
      : config(std::move(settings)), path(std::move(json)), application_name(std::move(name)),
        gateway_subscription(config.service_id, config.instance_id, config.gateway_eventgroup, config.gateway_event),
        vehicle_subscription(config.service_id, config.instance_id, config.vehicle_eventgroup, config.vehicle_event) {}

  void invalidate_locked(const std::string &reason) {
    subscription_requested = false;
    gateway_subscription.reset(subscription_enabled ? SubscriptionState::Unknown : SubscriptionState::Disabled);
    vehicle_subscription.reset(subscription_enabled ? SubscriptionState::Unknown : SubscriptionState::Disabled);
    session_reply_order.observe_revocation();
    current.session.Clear();
    current.observations.Clear();
    current.gateway_status_received_steady_ns = 0;
    current.vehicle_status_received_steady_ns = 0;
    current.failure = reason;
    publisher_instance.clear();
    gateway_sequence = 0;
    vehicle_sequence = 0;
    ++current.availability_generation;
    if (pending) {
      response_error = reason;
      response_ready = true;
    }
    wake.notify_all();
  }

  void state(vsomeip::state_type_e state) noexcept {
    std::lock_guard<std::mutex> lock(mutex);
    if (!current.running) return;
    try {
      current.registered = state == vsomeip::state_type_e::ST_REGISTERED;
      if (current.registered) {
        app->request_service(config.service_id, config.instance_id,
                             config.major_version, config.minor_version);
        app->request_event(config.service_id, config.instance_id, config.gateway_event,
            {config.gateway_eventgroup}, vsomeip::event_type_e::ET_EVENT,
            vsomeip::reliability_type_e::RT_RELIABLE);
        app->request_event(config.service_id, config.instance_id, config.vehicle_event,
            {config.vehicle_eventgroup}, vsomeip::event_type_e::ET_EVENT,
            vsomeip::reliability_type_e::RT_RELIABLE);
      } else {
        current.service_available = false;
        invalidate_locked("SOME/IP application deregistered");
      }
      wake.notify_all();
    } catch (const std::exception &failure) {
      current.service_available = false;
      invalidate_locked(failure.what());
    }
  }

  void availability(bool available) noexcept {
    std::lock_guard<std::mutex> lock(mutex);
    if (!current.running) return;
    try {
      if (!available && current.service_available) {
        invalidate_locked("SOME/IP service unavailable; old targets must be discarded");
      }
      current.service_available = available;
      if (available && subscription_enabled && !subscription_requested) request_subscriptions_locked();
      wake.notify_all();
    } catch (const std::exception &failure) {
      invalidate_locked(failure.what());
    }
  }

  void request_subscriptions_locked() {
    const auto now = steady_now_ns();
    const auto budget = static_cast<std::uint64_t>(config.reconnect_max_ms) * 1000000ULL;
    auto install = [&](detail::SubscriptionObservationState &observation, std::uint16_t group,
                       std::uint16_t event) {
      const auto generation = observation.begin(current.availability_generation, now, budget);
      app->register_subscription_status_handler(config.service_id, config.instance_id, group, event,
          [this, generation](vsomeip::service_t service, vsomeip::instance_t instance,
                            vsomeip::eventgroup_t group, vsomeip::event_t event, std::uint16_t error) {
            std::lock_guard<std::mutex> lock(mutex);
            if (!current.running || !current.service_available || !subscription_enabled || !subscription_requested) return;
            auto &state = group == config.gateway_eventgroup ? gateway_subscription : vehicle_subscription;
            state.observe(generation, service, instance, group, event, error, steady_now_ns());
            wake.notify_all();
          }, true);
    };
    subscription_requested = true;
    try {
      install(gateway_subscription, config.gateway_eventgroup, config.gateway_event);
      install(vehicle_subscription, config.vehicle_eventgroup, config.vehicle_event);
      app->subscribe(config.service_id, config.instance_id, config.gateway_eventgroup, config.major_version);
      app->subscribe(config.service_id, config.instance_id, config.vehicle_eventgroup, config.major_version);
    } catch (const std::exception &failure) {
      gateway_subscription.reset(SubscriptionState::Unknown);
      vehicle_subscription.reset(SubscriptionState::Unknown);
      current.failure = std::string("Readonly subscription request failed: ") + failure.what();
    }
  }

  bool valid_envelope(const wire::StatusEnvelope &envelope, bool vehicle_only = false) const {
    const bool valid_domain = envelope.status().has_opening_domain() &&
        valid_opening_domain(envelope.status().opening_domain());
    return envelope.schema_version_presence_case() == wire::StatusEnvelope::kSchemaVersion &&
           envelope.schema_version() == 1 &&
           envelope.schema_fingerprint() == fingerprint &&
           envelope.publisher_instance_token().size() == 16 &&
           envelope.publication_sequence_presence_case() == wire::StatusEnvelope::kPublicationSequence &&
           envelope.publication_sequence() > 0 &&
           envelope.published_system_presence_case() == wire::StatusEnvelope::kPublishedSystemNs &&
           envelope.published_boottime_presence_case() == wire::StatusEnvelope::kPublishedBoottimeNs &&
           envelope.published_boottime_ns() > 0 && envelope.has_status() &&
           (valid_domain || (vehicle_only && !envelope.status().has_opening_domain() &&
                             !envelope.status().has_session())) &&
           (!envelope.status().has_session() ||
            (envelope.status().session().has_opening_domain() &&
             same_opening_domain(envelope.status().session().opening_domain(),
                                  envelope.status().opening_domain()))) &&
           !has_control_token(envelope.status());
  }

  void merge_locked(const wire::StatusEnvelope &envelope, bool vehicle_only) {
    if (publisher_instance != envelope.publisher_instance_token()) {
      if (!publisher_instance.empty()) invalidate_locked("Status publisher generation changed");
      publisher_instance = envelope.publisher_instance_token();
    }
    auto &last_sequence = vehicle_only ? vehicle_sequence : gateway_sequence;
    if (envelope.publication_sequence() <= last_sequence) return;
    last_sequence = envelope.publication_sequence();
    const auto &incoming = envelope.status();
    auto &snapshot = current.observations;
    if (vehicle_only) {
      current.vehicle_status_received_steady_ns = steady_now_ns();
      if (incoming.has_vehicle()) *snapshot.mutable_vehicle() = incoming.vehicle();
      return;
    }
    current.gateway_status_received_steady_ns = steady_now_ns();
    *snapshot.mutable_opening_domain() = incoming.opening_domain();
    if (incoming.has_session()) {
      *snapshot.mutable_session() = incoming.session();
      // 只读事实可以撤销缓存的可提交提示，不能借无 token 的 event 建立或升级 holder。
      const bool readiness_revoked = !incoming.session().submission_ready() ||
          (current.session.has_opening_domain() &&
           !same_opening_domain(current.session.opening_domain(), incoming.opening_domain())) ||
          (!current.session.ingress_publisher().gid().empty() &&
           incoming.session().ingress_publisher().gid() != current.session.ingress_publisher().gid());
      if (readiness_revoked) {
        // 首个 Open 的回复还可能在锁外解码；缓存尚无 token 也须记住较晚撤准入。
        session_reply_order.observe_revocation();
        if (!current.session.session_token().empty()) {
          current.session.set_submission_ready(false);
          current.session.set_control_session_state(incoming.session().control_session_state());
          current.session.set_connection_state(incoming.session().connection_state());
        }
      }
    }
    if (incoming.has_transport()) *snapshot.mutable_transport() = incoming.transport();
    if (incoming.has_latest_forwarded()) *snapshot.mutable_latest_forwarded() = incoming.latest_forwarded();
    if (incoming.has_gateway()) *snapshot.mutable_gateway() = incoming.gateway();
    snapshot.mutable_sources()->CopyFrom(incoming.sources());
    if (incoming.has_latest_gateway_acceptance()) {
      *snapshot.mutable_latest_gateway_acceptance() = incoming.latest_gateway_acceptance();
    }
    snapshot.mutable_alerts()->CopyFrom(incoming.alerts());
  }

  void message(const std::shared_ptr<vsomeip::message> &incoming) noexcept {
    std::lock_guard<std::mutex> lock(mutex);
    if (!current.running || !incoming) return;
    try {
      if (incoming->get_service() != config.service_id ||
          incoming->get_instance() != config.instance_id ||
          incoming->get_interface_version() != config.major_version || !incoming->is_reliable()) return;
      const auto type = incoming->get_message_type();
      const auto payload = incoming->get_payload();
      if (type == vsomeip::message_type_e::MT_NOTIFICATION) {
        if (!current.service_available || !subscription_enabled || !payload ||
            payload->get_length() > config.max_payload_bytes) return;
        if (incoming->get_method() != config.gateway_event && incoming->get_method() != config.vehicle_event) return;
        auto envelope = decode_response<wire::StatusEnvelope>(std::string(
            reinterpret_cast<const char *>(payload->get_data()), payload->get_length()));
        if (!valid_envelope(envelope, incoming->get_method() == config.vehicle_event)) {
          current.failure = "Rejected status event schema/presence/fingerprint/token";
          return;
        }
        merge_locked(envelope, incoming->get_method() == config.vehicle_event);
        return;
      }
      if (!pending || pending_generation != current.availability_generation ||
          incoming->get_client() != pending->get_client() ||
          incoming->get_session() != pending->get_session() ||
          incoming->get_method() != pending->get_method() || response_ready ||
          (type != vsomeip::message_type_e::MT_RESPONSE && type != vsomeip::message_type_e::MT_ERROR)) return;
      if (type == vsomeip::message_type_e::MT_ERROR || incoming->get_return_code() != vsomeip::return_code_e::E_OK) {
        response_error = "SOME/IP error response code=" +
            std::to_string(static_cast<unsigned>(incoming->get_return_code()));
      } else if (!payload || payload->get_length() > config.max_payload_bytes) {
        response_error = "Response exceeds configured payload limit";
      } else {
        response_bytes.assign(reinterpret_cast<const char *>(payload->get_data()), payload->get_length());
        // 在真实 response callback 到达时取 revision，不能在锁外解码结束后补取。
        response_readiness_revision = session_reply_order.capture_reply();
      }
      response_ready = true;
      wake.notify_all();
    } catch (const std::exception &failure) {
      current.failure = failure.what();
      if (pending) {
        response_error = current.failure;
        response_ready = true;
        wake.notify_all();
      }
    }
  }

  Reply request(std::uint16_t method, const std::string &bytes,
                      std::chrono::milliseconds timeout) {
    if (bytes.size() > config.max_payload_bytes || timeout <= std::chrono::milliseconds::zero()) {
      throw std::invalid_argument("Request size or timeout outside configured bound");
    }
    std::unique_lock<std::mutex> lock(mutex);
    if (!current.running || !current.registered || !current.service_available) {
      throw std::runtime_error("SOME/IP service is not available");
    }
    if (pending) throw std::logic_error("Single-owner client permits only one in-flight request");
    auto output = vsomeip::runtime::get()->create_request(true);
    output->set_service(config.service_id);
    output->set_instance(config.instance_id);
    output->set_interface_version(config.major_version);
    output->set_method(method);
    auto payload = vsomeip::runtime::get()->create_payload();
    payload->set_data(reinterpret_cast<const vsomeip::byte_t *>(bytes.data()),
                      static_cast<vsomeip::length_t>(bytes.size()));
    output->set_payload(payload);
    pending = output;
    pending_generation = current.availability_generation;
    response_ready = false;
    response_bytes.clear();
    response_error.clear();
    response_readiness_revision = 0;
    try {
      // send 填充 request client/session；锁阻止极快回复在 request ID 尚未固定时提交。
      app->send(output);
      if (!wake.wait_for(lock, timeout, [this] { return response_ready || !current.running; })) {
        pending.reset();
        throw std::runtime_error("SOME/IP response timeout; holder keys remain unchanged");
      }
      pending.reset();
      if (!response_error.empty()) throw std::runtime_error(response_error);
      if (!current.running) throw std::runtime_error("SOME/IP client stopped");
      return {response_bytes, pending_generation, response_readiness_revision};
    } catch (...) {
      pending.reset();
      throw;
    }
  }

  wire::OpenSessionResponse session_reply(std::uint16_t method,
      const google::protobuf::MessageLite &request_message, const std::string &key,
      const wire::OpenAdmissionDomain &requested_domain,
      std::chrono::milliseconds timeout) {
    const auto received = request(method, serialize_request(request_message), timeout);
    auto result = decode_response<wire::OpenSessionResponse>(received.bytes);
    if (!result.has_capabilities() || result.capabilities().schema_version() != 1 ||
        result.capabilities().schema_fingerprint() != fingerprint ||
        result.capabilities().planning_transport() != wire::PLANNING_TRANSPORT_SOMEIP ||
        !result.capabilities().has_opening_domain() ||
        !valid_opening_domain(result.capabilities().opening_domain()) || result.open_request_id() != key) {
      throw std::runtime_error("Session response schema/fingerprint/request key mismatch");
    }
    if (!wire::SessionOperationResult_IsValid(result.result()) ||
        result.result() == wire::SESSION_OPERATION_RESULT_UNKNOWN) {
      throw std::runtime_error("Session response has unknown operation result");
    }
    const bool denied = result.result() == wire::SESSION_OPERATION_RESULT_BUSY ||
        result.result() == wire::SESSION_OPERATION_RESULT_REJECTED ||
        result.result() == wire::SESSION_OPERATION_RESULT_RESOURCE_EXHAUSTED;
    if (denied && result.has_session()) {
      throw std::runtime_error("Denied session response leaked a holder identity");
    }
    if (result.has_session()) {
      const auto &session = result.session();
      if (session.session_token().size() != 16 ||
          !session.has_opening_domain() ||
          !same_opening_domain(session.opening_domain(), requested_domain) ||
          !same_opening_domain(result.capabilities().opening_domain(), requested_domain) ||
          session.planning_transport() != wire::PLANNING_TRANSPORT_SOMEIP ||
          session.clock_domain() != wire::CLOCK_DOMAIN_SYSTEM ||
          (!session.ingress_publisher().gid().empty() && session.ingress_publisher().gid().size() != 24) ||
          (session.submission_ready() && session.ingress_publisher().gid().size() != 24)) {
        throw std::runtime_error("Session response identity/domain mismatch");
      }
      std::lock_guard<std::mutex> lock(mutex);
      if (!current.running || !current.service_available ||
          received.generation != current.availability_generation) {
        throw std::runtime_error("Session reply superseded by service/generation change");
      }
      // generation 管服务/身份换代；revision 只保护已观察的较晚撤准入，二者不能互代。
      session_reply_order.commit_reply(received.readiness_revision, *result.mutable_session(), current.session);
    } else if (!denied) {
      throw std::runtime_error("Session response omitted holder identity");
    }
    return result;
  }
};

SomeipClient::SomeipClient(Config config, std::string vsomeip_config_path, std::string application_name)
    : impl_(std::make_unique<Impl>(std::move(config), std::move(vsomeip_config_path),
                                  std::move(application_name))) {}

SomeipClient::~SomeipClient() { stop(); }

void SomeipClient::start() {
  if (impl_->started) throw std::logic_error("SomeipClient cannot be started twice");
  validate_config(impl_->config);
  validate_vsomeip_configuration(impl_->config, impl_->path, impl_->application_name, false);
  impl_->fingerprint = network_schema_fingerprint();
  impl_->app = initialize_client(impl_->application_name, impl_->path);
  impl_->started = true;
  try {
    impl_->app->register_state_handler([this](vsomeip::state_type_e state) { impl_->state(state); });
    impl_->app->register_availability_handler(impl_->config.service_id, impl_->config.instance_id,
        [this](vsomeip::service_t, vsomeip::instance_t, bool available) {
          impl_->availability(available);
        }, impl_->config.major_version, impl_->config.minor_version);
    impl_->app->register_message_handler(impl_->config.service_id, impl_->config.instance_id,
        vsomeip::ANY_METHOD, [this](const std::shared_ptr<vsomeip::message> &message) {
          impl_->message(message);
        });
    impl_->current.running = true;
    impl_->app_thread = std::thread([this] {
      detail::run_application_loop([this] { impl_->app->start(); },
          [this](const char *reason) {
            std::lock_guard<std::mutex> lock(impl_->mutex);
            impl_->current.running = false;
            impl_->current.registered = false;
            impl_->current.service_available = false;
            impl_->invalidate_locked(reason);
          }, [this] {
            std::lock_guard<std::mutex> lock(impl_->mutex);
            impl_->current.running = false;
            impl_->current.registered = false;
            impl_->current.service_available = false;
            impl_->invalidate_locked("SOME/IP application loop exited");
          });
    });
  } catch (...) {
    stop();
    throw;
  }
}

void SomeipClient::stop() noexcept {
  if (!impl_ || !impl_->started || !impl_->app) return;
  {
    std::lock_guard<std::mutex> lock(impl_->mutex);
    impl_->current.running = false;
    impl_->current.registered = false;
    impl_->current.service_available = false;
    impl_->invalidate_locked("SOME/IP local stop");
    impl_->gateway_subscription.reset(SubscriptionState::Disabled);
    impl_->vehicle_subscription.reset(SubscriptionState::Disabled);
  }
  impl_->wake.notify_all();
  try {
    impl_->app->unsubscribe(impl_->config.service_id, impl_->config.instance_id,
                            impl_->config.gateway_eventgroup);
    impl_->app->unsubscribe(impl_->config.service_id, impl_->config.instance_id,
                            impl_->config.vehicle_eventgroup);
    impl_->app->release_event(impl_->config.service_id, impl_->config.instance_id,
                              impl_->config.gateway_event);
    impl_->app->release_event(impl_->config.service_id, impl_->config.instance_id,
                              impl_->config.vehicle_event);
    impl_->app->release_service(impl_->config.service_id, impl_->config.instance_id);
  } catch (...) {
  }
  try {
    if (impl_->app_thread.joinable()) impl_->app->stop();
  } catch (...) {
  }
  if (impl_->app_thread.joinable()) impl_->app_thread.join();
  try {
    impl_->app->clear_all_handler();
  } catch (...) {
  }
  impl_->app.reset();
  std::lock_guard<std::mutex> lock(impl_->mutex);
  impl_->gateway_subscription.reset(SubscriptionState::Disabled);
  impl_->vehicle_subscription.reset(SubscriptionState::Disabled);
}

bool SomeipClient::wait_available(std::chrono::milliseconds timeout) {
  std::unique_lock<std::mutex> lock(impl_->mutex);
  impl_->wake.wait_for(lock, timeout, [this] {
    return !impl_->current.running ||
           (impl_->current.registered && impl_->current.service_available);
  });
  return impl_->current.running && impl_->current.registered && impl_->current.service_available;
}

void SomeipClient::set_status_subscription(bool enabled) {
  std::lock_guard<std::mutex> lock(impl_->mutex);
  if (!impl_->current.running) throw std::runtime_error("SOME/IP client is not running");
  const bool was_requested = impl_->subscription_requested;
  impl_->subscription_enabled = enabled;
  if (enabled && !impl_->subscription_requested && impl_->current.service_available) {
    impl_->request_subscriptions_locked();
  } else if (!enabled) {
    impl_->subscription_requested = false;
    impl_->gateway_subscription.reset(SubscriptionState::Disabled);
    impl_->vehicle_subscription.reset(SubscriptionState::Disabled);
    impl_->current.gateway_status_received_steady_ns = 0;
    impl_->current.vehicle_status_received_steady_ns = 0;
    if (!was_requested) return;
    impl_->app->unsubscribe(impl_->config.service_id, impl_->config.instance_id,
                            impl_->config.gateway_eventgroup);
    impl_->app->unsubscribe(impl_->config.service_id, impl_->config.instance_id,
                            impl_->config.vehicle_eventgroup);
  }
}

wire::OpenSessionResponse SomeipClient::open(const wire::OpenSessionRequest &request,
                                            std::chrono::milliseconds timeout) {
  require_opening_domain(request);
  return impl_->session_reply(impl_->config.open_method, request, request.open_request_id(),
                             request.opening_domain(), timeout);
}

wire::OpenSessionResponse SomeipClient::keep(const wire::KeepSessionRequest &request,
                                            std::chrono::milliseconds timeout) {
  require_opening_domain(request);
  return impl_->session_reply(impl_->config.keep_method, request, request.open_request_id(),
                             request.opening_domain(), timeout);
}

wire::OpenSessionResponse SomeipClient::close(const wire::CloseSessionRequest &request,
                                             std::chrono::milliseconds timeout) {
  require_opening_domain(request);
  auto response = impl_->session_reply(impl_->config.close_method, request, request.open_request_id(),
                                      request.opening_domain(), timeout);
  if (response.result() == wire::SESSION_OPERATION_RESULT_REVOKED) {
    std::lock_guard<std::mutex> lock(impl_->mutex);
    impl_->current.session.Clear();
  }
  return response;
}

wire::SubmitReceipt SomeipClient::submit(const control_link_edge_protocol::CommandFrame &target,
                                        std::chrono::milliseconds timeout) {
  std::array<std::byte, control_link_edge_protocol::kCommandFrameSize> bytes{};
  if (!control_link_edge_protocol::encode_command_layout(target, bytes.data(), bytes.size())) {
    throw std::invalid_argument("Target codec layout/domain/token invalid");
  }
  const auto received = impl_->request(impl_->config.submit_method,
      std::string(reinterpret_cast<const char *>(bytes.data()), bytes.size()), timeout);
  auto response = decode_response<wire::SubmitReceipt>(received.bytes);
  const std::string token(reinterpret_cast<const char *>(target.session_token.bytes.data()),
                          target.session_token.bytes.size());
  if (response.schema_version_presence_case() != wire::SubmitReceipt::kSchemaVersion ||
      response.schema_version() != 1 || !response.rpc_received() ||
      response.rpc_received_boottime_presence_case() != wire::SubmitReceipt::kRpcReceivedBoottimeNs ||
      !response.has_key() || response.key().session_token() != token ||
      response.key().source_sequence_presence_case() != wire::CommandKey::kSourceSequence ||
      response.key().source_sequence() != target.command_sequence) {
    throw std::runtime_error("Submit receipt schema/presence/target key mismatch");
  }
  return response;
}

wire::StatusEnvelope SomeipClient::get_status(std::chrono::milliseconds timeout) {
  wire::GetStatusRequest request;
  request.set_schema_version(1);
  const auto received = impl_->request(
      impl_->config.get_status_method, serialize_request(request), timeout);
  auto response = decode_response<wire::StatusEnvelope>(received.bytes);
  std::lock_guard<std::mutex> lock(impl_->mutex);
  if (!impl_->current.running || !impl_->current.service_available ||
      received.generation != impl_->current.availability_generation) {
    throw std::runtime_error("GetStatus reply superseded by service/generation change");
  }
  if (!impl_->valid_envelope(response)) throw std::runtime_error("GetStatus schema/presence/token mismatch");
  impl_->merge_locked(response, false);
  impl_->merge_locked(response, true);
  return response;
}

ClientStatus SomeipClient::status() const {
  std::lock_guard<std::mutex> lock(impl_->mutex);
  auto result = impl_->current;
  const auto now = steady_now_ns();
  result.gateway_subscription = impl_->gateway_subscription.snapshot(now);
  result.vehicle_subscription = impl_->vehicle_subscription.snapshot(now);
  // 只评估网络观察是否停更；原组的业务时效/validity 已由 server 的 immutable Contract 判断。
  // 此预算不能延长命令、Gateway 或 Vehicle 的有效期，原时间元数据不改写。
  const auto budget = static_cast<std::uint64_t>(impl_->config.keep_period_ms) * 1000000ULL;
  const auto expired = [now, budget](std::uint64_t received) {
    return received == 0 || received > now || now - received > budget;
  };
  if (expired(result.gateway_status_received_steady_ns)) {
    if (result.observations.has_gateway()) {
      result.observations.mutable_gateway()->mutable_observation()->set_validity(wire::OBSERVATION_VALIDITY_UNKNOWN);
    }
    for (auto &source : *result.observations.mutable_sources()) {
      source.mutable_observation()->set_validity(wire::OBSERVATION_VALIDITY_UNKNOWN);
    }
    if (result.observations.has_latest_gateway_acceptance()) {
      result.observations.mutable_latest_gateway_acceptance()->mutable_source_observation()->set_validity(
          wire::OBSERVATION_VALIDITY_UNKNOWN);
    }
    for (auto &alert : *result.observations.mutable_alerts()) {
      alert.mutable_observation()->set_validity(wire::OBSERVATION_VALIDITY_UNKNOWN);
    }
  }
  if (expired(result.vehicle_status_received_steady_ns) && result.observations.has_vehicle()) {
    result.observations.mutable_vehicle()->mutable_observation()->set_validity(wire::OBSERVATION_VALIDITY_UNKNOWN);
  }
  return result;
}

namespace {

struct ExampleOptions {
  std::string protocol_config;
  std::string vsomeip_config;
  std::string application_name{"control_link_someip_client"};
  std::uint32_t period_ms{50};
  std::uint32_t run_seconds{10};
  std::uint32_t keep_period_ms{0};
  std::uint32_t close_after_ms{0};
  std::uint32_t pause_after_ms{0};
  std::uint32_t pause_duration_ms{0};
  double velocity{0.3};
  double yaw{0.0};
  bool no_command{false};
  bool no_keep{false};
  bool no_close{false};
  bool readonly_only{false};
  bool hold{false};
};

std::string identity_hex(const std::string &bytes) {
  std::ostringstream output;
  output << std::hex << std::setfill('0');
  for (unsigned char byte : bytes) output << std::setw(2) << static_cast<unsigned>(byte);
  return output.str();
}

std::uint32_t unsigned_argument(const std::string &text) {
  if (text.empty() || text.front() == '-') throw std::invalid_argument("Expected unsigned option value");
  std::size_t consumed = 0;
  const auto value = std::stoull(text, &consumed);
  if (consumed != text.size() || value > std::numeric_limits<std::uint32_t>::max()) {
    throw std::invalid_argument("Numeric option outside uint32 range");
  }
  return static_cast<std::uint32_t>(value);
}

ExampleOptions example_options(int argc, char **argv) {
  ExampleOptions options;
  for (int index = 1; index < argc; ++index) {
    const std::string name(argv[index]);
    if (name == "--no-command" || name == "--keep-only") options.no_command = true;
    else if (name == "--no-keep") options.no_keep = true;
    else if (name == "--no-close") options.no_close = true;
    else if (name == "--readonly-only") options.readonly_only = true;
    else {
      if (index + 1 == argc) throw std::invalid_argument("Missing value for " + name);
      const std::string value(argv[++index]);
      if (name == "--config" || name == "--protocol-config") options.protocol_config = value;
      else if (name == "--vsomeip-config" || name == "--network-config") options.vsomeip_config = value;
      else if (name == "--application" || name == "--application-name") options.application_name = value;
      else if (name == "--period-ms") options.period_ms = unsigned_argument(value);
      else if (name == "--run-seconds") options.run_seconds = unsigned_argument(value);
      else if (name == "--keep-period-ms") options.keep_period_ms = unsigned_argument(value);
      else if (name == "--close-after-ms") options.close_after_ms = unsigned_argument(value);
      else if (name == "--pause-command-after-ms") options.pause_after_ms = unsigned_argument(value);
      else if (name == "--pause-command-duration-ms") options.pause_duration_ms = unsigned_argument(value);
      else if (name == "--velocity" || name == "--linear-velocity") options.velocity = std::stod(value);
      else if (name == "--yaw" || name == "--angular-velocity") options.yaw = std::stod(value);
      else if (name == "--mode") {
        if (value != "normal" && value != "hold") throw std::invalid_argument("Mode must be normal or hold");
        options.hold = value == "hold";
      } else throw std::invalid_argument("Unknown option " + name);
    }
  }
  if (options.protocol_config.empty() || options.vsomeip_config.empty() ||
      options.period_ms == 0 || options.run_seconds == 0 || options.run_seconds > 3600) {
    throw std::invalid_argument("Explicit --config and --vsomeip-config required; period > 0 and run-seconds in [1,3600]");
  }
  return options;
}

}

int run_planning_example(int argc, char **argv) {
  std::cout.setf(std::ios::unitbuf);
  if (argc == 2 && std::string(argv[1]) == "--help") {
    std::cout << "--config YAML --vsomeip-config JSON --application NAME "
                 "[--run-seconds 10 --period-ms 50 --velocity 0.3 --yaw 0 --mode normal|hold] "
                 "[--keep-period-ms N --no-command --no-keep --no-close --readonly-only] "
                 "[--close-after-ms N --pause-command-after-ms N --pause-command-duration-ms N]\n";
    return 0;
  }
  try {
    ExampleSignalScope signal_scope;
    const auto options = example_options(argc, argv);
    const auto config = load_config(options.protocol_config);
    const auto keep_period = std::chrono::milliseconds(
        options.keep_period_ms == 0 ? config.keep_period_ms : options.keep_period_ms);
    if (keep_period <= std::chrono::milliseconds::zero() ||
        keep_period >= std::chrono::milliseconds(config.idle_timeout_ms)) {
      throw std::invalid_argument("Example Keep period must be less than configured network idle timeout");
    }
    SomeipClient client(config, options.vsomeip_config, options.application_name);
    client.start();
    std::cout << "client pid=" << ::getpid() << " exe="
              << std::filesystem::read_symlink("/proc/self/exe").string()
              << " period_ms=" << options.period_ms << " keep_period_ms=" << keep_period.count()
              << " source=planning frame=body clock=SYSTEM units=m/s,rad/s\n";
    auto holder_nonce = random_identity();
    auto open_request_id = random_identity();
    wire::OpenAdmissionDomain opening_domain;
    std::string current_token;
    std::uint64_t business_sequence = 0;
    std::uint64_t keep_sequence = 0;
    std::uint64_t observed_generation = client.status().availability_generation;
    std::uint32_t open_retry_delay_ms = config.reconnect_initial_ms;
    bool held{false};
    bool ready{false};
    bool pause_reported{false};
    const auto started = std::chrono::steady_clock::now();
    const auto deadline = started + std::chrono::seconds(options.run_seconds);
    auto next_open = started;
    auto next_keep = started;
    auto next_command = started;
    auto next_observation = started;
    auto next_query = started;
    auto first_ready = std::chrono::steady_clock::time_point{};
    auto last_error_log = started - std::chrono::seconds(1);
    const auto new_holder = [&] {
      holder_nonce = random_identity();
      open_request_id = random_identity();
      opening_domain.Clear();
      current_token.clear();
      held = false;
      ready = false;
      keep_sequence = 0;
      business_sequence = 0;
      next_open = std::chrono::steady_clock::now();
      next_keep = next_open;
      open_retry_delay_ms = config.reconnect_initial_ms;
    };
    const auto observe_session = [&](const wire::OpenSessionResponse &response) {
      if (response.has_session()) {
        held = true;
        const auto &session = response.session();
        if (session.session_token() != current_token) {
          current_token = session.session_token();
          business_sequence = 0;
          std::cout << "new_session token=" << identity_hex(current_token)
                    << " gid=" << identity_hex(session.ingress_publisher().gid())
                    << " wall_clock_ns=" << system_now_ns() << '\n';
        }
        const bool was_ready = ready;
        ready = session.submission_ready();
        if (ready && !was_ready) {
          if (first_ready == std::chrono::steady_clock::time_point{}) first_ready = std::chrono::steady_clock::now();
          next_command = std::chrono::steady_clock::now();
          std::cout << "session_ready=true gid=" << identity_hex(session.ingress_publisher().gid())
                    << " wall_clock_ns=" << system_now_ns() << '\n';
        }
      } else if (response.result() == wire::SESSION_OPERATION_RESULT_REVOKED ||
                 response.result() == wire::SESSION_OPERATION_RESULT_REJECTED) {
        new_holder();
      }
    };
    while (!example_stop_signal && std::chrono::steady_clock::now() < deadline) {
      auto now = std::chrono::steady_clock::now();
      auto state = client.status();
      if (!state.running) break;
      if (state.availability_generation != observed_generation) {
        observed_generation = state.availability_generation;
        new_holder();
      }
      if (!state.service_available) {
        client.wait_available(std::chrono::milliseconds(50));
        continue;
      }
      if (held && !state.session.submission_ready()) ready = false;
      try {
        if (options.readonly_only) {
          if (now >= next_query) {
            client.get_status(std::chrono::milliseconds(100));
            next_query = now + std::chrono::milliseconds(100);
          }
        } else {
          if (held && !options.no_keep && now >= next_keep) {
            wire::KeepSessionRequest request;
            request.set_schema_version(1);
            request.set_holder_nonce(holder_nonce);
            request.set_open_request_id(open_request_id);
            *request.mutable_opening_domain() = opening_domain;
            request.set_keepalive_sequence(++keep_sequence);
            next_keep = now + keep_period;
            observe_session(client.keep(request, std::chrono::milliseconds(20)));
          }
          now = std::chrono::steady_clock::now();
          if (!ready && now >= next_open &&
              !(options.no_keep && held && first_ready != std::chrono::steady_clock::time_point{})) {
            // 只在新键对首次开立前发现域；超时、迟到回复及只读更新均不重绑定旧请求。
            if (!valid_opening_domain(opening_domain)) {
              opening_domain = client.get_status(std::chrono::milliseconds(100)).status().opening_domain();
            }
            wire::OpenSessionRequest request;
            request.set_schema_version(1);
            request.set_holder_nonce(holder_nonce);
            request.set_open_request_id(open_request_id);
            *request.mutable_opening_domain() = opening_domain;
            next_open = now + std::chrono::milliseconds(open_retry_delay_ms);
            open_retry_delay_ms = static_cast<std::uint32_t>(std::min<std::uint64_t>(
                config.reconnect_max_ms, static_cast<std::uint64_t>(open_retry_delay_ms) * 2));
            // 超时仍重试相同键，不能把响应丢失误当 session/物理连接终止。
            const auto response = client.open(request, std::chrono::milliseconds(100));
            if (response.has_session()) open_retry_delay_ms = config.reconnect_initial_ms;
            observe_session(response);
          }
          now = std::chrono::steady_clock::now();
          if (ready && first_ready != std::chrono::steady_clock::time_point{} &&
              options.close_after_ms > 0 && now - first_ready >= std::chrono::milliseconds(options.close_after_ms)) {
            wire::CloseSessionRequest request;
            request.set_schema_version(1);
            request.set_holder_nonce(holder_nonce);
            request.set_open_request_id(open_request_id);
            *request.mutable_opening_domain() = opening_domain;
            const auto response = client.close(request, std::chrono::milliseconds(100));
            std::cout << "explicit_close result=" << response.result() << '\n';
            held = false;
            ready = false;
            break;
          }
          const bool pause = options.pause_after_ms > 0 &&
              first_ready != std::chrono::steady_clock::time_point{} &&
              now - first_ready >= std::chrono::milliseconds(options.pause_after_ms) &&
              now - first_ready < std::chrono::milliseconds(
                  static_cast<std::uint64_t>(options.pause_after_ms) + options.pause_duration_ms);
          if (pause != pause_reported) {
            pause_reported = pause;
            std::cout << "command_pause=" << (pause ? "true" : "false")
                      << " wall_clock_ns=" << system_now_ns()
                      << " business_sequence=" << business_sequence << '\n';
          }
          if (ready && !options.no_command && !pause && now >= next_command) {
            if (business_sequence == std::numeric_limits<std::uint64_t>::max()) {
              throw std::runtime_error("Business sequence exhausted; explicit session termination required");
            }
            control_link_edge_protocol::CommandFrame target;
            std::memcpy(target.session_token.bytes.data(), current_token.data(), current_token.size());
            target.command_sequence = ++business_sequence;
            target.source_stamp_ns = system_now_ns();
            target.mode = options.hold ? control_link_edge_protocol::CommandMode::kHold :
                                       control_link_edge_protocol::CommandMode::kNormal;
            target.linear_velocity_mps = options.hold ? 0.0 : options.velocity;
            target.angular_velocity_radps = options.hold ? 0.0 : options.yaw;
            next_command += std::chrono::milliseconds(options.period_ms);
            if (next_command <= now) next_command = now + std::chrono::milliseconds(options.period_ms);
            const auto receipt = client.submit(target, std::chrono::milliseconds(20));
            std::cout << "submit sequence=" << target.command_sequence << " source_stamp_ns="
                      << target.source_stamp_ns << " rpc_received=" << receipt.rpc_received()
                      << " forwarding_state=" << receipt.forwarding_state()
                      << " gateway_acceptance=" << receipt.gateway_acceptance().state() << '\n';
          }
        }
      } catch (const std::exception &failure) {
        now = std::chrono::steady_clock::now();
        if (now - last_error_log >= std::chrono::seconds(1)) {
          std::cerr << "client operation: " << failure.what() << '\n';
          last_error_log = now;
        }
      }
      now = std::chrono::steady_clock::now();
      if (now >= next_observation) {
        state = client.status();
        const auto &snapshot = state.observations;
        std::cout << "observation wall_clock_ns=" << system_now_ns()
                  << " gateway_present=" << snapshot.has_gateway()
                  << " gateway_state=" << snapshot.gateway().state()
                  << " gateway_validity=" << snapshot.gateway().observation().validity()
                  << " gateway_observed_ns=" << snapshot.gateway().observation().observed_system_ns()
                  << " active_source=" << snapshot.gateway().active_source_id()
                  << " vehicle_present=" << snapshot.has_vehicle()
                  << " vehicle_validity=" << snapshot.vehicle().observation().validity()
                  << " vehicle_observed_ns=" << snapshot.vehicle().observation().observed_system_ns();
        if (snapshot.sources_size() > 0) {
          std::cout << " accepted_count=" << snapshot.sources(0).accepted_count()
                    << " source_validity=" << snapshot.sources(0).observation().validity()
                    << " source_sequence=" << snapshot.sources(0).last_source_sequence();
        }
        std::cout << '\n';
        next_observation = now + std::chrono::milliseconds(100);
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
    if (held && !options.no_close && !options.readonly_only && client.status().service_available) {
      wire::CloseSessionRequest request;
      request.set_schema_version(1);
      request.set_holder_nonce(holder_nonce);
      request.set_open_request_id(open_request_id);
      *request.mutable_opening_domain() = opening_domain;
      try {
        client.close(request, std::chrono::milliseconds(100));
      } catch (const std::exception &failure) {
        std::cerr << "close: " << failure.what() << '\n';
      }
    }
    const bool observed = client.status().observations.has_gateway();
    client.stop();
    if ((!options.readonly_only && first_ready == std::chrono::steady_clock::time_point{}) ||
        (options.readonly_only && !observed)) {
      std::cerr << "Example never reached its required ingress/readonly readiness; no business PASS claimed\n";
      return 2;
    }
    return 0;
  } catch (const std::exception &failure) {
    std::cerr << "someip_planning_client: " << failure.what() << '\n';
    return 1;
  }
}

}
