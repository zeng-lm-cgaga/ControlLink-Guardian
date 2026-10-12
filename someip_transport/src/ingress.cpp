#include "control_link_someip_transport/ingress.hpp"

#include <chrono>
#include <cstdint>
#include <limits>
#include <mutex>
#include <optional>
#include <set>
#include <stdexcept>
#include <string>
#include <utility>

#ifndef CONTROL_LINK_NETWORK_SCHEMA_SHA256_HEX
#error "The ingress target must define CONTROL_LINK_NETWORK_SCHEMA_SHA256_HEX from its .proto input"
#endif

namespace control_link::someip {
namespace {

namespace codec = control_link_edge_protocol;

wire::Capabilities normalized_capabilities(wire::Capabilities capabilities) {
  const std::string fingerprint_hex = CONTROL_LINK_NETWORK_SCHEMA_SHA256_HEX;
  if (fingerprint_hex.size() != 64) throw std::invalid_argument("invalid build-time schema fingerprint");
  std::string fingerprint;
  fingerprint.reserve(32);
  for (std::size_t offset = 0; offset < fingerprint_hex.size(); offset += 2) {
    const auto pair = fingerprint_hex.substr(offset, 2);
    std::size_t consumed = 0;
    const auto byte = std::stoul(pair, &consumed, 16);
    if (consumed != 2 || byte > 255) throw std::invalid_argument("invalid build-time schema fingerprint");
    fingerprint.push_back(static_cast<char>(byte));
  }
  capabilities.set_schema_version(1);
  capabilities.set_schema_fingerprint(std::move(fingerprint));
  return capabilities;
}

std::string payload_capacity_error(const Config &config, const wire::Capabilities &capabilities,
                                   const std::string &rmw) {
  try {
    validate_protocol_payload_budget(config, capabilities, rmw);
    return {};
  } catch (const std::invalid_argument &error) {
    return error.what();
  }
}

std::string token_bytes(const codec::SessionToken &token) {
  return std::string(reinterpret_cast<const char *>(token.bytes.data()), token.bytes.size());
}

bool valid_identity(const std::string &value) {
  return value.size() == codec::kSessionTokenSize;
}

bool representable_stamp(std::int64_t stamp) {
  constexpr std::int64_t nanoseconds_per_second = 1000000000;
  constexpr auto minimum = static_cast<std::int64_t>(std::numeric_limits<std::int32_t>::min()) * nanoseconds_per_second;
  constexpr auto maximum = static_cast<std::int64_t>(std::numeric_limits<std::int32_t>::max()) * nanoseconds_per_second + nanoseconds_per_second - 1;
  return stamp >= minimum && stamp <= maximum;
}

void redact_tokens(google::protobuf::Message *message) {
  const auto *reflection = message->GetReflection();
  for (int index = 0; index < message->GetDescriptor()->field_count(); ++index) {
    const auto *field = message->GetDescriptor()->field(index);
    if (field->name() == "session_token") {
      reflection->ClearField(message, field);
    } else if (field->cpp_type() == google::protobuf::FieldDescriptor::CPPTYPE_MESSAGE) {
      if (field->is_repeated()) {
        for (int element = 0; element < reflection->FieldSize(*message, field); ++element) {
          redact_tokens(reflection->MutableRepeatedMessage(message, field, element));
        }
      } else if (reflection->HasField(*message, field)) {
        redact_tokens(reflection->MutableMessage(message, field));
      }
    }
  }
}

}

struct PlanningIngress::Impl {
  struct Holder {
    std::uint16_t peer;
    std::string nonce;
    std::string request_id;

    bool matches(std::uint16_t candidate_peer, const std::string &candidate_nonce,
                 const std::string &candidate_request_id) const {
      return peer == candidate_peer && nonce == candidate_nonce && request_id == candidate_request_id;
    }
  };

  explicit Impl(Config configuration, wire::Capabilities advertised)
      : config(std::move(configuration)), capabilities(normalized_capabilities(std::move(advertised))) {
    // RMW 尚由 ROS owner 装配；真实身份须预检，构造仍拒绝已知 Capabilities 的必然超限。
    validate_protocol_payload_budget(config, capabilities, "r");
    reset_session_locked();
  }

  void reset_session_locked() {
    session.Clear();
    session.set_planning_transport(wire::PLANNING_TRANSPORT_SOMEIP);
    session.set_clock_domain(wire::CLOCK_DOMAIN_SYSTEM);
    session.set_control_session_state(state);
    session.set_connection_state(wire::CONNECTION_STATE_DISCONNECTED);
  }

  bool live_locked() const {
    return state == wire::CONTROL_SESSION_STATE_OPENING || state == wire::CONTROL_SESSION_STATE_ADMITTED;
  }

  wire::OpenSessionResponse response_locked(wire::SessionOperationResult result,
                                            const std::string &request_id,
                                            bool include_session = false) const {
    wire::OpenSessionResponse response;
    response.set_result(result);
    response.set_open_request_id(request_id);
    *response.mutable_capabilities() = capabilities;
    if (include_session) *response.mutable_session() = session;
    return response;
  }

  wire::OpenSessionResponse rejected_locked(const std::string &request_id) {
    counters.set_protocol_rejected_count(counters.protocol_rejected_count() + 1);
    return response_locked(wire::SESSION_OPERATION_RESULT_REJECTED, request_id);
  }

  void drop_pending_locked() {
    if (pending) {
      counters.set_dropped_count(counters.dropped_count() + 1);
      pending.reset();
    }
  }

  void revoke_locked(const std::string &reason) {
    if (state == wire::CONTROL_SESSION_STATE_REVOKING) return;
    drop_pending_locked();
    if (epoch == std::numeric_limits<std::uint64_t>::max()) {
      failure = "EPOCH_EXHAUSTED";
    } else {
      ++epoch;
      failure = reason;
    }
    state = wire::CONTROL_SESSION_STATE_REVOKING;
    session.set_control_session_state(state);
    session.set_submission_ready(false);
    session.set_connection_state(wire::CONNECTION_STATE_CLOSING);
  }

  void expire_locked(std::uint64_t now) {
    if (!live_locked()) return;
    if (now < last_keep_ns) {
      revoke_locked("STEADY_CLOCK_REGRESSION");
    } else if (now - last_keep_ns >= static_cast<std::uint64_t>(config.idle_timeout_ms) * 1000000) {
      revoke_locked("HOLDER_IDLE_TIMEOUT");
    }
  }

  wire::StatusSnapshot status_locked() const {
    auto result = observations;
    *result.mutable_session() = session;
    *result.mutable_transport() = counters;
    result.mutable_transport()->set_connection_state(session.connection_state());
    if (latest_forwarded.has_key()) *result.mutable_latest_forwarded() = latest_forwarded;
    else result.clear_latest_forwarded();
    auto *acceptance = result.mutable_latest_gateway_acceptance();
    acceptance->Clear();
    acceptance->set_state(wire::GATEWAY_ACCEPTANCE_STATE_UNKNOWN);
    if (!live_locked() || !latest_forwarded.has_key() ||
        latest_forwarded.publisher().gid() != session.ingress_publisher().gid()) return result;
    for (const auto &source : result.sources()) {
      // USABLE=5 的字段 owner 是 SourceStatus.msg；这里只关联原观察，不重做 Binding。
      if (source.source_id() != "planning" ||
          source.observation().validity() != wire::OBSERVATION_VALIDITY_VALID ||
          !source.endpoint_stable() || source.endpoint_state() != 5 ||
          source.endpoint_publisher_count() != 1 || !source.endpoint_generation_valid() ||
          source.accepted_count() == 0 ||
          source.endpoint_publisher().gid().size() != 24 ||
          source.endpoint_publisher().gid() != latest_forwarded.publisher().gid() ||
          source.endpoint_publisher().rmw_implementation() != latest_forwarded.publisher().rmw_implementation() ||
          source.last_source_sequence() != latest_forwarded.key().source_sequence()) continue;
      acceptance->set_state(wire::GATEWAY_ACCEPTANCE_STATE_LATEST_KEY_OBSERVED);
      *acceptance->mutable_key() = latest_forwarded.key();
      *acceptance->mutable_publisher() = source.endpoint_publisher();
      *acceptance->mutable_source_observation() = source.observation();
      acceptance->set_accepted_count(source.accepted_count());
      break;
    }
    return result;
  }

  const Config config;
  const wire::Capabilities capabilities;
  mutable std::mutex mutex;
  std::uint64_t epoch{0};
  wire::ControlSessionState state{wire::CONTROL_SESSION_STATE_IDLE};
  wire::SessionInfo session;
  std::optional<Holder> holder;
  std::set<std::pair<std::string, std::string>> retired;
  std::uint64_t last_keep_ns{0};
  std::uint64_t keep_sequence{0};
  std::optional<codec::CommandFrame> pending;
  wire::ForwardedCommandObservation latest_forwarded;
  wire::TransportCounters counters;
  wire::StatusSnapshot observations;
  std::string failure;
};

PlanningIngress::PlanningIngress(Config config, wire::Capabilities capabilities)
    : impl_(std::make_unique<Impl>(std::move(config), std::move(capabilities))) {}

PlanningIngress::~PlanningIngress() = default;

wire::OpenSessionResponse PlanningIngress::open(std::uint16_t peer,
                                               const wire::OpenSessionRequest &request,
                                               std::uint64_t steady_ns) {
  std::lock_guard<std::mutex> lock(impl_->mutex);
  impl_->counters.set_rpc_received_count(impl_->counters.rpc_received_count() + 1);
  impl_->expire_locked(steady_ns);
  if (request.schema_version_presence_case() != wire::OpenSessionRequest::kSchemaVersion ||
      request.schema_version() != 1 || peer == 0 || peer == UINT16_MAX ||
      !valid_identity(request.holder_nonce()) || !valid_identity(request.open_request_id())) {
    return impl_->rejected_locked(request.open_request_id());
  }
  if (impl_->live_locked()) {
    if (impl_->holder->matches(peer, request.holder_nonce(), request.open_request_id())) {
      return impl_->response_locked(wire::SESSION_OPERATION_RESULT_IDEMPOTENT, request.open_request_id(), true);
    }
    return impl_->response_locked(wire::SESSION_OPERATION_RESULT_BUSY, request.open_request_id());
  }
  if (impl_->state == wire::CONTROL_SESSION_STATE_REVOKING) {
    return impl_->response_locked(wire::SESSION_OPERATION_RESULT_BUSY, request.open_request_id());
  }
  const auto key = std::make_pair(request.holder_nonce(), request.open_request_id());
  if (impl_->retired.count(key) != 0) return impl_->rejected_locked(request.open_request_id());
  if (impl_->retired.size() >= impl_->config.retired_open_capacity) {
    impl_->failure = "RESOURCE_EXHAUSTED: retired open keys";
    impl_->counters.set_protocol_rejected_count(impl_->counters.protocol_rejected_count() + 1);
    return impl_->response_locked(wire::SESSION_OPERATION_RESULT_RESOURCE_EXHAUSTED,
                                  request.open_request_id());
  }
  if (impl_->epoch == std::numeric_limits<std::uint64_t>::max()) {
    impl_->failure = "EPOCH_EXHAUSTED";
    return impl_->rejected_locked(request.open_request_id());
  }
  std::string token;
  try {
    token = random_identity();
  } catch (const std::exception &) {
    impl_->failure = "RANDOM_SOURCE_FAILED";
    return impl_->rejected_locked(request.open_request_id());
  }
  ++impl_->epoch;
  impl_->holder = Impl::Holder{peer, request.holder_nonce(), request.open_request_id()};
  impl_->state = wire::CONTROL_SESSION_STATE_OPENING;
  impl_->reset_session_locked();
  impl_->session.set_session_token(std::move(token));
  impl_->session.set_connection_state(wire::CONNECTION_STATE_CONNECTING);
  impl_->last_keep_ns = steady_ns;
  impl_->keep_sequence = 0;
  impl_->latest_forwarded.Clear();
  impl_->failure.clear();
  return impl_->response_locked(wire::SESSION_OPERATION_RESULT_OPENING, request.open_request_id(), true);
}

wire::OpenSessionResponse PlanningIngress::keep(std::uint16_t peer,
                                               const wire::KeepSessionRequest &request,
                                               std::uint64_t steady_ns) {
  std::lock_guard<std::mutex> lock(impl_->mutex);
  impl_->counters.set_rpc_received_count(impl_->counters.rpc_received_count() + 1);
  impl_->expire_locked(steady_ns);
  if (request.schema_version_presence_case() != wire::KeepSessionRequest::kSchemaVersion ||
      request.schema_version() != 1 ||
      request.keepalive_sequence_presence_case() != wire::KeepSessionRequest::kKeepaliveSequence || !impl_->live_locked() ||
      !impl_->holder->matches(peer, request.holder_nonce(), request.open_request_id()) ||
      request.keepalive_sequence() <= impl_->keep_sequence) {
    return impl_->rejected_locked(request.open_request_id());
  }
  impl_->keep_sequence = request.keepalive_sequence();
  impl_->last_keep_ns = steady_ns;
  return impl_->response_locked(impl_->state == wire::CONTROL_SESSION_STATE_ADMITTED
      ? wire::SESSION_OPERATION_RESULT_ADMITTED : wire::SESSION_OPERATION_RESULT_OPENING,
      request.open_request_id(), true);
}

wire::OpenSessionResponse PlanningIngress::close(std::uint16_t peer,
                                                const wire::CloseSessionRequest &request) {
  std::lock_guard<std::mutex> lock(impl_->mutex);
  impl_->counters.set_rpc_received_count(impl_->counters.rpc_received_count() + 1);
  if (request.schema_version_presence_case() != wire::CloseSessionRequest::kSchemaVersion ||
      request.schema_version() != 1 || !impl_->holder ||
      !impl_->holder->matches(peer, request.holder_nonce(), request.open_request_id())) {
    return impl_->rejected_locked(request.open_request_id());
  }
  impl_->revoke_locked("HOLDER_CLOSED");
  return impl_->response_locked(wire::SESSION_OPERATION_RESULT_REVOKED, request.open_request_id(), true);
}

wire::SubmitReceipt PlanningIngress::submit(std::uint16_t peer, const codec::CommandFrame &frame,
                                           std::uint64_t received_boot_ns) {
  std::lock_guard<std::mutex> lock(impl_->mutex);
  impl_->counters.set_rpc_received_count(impl_->counters.rpc_received_count() + 1);
  wire::SubmitReceipt receipt;
  receipt.set_schema_version(1);
  receipt.set_rpc_received(true);
  receipt.set_rpc_received_boottime_ns(received_boot_ns);
  receipt.mutable_key()->set_session_token(token_bytes(frame.session_token));
  receipt.mutable_key()->set_source_sequence(frame.command_sequence);
  receipt.mutable_gateway_acceptance()->set_state(wire::GATEWAY_ACCEPTANCE_STATE_UNKNOWN);
  if (impl_->state != wire::CONTROL_SESSION_STATE_ADMITTED || !impl_->session.submission_ready() ||
      !impl_->holder || peer != impl_->holder->peer || received_boot_ns == 0 ||
      token_bytes(frame.session_token) != impl_->session.session_token() ||
      frame.clock_domain != codec::ClockDomain::kSystem || !representable_stamp(frame.source_stamp_ns)) {
    impl_->counters.set_protocol_rejected_count(impl_->counters.protocol_rejected_count() + 1);
    receipt.set_forwarding_state(wire::FORWARDING_STATE_REJECTED);
    return receipt;
  }
  if (impl_->pending) {
    impl_->counters.set_missed_count(impl_->counters.missed_count() + 1);
    impl_->drop_pending_locked();
  }
  // 这里只保留可表达原值；mode/finite/业务 sequence/时效由 Gateway 唯一判断。
  impl_->pending = frame;
  receipt.set_forwarding_state(wire::FORWARDING_STATE_PENDING);
  return receipt;
}

void PlanningIngress::expire(std::uint64_t steady_ns) {
  std::lock_guard<std::mutex> lock(impl_->mutex);
  impl_->expire_locked(steady_ns);
}

void PlanningIngress::revoke(const std::string &reason) {
  std::lock_guard<std::mutex> lock(impl_->mutex);
  impl_->revoke_locked(reason);
}

bool PlanningIngress::revoke_if_current(std::uint64_t epoch, const std::string &reason) {
  std::lock_guard<std::mutex> lock(impl_->mutex);
  if (epoch != impl_->epoch || impl_->state == wire::CONTROL_SESSION_STATE_REVOKING) return false;
  impl_->revoke_locked(reason);
  return true;
}

bool PlanningIngress::finish_release(std::uint64_t revoked_epoch) {
  std::lock_guard<std::mutex> lock(impl_->mutex);
  if (revoked_epoch != impl_->epoch || impl_->state != wire::CONTROL_SESSION_STATE_REVOKING) return false;
  if (impl_->holder) {
    const auto key = std::make_pair(impl_->holder->nonce, impl_->holder->request_id);
    if (impl_->retired.count(key) == 0 && impl_->retired.size() >= impl_->config.retired_open_capacity) {
      impl_->failure = "RESOURCE_EXHAUSTED: cannot retire open key";
      return false;
    }
    impl_->retired.insert(key);
  }
  impl_->holder.reset();
  impl_->state = wire::CONTROL_SESSION_STATE_IDLE;
  impl_->reset_session_locked();
  impl_->latest_forwarded.Clear();
  return true;
}

IngressSnapshot PlanningIngress::snapshot() const {
  std::lock_guard<std::mutex> lock(impl_->mutex);
  return IngressSnapshot{impl_->epoch, impl_->state, impl_->session, impl_->failure};
}

bool PlanningIngress::bind_publisher(std::uint64_t epoch, const std::string &gid, const std::string &rmw) {
  std::lock_guard<std::mutex> lock(impl_->mutex);
  if (epoch != impl_->epoch || impl_->state != wire::CONTROL_SESSION_STATE_OPENING || gid.size() != 24 || rmw.empty()) return false;
  const auto &publisher = impl_->session.ingress_publisher();
  if (!publisher.gid().empty() && (publisher.gid() != gid || publisher.rmw_implementation() != rmw)) return false;
  const auto capacity_error = payload_capacity_error(impl_->config, impl_->capabilities, rmw);
  if (!capacity_error.empty()) {
    impl_->revoke_locked(capacity_error);
    return false;
  }
  impl_->session.mutable_ingress_publisher()->set_gid(gid);
  impl_->session.mutable_ingress_publisher()->set_rmw_implementation(rmw);
  return true;
}

bool PlanningIngress::set_admitted(std::uint64_t epoch, bool ready,
    const std::function<bool(const wire::StatusSnapshot &)> &ready_guard) {
  std::lock_guard<std::mutex> lock(impl_->mutex);
  if (epoch != impl_->epoch || !impl_->live_locked()) return false;
  if (ready && (impl_->session.ingress_publisher().gid().size() != 24 ||
                impl_->session.ingress_publisher().rmw_implementation().empty())) return false;
  const bool guard_rejected = ready && ready_guard && !ready_guard(impl_->status_locked());
  if (guard_rejected) ready = false;
  if (!ready) impl_->drop_pending_locked();
  impl_->state = ready ? wire::CONTROL_SESSION_STATE_ADMITTED : wire::CONTROL_SESSION_STATE_OPENING;
  impl_->session.set_control_session_state(impl_->state);
  impl_->session.set_submission_ready(ready);
  impl_->session.set_connection_state(ready ? wire::CONNECTION_STATE_CONNECTED : wire::CONNECTION_STATE_CONNECTING);
  return !guard_rejected;
}

bool PlanningIngress::publish_pending(std::uint64_t epoch, std::uint64_t forwarded_boot_ns,
    const std::function<void(const codec::CommandFrame &)> &publish,
    const std::function<bool(const wire::StatusSnapshot &)> &ready_guard) {
  std::lock_guard<std::mutex> lock(impl_->mutex);
  if (epoch != impl_->epoch || impl_->state != wire::CONTROL_SESSION_STATE_ADMITTED ||
      !impl_->session.submission_ready() || !impl_->pending) return false;
  if (ready_guard && !ready_guard(impl_->status_locked())) {
    impl_->drop_pending_locked();
    impl_->session.set_submission_ready(false);
    return false;
  }
  // 最终 epoch 复核与 ROS publish 提交在同锁边界；callback 禁止重入或网络 I/O。
  if (!publish || forwarded_boot_ns == 0) {
    impl_->revoke_locked("INVALID_PUBLISH_COMMIT");
    return false;
  }
  try {
    publish(*impl_->pending);
  } catch (...) {
    impl_->revoke_locked("PUBLISH_FAILED");
    return false;
  }
  auto *key = impl_->latest_forwarded.mutable_key();
  key->set_session_token(token_bytes(impl_->pending->session_token));
  key->set_source_sequence(impl_->pending->command_sequence);
  *impl_->latest_forwarded.mutable_publisher() = impl_->session.ingress_publisher();
  impl_->latest_forwarded.set_forwarded_boottime_ns(forwarded_boot_ns);
  impl_->counters.set_forwarded_count(impl_->counters.forwarded_count() + 1);
  impl_->pending.reset();
  return true;
}

void PlanningIngress::update_observations(const std::function<void(wire::StatusSnapshot &)> &update) {
  std::lock_guard<std::mutex> lock(impl_->mutex);
  auto observations = impl_->observations;
  update(observations);
  // ROS owner 只能更新独立原始观察，不能借映射回调改变网络 holder/计数或伪造转发。
  observations.clear_session();
  observations.clear_transport();
  observations.clear_latest_forwarded();
  observations.clear_latest_gateway_acceptance();
  impl_->observations = std::move(observations);
}

void PlanningIngress::record_protocol_rejection() {
  std::lock_guard<std::mutex> lock(impl_->mutex);
  impl_->counters.set_rpc_received_count(impl_->counters.rpc_received_count() + 1);
  impl_->counters.set_protocol_rejected_count(impl_->counters.protocol_rejected_count() + 1);
}

wire::StatusSnapshot PlanningIngress::readonly_status() const {
  std::lock_guard<std::mutex> lock(impl_->mutex);
  auto result = impl_->status_locked();
  redact_tokens(&result);
  return result;
}

std::uint64_t steady_now_ns() {
  const auto elapsed = std::chrono::steady_clock::now().time_since_epoch();
  const auto count = std::chrono::duration_cast<std::chrono::nanoseconds>(elapsed).count();
  return count < 0 ? 0 : static_cast<std::uint64_t>(count);
}

std::uint64_t boottime_now_ns() { return codec::boottime_now_ns(); }

std::int64_t system_now_ns() { return codec::system_time_now_ns(); }

std::string random_identity() {
  codec::SessionToken token;
  if (!codec::generate_session_token(&token)) throw std::runtime_error("getrandom failed to generate 16B session identity");
  return token_bytes(token);
}

}
