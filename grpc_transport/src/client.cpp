#include "control_link_grpc_transport/client.hpp"

#include "control_link_network_v1.grpc.pb.h"
#include "client_reply_policy.hpp"

#include <grpcpp/grpcpp.h>

#include <algorithm>
#include <condition_variable>
#include <cerrno>
#include <cmath>
#include <cstring>
#include <filesystem>
#include <functional>
#include <iostream>
#include <limits>
#include <map>
#include <mutex>
#include <stdexcept>
#include <thread>
#include <type_traits>
#include <utility>
#include <unistd.h>
#include <pthread.h>
#include <signal.h>

namespace control_link::grpc_transport {
namespace {

namespace network = ::control_link::network;
using Milliseconds = std::chrono::milliseconds;

std::chrono::system_clock::time_point rpc_deadline(Milliseconds budget) {
  const auto count = static_cast<std::uint64_t>(budget.count());
  std::uint64_t quantum = 1;
  while (count / quantum >= 1000) quantum *= 10;
  // 锁定gRPC的wire timeout向上取三位有效数；缩短两量化单位，绝不扩大服务端预算。
  const auto nanoseconds = count > 2 * quantum ? (count - 2 * quantum) * 1000000 : count * 500000;
  return std::chrono::system_clock::now() + std::chrono::nanoseconds(nanoseconds);
}

void bounded_timeout(Milliseconds timeout, std::uint32_t maximum) {
  if (timeout <= Milliseconds::zero() || timeout.count() > maximum) {
    throw std::invalid_argument("gRPC timeout must be positive and within the configured RPC budget");
  }
}

void bounded_message(const google::protobuf::MessageLite &message, std::uint32_t maximum) {
  if (message.ByteSizeLong() > maximum) {
    throw std::invalid_argument("gRPC message exceeds configured max_message_bytes");
  }
}

bool readonly_has_token(const wire::StatusSnapshot &snapshot) {
  return !snapshot.session().session_token().empty() ||
      !snapshot.latest_forwarded().key().session_token().empty() ||
      !snapshot.latest_gateway_acceptance().key().session_token().empty();
}

void valid_open(const wire::OpenSessionRequest &request) {
  if (request.schema_version_presence_case() != wire::OpenSessionRequest::kSchemaVersion || request.schema_version() != 1 ||
      request.holder_nonce().size() != 16 || request.open_request_id().size() != 16) {
    throw std::invalid_argument("gRPC open requires V1 presence and two independent 16B holder keys");
  }
}

std::string token_bytes(const control_link_edge_protocol::SessionToken &token) {
  return std::string(reinterpret_cast<const char *>(token.bytes.data()), token.bytes.size());
}

std::string rpc_failure(const grpc::Status &status) {
  return "gRPC code=" + std::to_string(static_cast<int>(status.error_code())) +
      " " + status.error_message();
}

}

struct GrpcClient::Impl {
  struct Tag {
    std::function<void(bool)> complete;
  };

  struct ControlExchange {
    wire::ControlSessionRequest request;
    wire::OpenSessionResponse response;
    std::uint64_t readiness_revision{0};
    bool write_complete{false};
    bool response_complete{false};
    std::string error;
  };

  struct ControlCall {
    grpc::ClientContext context;
    std::unique_ptr<grpc::ClientAsyncReaderWriter<wire::ControlSessionRequest,
        wire::OpenSessionResponse>> stream;
    grpc::Status result;
    wire::OpenSessionResponse incoming;
    std::shared_ptr<ControlExchange> exchange;
    std::string holder_nonce;
    std::string open_request_id;
    std::uint64_t generation{0};
    bool start_pending{false};
    bool started{false};
    bool read_pending{false};
    bool write_pending{false};
    bool writes_done_pending{false};
    bool writes_done_sent{false};
    bool half_closed{false};
    bool closing{false};
    bool finish_pending{false};
    bool finished{false};
  };

  struct ReadonlyCall {
    grpc::ClientContext context;
    wire::SubscribeStatusRequest request;
    std::unique_ptr<grpc::ClientAsyncReader<wire::StatusEnvelope>> stream;
    grpc::Status result;
    wire::StatusEnvelope incoming;
    bool start_pending{false};
    bool started{false};
    bool read_pending{false};
    bool closing{false};
    bool finish_pending{false};
  };

  struct UnaryBase {
    grpc::ClientContext context;
    grpc::Status result;
    std::uint64_t generation{0};
    bool submission{false};
    bool done{false};
    std::string error;
    virtual ~UnaryBase() = default;
  };

  template <typename Request, typename Response>
  struct UnaryCall : UnaryBase {
    Request request;
    Response response;
    std::unique_ptr<grpc::ClientAsyncResponseReader<Response>> reader;
  };

  Config config;
  std::string fingerprint;
  std::shared_ptr<grpc::Channel> channel;
  std::unique_ptr<wire::PlanningIngress::Stub> stub;
  grpc::CompletionQueue cq;
  std::thread poller;
  mutable std::mutex mutex;
  std::condition_variable wake;
  ClientStatus current;
  std::shared_ptr<ControlCall> control;
  std::shared_ptr<ReadonlyCall> readonly;
  std::map<std::uint64_t, std::shared_ptr<UnaryBase>> unary_calls;
  std::uint64_t next_unary_id{0};
  std::uint64_t outstanding_tags{0};
  std::uint64_t readiness_revision{0};
  std::string publisher_instance;
  std::uint64_t publication_sequence{0};
  bool started{false};
  bool stopping{false};
  bool queue_shutdown{false};

  explicit Impl(Config settings) : config(std::move(settings)), fingerprint(network::schema_fingerprint()) {}

  ~Impl() {
    // 配置/构造失败也要关闭尚无操作的 CQ；正常运行由外部 owner 的 stop 完成 join/drain。
    if (!started && !queue_shutdown) {
      cq.Shutdown();
      void *unused_tag = nullptr;
      bool unused_ok = false;
      while (cq.Next(&unused_tag, &unused_ok)) {}
    }
  }

  Tag *tag(std::function<void(bool)> callback) {
    if (queue_shutdown) throw std::logic_error("gRPC enqueue after CQ shutdown");
    auto *result = new Tag{std::move(callback)};
    ++outstanding_tags;
    return result;
  }

  void require_running_locked() const {
    if (!current.running || stopping) throw std::runtime_error("gRPC client is not running");
  }

  void invalidate_control_locked(const std::string &reason) {
    ++readiness_revision;
    current.control_active = false;
    current.session.Clear();
    current.failure = reason;
    ++current.availability_generation;
    if (current.observations.has_session()) {
      current.observations.mutable_session()->set_submission_ready(false);
    }
    wake.notify_all();
  }

  void validate_capabilities(const wire::Capabilities &capabilities) const {
    if (capabilities.schema_version() != 1 || capabilities.schema_fingerprint() != fingerprint ||
        capabilities.planning_transport() != wire::PLANNING_TRANSPORT_GRPC) {
      throw std::runtime_error("gRPC capabilities schema/fingerprint/transport mismatch");
    }
    bounded_message(capabilities, config.max_message_bytes);
  }

  void validate_session_reply(const ControlCall &call, const wire::OpenSessionResponse &response) const {
    bounded_message(response, config.max_message_bytes);
    if (!response.has_capabilities() || response.open_request_id() != call.open_request_id) {
      throw std::runtime_error("gRPC control reply omitted capabilities or mismatched holder key");
    }
    validate_capabilities(response.capabilities());
    if (!wire::SessionOperationResult_IsValid(response.result()) ||
        response.result() == wire::SESSION_OPERATION_RESULT_UNKNOWN) {
      throw std::runtime_error("gRPC control reply has unknown operation result");
    }
    const bool denied = response.result() == wire::SESSION_OPERATION_RESULT_BUSY ||
        response.result() == wire::SESSION_OPERATION_RESULT_REJECTED ||
        response.result() == wire::SESSION_OPERATION_RESULT_RESOURCE_EXHAUSTED;
    if (denied) {
      if (response.has_session()) throw std::runtime_error("gRPC denied control reply leaked a holder session");
      return;
    }
    if (!response.has_session()) throw std::runtime_error("gRPC control reply omitted holder session");
    const auto &session = response.session();
    if (response.result() == wire::SESSION_OPERATION_RESULT_REVOKED &&
        (!call.exchange || !call.exchange->request.has_close() ||
         session.control_session_state() != wire::CONTROL_SESSION_STATE_REVOKING ||
         session.session_token() != current.session.session_token())) {
      throw std::runtime_error("gRPC revoked session is not the matching private Close receipt");
    }
    if (session.session_token().size() != 16 ||
        session.planning_transport() != wire::PLANNING_TRANSPORT_GRPC ||
        session.clock_domain() != wire::CLOCK_DOMAIN_SYSTEM ||
        (!session.ingress_publisher().gid().empty() && session.ingress_publisher().gid().size() != 24) ||
        (session.submission_ready() && (session.ingress_publisher().gid().size() != 24 ||
         session.ingress_publisher().rmw_implementation().empty() ||
         session.control_session_state() != wire::CONTROL_SESSION_STATE_ADMITTED))) {
      throw std::runtime_error("gRPC holder token/domain/publisher representation mismatch");
    }
  }

  void validate_envelope(const wire::StatusEnvelope &envelope) const {
    bounded_message(envelope, config.max_message_bytes);
    if (envelope.schema_version_presence_case() != wire::StatusEnvelope::kSchemaVersion || envelope.schema_version() != 1 ||
        envelope.schema_fingerprint() != fingerprint || envelope.publisher_instance_token().size() != 16 ||
        envelope.publication_sequence_presence_case() != wire::StatusEnvelope::kPublicationSequence || envelope.publication_sequence() == 0 ||
        envelope.published_system_presence_case() != wire::StatusEnvelope::kPublishedSystemNs ||
        envelope.published_boottime_presence_case() != wire::StatusEnvelope::kPublishedBoottimeNs ||
        envelope.published_boottime_ns() == 0 || !envelope.has_status() ||
        readonly_has_token(envelope.status())) {
      throw std::runtime_error("gRPC readonly envelope schema/presence/token-redaction mismatch");
    }
  }

  void merge_envelope_locked(const wire::StatusEnvelope &envelope) {
    validate_envelope(envelope);
    if (publisher_instance != envelope.publisher_instance_token()) {
      if (!publisher_instance.empty() && control && !control->closing) {
        cancel_control_locked(control, "gRPC status producer generation changed");
      }
      publisher_instance = envelope.publisher_instance_token();
      publication_sequence = 0;
    }
    if (envelope.publication_sequence() <= publication_sequence) return;
    publication_sequence = envelope.publication_sequence();
    if (current.status_received_steady_ns != 0) ++current.readonly_overwritten_count;
    current.status_received_steady_ns = network::steady_now_ns();
    current.observations = envelope.status();
    // readonly 只能撤可提交提示；不得借脱敏状态开立、升级或旋转控制 holder。
    if (envelope.status().has_session() &&
        (!envelope.status().session().submission_ready() ||
         (!current.session.ingress_publisher().gid().empty() &&
          current.session.ingress_publisher().gid() != envelope.status().session().ingress_publisher().gid()))) {
      ++readiness_revision;
      current.session.set_submission_ready(false);
    }
  }

  void finish_control_locked(const std::shared_ptr<ControlCall> &call) {
    if (!call->closing || call->finish_pending || call->finished || call->start_pending ||
        call->read_pending || call->write_pending || call->writes_done_pending || call->half_closed) return;
    call->finish_pending = true;
    // Finish 与 Read/Write/WritesDone 不并发；tag 的强引用保留 context 至完成及全部 drain。
    call->stream->Finish(&call->result, tag([this, call](bool ok) {
      call->finish_pending = false;
      call->finished = true;
      if (call->exchange && !call->exchange->response_complete && call->exchange->error.empty()) {
        call->exchange->error = ok ? rpc_failure(call->result) : "gRPC control Finish completion failed";
      }
      if (control == call) {
        control.reset();
        current.control_rpc_live = false;
      }
      wake.notify_all();
    }));
  }

  void cancel_control_locked(const std::shared_ptr<ControlCall> &call, const std::string &reason) {
    if (!call->closing) {
      call->closing = true;
      if (control == call) invalidate_control_locked(reason);
    }
    if (call->exchange && !call->exchange->response_complete && call->exchange->error.empty()) {
      call->exchange->error = reason;
    }
    call->half_closed = false;
    call->context.TryCancel();
    finish_control_locked(call);
    wake.notify_all();
  }

  void read_control_locked(const std::shared_ptr<ControlCall> &call) {
    if (!call->started || call->read_pending || (call->closing && !call->half_closed) ||
        call->finish_pending) return;
    call->incoming.Clear();
    call->read_pending = true;
    call->stream->Read(&call->incoming, tag([this, call](bool ok) {
      call->read_pending = false;
      if (!ok) {
        cancel_control_locked(call, "gRPC control Read EOF/termination; old targets discarded");
        return;
      }
      if (!call->closing && control == call && current.running) {
        try {
          validate_session_reply(*call, call->incoming);
          if (call->incoming.has_session()) {
            call->incoming.mutable_session()->set_submission_ready(reply_preserves_readiness(
                call->incoming.session().submission_ready(),
                call->exchange ? call->exchange->readiness_revision : 0, readiness_revision));
          }
          current.capabilities = call->incoming.capabilities();
          if (call->incoming.has_session()) {
            current.session = call->incoming.session();
            current.control_active = true;
          }
          if (call->exchange) {
            call->exchange->response = call->incoming;
            call->exchange->response_complete = true;
            if (call->exchange->request.has_close()) {
              cancel_control_locked(call, "gRPC explicit control Close");
            }
          }
          if (!call->incoming.has_session()) {
            cancel_control_locked(call, "gRPC holder rejected or revoked");
          }
        } catch (const std::exception &failure) {
          if (call->exchange) call->exchange->error = failure.what();
          cancel_control_locked(call, failure.what());
        }
      }
      if (call->exchange && call->exchange->write_complete && call->exchange->response_complete) {
        call->exchange.reset();
      }
      read_control_locked(call);
      finish_control_locked(call);
      wake.notify_all();
    }));
  }

  void writes_done_locked(const std::shared_ptr<ControlCall> &call) {
    if (!call->half_closed || !call->started || call->write_pending || call->writes_done_sent ||
        call->start_pending || call->finish_pending) return;
    call->writes_done_sent = true;
    call->writes_done_pending = true;
    call->stream->WritesDone(tag([this, call](bool ok) {
      call->writes_done_pending = false;
      if (!ok) cancel_control_locked(call, "gRPC WritesDone completion failed");
      finish_control_locked(call);
      wake.notify_all();
    }));
  }

  void write_control_locked(const std::shared_ptr<ControlCall> &call) {
    if (!call->started || call->closing || call->write_pending || !call->exchange) return;
    const auto exchange = call->exchange;
    if (exchange->write_complete) return;
    call->write_pending = true;
    call->stream->Write(exchange->request, tag([this, call, exchange](bool ok) {
      call->write_pending = false;
      exchange->write_complete = true;
      if (!ok) {
        exchange->error = "gRPC control Write completion failed";
        cancel_control_locked(call, exchange->error);
      }
      if (exchange->response_complete && call->exchange == exchange) call->exchange.reset();
      writes_done_locked(call);
      finish_control_locked(call);
      wake.notify_all();
    }));
  }

  std::shared_ptr<ControlCall> new_control_locked(const wire::OpenSessionRequest &request) {
    if (control) throw std::runtime_error("gRPC previous control RPC is still draining");
    if (current.control_generation == std::numeric_limits<std::uint64_t>::max()) {
      throw std::runtime_error("gRPC local control generation exhausted");
    }
    auto call = std::make_shared<ControlCall>();
    call->holder_nonce = request.holder_nonce();
    call->open_request_id = request.open_request_id();
    call->generation = ++current.control_generation;
    call->context.set_deadline(rpc_deadline(Milliseconds(config.max_control_stream_lifetime_ms)));
    call->context.set_wait_for_ready(false);
    call->stream = stub->PrepareAsyncOpenSession(&call->context, &cq);
    if (!call->stream) throw std::runtime_error("gRPC PrepareAsyncOpenSession failed");
    control = call;
    current.control_rpc_live = true;
    call->start_pending = true;
    call->stream->StartCall(tag([this, call](bool ok) {
      call->start_pending = false;
      call->started = true;
      if (!ok || !current.running || call->closing) {
        cancel_control_locked(call, "gRPC control Start completion failed or stopped");
      } else {
        read_control_locked(call);
        write_control_locked(call);
      }
      writes_done_locked(call);
      finish_control_locked(call);
      wake.notify_all();
    }));
    return call;
  }

  wire::OpenSessionResponse control_exchange(const wire::ControlSessionRequest &request,
                                            Milliseconds timeout) {
    bounded_timeout(timeout, config.max_control_stream_lifetime_ms);
    bounded_message(request, config.max_message_bytes);
    std::unique_lock<std::mutex> lock(mutex);
    require_running_locked();
    auto call = control;
    if (!call) {
      if (!request.has_open()) throw std::runtime_error("gRPC no live control RPC");
      call = new_control_locked(request.open());
    }
    std::string nonce;
    std::string key;
    if (request.has_open()) { nonce = request.open().holder_nonce(); key = request.open().open_request_id(); }
    if (request.has_keep()) { nonce = request.keep().holder_nonce(); key = request.keep().open_request_id(); }
    if (request.has_close()) { nonce = request.close().holder_nonce(); key = request.close().open_request_id(); }
    if (nonce != call->holder_nonce || key != call->open_request_id) {
      throw std::invalid_argument("gRPC live RPC requires unchanged holder nonce/open_request_id");
    }
    if (call->closing || call->exchange) {
      throw std::runtime_error("gRPC control request slot busy or RPC closing");
    }
    const auto exchange = std::make_shared<ControlExchange>();
    exchange->readiness_revision = readiness_revision;
    exchange->request = request;
    call->exchange = exchange;
    write_control_locked(call);
    if (!wake.wait_for(lock, timeout, [&] {
          return !current.running || !exchange->error.empty() ||
              (exchange->response_complete && exchange->write_complete);
        })) {
      // timeout 不重试/旋转 RPC；未完成请求继续占容量1，迟到回复按同 RPC 顺序消费。
      throw std::runtime_error("gRPC control response timeout; original holder keys retained");
    }
    if (!exchange->error.empty()) throw std::runtime_error(exchange->error);
    if (!current.running) throw std::runtime_error("gRPC client stopped while awaiting control reply");
    auto response = exchange->response;
    if (response.has_session() &&
        (control != call || call->closing || exchange->readiness_revision != readiness_revision)) {
      response.mutable_session()->set_submission_ready(false);
    }
    return response;
  }

  void finish_readonly_locked(const std::shared_ptr<ReadonlyCall> &call) {
    if (!call->closing || call->start_pending || call->read_pending || call->finish_pending) return;
    call->finish_pending = true;
    call->stream->Finish(&call->result, tag([this, call](bool ok) {
      call->finish_pending = false;
      if (readonly == call) {
        readonly.reset();
        current.readonly_subscribed = false;
        if (current.running && !call->result.ok()) current.failure = rpc_failure(call->result);
        if (!ok) current.failure = "gRPC readonly Finish completion failed";
      }
      wake.notify_all();
    }));
  }

  void cancel_readonly_locked(const std::shared_ptr<ReadonlyCall> &call) {
    call->closing = true;
    current.readonly_subscribed = false;
    call->context.TryCancel();
    finish_readonly_locked(call);
  }

  void read_status_locked(const std::shared_ptr<ReadonlyCall> &call) {
    if (!call->started || call->closing || call->read_pending || call->finish_pending) return;
    call->incoming.Clear();
    call->read_pending = true;
    call->stream->Read(&call->incoming, tag([this, call](bool ok) {
      call->read_pending = false;
      if (!ok) {
        call->closing = true;
        current.readonly_subscribed = false;
      } else if (!call->closing && current.running && readonly == call) {
        try {
          merge_envelope_locked(call->incoming);
        } catch (const std::exception &failure) {
          current.failure = failure.what();
          cancel_readonly_locked(call);
        }
      }
      read_status_locked(call);
      finish_readonly_locked(call);
      wake.notify_all();
    }));
  }

  void start_readonly_locked() {
    if (readonly) return;
    auto call = std::make_shared<ReadonlyCall>();
    call->request.set_schema_version(1);
    call->context.set_deadline(rpc_deadline(Milliseconds(config.max_readonly_stream_lifetime_ms)));
    call->context.set_wait_for_ready(false);
    call->stream = stub->PrepareAsyncSubscribeStatus(&call->context, call->request, &cq);
    if (!call->stream) throw std::runtime_error("gRPC PrepareAsyncSubscribeStatus failed");
    readonly = call;
    call->start_pending = true;
    call->stream->StartCall(tag([this, call](bool ok) {
      call->start_pending = false;
      call->started = true;
      if (!ok || !current.running || call->closing) cancel_readonly_locked(call);
      else {
        current.readonly_subscribed = true;
        read_status_locked(call);
      }
      finish_readonly_locked(call);
      wake.notify_all();
    }));
  }

  template <typename Request, typename Response, typename Start>
  Response unary(const Request &request, Milliseconds timeout, bool submission, Start start_call) {
    bounded_timeout(timeout, config.max_unary_deadline_ms);
    bounded_message(request, config.max_message_bytes);
    std::unique_lock<std::mutex> lock(mutex);
    require_running_locked();
    std::size_t matching = 0;
    for (const auto &entry : unary_calls) if (entry.second->submission == submission) ++matching;
    if (matching >= (submission ? 1U : config.readonly_inflight_limit)) {
      throw std::runtime_error("gRPC unary capacity exhausted; no unbounded retry queue");
    }
    if (next_unary_id == std::numeric_limits<std::uint64_t>::max()) {
      throw std::runtime_error("gRPC unary identity exhausted");
    }
    auto call = std::make_shared<UnaryCall<Request, Response>>();
    call->request = request;
    call->submission = submission;
    call->generation = current.control_generation;
    if constexpr (std::is_same_v<Request, wire::SubmitRequest>) {
      if (!control || control->closing || !current.control_active ||
          !current.session.submission_ready() ||
          request.target().key().session_token() != current.session.session_token()) {
        throw std::runtime_error("gRPC target belongs to an old/unready control session; not resubmitted");
      }
    }
    call->context.set_deadline(rpc_deadline(timeout));
    call->context.set_wait_for_ready(false);
    call->reader = start_call(&call->context, call->request, &cq);
    if (!call->reader) throw std::runtime_error("gRPC unary preparation failed");
    const auto identifier = ++next_unary_id;
    unary_calls.emplace(identifier, call);
    call->reader->Finish(&call->response, &call->result, tag([this, call, identifier](bool ok) {
      call->done = true;
      if (!ok) call->error = "gRPC unary Finish completion failed";
      else if (!call->result.ok()) call->error = rpc_failure(call->result);
      else if (call->response.ByteSizeLong() > config.max_message_bytes) {
        call->error = "gRPC unary response exceeds max_message_bytes";
      }
      unary_calls.erase(identifier);
      wake.notify_all();
    }));
    if (!wake.wait_for(lock, timeout, [&] { return call->done || !current.running; })) {
      call->context.TryCancel();
      throw std::runtime_error("gRPC unary deadline/response timeout; already-published command cannot be recalled");
    }
    if (!current.running) {
      call->context.TryCancel();
      throw std::runtime_error("gRPC client stopped while awaiting unary reply");
    }
    if (!call->error.empty()) throw std::runtime_error(call->error);
    if (submission && (call->generation != current.control_generation || !control || control->closing)) {
      throw std::runtime_error("gRPC submit completion superseded by control termination; acceptance unknown");
    }
    return call->response;
  }

  void poll() noexcept {
    void *raw = nullptr;
    bool ok = false;
    while (cq.Next(&raw, &ok)) {
      std::unique_ptr<Tag> completed(static_cast<Tag *>(raw));
      std::lock_guard<std::mutex> lock(mutex);
      --outstanding_tags;
      try {
        completed->complete(ok);
      } catch (const std::exception &failure) {
        current.failure = failure.what();
        current.running = false;
        if (control) cancel_control_locked(control, "gRPC CQ processing failure");
        if (readonly) cancel_readonly_locked(readonly);
        for (const auto &entry : unary_calls) entry.second->context.TryCancel();
      }
      wake.notify_all();
    }
  }
};

GrpcClient::GrpcClient(Config config) : impl_(std::make_unique<Impl>(std::move(config))) {
  validate_config(impl_->config);
}

GrpcClient::~GrpcClient() { stop(); }

void GrpcClient::start() {
  std::lock_guard<std::mutex> lock(impl_->mutex);
  if (impl_->started) throw std::runtime_error("gRPC client cannot be started twice");
  grpc::ChannelArguments arguments;
  arguments.SetMaxReceiveMessageSize(static_cast<int>(impl_->config.max_message_bytes));
  arguments.SetMaxSendMessageSize(static_cast<int>(impl_->config.max_message_bytes));
  arguments.SetInt("grpc.enable_retries", 0);
  arguments.SetInt("grpc.max_metadata_size", static_cast<int>(impl_->config.max_message_bytes));
  arguments.SetInt("grpc.initial_reconnect_backoff_ms", static_cast<int>(impl_->config.reconnect_initial_ms));
  arguments.SetInt("grpc.min_reconnect_backoff_ms", static_cast<int>(impl_->config.reconnect_initial_ms));
  arguments.SetInt("grpc.max_reconnect_backoff_ms", static_cast<int>(impl_->config.reconnect_max_ms));
  impl_->channel = grpc::CreateCustomChannel(impl_->config.listen_address,
      grpc::InsecureChannelCredentials(), arguments);
  impl_->stub = wire::PlanningIngress::NewStub(impl_->channel);
  impl_->current.running = true;
  impl_->started = true;
  try {
    impl_->poller = std::thread([this] { impl_->poll(); });
  } catch (...) {
    impl_->current.running = false;
    impl_->cq.Shutdown();
    impl_->queue_shutdown = true;
    throw;
  }
}

bool GrpcClient::wait_available(Milliseconds timeout) {
  bounded_timeout(timeout, impl_->config.max_unary_deadline_ms);
  std::shared_ptr<grpc::Channel> channel;
  {
    std::lock_guard<std::mutex> lock(impl_->mutex);
    impl_->require_running_locked();
    channel = impl_->channel;
  }
  const bool connected = channel->WaitForConnected(std::chrono::system_clock::now() + timeout);
  std::lock_guard<std::mutex> lock(impl_->mutex);
  impl_->current.service_available = connected && impl_->current.running;
  return impl_->current.service_available;
}

wire::OpenSessionResponse GrpcClient::open(const wire::OpenSessionRequest &request, Milliseconds timeout) {
  valid_open(request);
  wire::ControlSessionRequest operation;
  *operation.mutable_open() = request;
  return impl_->control_exchange(operation, timeout);
}

wire::OpenSessionResponse GrpcClient::keep(const wire::KeepSessionRequest &request, Milliseconds timeout) {
  if (request.schema_version_presence_case() != wire::KeepSessionRequest::kSchemaVersion || request.schema_version() != 1 ||
      request.holder_nonce().size() != 16 || request.open_request_id().size() != 16 ||
      request.keepalive_sequence_presence_case() != wire::KeepSessionRequest::kKeepaliveSequence) {
    throw std::invalid_argument("gRPC keep requires V1/key/independent keepalive sequence presence");
  }
  wire::ControlSessionRequest operation;
  *operation.mutable_keep() = request;
  return impl_->control_exchange(operation, timeout);
}

wire::OpenSessionResponse GrpcClient::close(const wire::CloseSessionRequest &request, Milliseconds timeout) {
  if (request.schema_version_presence_case() != wire::CloseSessionRequest::kSchemaVersion || request.schema_version() != 1 ||
      request.holder_nonce().size() != 16 || request.open_request_id().size() != 16) {
    throw std::invalid_argument("gRPC close requires V1 presence and exact holder keys");
  }
  wire::ControlSessionRequest operation;
  *operation.mutable_close() = request;
  try {
    return impl_->control_exchange(operation, timeout);
  } catch (const std::invalid_argument &) {
    // 错键/非法边界请求不能借 SDK 的取消路径撤销合法 holder。
    throw;
  } catch (...) {
    cancel_control();
    throw;
  }
}

wire::SubmitReceipt GrpcClient::submit(const control_link_edge_protocol::CommandFrame &target,
                                     Milliseconds timeout) {
  wire::SubmitRequest request;
  auto *payload = request.mutable_target();
  payload->set_schema_version(1);
  payload->mutable_key()->set_session_token(token_bytes(target.session_token));
  payload->mutable_key()->set_source_sequence(target.command_sequence);
  payload->set_source_stamp_ns(target.source_stamp_ns);
  if (target.clock_domain != control_link_edge_protocol::ClockDomain::kSystem) {
    throw std::invalid_argument("gRPC targets require the explicit SYSTEM timestamp domain");
  }
  payload->set_clock_domain(wire::CLOCK_DOMAIN_SYSTEM);
  payload->set_mode(static_cast<wire::CommandMode>(target.mode));
  payload->set_linear_velocity_mps(target.linear_velocity_mps);
  payload->set_angular_velocity_radps(target.angular_velocity_radps);
  // presence 与数值原样映射；finite/mode/重复 sequence/age 仍由唯一 Gateway 判断。
  auto response = impl_->unary<wire::SubmitRequest, wire::SubmitReceipt>(request, timeout, true,
      [this](grpc::ClientContext *context, const wire::SubmitRequest &input, grpc::CompletionQueue *queue) {
        return impl_->stub->AsyncSubmit(context, input, queue);
      });
  if (response.schema_version_presence_case() != wire::SubmitReceipt::kSchemaVersion || response.schema_version() != 1 || !response.has_key() ||
      response.key().session_token() != payload->key().session_token() ||
      response.key().source_sequence_presence_case() != wire::CommandKey::kSourceSequence ||
      response.key().source_sequence() != target.command_sequence) {
    throw std::runtime_error("gRPC SubmitReceipt schema/target-key mismatch; acceptance unknown");
  }
  return response;
}

wire::StatusEnvelope GrpcClient::get_status(Milliseconds timeout) {
  wire::GetStatusRequest request;
  request.set_schema_version(1);
  auto response = impl_->unary<wire::GetStatusRequest, wire::StatusEnvelope>(request, timeout, false,
      [this](grpc::ClientContext *context, const wire::GetStatusRequest &input, grpc::CompletionQueue *queue) {
        return impl_->stub->AsyncGetStatus(context, input, queue);
      });
  std::lock_guard<std::mutex> lock(impl_->mutex);
  impl_->merge_envelope_locked(response);
  return response;
}

wire::Capabilities GrpcClient::get_capabilities(Milliseconds timeout) {
  wire::GetCapabilitiesRequest request;
  request.set_schema_version(1);
  auto response = impl_->unary<wire::GetCapabilitiesRequest, wire::Capabilities>(request, timeout, false,
      [this](grpc::ClientContext *context, const wire::GetCapabilitiesRequest &input, grpc::CompletionQueue *queue) {
        return impl_->stub->AsyncGetCapabilities(context, input, queue);
      });
  std::lock_guard<std::mutex> lock(impl_->mutex);
  impl_->validate_capabilities(response);
  impl_->current.capabilities = response;
  return response;
}

void GrpcClient::set_status_subscription(bool enabled) {
  std::lock_guard<std::mutex> lock(impl_->mutex);
  impl_->require_running_locked();
  if (enabled) impl_->start_readonly_locked();
  else if (impl_->readonly) impl_->cancel_readonly_locked(impl_->readonly);
}

ClientStatus GrpcClient::status() const {
  std::lock_guard<std::mutex> lock(impl_->mutex);
  auto result = impl_->current;
  if (impl_->channel) result.service_available = result.running &&
      impl_->channel->GetState(false) == GRPC_CHANNEL_READY;
  return result;
}

void GrpcClient::half_close() {
  std::lock_guard<std::mutex> lock(impl_->mutex);
  impl_->require_running_locked();
  const auto call = impl_->control;
  if (!call || call->closing) return;
  call->half_closed = true;
  call->closing = true;
  impl_->invalidate_control_locked("gRPC client WritesDone relinquishes control");
  if (call->exchange && !call->exchange->response_complete) {
    call->exchange->error = "gRPC client half-closed control";
  }
  impl_->writes_done_locked(call);
}

void GrpcClient::cancel_control() noexcept {
  std::lock_guard<std::mutex> lock(impl_->mutex);
  if (impl_->control) impl_->cancel_control_locked(impl_->control, "gRPC client cancellation relinquishes control");
}

void GrpcClient::stop() noexcept {
  std::unique_lock<std::mutex> lock(impl_->mutex);
  if (!impl_->started || impl_->queue_shutdown) return;
  impl_->stopping = true;
  impl_->current.running = false;
  impl_->current.service_available = false;
  impl_->invalidate_control_locked("gRPC client stop");
  if (impl_->control) impl_->cancel_control_locked(impl_->control, "gRPC client stop");
  if (impl_->readonly) impl_->cancel_readonly_locked(impl_->readonly);
  for (const auto &entry : impl_->unary_calls) entry.second->context.TryCancel();
  impl_->wake.notify_all();
  const auto drained = [this] {
    return impl_->outstanding_tags == 0 && !impl_->control && !impl_->readonly && impl_->unary_calls.empty();
  };
  if (!impl_->wake.wait_for(lock, Milliseconds(impl_->config.shutdown_timeout_ms), drained)) {
    // 不能以丢 context 或在已 Shutdown CQ 后补 Finish 伪造有界关闭；超预算保持可观测并安全 drain。
    impl_->current.failure = "gRPC cancellation drain exceeded shutdown_timeout_ms";
    impl_->wake.wait(lock, drained);
  }
  impl_->queue_shutdown = true;
  impl_->cq.Shutdown();
  lock.unlock();
  if (impl_->poller.joinable()) impl_->poller.join();
  lock.lock();
  impl_->stub.reset();
  impl_->channel.reset();
}

namespace {

// 示例在创建网络线程前屏蔽退出信号，由 owner 轮询，不引入全局可变 handler 或 detached 线程。
class ExampleStopSignals final {
 public:
  ExampleStopSignals() {
    sigemptyset(&signals_);
    sigaddset(&signals_, SIGINT);
    sigaddset(&signals_, SIGTERM);
    if (pthread_sigmask(SIG_BLOCK, &signals_, &previous_) != 0) {
      throw std::runtime_error("Cannot establish example stop-signal owner");
    }
  }
  ~ExampleStopSignals() { pthread_sigmask(SIG_SETMASK, &previous_, nullptr); }
  bool requested() const {
    const timespec immediate{0, 0};
    const int signal = sigtimedwait(&signals_, nullptr, &immediate);
    if (signal == SIGINT || signal == SIGTERM) return true;
    if (signal < 0 && errno != EAGAIN && errno != EINTR) {
      throw std::runtime_error("Example signal observation failed");
    }
    return false;
  }
 private:
  sigset_t signals_{};
  sigset_t previous_{};
};

struct ExampleOptions {
  std::string path;
  std::string address;
  std::uint32_t period_ms{50};
  std::uint32_t keep_period_ms{1000};
  std::uint32_t run_seconds{10};
  std::uint32_t pause_after_ms{0};
  std::uint32_t pause_duration_ms{0};
  std::uint32_t half_close_after_ms{0};
  std::uint32_t cancel_after_ms{0};
  double velocity{0.3};
  double yaw{0.0};
  bool hold{false};
  bool readonly_only{false};
  bool no_keep{false};
  bool no_command{false};
  bool no_close{false};
};

std::uint32_t example_integer(const std::string &value) {
  if (value.empty() || value.find_first_not_of("0123456789") != std::string::npos) {
    throw std::invalid_argument("Example arguments require unsigned decimal values");
  }
  std::size_t consumed = 0;
  const auto result = std::stoull(value, &consumed);
  if (consumed != value.size() || result > std::numeric_limits<std::uint32_t>::max()) {
    throw std::invalid_argument("Example argument exceeds uint32");
  }
  return static_cast<std::uint32_t>(result);
}

double example_number(const std::string &value) {
  std::size_t consumed = 0;
  const auto result = std::stod(value, &consumed);
  if (consumed != value.size() || !std::isfinite(result)) {
    throw std::invalid_argument("Example velocity/yaw must be finite");
  }
  return result;
}

ExampleOptions example_options(int argc, char **argv) {
  ExampleOptions result;
  for (int index = 1; index < argc; ++index) {
    const std::string option = argv[index];
    if (option == "--readonly-only") { result.readonly_only = true; continue; }
    if (option == "--no-keep") { result.no_keep = true; continue; }
    if (option == "--no-command") { result.no_command = true; continue; }
    if (option == "--no-close") { result.no_close = true; continue; }
    if (++index == argc) throw std::invalid_argument("Missing example value for " + option);
    const std::string value = argv[index];
    if (option == "--config") result.path = value;
    else if (option == "--address") result.address = value;
    else if (option == "--period-ms") result.period_ms = example_integer(value);
    else if (option == "--keep-period-ms") result.keep_period_ms = example_integer(value);
    else if (option == "--run-seconds") result.run_seconds = example_integer(value);
    else if (option == "--velocity") result.velocity = example_number(value);
    else if (option == "--yaw") result.yaw = example_number(value);
    else if (option == "--pause-command-after-ms") result.pause_after_ms = example_integer(value);
    else if (option == "--pause-command-duration-ms") result.pause_duration_ms = example_integer(value);
    else if (option == "--half-close-after-ms") result.half_close_after_ms = example_integer(value);
    else if (option == "--cancel-after-ms") result.cancel_after_ms = example_integer(value);
    else if (option == "--mode") {
      if (value != "normal" && value != "hold") throw std::invalid_argument("--mode requires normal or hold");
      result.hold = value == "hold";
    } else throw std::invalid_argument("Unknown example option " + option);
  }
  if (result.path.empty() || !result.period_ms || !result.keep_period_ms ||
      !result.run_seconds || result.run_seconds > 3600 ||
      (result.pause_after_ms != 0 && result.pause_duration_ms == 0)) {
    throw std::invalid_argument("Explicit --config and positive periods/run-seconds [1,3600] required");
  }
  return result;
}

void print_observations(const ClientStatus &state) {
  const auto &snapshot = state.observations;
  const auto &gateway = snapshot.gateway();
  const auto &vehicle = snapshot.vehicle();
  std::cout << "observations wall_clock_ns=" << network::system_now_ns()
      << " received_steady_ns=" << state.status_received_steady_ns
      << " control_generation=" << state.control_generation
      << " ready=" << state.session.submission_ready()
      << " gateway_state=" << gateway.state()
      << " gateway_validity=" << gateway.observation().validity()
      << " gateway_observed_system_ns=" << gateway.observation().observed_system_ns()
      << " active_source=" << gateway.active_source_id()
      << " active_sequence=" << gateway.active_source_sequence()
      << " vehicle_state=" << vehicle.state()
      << " vehicle_validity=" << vehicle.observation().validity()
      << " vehicle_observed_system_ns=" << vehicle.observation().observed_system_ns()
      << " vehicle_velocity=" << vehicle.linear_velocity_mps();
  for (const auto &source : snapshot.sources()) {
    if (source.source_id() == "planning") {
      std::cout << " planning_accepted=" << source.accepted_count()
          << " planning_rejected=" << source.rejected_count()
          << " planning_sequence=" << source.last_source_sequence()
          << " planning_validity=" << source.observation().validity()
          << " planning_observed_system_ns=" << source.observation().observed_system_ns()
          << " publisher_count=" << source.endpoint_publisher_count();
    }
  }
  std::cout << " latest_acceptance_observation=" << snapshot.latest_gateway_acceptance().state()
      << " status_overwrites=" << state.readonly_overwritten_count << std::endl;
}

}

int run_planning_example(int argc, char **argv) {
  if (argc == 2 && std::string(argv[1]) == "--help") {
    std::cout << "--config YAML [--address 127.0.0.1:50051] [--period-ms 50 --keep-period-ms 1000 "
        "--run-seconds 10 --velocity 0.3 --yaw 0 --mode normal|hold] "
        "[--readonly-only --no-command --no-keep --no-close] "
        "[--pause-command-after-ms N --pause-command-duration-ms N] "
        "[--half-close-after-ms N --cancel-after-ms N]" << std::endl;
    return 0;
  }
  try {
    ExampleStopSignals signals;
    const auto options = example_options(argc, argv);
    auto config = load_config(options.path);
    if (!options.address.empty()) config.listen_address = options.address;
    validate_config(config);
    GrpcClient client(config);
    client.start();
    std::cout << "client pid=" << ::getpid() << " exe="
        << std::filesystem::read_symlink("/proc/self/exe").string()
        << " address=" << config.listen_address << " period_ms=" << options.period_ms
        << " keep_period_ms=" << options.keep_period_ms
        << " source=planning frame=body clock=SYSTEM units=m/s,rad/s" << std::endl;
    auto nonce = network::random_identity();
    auto open_key = network::random_identity();
    std::string token;
    std::uint64_t sequence = 0;
    std::uint64_t keep_sequence = 0;
    bool used_rpc = false;
    bool held = false;
    bool ready = false;
    bool pause_reported = false;
    bool control_fault_injected = false;
    std::uint32_t open_backoff = config.reconnect_initial_ms;
    const auto started = std::chrono::steady_clock::now();
    const auto finish = started + std::chrono::seconds(options.run_seconds);
    auto first_ready = std::chrono::steady_clock::time_point{};
    auto next_open = started;
    auto next_keep = started;
    auto next_command = started;
    auto next_observation = started;
    auto next_subscription = started;
    auto next_failure_log = started;
    const auto command_wait = Milliseconds(std::min<std::uint32_t>(20, config.max_unary_deadline_ms));
    const auto observe_reply = [&](const wire::OpenSessionResponse &response) {
      held = response.has_session();
      ready = held && response.session().submission_ready();
      if (held && token != response.session().session_token()) {
        // 只有真实新 token 重置业务 generation；同 RPC 重试/拒绝不能重置 sequence。
        token = response.session().session_token();
        sequence = 0;
        next_command = std::chrono::steady_clock::now();
      }
      if (ready && first_ready == std::chrono::steady_clock::time_point{}) {
        first_ready = std::chrono::steady_clock::now();
      }
      std::cout << "session result=" << response.result() << " ready=" << ready
          << " control_generation=" << client.status().control_generation
          << " wall_clock_ns=" << network::system_now_ns() << std::endl;
    };
    while (std::chrono::steady_clock::now() < finish && !signals.requested()) {
      auto now = std::chrono::steady_clock::now();
      auto state = client.status();
      if (!state.running) break;
      if (used_rpc && !state.control_rpc_live) {
        nonce = network::random_identity();
        open_key = network::random_identity();
        token.clear();
        sequence = 0;
        keep_sequence = 0;
        used_rpc = false;
        held = false;
        ready = false;
        first_ready = std::chrono::steady_clock::time_point{};
        next_open = now + Milliseconds(open_backoff);
      }
      if (!state.service_available) {
        client.wait_available(Milliseconds(std::min<std::uint32_t>(50, config.max_unary_deadline_ms)));
        std::this_thread::sleep_for(Milliseconds(5));
        continue;
      }
      if (held && !state.session.submission_ready()) ready = false;
      try {
        if (!state.readonly_subscribed && now >= next_subscription) {
          client.set_status_subscription(true);
          next_subscription = now + Milliseconds(config.reconnect_initial_ms);
        }
        if (!options.readonly_only) {
          if (held && !options.no_keep && now >= next_keep) {
            wire::KeepSessionRequest request;
            request.set_schema_version(1);
            request.set_holder_nonce(nonce);
            request.set_open_request_id(open_key);
            if (keep_sequence == std::numeric_limits<std::uint64_t>::max()) {
              throw std::runtime_error("Keep sequence exhausted; explicit new RPC required");
            }
            request.set_keepalive_sequence(++keep_sequence);
            next_keep = now + Milliseconds(options.keep_period_ms);
            observe_reply(client.keep(request, command_wait));
          }
          now = std::chrono::steady_clock::now();
          if (!ready && now >= next_open) {
            wire::OpenSessionRequest request;
            request.set_schema_version(1);
            request.set_holder_nonce(nonce);
            request.set_open_request_id(open_key);
            next_open = now + Milliseconds(open_backoff);
            open_backoff = static_cast<std::uint32_t>(std::min<std::uint64_t>(
                config.reconnect_max_ms, static_cast<std::uint64_t>(open_backoff) * 2));
            used_rpc = true;
            const auto response = client.open(request, command_wait);
            if (response.has_session()) open_backoff = config.reconnect_initial_ms;
            observe_reply(response);
          }
          now = std::chrono::steady_clock::now();
          const auto since_ready = first_ready == std::chrono::steady_clock::time_point{} ?
              Milliseconds::zero() : std::chrono::duration_cast<Milliseconds>(now - first_ready);
          if (!control_fault_injected && ready && options.half_close_after_ms &&
              since_ready >= Milliseconds(options.half_close_after_ms)) {
            client.half_close();
            control_fault_injected = true;
            ready = false;
            held = false;
            std::cout << "half_close wall_clock_ns=" << network::system_now_ns() << std::endl;
          } else if (!control_fault_injected && ready && options.cancel_after_ms &&
                     since_ready >= Milliseconds(options.cancel_after_ms)) {
            client.cancel_control();
            control_fault_injected = true;
            ready = false;
            held = false;
            std::cout << "cancel_control wall_clock_ns=" << network::system_now_ns() << std::endl;
          }
          const bool pause = options.pause_after_ms && first_ready != std::chrono::steady_clock::time_point{} &&
              since_ready >= Milliseconds(options.pause_after_ms) &&
              since_ready < Milliseconds(static_cast<std::uint64_t>(options.pause_after_ms) + options.pause_duration_ms);
          if (pause != pause_reported) {
            pause_reported = pause;
            std::cout << "command_pause=" << pause << " wall_clock_ns=" << network::system_now_ns()
                << " business_sequence=" << sequence << std::endl;
          }
          if (ready && !options.no_command && !pause && now >= next_command) {
            if (sequence == std::numeric_limits<std::uint64_t>::max()) {
              throw std::runtime_error("Business sequence exhausted; explicit new RPC required");
            }
            control_link_edge_protocol::CommandFrame target;
            std::memcpy(target.session_token.bytes.data(), token.data(), token.size());
            target.command_sequence = ++sequence;
            target.source_stamp_ns = network::system_now_ns();
            target.mode = options.hold ? control_link_edge_protocol::CommandMode::kHold :
                control_link_edge_protocol::CommandMode::kNormal;
            target.linear_velocity_mps = options.hold ? 0.0 : options.velocity;
            target.angular_velocity_radps = options.hold ? 0.0 : options.yaw;
            next_command += Milliseconds(options.period_ms);
            if (next_command <= now) next_command = now + Milliseconds(options.period_ms);
            const auto receipt = client.submit(target, command_wait);
            std::cout << "submit sequence=" << target.command_sequence
                << " source_stamp_ns=" << target.source_stamp_ns
                << " rpc_received=" << receipt.rpc_received()
                << " forwarding_state=" << receipt.forwarding_state()
                << " gateway_acceptance=" << receipt.gateway_acceptance().state() << std::endl;
          }
        }
      } catch (const std::exception &failure) {
        now = std::chrono::steady_clock::now();
        if (now >= next_failure_log) {
          std::cerr << "boundary failure=" << failure.what() << " wall_clock_ns=" << network::system_now_ns() << std::endl;
          next_failure_log = now + Milliseconds(1000);
        }
        // 拒绝/timeout 的目标已消费其业务 sequence，绝不重新换 token/盖章重发。
      }
      now = std::chrono::steady_clock::now();
      if (now >= next_observation) {
        print_observations(client.status());
        next_observation = now + Milliseconds(config.status_event_period_ms);
      }
      std::this_thread::sleep_for(Milliseconds(2));
    }
    if (!options.no_close && held && client.status().control_active) {
      wire::CloseSessionRequest request;
      request.set_schema_version(1);
      request.set_holder_nonce(nonce);
      request.set_open_request_id(open_key);
      try { client.close(request, Milliseconds(std::min<std::uint32_t>(100, config.max_unary_deadline_ms))); }
      catch (const std::exception &failure) { std::cerr << "close=" << failure.what() << std::endl; }
    }
    client.stop();
    return 0;
  } catch (const std::exception &failure) {
    std::cerr << "grpc_planning_client: " << failure.what() << std::endl;
    return 1;
  }
}

}
