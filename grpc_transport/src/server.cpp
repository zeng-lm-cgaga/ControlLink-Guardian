#include "control_link_grpc_transport/server.hpp"
#include "control_link_network_v1.grpc.pb.h"

#include <algorithm>
#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstring>
#include <iostream>
#include <limits>
#include <map>
#include <mutex>
#include <optional>
#include <stdexcept>
#include <thread>
#include <vector>

#include <grpcpp/grpcpp.h>

namespace control_link::grpc_transport {
namespace wire = ::control_link::network::v1;
namespace codec = ::control_link_edge_protocol;
namespace {

enum class Kind { control, submit, status, capabilities, subscription };
enum class Operation { request, read, write, finish, done };

bool decode_target(const wire::SubmitRequest &request, codec::CommandFrame &frame) {
  if (!request.has_target()) return false;
  const auto &target = request.target();
  if (target.schema_version_presence_case() != wire::CommandTarget::kSchemaVersion || target.schema_version() != 1 ||
      !target.has_key() || target.key().session_token().size() != codec::kSessionTokenSize ||
      target.key().source_sequence_presence_case() != wire::CommandKey::kSourceSequence ||
      target.source_stamp_presence_case() != wire::CommandTarget::kSourceStampNs ||
      target.clock_domain_presence_case() != wire::CommandTarget::kClockDomain ||
      target.clock_domain() != wire::CLOCK_DOMAIN_SYSTEM ||
      target.mode_presence_case() != wire::CommandTarget::kMode ||
      static_cast<int>(target.mode()) < 0 || static_cast<int>(target.mode()) > 255 ||
      target.linear_velocity_presence_case() != wire::CommandTarget::kLinearVelocityMps ||
      target.angular_velocity_presence_case() != wire::CommandTarget::kAngularVelocityRadps) return false;
  std::memcpy(frame.session_token.bytes.data(), target.key().session_token().data(), codec::kSessionTokenSize);
  frame.command_sequence = target.key().source_sequence();
  frame.source_stamp_ns = target.source_stamp_ns();
  frame.clock_domain = codec::ClockDomain::kSystem;
  frame.mode = static_cast<codec::CommandMode>(target.mode());
  frame.linear_velocity_mps = target.linear_velocity_mps();
  frame.angular_velocity_radps = target.angular_velocity_radps();
  return true;
}

}

struct GrpcServer::Impl {
  struct Call;
  struct Tag { Call *call; Operation operation; };

  // 同一call拥有context、全部缓冲和固定tag；Read/Write各至多一个，Finish不与二者并发。
  struct Call : std::enable_shared_from_this<Call> {
    explicit Call(Kind type, std::uint64_t identity) : kind(type), id(identity),
        request_tag{this, Operation::request}, read_tag{this, Operation::read},
        write_tag{this, Operation::write}, finish_tag{this, Operation::finish}, done_tag{this, Operation::done} {
      if (kind == Kind::control) control = std::make_unique<grpc::ServerAsyncReaderWriter<wire::OpenSessionResponse, wire::ControlSessionRequest>>(&context);
      if (kind == Kind::submit) submit = std::make_unique<grpc::ServerAsyncResponseWriter<wire::SubmitReceipt>>(&context);
      if (kind == Kind::status) status = std::make_unique<grpc::ServerAsyncResponseWriter<wire::StatusEnvelope>>(&context);
      if (kind == Kind::capabilities) capabilities = std::make_unique<grpc::ServerAsyncResponseWriter<wire::Capabilities>>(&context);
      if (kind == Kind::subscription) subscription = std::make_unique<grpc::ServerAsyncWriter<wire::StatusEnvelope>>(&context);
    }
    Kind kind;
    std::uint64_t id;
    grpc::ServerContext context;
    Tag request_tag, read_tag, write_tag, finish_tag, done_tag;
    std::atomic<bool> started{false};
    bool request_pending{true}, done_pending{true}, read_pending{false}, write_pending{false}, finish_pending{false};
    bool closing{false}, finish_issued{false}, received_open{false}, done_seen{false}, end_after_reply{false};
    std::uint64_t submission_epoch{0};
    std::uint64_t holder_epoch{0}, opened_at{0}, next_status_at{0};
    grpc::Status end_status;
    wire::ControlSessionRequest control_request;
    wire::OpenSessionResponse control_response;
    std::optional<wire::OpenSessionResponse> latest_control_response;
    wire::SubmitRequest submit_request;
    wire::SubmitReceipt submit_response;
    wire::GetStatusRequest status_request;
    wire::GetCapabilitiesRequest capabilities_request;
    wire::SubscribeStatusRequest subscription_request;
    wire::StatusEnvelope status_response;
    std::optional<wire::StatusEnvelope> latest_status;
    wire::Capabilities capabilities_response;
    std::unique_ptr<grpc::ServerAsyncReaderWriter<wire::OpenSessionResponse, wire::ControlSessionRequest>> control;
    std::unique_ptr<grpc::ServerAsyncResponseWriter<wire::SubmitReceipt>> submit;
    std::unique_ptr<grpc::ServerAsyncResponseWriter<wire::StatusEnvelope>> status;
    std::unique_ptr<grpc::ServerAsyncResponseWriter<wire::Capabilities>> capabilities;
    std::unique_ptr<grpc::ServerAsyncWriter<wire::StatusEnvelope>> subscription;
  };

  Impl(Config configuration, std::shared_ptr<network::PlanningIngress> resource, wire::Capabilities advertised)
      : config(std::move(configuration)), ingress(std::move(resource)), capabilities(std::move(advertised)),
        instance(network::random_identity()) {
    validate_config(config);
    if (!ingress) throw std::invalid_argument("gRPC requires the shared PlanningIngress");
    capabilities.set_schema_version(1);
    capabilities.set_schema_fingerprint(network::schema_fingerprint());
    capabilities.set_planning_transport(wire::PLANNING_TRANSPORT_GRPC);
    *capabilities.mutable_opening_domain() = ingress->opening_domain();
    if (config.max_message_bytes < network::required_protocol_payload_bytes(capabilities, "r"))
      throw std::invalid_argument("gRPC payload cannot hold mandatory capabilities/session/status replies");
  }

  std::uint64_t limit(Kind kind) const {
    return kind == Kind::subscription ? config.max_status_subscribers : config.readonly_inflight_limit;
  }

  void arm(Kind kind) {
    if (stopping.load()) return;
    std::size_t active = 0;
    for (const auto &entry : calls) {
      if (entry.second->kind != kind) continue;
      if (entry.second->request_pending) return;
      ++active;
    }
    if (active >= limit(kind) + 1) return;
    if (next_id == std::numeric_limits<std::uint64_t>::max()) throw std::runtime_error("gRPC RPC identity exhausted");
    auto call = std::make_shared<Call>(kind, ++next_id);
    {
      std::lock_guard<std::mutex> lock(calls_mutex);
      calls.emplace(call->id, call);
    }
    call->context.AsyncNotifyWhenDone(&call->done_tag);
    if (kind == Kind::control) service.RequestOpenSession(&call->context, call->control.get(), cq.get(), cq.get(), &call->request_tag);
    if (kind == Kind::submit) service.RequestSubmit(&call->context, &call->submit_request, call->submit.get(), cq.get(), cq.get(), &call->request_tag);
    if (kind == Kind::status) service.RequestGetStatus(&call->context, &call->status_request, call->status.get(), cq.get(), cq.get(), &call->request_tag);
    if (kind == Kind::capabilities) service.RequestGetCapabilities(&call->context, &call->capabilities_request, call->capabilities.get(), cq.get(), cq.get(), &call->request_tag);
    if (kind == Kind::subscription) service.RequestSubscribeStatus(&call->context, &call->subscription_request, call->subscription.get(), cq.get(), cq.get(), &call->request_tag);
    peak_calls.store(std::max<std::uint64_t>(peak_calls.load(), calls.size()));
  }

  bool deadline_valid(const Call &call, std::uint32_t maximum_ms) const {
    const auto deadline = call.context.deadline();
    const auto now = std::chrono::system_clock::now();
    return deadline != std::chrono::system_clock::time_point::max() && deadline > now &&
           deadline - now <= std::chrono::milliseconds(maximum_ms);
  }

  void revoke_holder(Call &call, const std::string &reason) {
    if (call.holder_epoch != 0) ingress->revoke_if_current(call.holder_epoch, reason);
    call.latest_control_response.reset();
  }

  void terminate(Call &call, const grpc::Status &status, const std::string &reason, bool cancel = false) {
    call.closing = true;
    call.end_status = status;
    revoke_holder(call, reason);
    call.latest_status.reset();
    if (cancel && call.started.load()) call.context.TryCancel();
  }

  wire::StatusEnvelope envelope() {
    wire::StatusEnvelope result;
    result.set_schema_version(1);
    result.set_schema_fingerprint(network::schema_fingerprint());
    result.set_publisher_instance_token(instance);
    if (publication_sequence == std::numeric_limits<std::uint64_t>::max()) throw std::runtime_error("gRPC publication sequence exhausted");
    result.set_publication_sequence(++publication_sequence);
    result.set_published_system_ns(network::system_now_ns());
    result.set_published_boottime_ns(network::boottime_now_ns());
    *result.mutable_status() = ingress->readonly_status();
    return result;
  }

  void read(Call &call) {
    if (call.closing || call.read_pending || cq_closing.load()) return;
    call.control_request.Clear();
    call.read_pending = true;
    call.control->Read(&call.control_request, &call.read_tag);
  }

  void write(Call &call) {
    if (call.closing || call.write_pending || cq_closing.load()) return;
    if (call.kind == Kind::control && call.latest_control_response) {
      call.control_response = std::move(*call.latest_control_response);
      call.latest_control_response.reset();
      if (call.control_response.ByteSizeLong() > config.max_message_bytes) {
        terminate(call, grpc::Status(grpc::StatusCode::RESOURCE_EXHAUSTED, "Control reply exceeds configured payload"), "PAYLOAD_EXHAUSTED");
        return;
      }
      call.write_pending = true;
      call.control->Write(call.control_response, &call.write_tag);
      if (call.end_after_reply) call.closing = true;
    } else if (call.kind == Kind::subscription && call.latest_status) {
      call.status_response = std::move(*call.latest_status);
      call.latest_status.reset();
      if (call.status_response.ByteSizeLong() > config.max_message_bytes) {
        terminate(call, grpc::Status(grpc::StatusCode::RESOURCE_EXHAUSTED, "Status exceeds configured payload"), "PAYLOAD_EXHAUSTED");
        return;
      }
      call.write_pending = true;
      call.subscription->Write(call.status_response, &call.write_tag);
    }
  }

  void finish(Call &call) {
    if (!call.started.load() || !call.closing || call.read_pending || call.write_pending || call.finish_issued || cq_closing.load()) return;
    call.finish_issued = true;
    call.finish_pending = true;
    if (call.kind == Kind::control) call.control->Finish(call.end_status, &call.finish_tag);
    if (call.kind == Kind::subscription) call.subscription->Finish(call.end_status, &call.finish_tag);
    if (call.kind == Kind::submit) call.submit->Finish(call.submit_response, call.end_status, &call.finish_tag);
    if (call.kind == Kind::status) call.status->Finish(call.status_response, call.end_status, &call.finish_tag);
    if (call.kind == Kind::capabilities) call.capabilities->Finish(call.capabilities_response, call.end_status, &call.finish_tag);
  }

  void receive_control(Call &call) {
    const auto &request = call.control_request;
    if (request.ByteSizeLong() > config.max_message_bytes ||
        (!call.received_open && !request.has_open())) {
      terminate(call, grpc::Status(grpc::StatusCode::INVALID_ARGUMENT, "First control frame must be OpenSession"), "INVALID_CONTROL", true);
      return;
    }
    wire::OpenSessionResponse response;
    if (request.has_open()) {
      response = ingress->open(call.id, request.open(), network::steady_now_ns());
      call.received_open = true;
      if (response.has_session() && response.session().session_token().size() == codec::kSessionTokenSize) {
        call.holder_epoch = ingress->snapshot().epoch;
      }
    } else if (request.has_keep()) {
      response = ingress->keep(call.id, request.keep(), network::steady_now_ns());
    } else if (request.has_close()) {
      response = ingress->close(call.id, request.close());
    } else {
      terminate(call, grpc::Status(grpc::StatusCode::INVALID_ARGUMENT, "Missing control operation"), "INVALID_CONTROL", true);
      return;
    }
    if (call.latest_control_response) {
      ++dropped_updates;
      ingress->record_transport_drop();
    }
    const bool end = (response.result() == wire::SESSION_OPERATION_RESULT_BUSY && call.holder_epoch == 0) ||
        response.result() == wire::SESSION_OPERATION_RESULT_RESOURCE_EXHAUSTED ||
        (request.has_open() && response.result() == wire::SESSION_OPERATION_RESULT_REJECTED && call.holder_epoch == 0) ||
        (request.has_close() && response.result() == wire::SESSION_OPERATION_RESULT_REVOKED);
    call.end_after_reply = end;
    call.latest_control_response = response;
    write(call);
    // 最终回复slot也有界；若旧Write在途则待其完成再发Close/BUSY，不能丢回复或旋转holder。
    if (!end) read(call);
  }

  void receive_request(Call &call) {
    std::uint64_t active = 0;
    for (const auto &entry : calls)
      if (entry.second->kind == call.kind && entry.second->started.load()) ++active;
    if (active > limit(call.kind)) {
      terminate(call, grpc::Status(grpc::StatusCode::RESOURCE_EXHAUSTED, "Configured RPC capacity exhausted"), "RPC_CAPACITY");
      return;
    }
    const auto maximum = call.kind == Kind::control ? config.max_control_stream_lifetime_ms :
                         call.kind == Kind::subscription ? config.max_readonly_stream_lifetime_ms : config.max_unary_deadline_ms;
    if (!deadline_valid(call, maximum)) {
      terminate(call, grpc::Status(grpc::StatusCode::INVALID_ARGUMENT, "Explicit finite deadline exceeds configured RPC budget"), "INVALID_DEADLINE");
      return;
    }
    if (call.kind == Kind::control) {
      call.opened_at = network::steady_now_ns();
      read(call);
      return;
    }
    if (call.kind == Kind::submit) {
      codec::CommandFrame frame;
      if (call.submit_request.ByteSizeLong() > config.max_message_bytes || !decode_target(call.submit_request, frame)) {
        ingress->record_protocol_rejection();
        call.submit_response.set_schema_version(1);
        call.submit_response.set_rpc_received(true);
        call.submit_response.set_rpc_received_boottime_ns(network::boottime_now_ns());
        if (call.submit_request.has_target() && call.submit_request.target().has_key())
          *call.submit_response.mutable_key() = call.submit_request.target().key();
        call.submit_response.set_forwarding_state(wire::FORWARDING_STATE_REJECTED);
        call.submit_response.mutable_gateway_acceptance()->set_state(wire::GATEWAY_ACCEPTANCE_STATE_UNKNOWN);
      } else {
        // 短unary没有控制holder身份；唯一存活holder的token与最终epoch资格由公共入口同步检查。
        call.submission_epoch = ingress->snapshot().epoch;
        call.submit_response = ingress->submit(0, frame, network::boottime_now_ns(), call.id);
      }
    } else if (call.kind == Kind::capabilities) {
      if (call.capabilities_request.schema_version_presence_case() != wire::GetCapabilitiesRequest::kSchemaVersion ||
          call.capabilities_request.schema_version() != 1) {
        terminate(call, grpc::Status(grpc::StatusCode::INVALID_ARGUMENT, "Missing/unknown schema version"), "INVALID_SCHEMA");
        return;
      }
      call.capabilities_response = ingress->capabilities();
    } else if (call.kind == Kind::status || call.kind == Kind::subscription) {
      const bool valid = call.kind == Kind::status ?
          call.status_request.schema_version_presence_case() == wire::GetStatusRequest::kSchemaVersion && call.status_request.schema_version() == 1 :
          call.subscription_request.schema_version_presence_case() == wire::SubscribeStatusRequest::kSchemaVersion && call.subscription_request.schema_version() == 1;
      if (!valid) {
        terminate(call, grpc::Status(grpc::StatusCode::INVALID_ARGUMENT, "Missing/unknown schema version"), "INVALID_SCHEMA");
        return;
      }
      if (call.kind == Kind::subscription) {
        call.next_status_at = network::steady_now_ns();
        return;
      }
      call.status_response = envelope();
    }
    if (call.submit_response.ByteSizeLong() > config.max_message_bytes ||
        call.status_response.ByteSizeLong() > config.max_message_bytes ||
        call.capabilities_response.ByteSizeLong() > config.max_message_bytes) {
      terminate(call, grpc::Status(grpc::StatusCode::RESOURCE_EXHAUSTED, "Reply exceeds configured payload"), "PAYLOAD_EXHAUSTED");
    } else call.closing = true;
  }

  void process(Tag *tag, bool ok) {
    auto call = tag->call->shared_from_this();
    ++completed_tags;
    if (tag->operation == Operation::request) {
      call->request_pending = false;
      if (!ok) {
        // 未匹配RPC不保证有Done；保守留到CQ彻底drain，既不等待缺失tag也不提前释放地址。
        call->done_pending = false;
        ++unstarted_requests;
        unstarted_retired.push_back(call);
        call->closing = true;
      } else {
        call->started.store(true);
        if (stopping.load() || call->done_seen)
          terminate(*call, grpc::Status(grpc::StatusCode::CANCELLED, "RPC already terminated or server stopping"), "STOP_OR_EARLY_DONE", true);
        else receive_request(*call);
      }
    } else if (tag->operation == Operation::read) {
      call->read_pending = false;
      if (!ok) terminate(*call, grpc::Status::OK, "CONTROL_READ_ENDED");
      else if (!call->closing) receive_control(*call);
    } else if (tag->operation == Operation::write) {
      call->write_pending = false;
      if (!ok) terminate(*call, grpc::Status(grpc::StatusCode::CANCELLED, "Write ended"), "WRITE_ENDED", true);
      else write(*call);
    } else if (tag->operation == Operation::finish) {
      call->finish_pending = false;
    } else if (tag->operation == Operation::done) {
      call->done_pending = false;
      call->done_seen = true;
      // IsCancelled在Done交付后才合法；Read !ok和半关闭本身不用于推断取消。
      const auto cancelled = call->started.load() && call->context.IsCancelled();
      if (cancelled && call->kind == Kind::submit && call->submission_epoch != 0)
        ingress->cancel_pending_submission(call->submission_epoch, call->id);
      terminate(*call, cancelled ? grpc::Status(grpc::StatusCode::CANCELLED, "RPC terminated") : grpc::Status::OK,
                "CONTROL_RPC_DONE");
    }
    finish(*call);
    if (!call->request_pending && !call->read_pending && !call->write_pending &&
        !call->finish_pending && !call->done_pending && (!call->started.load() || call->finish_issued)) {
      std::lock_guard<std::mutex> lock(calls_mutex);
      calls.erase(call->id);
      calls_drained.notify_all();
    }
    for (const auto kind : {Kind::control, Kind::submit, Kind::status, Kind::capabilities, Kind::subscription}) arm(kind);
  }

  void tick() {
    const auto now = network::steady_now_ns();
    for (const auto &entry : calls) {
      auto &call = *entry.second;
      if (!call.started.load() || call.closing) continue;
      if (stopping.load()) {
        terminate(call, grpc::Status(grpc::StatusCode::CANCELLED, "Server stopping"), "STOP", true);
      } else if (call.kind == Kind::control && !call.received_open &&
                 now - call.opened_at >= static_cast<std::uint64_t>(config.control_open_timeout_ms) * 1000000) {
        terminate(call, grpc::Status(grpc::StatusCode::DEADLINE_EXCEEDED, "First OpenSession timeout"), "OPEN_TIMEOUT", true);
      } else if (call.kind == Kind::subscription && now >= call.next_status_at) {
        if (call.latest_status) { ++dropped_updates; ingress->record_transport_drop(); }
        call.latest_status = envelope();
        call.next_status_at = now + static_cast<std::uint64_t>(config.status_event_period_ms) * 1000000;
        write(call);
      }
      finish(call);
    }
  }

  void run() {
    void *raw = nullptr;
    bool ok = false;
    while (true) {
      const auto event = cq->AsyncNext(&raw, &ok, std::chrono::system_clock::now() + std::chrono::milliseconds(10));
      if (event == grpc::CompletionQueue::SHUTDOWN) break;
      std::lock_guard<std::mutex> enqueue_lock(enqueue_mutex);
      try {
        if (event == grpc::CompletionQueue::GOT_EVENT) process(static_cast<Tag *>(raw), ok);
        if (!cq_closing.load()) tick();
      } catch (const std::exception &error) {
        processing_failed.store(true);
        stopping.store(true);
        ingress->revoke("GRPC_CQ_FAILURE");
        std::cerr << "gRPC CQ processing failed: " << error.what() << "\n";
        for (const auto &entry : calls)
          if (entry.second->started.load()) entry.second->context.TryCancel();
      }
    }
    {
      std::lock_guard<std::mutex> lock(calls_mutex);
      unresolved_calls.store(calls.size());
      calls.clear();
      unstarted_retired.clear();
    }
    cq_drained.store(true);
  }

  void start() {
    if (server || stopping.load()) throw std::logic_error("GrpcServer cannot be started twice");
    grpc::ServerBuilder builder;
    builder.SetMaxReceiveMessageSize(static_cast<int>(config.max_message_bytes));
    builder.SetMaxSendMessageSize(static_cast<int>(config.max_message_bytes));
    // 五种RPC的acceptor/在途额度共用库级内存quota；这不是另一套命令lease。
    const auto call_budget = std::uint64_t{4} * config.readonly_inflight_limit + config.max_status_subscribers + 5;
    grpc::ResourceQuota quota;
    quota.Resize(static_cast<std::size_t>((config.max_message_bytes + std::uint64_t{65536}) * call_budget * 4));
    quota.SetMaxThreads(static_cast<int>(call_budget));
    builder.SetResourceQuota(quota);
    builder.AddChannelArgument("grpc.max_concurrent_streams", static_cast<int>(call_budget));
    int port = 0;
    builder.AddListeningPort(config.listen_address, grpc::InsecureServerCredentials(), &port);
    builder.RegisterService(&service);
    cq = builder.AddCompletionQueue();
    server = builder.BuildAndStart();
    if (!server || port == 0) {
      cq->Shutdown();
      void *raw = nullptr; bool ok = false;
      while (cq->Next(&raw, &ok)) {}
      cq.reset(); server.reset();
      throw std::runtime_error("gRPC could not bind configured loopback listener: " + config.listen_address);
    }
    for (const auto kind : {Kind::control, Kind::submit, Kind::status, Kind::capabilities, Kind::subscription}) arm(kind);
    poller = std::thread([this] { run(); });
  }

  void stop() noexcept {
    std::lock_guard<std::mutex> shutdown_lock(shutdown_mutex);
    if (!server) return;
    stopping.store(true);
    ingress->stop();
    std::vector<std::shared_ptr<Call>> active;
    {
      std::lock_guard<std::mutex> lock(calls_mutex);
      for (const auto &entry : calls) if (entry.second->started.load()) active.push_back(entry.second);
    }
    for (const auto &call : active) call->context.TryCancel();
    // 必须在非CQ线程Shutdown；poller继续处理取消/Read/Write/Done及必要Finish。
    server->Shutdown(std::chrono::system_clock::now() + std::chrono::milliseconds(config.shutdown_timeout_ms));
    {
      // Shutdown可先于应用Read/Done tag返回；Finish义务清空后才能关闭enqueue/CQ。
      std::unique_lock<std::mutex> lock(calls_mutex);
      if (!calls_drained.wait_for(lock, std::chrono::milliseconds(config.shutdown_timeout_ms), [this] { return calls.empty(); })) {
        std::cerr << "gRPC application tag drain exceeded shutdown_timeout_ms; retaining contexts until completed\n";
        calls_drained.wait(lock, [this] { return calls.empty(); });
      }
    }
    {
      std::lock_guard<std::mutex> enqueue_lock(enqueue_mutex);
      cq_closing.store(true);
      cq->Shutdown();
    }
    if (poller.joinable()) poller.join();
    server.reset(); cq.reset();
    if (unresolved_calls.load() != 0)
      std::cerr << "gRPC shutdown drained CQ with unresolved call obligations=" << unresolved_calls.load() << '\n';
  }

  Config config;
  std::shared_ptr<network::PlanningIngress> ingress;
  wire::Capabilities capabilities;
  std::string instance;
  wire::PlanningIngress::AsyncService service;
  std::unique_ptr<grpc::ServerCompletionQueue> cq;
  std::unique_ptr<grpc::Server> server;
  std::thread poller;
  std::map<std::uint64_t, std::shared_ptr<Call>> calls;
  std::vector<std::shared_ptr<Call>> unstarted_retired;
  mutable std::mutex calls_mutex;
  std::condition_variable calls_drained;
  std::mutex enqueue_mutex, shutdown_mutex;
  std::uint64_t next_id{0}, publication_sequence{0};
  std::atomic<bool> stopping{false}, cq_closing{false}, cq_drained{false}, processing_failed{false};
  std::atomic<std::uint64_t> peak_calls{0}, completed_tags{0}, dropped_updates{0},
                             unstarted_requests{0}, unresolved_calls{0};
};

GrpcServer::GrpcServer(Config config, std::shared_ptr<network::PlanningIngress> ingress, wire::Capabilities capabilities)
    : impl_(std::make_unique<Impl>(std::move(config), std::move(ingress), std::move(capabilities))) {}
GrpcServer::~GrpcServer() { stop(); }
void GrpcServer::start() { impl_->start(); }
void GrpcServer::stop() noexcept { impl_->stop(); }
ServerRuntime GrpcServer::runtime() const {
  std::lock_guard<std::mutex> lock(impl_->calls_mutex);
  return {impl_->calls.size(), impl_->peak_calls.load(), impl_->completed_tags.load(),
          impl_->dropped_updates.load(), impl_->unstarted_requests.load(), impl_->unresolved_calls.load(),
          impl_->cq_drained.load(), impl_->processing_failed.load()};
}

}
