#include "control_link_grpc_transport/server.hpp"
#include "control_link_grpc_transport/client.hpp"
#include "client_reply_policy.hpp"
#include "control_link_network_v1.grpc.pb.h"

#include <arpa/inet.h>
#include <sys/socket.h>
#include <unistd.h>

#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <limits>
#include <memory>
#include <stdexcept>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include <google/protobuf/message.h>
#include <google/protobuf/unknown_field_set.h>
#include <grpc/impl/codegen/grpc_types.h>
#include <grpcpp/grpcpp.h>
#include <gtest/gtest.h>

namespace control_link::grpc_transport {
namespace {

namespace wire = ::control_link::network::v1;
namespace codec = ::control_link_edge_protocol;
using namespace std::chrono_literals;

template <typename Predicate>
bool await(Predicate predicate, std::chrono::milliseconds budget = 2s) {
  const auto deadline = std::chrono::steady_clock::now() + budget;
  while (std::chrono::steady_clock::now() < deadline) {
    if (predicate()) return true;
    std::this_thread::sleep_for(2ms);
  }
  return predicate();
}

// 只改变测试 listener：由内核选空闲 loopback 端口，在生产 start 前释放保留 fd。
class LoopbackPort final {
 public:
  LoopbackPort() : descriptor_(::socket(AF_INET, SOCK_STREAM | SOCK_CLOEXEC, 0)) {
    if (descriptor_ < 0) throw std::runtime_error("fixture socket failed");
    sockaddr_in address{};
    address.sin_family = AF_INET;
    address.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
    if (::bind(descriptor_, reinterpret_cast<sockaddr *>(&address), sizeof(address)) != 0) {
      close();
      throw std::runtime_error("fixture loopback reservation failed");
    }
    socklen_t size = sizeof(address);
    if (::getsockname(descriptor_, reinterpret_cast<sockaddr *>(&address), &size) != 0) {
      close();
      throw std::runtime_error("fixture loopback getsockname failed");
    }
    address_ = "127.0.0.1:" + std::to_string(ntohs(address.sin_port));
  }
  ~LoopbackPort() { close(); }
  LoopbackPort(const LoopbackPort &) = delete;
  LoopbackPort &operator=(const LoopbackPort &) = delete;
  const std::string &address() const { return address_; }
  void close() {
    if (descriptor_ >= 0) {
      ::close(descriptor_);
      descriptor_ = -1;
    }
  }

 private:
  int descriptor_;
  std::string address_;
};

wire::ControlSessionRequest make_open_request(char identity) {
  wire::ControlSessionRequest request;
  auto *open = request.mutable_open();
  open->set_schema_version(1);
  open->set_holder_nonce(std::string(16, identity));
  open->set_open_request_id(std::string(16, identity));
  return request;
}

wire::ControlSessionRequest keep_request(const wire::ControlSessionRequest &open,
                                       std::uint64_t sequence) {
  wire::ControlSessionRequest request;
  auto *keep = request.mutable_keep();
  keep->set_schema_version(1);
  keep->set_holder_nonce(open.open().holder_nonce());
  keep->set_open_request_id(open.open().open_request_id());
  keep->set_keepalive_sequence(sequence);
  *keep->mutable_opening_domain() = open.open().opening_domain();
  return request;
}

wire::SubmitRequest target_request(const std::string &token, std::uint64_t sequence) {
  wire::SubmitRequest request;
  auto *target = request.mutable_target();
  target->set_schema_version(1);
  target->mutable_key()->set_session_token(token);
  target->mutable_key()->set_source_sequence(sequence);
  target->set_source_stamp_ns(1700000000123456789LL);
  target->set_clock_domain(wire::CLOCK_DOMAIN_SYSTEM);
  target->set_mode(wire::COMMAND_MODE_NORMAL);
  target->set_linear_velocity_mps(0.5);
  target->set_angular_velocity_radps(0.1);
  return request;
}

// 同步 stub 仅作测试驱动；服务仍调用真实生产 CQ engine，不替代生产异步 SDK。
class ControlStream final {
 public:
  ControlStream(wire::PlanningIngress::Stub &stub, std::chrono::milliseconds lifetime) {
    context_.set_deadline(std::chrono::system_clock::now() + lifetime);
    stream_ = stub.OpenSession(&context_);
    if (!stream_) throw std::runtime_error("fixture control stream creation failed");
  }
  ~ControlStream() {
    if (!finished_) {
      cancel();
      finish();
    }
  }
  ControlStream(const ControlStream &) = delete;
  ControlStream &operator=(const ControlStream &) = delete;
  bool write(const wire::ControlSessionRequest &request) { return stream_->Write(request); }
  bool read(wire::OpenSessionResponse *response) { return stream_->Read(response); }
  bool half_close() { return stream_->WritesDone(); }
  void cancel() { context_.TryCancel(); }
  grpc::Status finish() {
    if (!finished_) {
      result_ = stream_->Finish();
      finished_ = true;
    }
    return result_;
  }

 private:
  grpc::ClientContext context_;
  std::unique_ptr<grpc::ClientReaderWriter<wire::ControlSessionRequest,
                                          wire::OpenSessionResponse>> stream_;
  bool finished_{false};
  grpc::Status result_;
};

class StatusStream final {
 public:
  StatusStream(wire::PlanningIngress::Stub &stub, std::chrono::milliseconds lifetime) {
    context_.set_deadline(std::chrono::system_clock::now() + lifetime);
    wire::SubscribeStatusRequest request;
    request.set_schema_version(1);
    stream_ = stub.SubscribeStatus(&context_, request);
  }
  ~StatusStream() {
    if (!finished_) {
      cancel();
      finish();
    }
  }
  StatusStream(const StatusStream &) = delete;
  StatusStream &operator=(const StatusStream &) = delete;
  bool read(wire::StatusEnvelope *response) { return stream_->Read(response); }
  void cancel() { context_.TryCancel(); }
  grpc::Status finish() {
    if (!finished_) {
      result_ = stream_->Finish();
      finished_ = true;
    }
    return result_;
  }

 private:
  grpc::ClientContext context_;
  std::unique_ptr<grpc::ClientReader<wire::StatusEnvelope>> stream_;
  bool finished_{false};
  grpc::Status result_;
};

// 单独一个 writer 制造慢控制回复；任意 ASSERT 提前退出时仍 cancel→join，禁止 detached。
class OpenBurst final {
 public:
  OpenBurst(ControlStream &stream, wire::ControlSessionRequest request) : stream_(stream),
      worker_([this, request = std::move(request)] {
        for (std::uint64_t count = 0; count < 4096 && !stopping_.load(); ++count) {
          if (!stream_.write(request)) break;
          ++sent_;
        }
      }) {}
  ~OpenBurst() { stop(); }
  void stop() {
    stopping_.store(true);
    stream_.cancel();
    if (worker_.joinable()) worker_.join();
  }
  std::uint64_t sent() const { return sent_.load(); }

 private:
  ControlStream &stream_;
  std::atomic<bool> stopping_{false};
  std::atomic<std::uint64_t> sent_{0};
  std::thread worker_;
};

std::size_t expect_redacted(const google::protobuf::Message &message) {
  const auto *reflection = message.GetReflection();
  const auto *descriptor = message.GetDescriptor();
  std::size_t checked = 0;
  for (int index = 0; index < descriptor->field_count(); ++index) {
    const auto *field = descriptor->field(index);
    if (field->name() == "session_token") {
      EXPECT_TRUE(reflection->GetString(message, field).empty()) << field->full_name();
      ++checked;
    } else if (field->cpp_type() == google::protobuf::FieldDescriptor::CPPTYPE_MESSAGE) {
      if (field->is_repeated()) {
        for (int entry = 0; entry < reflection->FieldSize(message, field); ++entry)
          checked += expect_redacted(reflection->GetRepeatedMessage(message, field, entry));
      } else if (reflection->HasField(message, field)) {
        checked += expect_redacted(reflection->GetMessage(message, field));
      }
    }
  }
  return checked;
}

class GrpcLifecycle : public ::testing::Test {
 protected:
  wire::ControlSessionRequest open_request(char identity) {
    auto request = make_open_request(identity);
    *request.mutable_open()->mutable_opening_domain() = ingress_->opening_domain();
    return request;
  }
  void SetUp() override {
    config_ = load_config(CONTROL_LINK_PROTOCOL_CONFIG_PATH);
    start_fresh();
  }
  void TearDown() override {
    if (server_) {
      server_->stop();
      expect_drained();
    }
  }
  void start_fresh() {
    LoopbackPort reservation;
    config_.listen_address = reservation.address();
    wire::Capabilities capabilities;
    capabilities.set_contract_id("grpc-lifecycle-fixture");
    capabilities.set_contract_hash(std::string(64, 'c'));
    capabilities.set_decision_config_hash(std::string(64, 'd'));
    capabilities.set_max_command_bytes(codec::kCommandFrameSize);
    capabilities.set_max_status_bytes(config_.max_message_bytes);
    capabilities.set_max_pending_targets(1);
    capabilities.set_max_status_subscribers(config_.max_status_subscribers);
    ingress_ = std::make_shared<network::PlanningIngress>(config_.ingress, capabilities,
                                                         wire::PLANNING_TRANSPORT_GRPC);
    server_ = std::make_unique<GrpcServer>(config_, ingress_, capabilities);
    reservation.close();
    server_->start();
    grpc::ChannelArguments arguments;
    arguments.SetInt(GRPC_ARG_HTTP2_BDP_PROBE, 0);
    arguments.SetInt(GRPC_ARG_HTTP2_STREAM_LOOKAHEAD_BYTES, 5);
    channel_ = grpc::CreateCustomChannel(config_.listen_address,
                                         grpc::InsecureChannelCredentials(), arguments);
    ASSERT_TRUE(channel_->WaitForConnected(std::chrono::system_clock::now() + 2s));
    stub_ = wire::PlanningIngress::NewStub(channel_);
    ASSERT_TRUE(await([this] { return server_->runtime().active_calls == 5; }));
  }
  // 显式替代 ROS resource owner 的测试装配，不宣称 Graph/SourceStatus/Gateway 准入已验证。
  bool admit(std::uint64_t epoch, char generation = 'g') {
    return ingress_->bind_publisher(epoch, std::string(24, generation), "rmw_fastrtps_cpp") &&
           ingress_->set_admitted(epoch, true);
  }
  bool release_revoked() {
    const auto snapshot = ingress_->snapshot();
    return snapshot.state == wire::CONTROL_SESSION_STATE_REVOKING &&
           ingress_->finish_release(snapshot.epoch);
  }
  grpc::Status submit(const wire::SubmitRequest &request, wire::SubmitReceipt *receipt) {
    grpc::ClientContext context;
    context.set_deadline(std::chrono::system_clock::now() + 1s);
    return stub_->Submit(&context, request, receipt);
  }
  grpc::Status get_status(wire::StatusEnvelope *response) {
    grpc::ClientContext context;
    context.set_deadline(std::chrono::system_clock::now() + 1s);
    wire::GetStatusRequest request;
    request.set_schema_version(1);
    return stub_->GetStatus(&context, request, response);
  }
  void expect_drained() {
    const auto runtime = server_->runtime();
    EXPECT_TRUE(runtime.cq_drained);
    EXPECT_EQ(runtime.active_calls, 0U);
    EXPECT_EQ(runtime.unresolved_calls, 0U);
    EXPECT_GT(runtime.completed_tags, 0U);
    EXPECT_LE(runtime.peak_calls, std::uint64_t{4} * config_.readonly_inflight_limit +
                                     config_.max_status_subscribers + 5);
  }

  Config config_;
  std::shared_ptr<network::PlanningIngress> ingress_;
  std::unique_ptr<GrpcServer> server_;
  std::shared_ptr<grpc::Channel> channel_;
  std::unique_ptr<wire::PlanningIngress::Stub> stub_;
};

TEST(GrpcReplyPolicy, LaterReadonlyRevocationCannotBeUndoneByEarlierRequestReply) {
  EXPECT_TRUE(reply_preserves_readiness(true, 2, 2));
  EXPECT_FALSE(reply_preserves_readiness(true, 2, 3));
  EXPECT_FALSE(reply_preserves_readiness(false, 3, 3));
  EXPECT_TRUE(reply_preserves_readiness(true, 3, 3));
}

TEST_F(GrpcLifecycle, BoundedOpeningDomainsRotateThroughRealRpcAndRejectFrozenOldRequests) {
  server_->stop();
  expect_drained();
  config_.ingress.retired_open_capacity = 1;
  start_fresh();
  wire::ControlSessionRequest original;
  std::string previous_token;
  for (std::uint64_t iteration = 0; iteration < 128; ++iteration) {
    grpc::ClientContext query_context;
    query_context.set_deadline(std::chrono::system_clock::now() + 1s);
    wire::GetCapabilitiesRequest query;
    query.set_schema_version(1);
    wire::Capabilities observed;
    ASSERT_TRUE(stub_->GetCapabilities(&query_context, query, &observed).ok());
    ASSERT_EQ(observed.opening_domain().domain_version(), 1U);
    ASSERT_EQ(observed.opening_domain().generation(), iteration + 1);
    ASSERT_EQ(observed.opening_domain().service_instance_id().size(), 16U);
    auto request = make_open_request('r');
    *request.mutable_open()->mutable_opening_domain() = observed.opening_domain();
    if (iteration == 0) original = request;
    ControlStream holder(*stub_, 5s);
    wire::OpenSessionResponse response;
    ASSERT_TRUE(holder.write(request));
    ASSERT_TRUE(holder.read(&response));
    ASSERT_EQ(response.result(), wire::SESSION_OPERATION_RESULT_OPENING);
    ASSERT_TRUE(response.has_session());
    const auto token = response.session().session_token();
    EXPECT_NE(token, previous_token);
    previous_token = token;
    const auto epoch = ingress_->snapshot().epoch;
    ASSERT_TRUE(holder.write(request));
    ASSERT_TRUE(holder.read(&response));
    EXPECT_EQ(response.result(), wire::SESSION_OPERATION_RESULT_IDEMPOTENT);
    EXPECT_EQ(response.session().session_token(), token);
    EXPECT_EQ(ingress_->snapshot().epoch, epoch);
    if (iteration == 0) {
      ControlStream missing(*stub_, 2s);
      ASSERT_TRUE(missing.write(make_open_request('x')));
      ASSERT_TRUE(missing.read(&response));
      EXPECT_EQ(response.result(), wire::SESSION_OPERATION_RESULT_REJECTED);
      EXPECT_EQ(response.failure_reason(), "OPEN_ADMISSION_DOMAIN_REQUIRED_OR_STALE");
      EXPECT_FALSE(response.has_session());
      missing.cancel();
      missing.finish();
      ControlStream competing(*stub_, 2s);
      ASSERT_TRUE(competing.write(request));
      ASSERT_TRUE(competing.read(&response));
      EXPECT_EQ(response.result(), wire::SESSION_OPERATION_RESULT_BUSY);
      EXPECT_FALSE(response.has_session());
      competing.cancel();
      competing.finish();
    } else {
      ControlStream stale(*stub_, 2s);
      ASSERT_TRUE(stale.write(original));
      ASSERT_TRUE(stale.read(&response));
      EXPECT_EQ(response.result(), wire::SESSION_OPERATION_RESULT_REJECTED);
      EXPECT_EQ(response.failure_reason(), "OPEN_ADMISSION_DOMAIN_REQUIRED_OR_STALE");
      EXPECT_FALSE(response.has_session());
      stale.cancel();
      stale.finish();
    }
    ASSERT_TRUE(holder.write(keep_request(request, 1)));
    ASSERT_TRUE(holder.read(&response));
    EXPECT_EQ(response.session().session_token(), token);
    ASSERT_TRUE(holder.half_close());
    ASSERT_TRUE(await([this] { return ingress_->snapshot().state == wire::CONTROL_SESSION_STATE_REVOKING; }));
    holder.finish();
    EXPECT_FALSE(ingress_->publish_pending(epoch, 100, [](const auto &) { FAIL() << "old epoch published"; }));
    ASSERT_TRUE(release_revoked());
    EXPECT_EQ(ingress_->snapshot().retired_open_count, 0U);
    EXPECT_EQ(ingress_->snapshot().opening_domain.generation(), iteration + 2);
  }
}

TEST_F(GrpcLifecycle, InstalledSdkControlAndReadonlyCloseKeepSeparateResponsibilities) {
  GrpcClient client(config_);
  client.start();
  ASSERT_TRUE(client.wait_available(1s));
  const auto request = open_request(static_cast<char>(109));
  const auto opened = client.open(request.open());
  const auto epoch = ingress_->snapshot().epoch;
  ASSERT_TRUE(admit(epoch));
  const auto ready = client.keep(keep_request(request, 1).keep());
  ASSERT_TRUE(ready.session().submission_ready());
  EXPECT_TRUE(client.get_status().has_status());
  const auto before_subscription = client.status().status_received_steady_ns;
  client.set_status_subscription(true);
  ASSERT_TRUE(await([&] { return client.status().status_received_steady_ns > before_subscription; }))
      << "running=" << client.status().running << " subscribed=" << client.status().readonly_subscribed
      << " failure=" << client.status().failure;
  client.set_status_subscription(false);
  EXPECT_EQ(ingress_->snapshot().epoch, epoch);
  wire::CloseSessionRequest close;
  close.set_schema_version(1);
  close.set_holder_nonce(request.open().holder_nonce());
  close.set_open_request_id(request.open().open_request_id());
  *close.mutable_opening_domain() = request.open().opening_domain();
  const auto closed = client.close(close);
  EXPECT_EQ(closed.result(), wire::SESSION_OPERATION_RESULT_REVOKED);
  EXPECT_EQ(closed.session().session_token(), opened.session().session_token());
  EXPECT_EQ(closed.session().control_session_state(), wire::CONTROL_SESSION_STATE_REVOKING);
  EXPECT_FALSE(client.status().control_active);
  client.stop();
  EXPECT_FALSE(client.status().running);
}

TEST_F(GrpcLifecycle, SubscriptionOverloadAndInvalidCapacityHaveExplicitDiagnostics) {
  auto invalid = config_;
  invalid.max_message_bytes = invalid.ingress.max_payload_bytes = 64;
  EXPECT_THROW(validate_config(invalid), std::invalid_argument);
  invalid = config_;
  invalid.max_status_subscribers = std::numeric_limits<std::uint32_t>::max();
  EXPECT_THROW(validate_config(invalid), std::invalid_argument);
  std::vector<std::unique_ptr<StatusStream>> streams;
  wire::StatusEnvelope response;
  for (std::uint32_t count = 0; count < config_.max_status_subscribers; ++count) {
    auto stream = std::make_unique<StatusStream>(*stub_, 10s);
    ASSERT_TRUE(stream->read(&response));
    streams.push_back(std::move(stream));
  }
  StatusStream rejected(*stub_, 5s);
  EXPECT_FALSE(rejected.read(&response));
  EXPECT_EQ(rejected.finish().error_code(), grpc::StatusCode::RESOURCE_EXHAUSTED);
  for (auto &stream : streams) {
    stream->cancel();
    EXPECT_EQ(stream->finish().error_code(), grpc::StatusCode::CANCELLED);
  }
}

TEST_F(GrpcLifecycle, SameRpcIdempotenceCompetitorAndReadonlyIsolation) {
  const auto request = open_request('a');
  ControlStream holder(*stub_, 10s);
  ASSERT_TRUE(holder.write(request));
  wire::OpenSessionResponse opened;
  ASSERT_TRUE(holder.read(&opened));
  ASSERT_EQ(opened.result(), wire::SESSION_OPERATION_RESULT_OPENING);
  ASSERT_EQ(opened.session().session_token().size(), codec::kSessionTokenSize);
  const auto epoch = ingress_->snapshot().epoch;
  ASSERT_TRUE(holder.write(request));
  wire::OpenSessionResponse retried;
  ASSERT_TRUE(holder.read(&retried));
  EXPECT_EQ(retried.result(), wire::SESSION_OPERATION_RESULT_IDEMPOTENT);
  EXPECT_EQ(retried.session().session_token(), opened.session().session_token());
  EXPECT_EQ(ingress_->snapshot().epoch, epoch);

  ControlStream competitor(*stub_, 10s);
  ASSERT_TRUE(competitor.write(request));
  wire::OpenSessionResponse busy;
  ASSERT_TRUE(competitor.read(&busy));
  EXPECT_EQ(busy.result(), wire::SESSION_OPERATION_RESULT_BUSY);
  EXPECT_FALSE(busy.has_session());
  EXPECT_EQ(busy.open_request_id(), request.open().open_request_id());
  EXPECT_FALSE(competitor.read(&busy));
  EXPECT_TRUE(competitor.finish().ok());
  ASSERT_TRUE(admit(epoch));
  ASSERT_TRUE(holder.write(keep_request(request, 1)));
  ASSERT_TRUE(holder.read(&retried));
  EXPECT_TRUE(retried.session().submission_ready());
  EXPECT_EQ(retried.session().session_token(), opened.session().session_token());

  wire::SubmitReceipt receipt;
  ASSERT_TRUE(submit(target_request(opened.session().session_token(), 10), &receipt).ok());
  ASSERT_EQ(receipt.forwarding_state(), wire::FORWARDING_STATE_PENDING);
  ASSERT_TRUE(ingress_->publish_pending(epoch, network::boottime_now_ns(), [](const auto &) {}));
  // 构造明确非空的嵌套关联 key，避免只检查原本没有 token 的空观测。
  ingress_->update_observations([](auto &status) {
    auto *source = status.add_sources();
    source->set_source_id("planning");
    source->mutable_observation()->set_validity(wire::OBSERVATION_VALIDITY_VALID);
    source->set_endpoint_stable(true);
    source->set_endpoint_state(5);
    source->set_endpoint_publisher_count(1);
    source->set_endpoint_generation_valid(true);
    source->mutable_endpoint_publisher()->set_gid(std::string(24, 'g'));
    source->mutable_endpoint_publisher()->set_rmw_implementation("rmw_fastrtps_cpp");
    source->set_last_source_sequence(10);
    source->set_accepted_count(1);
  });
  wire::StatusEnvelope status;
  ASSERT_TRUE(get_status(&status).ok());
  ASSERT_TRUE(status.status().latest_forwarded().has_key());
  ASSERT_TRUE(status.status().latest_gateway_acceptance().has_key());
  EXPECT_EQ(status.status().latest_gateway_acceptance().state(),
            wire::GATEWAY_ACCEPTANCE_STATE_LATEST_KEY_OBSERVED);
  EXPECT_GE(expect_redacted(status), 3U);
  EXPECT_EQ(status.status().session().ingress_publisher().gid(), std::string(24, 'g'));

  StatusStream readonly(*stub_, 10s);
  ASSERT_TRUE(readonly.read(&status));
  EXPECT_GE(expect_redacted(status), 3U);
  const auto before_cancel = ingress_->snapshot();
  readonly.cancel();
  EXPECT_EQ(readonly.finish().error_code(), grpc::StatusCode::CANCELLED);
  ASSERT_TRUE(holder.write(keep_request(request, 2)));
  ASSERT_TRUE(holder.read(&retried));
  EXPECT_EQ(ingress_->snapshot().epoch, before_cancel.epoch);
  EXPECT_EQ(ingress_->snapshot().session.session_token(), before_cancel.session.session_token());
  EXPECT_EQ(ingress_->snapshot().state, wire::CONTROL_SESSION_STATE_ADMITTED);
  StatusStream resubscribed(*stub_, 10s);
  ASSERT_TRUE(resubscribed.read(&status));
  EXPECT_GE(expect_redacted(status), 3U);
  EXPECT_EQ(ingress_->snapshot().epoch, epoch);

  grpc::ClientContext context;
  context.set_deadline(std::chrono::system_clock::now() + 1s);
  wire::GetCapabilitiesRequest capabilities_request;
  capabilities_request.set_schema_version(1);
  wire::Capabilities capabilities;
  ASSERT_TRUE(stub_->GetCapabilities(&context, capabilities_request, &capabilities).ok());
  EXPECT_EQ(capabilities.planning_transport(), wire::PLANNING_TRANSPORT_GRPC);
  EXPECT_EQ(capabilities.schema_fingerprint(), network::schema_fingerprint());
}

TEST_F(GrpcLifecycle, BoundaryRejectionsDoNotReplaceRawBusinessValues) {
  auto undersized = config_;
  undersized.max_message_bytes = 64;
  undersized.ingress.max_payload_bytes = 64;
  EXPECT_THROW(validate_config(undersized), std::invalid_argument);
  wire::Capabilities oversized_capabilities;
  oversized_capabilities.set_contract_id(std::string(config_.max_message_bytes + 1, 'x'));
  EXPECT_THROW(GrpcServer(config_, ingress_, oversized_capabilities), std::invalid_argument);

  ControlStream holder(*stub_, 10s);
  ASSERT_TRUE(holder.write(open_request('b')));
  wire::OpenSessionResponse opened;
  ASSERT_TRUE(holder.read(&opened));
  const auto epoch = ingress_->snapshot().epoch;
  ASSERT_TRUE(admit(epoch));
  const auto valid = target_request(opened.session().session_token(), 10);
  wire::SubmitReceipt receipt;
  ASSERT_TRUE(submit(valid, &receipt).ok());
  ASSERT_EQ(receipt.forwarding_state(), wire::FORWARDING_STATE_PENDING);
  EXPECT_EQ(receipt.gateway_acceptance().state(), wire::GATEWAY_ACCEPTANCE_STATE_UNKNOWN);

  std::vector<wire::SubmitRequest> invalid;
  invalid.emplace_back();
  invalid.push_back(valid);
  invalid.back().mutable_target()->clear_schema_version();
  invalid.push_back(valid);
  invalid.back().mutable_target()->set_schema_version(2);
  invalid.push_back(valid);
  invalid.back().mutable_target()->mutable_key()->clear_source_sequence();
  invalid.push_back(valid);
  invalid.back().mutable_target()->clear_source_stamp_ns();
  invalid.push_back(valid);
  invalid.back().mutable_target()->clear_linear_velocity_mps();
  invalid.push_back(valid);
  invalid.back().mutable_target()->set_clock_domain(wire::CLOCK_DOMAIN_UNSPECIFIED);
  invalid.push_back(valid);
  invalid.back().mutable_target()->set_mode(static_cast<wire::CommandMode>(256));
  invalid.push_back(valid);
  invalid.back().mutable_target()->set_source_stamp_ns(std::numeric_limits<std::int64_t>::max());
  invalid.push_back(valid);
  invalid.back().mutable_target()->mutable_key()->set_session_token(std::string(15, 'x'));
  invalid.push_back(valid);
  invalid.back().mutable_target()->mutable_key()->set_session_token(std::string(16, 'x'));
  for (std::size_t index = 0; index < invalid.size(); ++index) {
    SCOPED_TRACE(index);
    receipt.Clear();
    ASSERT_TRUE(submit(invalid[index], &receipt).ok());
    EXPECT_TRUE(receipt.rpc_received());
    EXPECT_EQ(receipt.forwarding_state(), wire::FORWARDING_STATE_REJECTED);
    EXPECT_EQ(receipt.gateway_acceptance().state(), wire::GATEWAY_ACCEPTANCE_STATE_UNKNOWN);
    EXPECT_EQ(ingress_->snapshot().epoch, epoch);
  }
  codec::CommandFrame forwarded;
  ASSERT_TRUE(ingress_->publish_pending(epoch, network::boottime_now_ns(),
                                       [&](const auto &frame) { forwarded = frame; }));
  EXPECT_EQ(forwarded.command_sequence, valid.target().key().source_sequence());
  EXPECT_EQ(forwarded.source_stamp_ns, valid.target().source_stamp_ns());

  auto too_large = valid;
  auto *target = too_large.mutable_target();
  target->GetReflection()->MutableUnknownFields(target)->AddLengthDelimited(
      999, std::string(config_.max_message_bytes + 1, 'x'));
  receipt.Clear();
  EXPECT_EQ(submit(too_large, &receipt).error_code(), grpc::StatusCode::RESOURCE_EXHAUSTED);
  EXPECT_EQ(ingress_->snapshot().epoch, epoch);
  EXPECT_FALSE(ingress_->publish_pending(epoch, network::boottime_now_ns(),
                                        [](const auto &) { ADD_FAILURE() << "oversized target published"; }));

  // 重复/倒退/零 sequence、陈旧 stamp、NaN/Inf 与可表示未知模式原样交唯一 Gateway 判断。
  for (const auto sequence : {10U, 10U, 9U, 0U}) {
    auto business = target_request(opened.session().session_token(), sequence);
    business.mutable_target()->set_source_stamp_ns(1);
    business.mutable_target()->set_linear_velocity_mps(std::numeric_limits<double>::quiet_NaN());
    business.mutable_target()->set_angular_velocity_radps(std::numeric_limits<double>::infinity());
    business.mutable_target()->set_mode(static_cast<wire::CommandMode>(200));
    ASSERT_TRUE(submit(business, &receipt).ok());
    ASSERT_EQ(receipt.forwarding_state(), wire::FORWARDING_STATE_PENDING);
    ASSERT_TRUE(ingress_->publish_pending(epoch, network::boottime_now_ns(),
                                         [&](const auto &frame) { forwarded = frame; }));
    EXPECT_EQ(forwarded.command_sequence, sequence);
    EXPECT_EQ(forwarded.source_stamp_ns, 1);
    EXPECT_TRUE(std::isnan(forwarded.linear_velocity_mps));
    EXPECT_TRUE(std::isinf(forwarded.angular_velocity_radps));
    EXPECT_EQ(static_cast<unsigned>(forwarded.mode), 200U);
    EXPECT_EQ(receipt.gateway_acceptance().state(), wire::GATEWAY_ACCEPTANCE_STATE_UNKNOWN);
  }
  for (const bool missing_schema : {true, false}) {
    grpc::ClientContext context;
    context.set_deadline(std::chrono::system_clock::now() + 1s);
    wire::GetStatusRequest request;
    if (!missing_schema) request.set_schema_version(2);
    wire::StatusEnvelope response;
    EXPECT_EQ(stub_->GetStatus(&context, request, &response).error_code(),
              grpc::StatusCode::INVALID_ARGUMENT);
    EXPECT_EQ(ingress_->snapshot().epoch, epoch);
  }
  grpc::ClientContext missing_deadline;
  wire::GetCapabilitiesRequest request;
  request.set_schema_version(1);
  wire::Capabilities response;
  EXPECT_EQ(stub_->GetCapabilities(&missing_deadline, request, &response).error_code(),
            grpc::StatusCode::INVALID_ARGUMENT);
}

TEST_F(GrpcLifecycle, HalfCloseLateCancellationDeadlineAndFirstOpenTimeout) {
  {
    ControlStream half_closed(*stub_, 10s);
    ASSERT_TRUE(half_closed.write(open_request('c')));
    wire::OpenSessionResponse opened;
    ASSERT_TRUE(half_closed.read(&opened));
    const auto epoch = ingress_->snapshot().epoch;
    ASSERT_TRUE(admit(epoch));
    wire::SubmitReceipt receipt;
    ASSERT_TRUE(submit(target_request(opened.session().session_token(), 1), &receipt).ok());
    ASSERT_TRUE(half_closed.half_close());
    EXPECT_FALSE(half_closed.read(&opened));
    EXPECT_TRUE(half_closed.finish().ok());
    ASSERT_TRUE(await([this] { return ingress_->snapshot().state == wire::CONTROL_SESSION_STATE_REVOKING; }));
    EXPECT_FALSE(ingress_->publish_pending(epoch, network::boottime_now_ns(),
                                          [](const auto &) { ADD_FAILURE() << "half-closed epoch published"; }));
    ASSERT_TRUE(release_revoked());
  }
  ASSERT_TRUE(await([this] { return server_->runtime().active_calls == 5; }));

  ControlStream predecessor(*stub_, 10s);
  ASSERT_TRUE(predecessor.write(open_request('d')));
  wire::OpenSessionResponse prior;
  ASSERT_TRUE(predecessor.read(&prior));
  const auto old_epoch = ingress_->snapshot().epoch;
  ASSERT_TRUE(admit(old_epoch, 'p'));
  // 资源 owner 已撤销/释放旧代，但旧 RPC 仍有真实 pending Read；Done 在新代开立后触发。
  ASSERT_TRUE(ingress_->revoke_if_current(old_epoch, "fixture predecessor release"));
  ASSERT_TRUE(release_revoked());
  ControlStream successor(*stub_, 10s);
  ASSERT_TRUE(successor.write(open_request('e')));
  wire::OpenSessionResponse current;
  ASSERT_TRUE(successor.read(&current));
  const auto new_epoch = ingress_->snapshot().epoch;
  ASSERT_NE(new_epoch, old_epoch);
  ASSERT_NE(current.session().session_token(), prior.session().session_token());
  ASSERT_TRUE(admit(new_epoch, 'n'));
  const auto calls_before_cancel = server_->runtime().active_calls;
  predecessor.cancel();
  EXPECT_EQ(predecessor.finish().error_code(), grpc::StatusCode::CANCELLED);
  ASSERT_TRUE(await([&] { return server_->runtime().active_calls < calls_before_cancel; }));
  EXPECT_EQ(ingress_->snapshot().epoch, new_epoch);
  EXPECT_EQ(ingress_->snapshot().state, wire::CONTROL_SESSION_STATE_ADMITTED);
  EXPECT_EQ(ingress_->snapshot().session.session_token(), current.session().session_token());
  auto stale_token = target_request(prior.session().session_token(), 1);
  stale_token.mutable_target()->set_source_stamp_ns(network::system_now_ns());
  wire::SubmitReceipt receipt;
  ASSERT_TRUE(submit(stale_token, &receipt).ok());
  EXPECT_EQ(receipt.forwarding_state(), wire::FORWARDING_STATE_REJECTED);
  ASSERT_TRUE(submit(target_request(current.session().session_token(), 1), &receipt).ok());
  EXPECT_EQ(receipt.forwarding_state(), wire::FORWARDING_STATE_PENDING);
  successor.cancel();
  EXPECT_EQ(successor.finish().error_code(), grpc::StatusCode::CANCELLED);
  ASSERT_TRUE(await([this] { return ingress_->snapshot().state == wire::CONTROL_SESSION_STATE_REVOKING; }));
  ASSERT_TRUE(release_revoked());
  ASSERT_TRUE(await([this] { return server_->runtime().active_calls == 5; }));

  {
    const auto baseline = server_->runtime().active_calls;
    ControlStream deadline(*stub_, 1s);
    ASSERT_TRUE(await([&] { return server_->runtime().active_calls > baseline; }));
    ASSERT_TRUE(deadline.write(open_request('f')));
    wire::OpenSessionResponse opened;
    ASSERT_TRUE(deadline.read(&opened));
    ASSERT_EQ(opened.session().session_token().size(), codec::kSessionTokenSize);
    ASSERT_TRUE(await([this] { return ingress_->snapshot().state == wire::CONTROL_SESSION_STATE_REVOKING; }, 2s));
    EXPECT_FALSE(deadline.read(&opened));
    EXPECT_EQ(deadline.finish().error_code(), grpc::StatusCode::DEADLINE_EXCEEDED);
    ASSERT_TRUE(release_revoked());
  }
  ASSERT_TRUE(await([this] { return server_->runtime().active_calls == 5; }));
  {
    const auto baseline = server_->runtime().active_calls;
    const auto open_budget = std::chrono::milliseconds(config_.control_open_timeout_ms);
    ControlStream unopened(*stub_, open_budget + 8s);
    ASSERT_TRUE(await([&] { return server_->runtime().active_calls > baseline; }));
    ASSERT_TRUE(await([&] { return server_->runtime().active_calls == baseline; }, open_budget + 2s));
    wire::OpenSessionResponse response;
    EXPECT_FALSE(unopened.read(&response));
    const auto status = unopened.finish();
    EXPECT_TRUE(status.error_code() == grpc::StatusCode::CANCELLED ||
                status.error_code() == grpc::StatusCode::DEADLINE_EXCEEDED);
    EXPECT_EQ(ingress_->snapshot().state, wire::CONTROL_SESSION_STATE_IDLE);
    EXPECT_TRUE(ingress_->snapshot().session.session_token().empty());
    ASSERT_TRUE(await([this] { return server_->runtime().active_calls == 5; }));
  }
}

TEST_F(GrpcLifecycle, EmptyAcceptorsAndPendingControlWritesStopAndDrain) {
  server_->stop();
  expect_drained();
  const auto empty_stop = server_->runtime();
  EXPECT_GE(empty_stop.unstarted_requests, 5U);
  server_->stop();
  EXPECT_EQ(server_->runtime().completed_tags, empty_stop.completed_tags);
  EXPECT_EQ(server_->runtime().unstarted_requests, empty_stop.unstarted_requests);

  start_fresh();
  if (HasFatalFailure()) return;
  ControlStream holder(*stub_, 15s);
  const auto request = open_request('g');
  ASSERT_TRUE(holder.write(request));
  wire::OpenSessionResponse opened;
  ASSERT_TRUE(holder.read(&opened));
  const auto epoch = ingress_->snapshot().epoch;
  ASSERT_TRUE(admit(epoch));
  const auto baseline = server_->runtime().active_calls;
  ControlStream pending_read(*stub_, 15s);
  ASSERT_TRUE(await([&] { return server_->runtime().active_calls > baseline; }));
  const auto drops = server_->runtime().dropped_updates;
  OpenBurst burst(holder, request);
  ASSERT_TRUE(await([&] { return server_->runtime().dropped_updates > drops; }, 8s));
  EXPECT_GT(burst.sent(), 1U);
  // 不读控制回复，实际阻塞 Write 与另一 RPC 的 pending Read 同时存在才停服。
  server_->stop();
  burst.stop();
  expect_drained();
  EXPECT_EQ(ingress_->snapshot().state, wire::CONTROL_SESSION_STATE_REVOKING);
  EXPECT_FALSE(ingress_->publish_pending(epoch, network::boottime_now_ns(),
                                        [](const auto &) { ADD_FAILURE() << "stopped server published"; }));
  EXPECT_FALSE(holder.finish().ok());
  EXPECT_FALSE(pending_read.finish().ok());
  const auto after_stop = server_->runtime();
  server_->stop();
  EXPECT_EQ(server_->runtime().completed_tags, after_stop.completed_tags);
  EXPECT_EQ(server_->runtime().unresolved_calls, 0U);
}

TEST_F(GrpcLifecycle, SlowReadonlyIsBoundedAndCannotBlockControlOrShutdown) {
  ControlStream holder(*stub_, 20s);
  const auto request = open_request('h');
  ASSERT_TRUE(holder.write(request));
  wire::OpenSessionResponse opened;
  ASSERT_TRUE(holder.read(&opened));
  const auto epoch = ingress_->snapshot().epoch;
  ASSERT_TRUE(admit(epoch));
  ingress_->update_observations([this](auto &status) {
    auto *alert = status.add_alerts();
    alert->set_name("bounded slow-reader fixture");
    alert->set_message(std::string(config_.max_message_bytes - 1024, 's'));
  });
  ASSERT_LT(ingress_->readonly_status().ByteSizeLong() + 128, config_.max_message_bytes);
  StatusStream slow(*stub_, 20s);
  wire::StatusEnvelope observed;
  ASSERT_TRUE(slow.read(&observed));
  const auto drops = server_->runtime().dropped_updates;
  ASSERT_TRUE(await([&] { return server_->runtime().dropped_updates > drops; }, 8s));

  wire::SubmitReceipt receipt;
  ASSERT_TRUE(submit(target_request(opened.session().session_token(), 1), &receipt).ok());
  ASSERT_EQ(receipt.forwarding_state(), wire::FORWARDING_STATE_PENDING);
  std::uint64_t forwarded_sequence = 0;
  ASSERT_TRUE(ingress_->publish_pending(epoch, network::boottime_now_ns(),
                                       [&](const auto &frame) { forwarded_sequence = frame.command_sequence; }));
  EXPECT_EQ(forwarded_sequence, 1U);
  ASSERT_TRUE(holder.write(keep_request(request, 1)));
  ASSERT_TRUE(holder.read(&opened));
  EXPECT_TRUE(opened.session().submission_ready());
  EXPECT_EQ(ingress_->snapshot().epoch, epoch);
  ASSERT_TRUE(get_status(&observed).ok());
  EXPECT_GE(expect_redacted(observed), 2U);
  StatusStream healthy(*stub_, 10s);
  ASSERT_TRUE(healthy.read(&observed));
  EXPECT_EQ(observed.status().session().control_session_state(), wire::CONTROL_SESSION_STATE_ADMITTED);
  healthy.cancel();
  EXPECT_EQ(healthy.finish().error_code(), grpc::StatusCode::CANCELLED);
  ASSERT_TRUE(submit(target_request(opened.session().session_token(), 2), &receipt).ok());
  EXPECT_EQ(receipt.forwarding_state(), wire::FORWARDING_STATE_PENDING);
  EXPECT_EQ(ingress_->snapshot().epoch, epoch);
  EXPECT_GT(server_->runtime().dropped_updates, drops);
  EXPECT_LE(server_->runtime().peak_calls, std::uint64_t{4} * config_.readonly_inflight_limit +
                                          config_.max_status_subscribers + 5);

  server_->stop();
  expect_drained();
  EXPECT_FALSE(slow.finish().ok());
  EXPECT_FALSE(holder.finish().ok());
}

}
}
