#include "control_link_someip_transport/client.hpp"
#include "control_link_someip_transport/server.hpp"

#include <pthread.h>
#include <unistd.h>

#include <algorithm>
#include <cerrno>
#include <chrono>
#include <csignal>
#include <cstdint>
#include <filesystem>
#include <iostream>
#include <memory>
#include <stdexcept>
#include <string>
#include <thread>

namespace {

namespace net = control_link::someip;
namespace wire = net::wire;
using Clock = std::chrono::steady_clock;
using Milliseconds = std::chrono::milliseconds;

void require(bool condition, const std::string &reason) {
  if (!condition) throw std::runtime_error(reason);
}

struct Options {
  std::string role;
  std::string config_path;
  std::string json_path;
  std::string application;
  Milliseconds timeout{60000};
  std::uint32_t rotations{128};
};

Options parse_options(int argc, char **argv) {
  if (argc < 2) throw std::invalid_argument("Expected fixture or consumer role");
  Options options;
  options.role = argv[1];
  if (options.role != "fixture" && options.role != "consumer") {
    throw std::invalid_argument("Role must be fixture or consumer");
  }
  for (int index = 2; index < argc; ++index) {
    const std::string name(argv[index]);
    if (index + 1 == argc) throw std::invalid_argument("Missing value for " + name);
    const std::string value(argv[++index]);
    if (name == "--config") options.config_path = value;
    else if (name == "--vsomeip-config") options.json_path = value;
    else if (name == "--application") options.application = value;
    else if (name == "--timeout-ms") {
      if (value.empty() || value.front() == '-') throw std::invalid_argument("Invalid timeout");
      std::size_t consumed = 0;
      const auto number = std::stoull(value, &consumed);
      if (consumed != value.size() || number == 0 || number > 60000) {
        throw std::invalid_argument("Timeout must be 1..60000ms");
      }
      options.timeout = Milliseconds{number};
    } else if (name == "--rotations") {
      if (value.empty() || value.front() == '-') throw std::invalid_argument("Invalid rotations");
      std::size_t consumed = 0;
      const auto number = std::stoull(value, &consumed);
      if (consumed != value.size() || number == 0 || number > 256) {
        throw std::invalid_argument("Rotations must be 1..256");
      }
      options.rotations = static_cast<std::uint32_t>(number);
    } else throw std::invalid_argument("Unknown argument " + name);
  }
  if (options.config_path.empty() || options.json_path.empty() ||
      (options.role == "consumer" && options.application.empty()) ||
      (options.role == "fixture" && !options.application.empty())) {
    throw std::invalid_argument("Explicit YAML/JSON required; only consumer takes --application");
  }
  return options;
}

// fixture 启动线程前屏蔽停止信号；主线程等待信号后停止生产server并join，不异步销毁资源。
class StopSignalWait final {
 public:
  StopSignalWait() {
    if (::sigemptyset(&signals_) != 0 || ::sigaddset(&signals_, SIGINT) != 0 ||
        ::sigaddset(&signals_, SIGTERM) != 0 ||
        ::pthread_sigmask(SIG_BLOCK, &signals_, &previous_) != 0) {
      throw std::runtime_error("Could not configure cooperative fixture stop");
    }
  }
  ~StopSignalWait() { ::pthread_sigmask(SIG_SETMASK, &previous_, nullptr); }
  StopSignalWait(const StopSignalWait &) = delete;
  StopSignalWait &operator=(const StopSignalWait &) = delete;

  int wait() {
    const timespec timeout{0, 5000000};
    const auto received = ::sigtimedwait(&signals_, nullptr, &timeout);
    if (received >= 0) return received;
    if (errno == EAGAIN || errno == EINTR) return 0;
    throw std::runtime_error("Fixture stop wait failed");
  }

 private:
  sigset_t signals_{};
  sigset_t previous_{};
};

void log_identity(const Options &options, const net::Config &config) {
  std::cout << "role=" << options.role << " pid=" << ::getpid()
            << " exe=" << std::filesystem::read_symlink("/proc/self/exe").string()
            << " yaml_retired_capacity=" << config.retired_open_capacity
            << " timeout_ms=" << options.timeout.count()
            << " rotations=" << options.rotations
            << " scope=PROTOCOL_SDK_RESOURCE_ONLY no_ros=true no_gateway=true no_vcan=true\n";
}

std::string hex_bytes(const std::string &bytes) {
  constexpr char digits[] = "0123456789abcdef";
  std::string result;
  result.reserve(bytes.size() * 2);
  for (const unsigned char byte : bytes) {
    result.push_back(digits[byte >> 4]);
    result.push_back(digits[byte & 15]);
  }
  return result;
}

bool same_domain(const wire::OpenAdmissionDomain &left, const wire::OpenAdmissionDomain &right) {
  return left.domain_version() == 1 && right.domain_version() == 1 &&
      left.service_instance_id().size() == 16 &&
      left.service_instance_id() == right.service_instance_id() &&
      left.generation_presence_case() == wire::OpenAdmissionDomain::kGeneration &&
      right.generation_presence_case() == wire::OpenAdmissionDomain::kGeneration &&
      left.generation() > 0 && left.generation() == right.generation();
}

Milliseconds request_budget(Clock::time_point deadline, Milliseconds maximum = Milliseconds{500}) {
  const auto remaining = std::chrono::duration_cast<Milliseconds>(deadline - Clock::now());
  require(remaining > Milliseconds::zero(), "Probe outer deadline expired");
  return std::min(remaining, maximum);
}

void require_readonly(const wire::StatusSnapshot &snapshot,
                      const wire::OpenAdmissionDomain &domain) {
  require(snapshot.has_opening_domain() && same_domain(snapshot.opening_domain(), domain),
          "Readonly observation changed or omitted the opening domain");
  require(snapshot.has_session() && snapshot.session().has_opening_domain() &&
              same_domain(snapshot.session().opening_domain(), domain) &&
              snapshot.session().session_token().empty() &&
              !snapshot.session().submission_ready() && snapshot.session().ingress_publisher().gid().empty(),
          "Readonly observation leaked a token or pretended publisher admission");
  require(snapshot.latest_forwarded().key().session_token().empty() &&
              snapshot.latest_gateway_acceptance().key().session_token().empty(),
          "Readonly nested observation leaked a token");
  require(snapshot.has_transport() && snapshot.transport().forwarded_count() == 0,
          "Resource fixture unexpectedly forwarded business data");
}

void log_status(const std::string &phase, std::uint32_t iteration,
                const wire::StatusEnvelope &envelope) {
  std::cout << "client_status phase=" << phase << " iteration=" << iteration
            << " service_instance=" << hex_bytes(envelope.status().opening_domain().service_instance_id())
            << " domain=" << envelope.status().opening_domain().generation()
            << " publication_sequence=" << envelope.publication_sequence()
            << " decoded_envelope_proto_hex=" << hex_bytes(envelope.SerializeAsString()) << '\n';
}

void require_holder(const wire::OpenSessionResponse &response, wire::SessionOperationResult result,
                    const std::string &token, const wire::OpenAdmissionDomain &domain) {
  require(response.result() == result && response.has_session() &&
              response.session().session_token().size() == 16 && response.session().session_token() == token &&
              response.session().has_opening_domain() && same_domain(response.session().opening_domain(), domain) &&
              response.has_capabilities() && response.capabilities().has_opening_domain() &&
              same_domain(response.capabilities().opening_domain(), domain),
          "Holder reply changed the private token or bound domain");
  const auto received_state = response.session().control_session_state();
  std::cout << "client_holder_hint domain=" << domain.generation()
            << " result=" << response.result() << " received_session_state=" << received_state
            << " submission_ready=" << (response.session().submission_ready() ? "true" : "false")
            << " gid_bytes=" << response.session().ingress_publisher().gid().size()
            << " authority_claim=false\n";
  require((received_state == wire::CONTROL_SESSION_STATE_OPENING ||
           received_state == wire::CONTROL_SESSION_STATE_UNKNOWN) &&
              !response.session().submission_ready() && response.session().ingress_publisher().gid().empty(),
          "Holder hint must be OPENING or fail-closed UNKNOWN without readiness/GID; GetStatus confirms server state");
}

void observe_stream_event(net::SomeipClient &client, const net::Config &config,
                          const wire::OpenSessionRequest &request, const std::string &token,
                          std::uint32_t iteration, Clock::time_point deadline) {
  const auto previous_received = client.status().gateway_status_received_steady_ns;
  const auto event_budget = Milliseconds{static_cast<std::int64_t>(config.status_event_period_ms) * 3 + 100};
  const auto event_deadline = std::min(deadline, Clock::now() + event_budget);
  auto next_keep = Clock::now() + Milliseconds{config.keep_period_ms};
  wire::KeepSessionRequest keep;
  keep.set_schema_version(1);
  keep.set_holder_nonce(request.holder_nonce());
  keep.set_open_request_id(request.open_request_id());
  *keep.mutable_opening_domain() = request.opening_domain();
  std::uint64_t keep_sequence = 1;
  while (Clock::now() < event_deadline) {
    const auto state = client.status();
    if (state.gateway_status_received_steady_ns > previous_received) {
      require_readonly(state.observations, request.opening_domain());
      require(state.observations.session().control_session_state() == wire::CONTROL_SESSION_STATE_OPENING &&
                  state.session.session_token() == token && !state.session.submission_ready(),
              "Readonly event changed the live holder or fabricated readiness");
      std::cout << "client_event iteration=" << iteration
                << " domain=" << state.observations.opening_domain().generation()
                << " received_steady_ns=" << state.gateway_status_received_steady_ns
                << " decoded_snapshot_proto_hex=" << hex_bytes(state.observations.SerializeAsString())
                << " capture=SDK_DECODED_LATEST_EVENT_SNAPSHOT not_wire_envelope=true\n";
      return;
    }
    if (Clock::now() >= next_keep) {
      keep.set_keepalive_sequence(++keep_sequence);
      require_holder(client.keep(keep, request_budget(deadline)), wire::SESSION_OPERATION_RESULT_OPENING,
                     token, request.opening_domain());
      next_keep = Clock::now() + Milliseconds{config.keep_period_ms};
    }
    std::this_thread::sleep_for(Milliseconds{1});
  }
  throw std::runtime_error("No production readonly status event observed without GetStatus polling");
}

int run_fixture(const Options &options, net::Config config) {
  StopSignalWait stop;
  // 只改变此测试对象；不写YAML默认64，也不伪造publisher/Gateway admission。
  config.retired_open_capacity = 1;
  wire::Capabilities capabilities;
  capabilities.set_contract_id("capacity-fixture-no-gateway");
  capabilities.set_planning_transport(wire::PLANNING_TRANSPORT_SOMEIP);
  capabilities.set_max_command_bytes(control_link_edge_protocol::kCommandFrameSize);
  capabilities.set_max_status_bytes(config.max_payload_bytes);
  capabilities.set_max_pending_targets(1);
  capabilities.set_max_status_subscribers(config.readonly_inflight_limit);
  auto ingress = std::make_shared<net::PlanningIngress>(config, capabilities);
  net::SomeipServer server(config, ingress, options.json_path);
  server.start();
  std::cout << "fixture_started=true retired_capacity=1 rotations=" << options.rotations
            << " submission_ready=false\n";
  const auto deadline = Clock::now() + options.timeout;
  std::uint64_t holder_releases = 0;
  int stop_signal = 0;
  while (Clock::now() < deadline) {
    const auto snapshot = ingress->snapshot();
    require(snapshot.retired_open_count <= 1 && !snapshot.session.submission_ready() &&
                snapshot.session.ingress_publisher().gid().empty() &&
                snapshot.state != wire::CONTROL_SESSION_STATE_ADMITTED,
            "Fixture exceeded bounded storage or unexpectedly admitted a publisher");
    if (snapshot.state == wire::CONTROL_SESSION_STATE_REVOKING &&
        ingress->finish_release(snapshot.epoch)) {
      if (!snapshot.session.session_token().empty()) ++holder_releases;
      const auto released = ingress->snapshot();
      require(released.retired_open_count == 0 &&
                  released.opening_domain.service_instance_id() == snapshot.opening_domain.service_instance_id() &&
                  released.opening_domain.generation() > snapshot.opening_domain.generation() &&
                  released.opening_domain.generation() - snapshot.opening_domain.generation() == 1,
              "Successful release did not advance the domain before bounded reclamation");
      std::cout << "finish_release=true epoch=" << snapshot.epoch
                << " iteration=" << holder_releases << " holder_releases=" << holder_releases
                << " old_domain=" << snapshot.opening_domain.generation()
                << " domain=" << released.opening_domain.generation()
                << " service_instance=" << hex_bytes(released.opening_domain.service_instance_id())
                << " retired_count_before=" << snapshot.retired_open_count
                << " retired_count=" << released.retired_open_count << '\n';
    }
    stop_signal = stop.wait();
    if (stop_signal != 0) break;
  }
  server.stop();
  const auto closing = ingress->snapshot();
  if (closing.state == wire::CONTROL_SESSION_STATE_REVOKING) {
    require(ingress->finish_release(closing.epoch), "Final resource release failed");
    if (!closing.session.session_token().empty()) ++holder_releases;
  }
  const auto final_snapshot = ingress->snapshot();
  require(final_snapshot.retired_open_count <= 1 && ingress->readonly_status().transport().forwarded_count() == 0,
          "Stopped fixture exceeded storage or forwarded business data");
  std::cout << "fixture_stopped=true requested_signal=" << stop_signal
            << " holder_releases=" << holder_releases
            << " domain=" << final_snapshot.opening_domain.generation()
            << " retired_count=" << final_snapshot.retired_open_count << " forwarded_count=0\n";
  require(stop_signal == SIGINT, "Fixture requires a cooperative SIGINT stop, not timeout or another signal");
  require(holder_releases >= options.rotations, "Fixture did not observe all requested holder releases");
  return 0;
}

wire::OpenSessionRequest fresh_request(const wire::OpenAdmissionDomain &domain) {
  wire::OpenSessionRequest request;
  request.set_schema_version(1);
  request.set_holder_nonce(net::random_identity());
  request.set_open_request_id(net::random_identity());
  *request.mutable_opening_domain() = domain;
  return request;
}

int run_consumer(const Options &options, const net::Config &config) {
  const auto deadline = Clock::now() + options.timeout;
  net::SomeipClient client(config, options.json_path, options.application);
  client.start();
  require(client.wait_available(request_budget(deadline, options.timeout)),
          "SOME/IP service discovery did not become available");
  std::string original_bytes;
  std::string original_token;
  wire::OpenAdmissionDomain released_domain;
  for (std::uint32_t iteration = 1; iteration <= options.rotations; ++iteration) {
    const auto discovered = client.get_status(request_budget(deadline));
    const auto domain = discovered.status().opening_domain();
    require_readonly(discovered.status(), domain);
    require(discovered.status().session().control_session_state() == wire::CONTROL_SESSION_STATE_IDLE,
            "New rotation did not start in IDLE");
    if (iteration == 1) require(domain.generation() == 1, "Fresh fixture must start in domain 1");
    else require(same_domain(domain, released_domain), "Readonly discovery unexpectedly rotated the domain");
    log_status("before_open", iteration, discovered);
    const auto request = fresh_request(domain);
    if (iteration == 1) original_bytes = request.SerializeAsString();
    const auto opened = client.open(request, request_budget(deadline));
    const auto token = opened.session().session_token();
    require_holder(opened, wire::SESSION_OPERATION_RESULT_OPENING, token, domain);
    if (iteration == 1) original_token = token;
    std::cout << "client_open iteration=" << iteration << " domain=" << domain.generation()
              << " request_proto_hex=" << hex_bytes(request.SerializeAsString())
              << " response_proto_hex=" << hex_bytes(opened.SerializeAsString()) << '\n';
    require_holder(client.open(request, request_budget(deadline)), wire::SESSION_OPERATION_RESULT_IDEMPOTENT,
                   token, domain);
    wire::KeepSessionRequest keep;
    keep.set_schema_version(1);
    keep.set_holder_nonce(request.holder_nonce());
    keep.set_open_request_id(request.open_request_id());
    *keep.mutable_opening_domain() = domain;
    keep.set_keepalive_sequence(1);
    require_holder(client.keep(keep, request_budget(deadline)), wire::SESSION_OPERATION_RESULT_OPENING, token, domain);
    const auto observation = client.get_status(request_budget(deadline));
    require_readonly(observation.status(), domain);
    require(observation.status().session().control_session_state() == wire::CONTROL_SESSION_STATE_OPENING,
            "Readonly query changed the live holder state");
    log_status("opening_readonly", iteration, observation);
    if (iteration == 1 || iteration == options.rotations) {
      if (iteration == 1) {
        client.set_status_subscription(false);
        client.set_status_subscription(true);
      }
      observe_stream_event(client, config, request, token, iteration, deadline);
      require_holder(client.open(request, request_budget(deadline)), wire::SESSION_OPERATION_RESULT_IDEMPOTENT,
                     token, domain);
    }
    wire::CloseSessionRequest close;
    close.set_schema_version(1);
    close.set_holder_nonce(request.holder_nonce());
    close.set_open_request_id(request.open_request_id());
    *close.mutable_opening_domain() = domain;
    const auto closed = client.close(close, request_budget(deadline));
    require(closed.result() == wire::SESSION_OPERATION_RESULT_REVOKED && closed.has_session() &&
                closed.session().session_token() == token && closed.session().has_opening_domain() &&
                same_domain(closed.session().opening_domain(), domain) && !closed.session().submission_ready(),
            "Matching production CloseSession did not revoke the bound holder");
    require(client.status().session.session_token().empty(), "Close retained a cached control token");
    bool released = false;
    while (Clock::now() < deadline) {
      const auto envelope = client.get_status(request_budget(deadline));
      if (envelope.status().has_session() &&
          envelope.status().session().control_session_state() == wire::CONTROL_SESSION_STATE_IDLE) {
        released_domain = envelope.status().opening_domain();
        require_readonly(envelope.status(), released_domain);
        require(released_domain.service_instance_id() == domain.service_instance_id() &&
                    released_domain.generation() > domain.generation() &&
                    released_domain.generation() - domain.generation() == 1,
                "Release did not advance exactly one domain in the same service instance");
        log_status("after_release", iteration, envelope);
        released = true;
        break;
      }
      std::this_thread::sleep_for(Milliseconds{1});
    }
    require(released, "Fixture finish_release was not observed through production GetStatus");
    wire::OpenSessionRequest replay;
    require(replay.ParseFromString(original_bytes) && replay.SerializeAsString() == original_bytes,
            "Original serialized OpenSession request was not preserved");
    const auto retired = client.open(replay, request_budget(deadline));
    require(retired.result() == wire::SESSION_OPERATION_RESULT_REJECTED && !retired.has_session() &&
                retired.failure_reason() == "OPEN_ADMISSION_DOMAIN_REQUIRED_OR_STALE",
            "Original stale-domain request revived or lacked the explicit rejection reason");
    const auto retired_bytes = retired.SerializeAsString();
    require(retired_bytes.find(original_token) == std::string::npos &&
                retired_bytes.find(token) == std::string::npos &&
                retired.has_capabilities() && same_domain(retired.capabilities().opening_domain(), released_domain) &&
                client.status().session.session_token().empty(),
            "Stale request refusal exposed/cached a control token or changed the domain");
    std::cout << "client_replay iteration=" << iteration << " old_domain=" << replay.opening_domain().generation()
              << " current_domain=" << released_domain.generation() << " result=" << retired.result()
              << " failure_reason=" << retired.failure_reason() << " has_session=false revived=false"
              << " original_request_proto_hex=" << hex_bytes(original_bytes)
              << " response_proto_hex=" << hex_bytes(retired_bytes) << '\n';
  }
  const auto final_status = client.get_status(request_budget(deadline));
  require_readonly(final_status.status(), released_domain);
  require(final_status.status().session().control_session_state() == wire::CONTROL_SESSION_STATE_IDLE,
          "Final rotation did not leave the fixture IDLE");
  log_status("final", options.rotations, final_status);
  client.stop();
  std::cout << "PASS rotations=" << options.rotations << " forwarded_count=0"
            << " elapsed_ms=" << std::chrono::duration_cast<Milliseconds>(Clock::now() - (deadline - options.timeout)).count()
            << " scope=PRODUCTION_CROSSPROCESS_PROTOCOL_RESOURCE_ONLY"
            << " no_business_acceptance_claim=true no_full_sd_tcp_claim=true\n";
  return 0;
}

}

int main(int argc, char **argv) {
  std::cout.setf(std::ios::unitbuf);
  try {
    const auto options = parse_options(argc, argv);
    const auto config = net::load_config(options.config_path);
    log_identity(options, config);
    const auto result = options.role == "fixture" ? run_fixture(options, config) : run_consumer(options, config);
    std::cout.flush();
    std::cerr.flush();
    require(static_cast<bool>(std::cout) && static_cast<bool>(std::cerr), "Probe log flush failed");
    return result;
  } catch (const std::exception &error) {
    std::cerr << "capacity_probe failure: " << error.what() << '\n';
    return 1;
  }
}
