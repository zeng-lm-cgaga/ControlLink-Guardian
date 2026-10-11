#include "control_link_someip_transport/client.hpp"
#include "control_link_someip_transport/server.hpp"

#include <pthread.h>
#include <unistd.h>

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
  Milliseconds timeout{10000};
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
            << " scope=PROTOCOL_SDK_RESOURCE_ONLY no_ros=true no_gateway=true no_vcan=true\n";
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
  std::cout << "fixture_started=true retired_capacity=1 submission_ready=false\n";
  const auto deadline = Clock::now() + options.timeout;
  std::uint64_t holder_releases = 0;
  int stop_signal = 0;
  while (Clock::now() < deadline) {
    const auto snapshot = ingress->snapshot();
    if (snapshot.state == wire::CONTROL_SESSION_STATE_REVOKING &&
        ingress->finish_release(snapshot.epoch)) {
      if (!snapshot.session.session_token().empty()) ++holder_releases;
      std::cout << "finish_release=true epoch=" << snapshot.epoch
                << " holder_releases=" << holder_releases << '\n';
    }
    stop_signal = stop.wait();
    if (stop_signal != 0) break;
  }
  server.stop();
  const auto closing = ingress->snapshot();
  if (closing.state == wire::CONTROL_SESSION_STATE_REVOKING) {
    require(ingress->finish_release(closing.epoch), "Final resource release failed");
  }
  std::cout << "fixture_stopped=true requested_signal=" << stop_signal
            << " holder_releases=" << holder_releases << '\n';
  require(stop_signal != 0, "Fixture timeout is not a cooperative passing stop");
  require(holder_releases == 1, "Expected one observed holder retirement");
  return 0;
}

wire::OpenSessionRequest fresh_request() {
  wire::OpenSessionRequest request;
  request.set_schema_version(1);
  request.set_holder_nonce(net::random_identity());
  request.set_open_request_id(net::random_identity());
  return request;
}

int run_consumer(const Options &options, const net::Config &config) {
  net::SomeipClient client(config, options.json_path, options.application);
  client.start();
  require(client.wait_available(options.timeout), "SOME/IP service discovery did not become available");
  const auto original = fresh_request();
  const auto opened = client.open(original);
  require(opened.result() == wire::SESSION_OPERATION_RESULT_OPENING && opened.has_session(),
          "Initial production OpenSession did not create the fixture holder");
  const auto original_token = opened.session().session_token();
  require(original_token.size() == 16 && !opened.session().submission_ready() &&
              opened.session().ingress_publisher().gid().empty(),
          "Fixture must stay OPENING without pretending Gateway admission");
  std::cout << "open_result=" << opened.result() << " token_bytes=" << original_token.size()
            << " submission_ready=false\n";
  wire::CloseSessionRequest close;
  close.set_schema_version(1);
  close.set_holder_nonce(original.holder_nonce());
  close.set_open_request_id(original.open_request_id());
  const auto closed = client.close(close);
  require(closed.result() == wire::SESSION_OPERATION_RESULT_REVOKED,
          "Matching production CloseSession did not revoke the holder");
  require(client.status().session.session_token().empty(), "Close retained a cached control token");
  const auto release_deadline = Clock::now() + options.timeout;
  bool released = false;
  while (Clock::now() < release_deadline) {
    const auto envelope = client.get_status();
    if (envelope.status().has_session() &&
        envelope.status().session().control_session_state() == wire::CONTROL_SESSION_STATE_IDLE) {
      require(envelope.status().session().session_token().empty() &&
                  !envelope.status().session().submission_ready(),
              "Readonly release observation leaked a token or readiness");
      released = true;
      break;
    }
    std::this_thread::sleep_for(Milliseconds{10});
  }
  require(released, "Fixture finish_release was not observed through production GetStatus");
  auto replacement = fresh_request();
  require(replacement.holder_nonce() != original.holder_nonce() ||
              replacement.open_request_id() != original.open_request_id(), "Fixture keys are not distinct");
  const auto exhausted = client.open(replacement);
  require(exhausted.result() == wire::SESSION_OPERATION_RESULT_RESOURCE_EXHAUSTED &&
              !exhausted.has_session(), "Expected structured RESOURCE_EXHAUSTED without a session");
  require(exhausted.open_request_id() == replacement.open_request_id() &&
              exhausted.SerializeAsString().find(original_token) == std::string::npos &&
              client.status().session.session_token().empty(),
          "Resource refusal leaked or cached the retired holder token");
  std::cout << "new_key_result=" << exhausted.result() << " has_session=false sdk_threw=false\n";
  const auto retired = client.open(original);
  require(retired.result() == wire::SESSION_OPERATION_RESULT_REJECTED && !retired.has_session(),
          "Retired original key revived or was not specifically rejected");
  require(retired.SerializeAsString().find(original_token) == std::string::npos &&
              client.status().session.session_token().empty(), "Retired-key refusal exposed a control token");
  std::cout << "original_key_result=" << retired.result() << " has_session=false revived=false\n";
  const auto final_status = client.get_status();
  require(final_status.status().session().control_session_state() == wire::CONTROL_SESSION_STATE_IDLE &&
              !final_status.status().session().submission_ready() &&
              final_status.status().transport().forwarded_count() == 0,
          "Resource-only fixture unexpectedly became live or forwarded business data");
  client.stop();
  std::cout << "PASS scope=PROTOCOL_SDK_RESOURCE_ONLY no_business_acceptance_claim=true\n";
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
