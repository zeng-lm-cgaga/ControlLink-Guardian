#include "control_link_someip_transport/client.hpp"
#include "control_link_someip_transport/server.hpp"
#include <array>
#include <chrono>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <thread>
#include <unistd.h>

namespace {
namespace net = control_link::someip;
namespace wire = control_link::network::v1;
namespace codec = control_link_edge_protocol;
using namespace std::chrono_literals;

void require(bool value, const std::string &message) {
  if (!value) throw std::runtime_error(message);
}

std::string hex(const std::string &bytes) {
  std::ostringstream out;
  for (unsigned char byte : bytes) out << std::hex << std::setw(2) << std::setfill('0') << unsigned(byte);
  return out.str();
}

const char *state_name(net::SubscriptionState state) {
  switch (state) {
    case net::SubscriptionState::Disabled: return "disabled";
    case net::SubscriptionState::Pending: return "pending";
    case net::SubscriptionState::Accepted: return "accepted";
    case net::SubscriptionState::Rejected: return "rejected";
    case net::SubscriptionState::TimedOut: return "timeout_unknown";
    case net::SubscriptionState::Unknown: return "unknown";
  }
  throw std::logic_error("unknown enum");
}

void observation(std::ostream &log, const char *label, const net::ClientStatus &status) {
  for (const auto &entry : {status.gateway_subscription, status.vehicle_subscription}) {
    log << "{\"monotonic_ns\":" << net::steady_now_ns() << ",\"event\":\"subscription\",\"label\":\"" << label
        << "\",\"service\":" << entry.service << ",\"instance\":" << entry.instance
        << ",\"eventgroup\":" << entry.eventgroup << ",\"notifier\":" << entry.event
        << ",\"state\":\"" << state_name(entry.state) << "\",\"request_generation\":" << entry.request_generation
        << ",\"availability_generation\":" << entry.availability_generation
        << ",\"requested_ns\":" << entry.requested_steady_ns << ",\"deadline_ns\":" << entry.deadline_steady_ns
        << ",\"callback_ns\":" << entry.callback_steady_ns << ",\"callback_observed\":" << entry.callback_observed
        << ",\"callback_attempt_unambiguous\":" << entry.callback_attempt_unambiguous
        << ",\"error\":" << entry.error << ",\"ignored_callbacks\":" << entry.ignored_callbacks
        << ",\"token\":\"" << hex(status.session.session_token()) << "\",\"gid\":\""
        << hex(status.session.ingress_publisher().gid()) << "\"}\n";
  }
  log.flush();
}
}

int main(int argc, char **argv) {
  try {
    require(argc == 6, "usage: subscription_probe server-ack|server-nack|client-ack|client-nack YAML JSON LOG DONE");
    const std::string role = argv[1], done = argv[5];
    const bool negative = role.find("nack") != std::string::npos;
    auto config = net::load_config(argv[2]);
    require(!std::filesystem::exists(argv[4]), "refuse existing log");
    std::ofstream log(argv[4]);
    log.exceptions(std::ios::badbit | std::ios::failbit);
    log << "{\"event\":\"identity\",\"monotonic_ns\":" << net::steady_now_ns() << ",\"pid\":" << ::getpid()
        << ",\"scope\":\"synthetic GID; ROS-free TCP/SD only\"}\n";
    log.flush();
    if (role == "server-ack" || role == "server-nack") {
      wire::Capabilities capabilities;
      capabilities.set_schema_version(1);
      capabilities.set_contract_id("subscription-observation-fixture-not-gateway");
      capabilities.set_planning_transport(wire::PLANNING_TRANSPORT_SOMEIP);
      auto ingress = std::make_shared<net::PlanningIngress>(config, capabilities);
      net::SomeipServer server(config, ingress, argv[3]);
      server.start();
      const auto deadline = std::chrono::steady_clock::now() + 15s;
      unsigned forwarded = 0;
      while (!std::filesystem::exists(done) && std::chrono::steady_clock::now() < deadline) {
        const auto state = ingress->snapshot();
        if (state.state == wire::CONTROL_SESSION_STATE_OPENING) {
          require(ingress->bind_publisher(state.epoch, std::string(24, 'g'), "fixture-no-ROS"), "bind synthetic publisher");
          require(ingress->set_admitted(state.epoch, true), "admit synthetic publisher");
        }
        ingress->publish_pending(state.epoch, net::boottime_now_ns(), [&](const codec::CommandFrame &frame) {
          ++forwarded;
          log << "{\"event\":\"forwarded\",\"monotonic_ns\":" << net::steady_now_ns()
              << ",\"sequence\":" << frame.command_sequence << ",\"stamp_ns\":" << frame.source_stamp_ns << "}\n";
          log.flush();
        });
        std::this_thread::sleep_for(2ms);
      }
      require(std::filesystem::exists(done), "client completion deadline");
      const auto counters = server.rate_limits();
      server.stop();
      ingress->stop();
      const auto subscribe = counters[static_cast<std::size_t>(control_link::network::IngressMethod::Subscribe)];
      log << "{\"event\":\"server_final\",\"monotonic_ns\":" << net::steady_now_ns()
          << ",\"forwarded\":" << forwarded << ",\"subscribe_limited\":" << subscribe.caller_limited << "}\n";
      require(forwarded == 1 && (negative ? subscribe.caller_limited >= 1 : subscribe.caller_limited == 0),
          "actual admission counters disagree with case");
    } else {
      require(role == "client-ack" || role == "client-nack", "invalid role");
      net::SomeipClient client(config, argv[3], "rate_limit_peer");
      client.start();
      client.set_status_subscription(false);
      require(client.wait_available(4s), "service availability");
      observation(log, "initial", client.status());
      std::this_thread::sleep_for(250ms);
      const auto discovered = client.get_status();
      wire::OpenSessionRequest open;
      open.set_schema_version(1);
      open.set_holder_nonce(std::string(16, 'h'));
      open.set_open_request_id(std::string(16, 'r'));
      *open.mutable_opening_domain() = discovered.status().opening_domain();
      auto session = client.open(open).session();
      std::this_thread::sleep_for(100ms);
      wire::KeepSessionRequest keep;
      keep.set_schema_version(1);
      keep.set_holder_nonce(open.holder_nonce());
      keep.set_open_request_id(open.open_request_id());
      *keep.mutable_opening_domain() = open.opening_domain();
      std::uint64_t keep_sequence = 0;
      auto maintain = [&] {
        keep.set_keepalive_sequence(++keep_sequence);
        const auto maintained = client.keep(keep).session();
        require(maintained.submission_ready(), "Keep remains available");
        if (session.submission_ready()) {
          require(maintained.session_token() == session.session_token() &&
              maintained.ingress_publisher().gid() == session.ingress_publisher().gid(), "readonly must not rotate holder");
        }
        session = maintained;
      };
      maintain();
      codec::CommandFrame frame;
      require(session.session_token().size() == 16, "session token");
      std::memcpy(frame.session_token.bytes.data(), session.session_token().data(), 16);
      frame.command_sequence = 1;
      frame.source_stamp_ns = net::system_now_ns();
      frame.linear_velocity_mps = .3;
      client.submit(frame);
      client.set_status_subscription(true);
      auto wait_window = [&] {
        const auto end = std::chrono::steady_clock::now() + 2200ms;
        while (std::chrono::steady_clock::now() < end) {
          std::this_thread::sleep_for(10ms);
        }
      };
      wait_window();
      auto status = client.status();
      observation(log, "ack_and_timeout", status);
      if (negative) {
        require(status.vehicle_subscription.state == net::SubscriptionState::TimedOut &&
            !status.vehicle_subscription.callback_observed, "fixed routing-host NACK API gap expected, not invented rejection");
      } else {
        for (const auto &entry : {status.gateway_subscription, status.vehicle_subscription}) {
          require(entry.state == net::SubscriptionState::Accepted && entry.callback_observed && entry.error == 0,
              "both actual public ACK callbacks required in positive control");
        }
      }
      client.set_status_subscription(false);
      observation(log, "disabled", client.status());
      require(client.status().session.session_token() == session.session_token(), "disable preserved token");
      std::this_thread::sleep_for(250ms);
      client.get_status();
      maintain();
      observation(log, "fallback_query_keep", client.status());
      client.set_status_subscription(true);
      wait_window();
      status = client.status();
      observation(log, "reenabled", status);
      require(status.gateway_subscription.state != net::SubscriptionState::Accepted &&
          !status.gateway_subscription.callback_attempt_unambiguous,
          "re-registration callback must not claim unambiguous new ACK");
      client.set_status_subscription(false);
      std::this_thread::sleep_for(250ms);
      const auto queried = client.get_status();
      require(queried.status().transport().forwarded_count() == 1 &&
          queried.status().latest_forwarded().key().source_sequence() == 1, "readonly never republishes commands");
      maintain();
      require(client.status().session.session_token() == session.session_token(), "query/Keep preserved token");
      observation(log, "after_query_keep", client.status());
      wire::CloseSessionRequest close;
      close.set_schema_version(1);
      close.set_holder_nonce(open.holder_nonce());
      close.set_open_request_id(open.open_request_id());
      *close.mutable_opening_domain() = open.opening_domain();
      client.close(close);
      client.stop();
      client.stop();
      observation(log, "stopped", client.status());
      log << "{\"event\":\"client_final\",\"monotonic_ns\":" << net::steady_now_ns()
          << ",\"safe_observability\":true,\"strict_nack_callback_closed\":false,\"original_sequence\":1,\"original_stamp_ns\":"
          << frame.source_stamp_ns << "}\n";
      log.flush();
      std::ofstream(done) << "closed\n";
    }
    return 0;
  } catch (const std::exception &error) {
    std::cerr << "subscription observation fixture: " << error.what() << '\n';
    return 1;
  }
}
