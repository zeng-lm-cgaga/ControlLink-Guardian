#include "control_link_network_ingress/schema.hpp"

#include <cstddef>
#include <stdexcept>

#ifndef CONTROL_LINK_NETWORK_SCHEMA_SHA256_HEX
#error "The network ingress target must define CONTROL_LINK_NETWORK_SCHEMA_SHA256_HEX from its .proto input"
#endif

namespace control_link::network {

std::string schema_fingerprint() {
  const std::string hex = CONTROL_LINK_NETWORK_SCHEMA_SHA256_HEX;
  if (hex.size() != 64) throw std::runtime_error("network schema SHA256 must contain 64 hex digits");
  const auto decode = [](char value) -> unsigned {
    if (value >= '0' && value <= '9') return static_cast<unsigned>(value - '0');
    if (value >= 'a' && value <= 'f') return static_cast<unsigned>(value - 'a' + 10);
    if (value >= 'A' && value <= 'F') return static_cast<unsigned>(value - 'A' + 10);
    throw std::runtime_error("network schema SHA256 contains a non-hex digit");
  };
  std::string fingerprint;
  fingerprint.reserve(32);
  for (std::size_t offset = 0; offset < hex.size(); offset += 2) {
    fingerprint.push_back(static_cast<char>((decode(hex[offset]) << 4) | decode(hex[offset + 1])));
  }
  return fingerprint;
}

}
