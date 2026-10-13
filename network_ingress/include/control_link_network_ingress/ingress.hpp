#pragma once

#include "control_link_network_ingress/schema.hpp"
#include "control_link_someip_transport/ingress.hpp"

namespace control_link::network {

namespace wire = ::control_link::network::v1;

// 历史 C++ 名称保留源码接口归属；同一实现没有 vsomeip、SD、ROS 或 EdgeRuntime 依赖。
using PlanningIngress = ::control_link::someip::PlanningIngress;
using IngressResources = ::control_link::someip::IngressResources;
using IngressSnapshot = ::control_link::someip::IngressSnapshot;
using ::control_link::someip::required_protocol_payload_bytes;
using ::control_link::someip::steady_now_ns;
using ::control_link::someip::boottime_now_ns;
using ::control_link::someip::system_now_ns;
using ::control_link::someip::random_identity;
using ::control_link::someip::valid_opening_domain;
using ::control_link::someip::same_opening_domain;

}
