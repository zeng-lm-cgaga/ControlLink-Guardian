cmake_minimum_required(VERSION 3.16)
project(control_link_someip_focused_evidence LANGUAGES CXX)
set(CONTROL_LINK_REUSED_SOMEIP_BUILD "" CACHE PATH "Reviewed native static library build")
find_package(vsomeip3 3.4.10 EXACT CONFIG REQUIRED)
find_package(Protobuf 3.12.4 EXACT REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(Threads REQUIRED)
get_filename_component(CONTROL_LINK_ROOT ../.. ABSOLUTE)
add_executable(rate_limit_probe rate_limit_probe.cpp)
target_compile_features(rate_limit_probe PRIVATE cxx_std_17)
target_compile_options(rate_limit_probe PRIVATE -Wall -Wextra -Wpedantic)
target_include_directories(rate_limit_probe PRIVATE
  ${CONTROL_LINK_ROOT}/someip_transport/include ${CONTROL_LINK_ROOT}/network_ingress/include
  ${CONTROL_LINK_ROOT}/edge_protocol/include ${CONTROL_LINK_REUSED_SOMEIP_BUILD}/network_ingress/generated)
target_link_libraries(rate_limit_probe PRIVATE
  ${CONTROL_LINK_REUSED_SOMEIP_BUILD}/libcontrol_link_someip_transport.a
  ${CONTROL_LINK_REUSED_SOMEIP_BUILD}/network_ingress/libcontrol_link_network_ingress.a
  ${CONTROL_LINK_REUSED_SOMEIP_BUILD}/network_ingress/edge_protocol/libcontrol_link_edge_protocol.a
  vsomeip3 protobuf::libprotobuf yaml-cpp Threads::Threads)
