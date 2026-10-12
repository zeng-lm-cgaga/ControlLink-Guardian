cmake_minimum_required(VERSION 3.16)
project(control_link_grpc_evidence LANGUAGES CXX)

find_package(control_link_grpc_transport CONFIG REQUIRED)

add_executable(grpc_protocol_probe probe.cpp)
target_compile_features(grpc_protocol_probe PRIVATE cxx_std_17)
target_compile_options(grpc_protocol_probe PRIVATE -Wall -Wextra -Wpedantic)
target_link_libraries(grpc_protocol_probe PRIVATE ControlLink::control_link_grpc_transport)
install(TARGETS grpc_protocol_probe RUNTIME DESTINATION bin)
