cmake_minimum_required(VERSION 3.16)
project(grpc_planning_consumer LANGUAGES CXX)

find_package(control_link_grpc_transport CONFIG REQUIRED)

add_executable(grpc_planning_consumer main.cpp)
target_compile_features(grpc_planning_consumer PRIVATE cxx_std_17)
target_compile_options(grpc_planning_consumer PRIVATE -Wall -Wextra -Wpedantic)
target_link_libraries(grpc_planning_consumer PRIVATE ControlLink::control_link_grpc_transport)
install(TARGETS grpc_planning_consumer RUNTIME DESTINATION bin)
