cmake_minimum_required(VERSION 3.16)
project(control_link_protocol_dependency_probe LANGUAGES C CXX)

include("${CMAKE_CURRENT_LIST_DIR}/../../ros2_ws/src/control_link_adapters/cmake/ControlLinkNetworkDependencies.cmake")
control_link_probe_network_dependencies()
