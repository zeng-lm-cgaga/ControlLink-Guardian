cmake_minimum_required(VERSION 3.16)
project(control_link_network_probe LANGUAGES CXX)

find_package(control_link_someip_transport CONFIG REQUIRED)
find_package(vsomeip3 3.4.10 EXACT CONFIG REQUIRED)
find_package(OpenSSL REQUIRED COMPONENTS Crypto)
find_package(Threads REQUIRED)

add_executable(network_session_probe main.cpp)
target_compile_features(network_session_probe PRIVATE cxx_std_17)
target_compile_options(network_session_probe PRIVATE -Wall -Wextra -Wpedantic)
target_link_libraries(network_session_probe PRIVATE
  ControlLink::control_link_someip_transport vsomeip3 OpenSSL::Crypto Threads::Threads)
install(TARGETS network_session_probe RUNTIME DESTINATION bin)
