cmake_minimum_required(VERSION 3.16)
project(control_link_someip_capacity_probe LANGUAGES CXX)

find_package(control_link_someip_transport CONFIG REQUIRED)
find_package(Threads REQUIRED)

add_executable(someip_capacity_probe main.cpp)
target_compile_features(someip_capacity_probe PRIVATE cxx_std_17)
target_compile_options(someip_capacity_probe PRIVATE -Wall -Wextra -Wpedantic)
target_link_libraries(someip_capacity_probe PRIVATE
  ControlLink::control_link_someip_transport Threads::Threads)
install(TARGETS someip_capacity_probe RUNTIME DESTINATION bin)
