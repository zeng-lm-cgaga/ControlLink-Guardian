cmake_minimum_required(VERSION 3.16)
project(control_link_edge_sdk_consumer LANGUAGES CXX)

find_package(control_link_edge_client CONFIG REQUIRED)

add_executable(edge_planning_sdk_consumer main.cpp)
target_compile_features(edge_planning_sdk_consumer PRIVATE cxx_std_17)
target_compile_options(edge_planning_sdk_consumer PRIVATE -Wall -Wextra -Wpedantic)
target_link_libraries(edge_planning_sdk_consumer
	PRIVATE
		ControlLink::control_link_edge_client
)
