cmake_minimum_required(VERSION 3.8)
project(control_link_gateway)

if(NOT CMAKE_CXX_STANDARD)
	set(CMAKE_CXX_STANDARD 17)
endif()
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(ament_cmake REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(control_link_contract REQUIRED)
find_package(control_link_interfaces REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(lifecycle_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(rmw REQUIRED)

add_library(${PROJECT_NAME} SHARED
	src/control_gateway_node.cpp
	src/command_validator.cpp
	src/graph_monitor.cpp
	src/source_binding.cpp
	src/source_runtime.cpp
	src/source_arbiter.cpp
	src/state_machine.cpp
	src/critical_endpoint_monitor.cpp
	src/vehicle_state_validator.cpp
	src/vehicle_state_runtime.cpp
)

add_executable(${PROJECT_NAME}_node
	src/control_gateway_main.cpp
)

target_include_directories(${PROJECT_NAME}
	PUBLIC
		$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
		$<INSTALL_INTERFACE:include>
)

ament_target_dependencies(${PROJECT_NAME} PUBLIC
	builtin_interfaces
	control_link_contract
	control_link_interfaces
	diagnostic_msgs
	lifecycle_msgs
	rclcpp
	rclcpp_lifecycle
	rmw
)
target_compile_options(${PROJECT_NAME} PRIVATE -Wall -Wextra -Wpedantic)
target_compile_options(${PROJECT_NAME}_node PRIVATE -Wall -Wextra -Wpedantic)
target_link_libraries(${PROJECT_NAME}_node ${PROJECT_NAME})
ament_target_dependencies(${PROJECT_NAME}_node
	rclcpp
	rclcpp_lifecycle
)

if(BUILD_TESTING)
	find_package(ament_cmake_gtest REQUIRED)

	ament_add_gtest(test_gateway_components
		test/test_gateway_components.cpp
	)
	target_link_libraries(test_gateway_components ${PROJECT_NAME})
	target_compile_definitions(test_gateway_components PRIVATE
		CONTROL_LINK_TEST_CONTRACT_PATH="${CMAKE_CURRENT_SOURCE_DIR}/../../../config/common/gateway_contract.yaml"
		CONTROL_LINK_TEST_SOURCE_POLICY_PATH="${CMAKE_CURRENT_SOURCE_DIR}/../../../config/common/source_policy.yaml"
	)
	target_compile_options(test_gateway_components PRIVATE -Wall -Wextra -Wpedantic)
endif()

install(
	DIRECTORY include/
	DESTINATION include
)
install(
	TARGETS ${PROJECT_NAME}
	EXPORT export_${PROJECT_NAME}
	ARCHIVE DESTINATION lib
	LIBRARY DESTINATION lib
	RUNTIME DESTINATION bin
	INCLUDES DESTINATION include
)
install(
	TARGETS ${PROJECT_NAME}_node
	RUNTIME DESTINATION lib/${PROJECT_NAME}
)

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(
	builtin_interfaces
	control_link_contract
	control_link_interfaces
	diagnostic_msgs
	lifecycle_msgs
	rclcpp
	rclcpp_lifecycle
	rmw
)

ament_package()
