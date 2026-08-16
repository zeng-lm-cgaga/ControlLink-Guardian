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
find_package(nlohmann_json REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(rmw REQUIRED)

execute_process(
	COMMAND git rev-parse --verify HEAD
	WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/../../..
	RESULT_VARIABLE CONTROL_LINK_GIT_RESULT
	OUTPUT_VARIABLE CONTROL_LINK_GIT_COMMIT
	OUTPUT_STRIP_TRAILING_WHITESPACE
	ERROR_QUIET
)
string(LENGTH "${CONTROL_LINK_GIT_COMMIT}" CONTROL_LINK_GIT_COMMIT_LENGTH)
if(NOT CONTROL_LINK_GIT_RESULT EQUAL 0 OR
	NOT CONTROL_LINK_GIT_COMMIT_LENGTH EQUAL 40 OR
	NOT CONTROL_LINK_GIT_COMMIT MATCHES "^[0-9a-f]+$")
	set(CONTROL_LINK_GIT_COMMIT "unknown")
endif()
execute_process(
	COMMAND git status --porcelain --untracked-files=normal
	WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/../../..
	RESULT_VARIABLE CONTROL_LINK_GIT_STATUS_RESULT
	OUTPUT_VARIABLE CONTROL_LINK_GIT_STATUS
	OUTPUT_STRIP_TRAILING_WHITESPACE
	ERROR_QUIET
)
if(CONTROL_LINK_GIT_STATUS_RESULT EQUAL 0 AND CONTROL_LINK_GIT_STATUS STREQUAL "")
	set(CONTROL_LINK_GIT_DIRTY 0)
else()
	set(CONTROL_LINK_GIT_DIRTY 1)
endif()
if(CMAKE_BUILD_TYPE STREQUAL "")
	set(CONTROL_LINK_BUILD_TYPE "Unknown")
else()
	set(CONTROL_LINK_BUILD_TYPE "${CMAKE_BUILD_TYPE}")
endif()

add_library(${PROJECT_NAME} SHARED
	src/control_gateway_node.cpp
	src/command_validator.cpp
	src/decision_engine.cpp
	src/decision_replay.cpp
	src/decision_trace.cpp
	src/decision_trace_recorder.cpp
	src/graph_monitor.cpp
	src/source_binding.cpp
	src/source_runtime.cpp
	src/source_arbiter.cpp
	src/state_machine.cpp
	src/critical_endpoint_monitor.cpp
	src/vehicle_state_validator.cpp
	src/vehicle_state_runtime.cpp
)
target_link_libraries(${PROJECT_NAME} PRIVATE nlohmann_json::nlohmann_json)

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
target_compile_definitions(${PROJECT_NAME} PRIVATE
	CONTROL_LINK_BUILD_TYPE="${CONTROL_LINK_BUILD_TYPE}"
	CONTROL_LINK_GIT_COMMIT="${CONTROL_LINK_GIT_COMMIT}"
	CONTROL_LINK_GIT_DIRTY=${CONTROL_LINK_GIT_DIRTY}
)
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

	ament_add_gtest(test_decision_trace
		test/test_decision_trace.cpp
	)
	target_link_libraries(test_decision_trace ${PROJECT_NAME})
	target_compile_options(test_decision_trace PRIVATE -Wall -Wextra -Wpedantic)

	ament_add_gtest(test_decision_replay
		test/test_decision_replay.cpp
	)
	target_link_libraries(test_decision_replay ${PROJECT_NAME})
	target_compile_definitions(test_decision_replay PRIVATE
		CONTROL_LINK_TEST_CONFIG_ROOT="${CMAKE_CURRENT_SOURCE_DIR}/../../../config"
		CONTROL_LINK_TEST_ROBOT_PROFILE_PATH="${CMAKE_CURRENT_SOURCE_DIR}/../../../config/robot/robot_profile.yaml"
		CONTROL_LINK_TEST_DECISION_TRACE_PATH="${CMAKE_CURRENT_SOURCE_DIR}/test/fixtures/decision_traces/robot_nominal.jsonl"
	)
	target_compile_options(test_decision_replay PRIVATE -Wall -Wextra -Wpedantic)

	ament_add_gtest(test_decision_trace_recorder
		test/test_decision_trace_recorder.cpp
	)
	target_link_libraries(test_decision_trace_recorder ${PROJECT_NAME})
	target_compile_definitions(test_decision_trace_recorder PRIVATE
		CONTROL_LINK_TEST_DECISION_TRACE_PATH="${CMAKE_CURRENT_SOURCE_DIR}/test/fixtures/decision_traces/robot_nominal.jsonl"
	)
	target_compile_options(test_decision_trace_recorder PRIVATE -Wall -Wextra -Wpedantic)
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
