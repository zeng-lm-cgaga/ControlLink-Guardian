cmake_minimum_required(VERSION 3.8)
project(control_link_bringup)

option(
	CONTROL_LINK_ENABLE_VCAN_TESTS
	"Build ADAS launch tests that require an existing vcan0"
	OFF
)

if(NOT CMAKE_CXX_STANDARD)
	set(CMAKE_CXX_STANDARD 17)
endif()
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(control_link_contract REQUIRED)
find_package(control_link_adapters REQUIRED)
find_package(control_link_gateway REQUIRED)
find_package(control_link_interfaces REQUIRED)
find_package(controller_manager_msgs REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(lifecycle_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(nlohmann_json REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(ros_gz_interfaces REQUIRED)
find_package(rosgraph_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)

add_library(control_link_reconfiguration_core STATIC
	src/reconfiguration_transaction.cpp
)
target_include_directories(control_link_reconfiguration_core
	PUBLIC
		$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)
ament_target_dependencies(control_link_reconfiguration_core PUBLIC
	control_link_contract
)
target_link_libraries(control_link_reconfiguration_core PUBLIC
	nlohmann_json::nlohmann_json
)
target_compile_options(control_link_reconfiguration_core PRIVATE
	-Wall -Wextra -Wpedantic
)

if(BUILD_TESTING)
	find_package(ament_cmake_gtest REQUIRED)
	find_package(launch_testing_ament_cmake REQUIRED)
	ament_add_gtest(test_reconfiguration_transaction
		test/test_reconfiguration_transaction.cpp
	)
	target_link_libraries(test_reconfiguration_transaction
		control_link_reconfiguration_core
	)
	target_compile_options(test_reconfiguration_transaction PRIVATE
		-Wall -Wextra -Wpedantic
	)
endif()

ament_python_install_package(${PROJECT_NAME})

add_executable(robot_readiness_gate
	tools/robot_readiness_gate.cpp
)
add_executable(gateway_lifecycle_activator
	tools/gateway_lifecycle_activator.cpp
)
add_executable(send_fixed_nav_goal
	tools/send_fixed_nav_goal.cpp
)
add_executable(verify_teleop_takeover
	tools/verify_teleop_takeover.cpp
)
add_executable(verify_robot_tf_stale
	tools/verify_robot_tf_stale.cpp
)
add_executable(verify_robot_clock_stall
	tools/verify_robot_clock_stall.cpp
)
add_executable(verify_robot_controller_loss
	tools/verify_robot_controller_loss.cpp
)
add_executable(verify_robot_command_timeout
	tools/verify_robot_command_timeout.cpp
)
add_executable(vcan_vehicle_simulator
	tools/vcan_vehicle_simulator.cpp
)
add_executable(measure_runtime
	tools/measure_runtime.cpp
)
add_executable(verify_gateway_lifecycle
	tools/verify_gateway_lifecycle.cpp
)
add_executable(verify_adas_vcan
	tools/verify_adas_vcan.cpp
)
add_executable(replay_input_bridge
	tools/replay_input_bridge.cpp
)
add_executable(verify_mock_hardware_failure
	tools/verify_mock_hardware_failure.cpp
)
add_executable(replay_decisions
	tools/replay_decisions.cpp
)
add_executable(reconfigure_gateway
	tools/reconfigure_gateway.cpp
)

target_compile_options(robot_readiness_gate PRIVATE -Wall -Wextra -Wpedantic)
target_compile_options(gateway_lifecycle_activator PRIVATE -Wall -Wextra -Wpedantic)
target_compile_options(send_fixed_nav_goal PRIVATE -Wall -Wextra -Wpedantic)
target_compile_options(verify_teleop_takeover PRIVATE -Wall -Wextra -Wpedantic)
target_compile_options(verify_robot_tf_stale PRIVATE -Wall -Wextra -Wpedantic)
target_compile_options(verify_robot_clock_stall PRIVATE -Wall -Wextra -Wpedantic)
target_compile_options(verify_robot_controller_loss PRIVATE -Wall -Wextra -Wpedantic)
target_compile_options(verify_robot_command_timeout PRIVATE -Wall -Wextra -Wpedantic)
target_compile_options(vcan_vehicle_simulator PRIVATE -Wall -Wextra -Wpedantic)
target_compile_options(measure_runtime PRIVATE -Wall -Wextra -Wpedantic)
target_compile_options(verify_gateway_lifecycle PRIVATE -Wall -Wextra -Wpedantic)
target_compile_options(verify_adas_vcan PRIVATE -Wall -Wextra -Wpedantic)
target_compile_options(replay_input_bridge PRIVATE -Wall -Wextra -Wpedantic)
target_compile_options(verify_mock_hardware_failure PRIVATE -Wall -Wextra -Wpedantic)
target_compile_options(replay_decisions PRIVATE -Wall -Wextra -Wpedantic)
target_compile_options(reconfigure_gateway PRIVATE -Wall -Wextra -Wpedantic)

ament_target_dependencies(robot_readiness_gate
	controller_manager_msgs
	lifecycle_msgs
	rclcpp
	tf2
	tf2_ros
)
ament_target_dependencies(gateway_lifecycle_activator
	lifecycle_msgs
	rclcpp
)
ament_target_dependencies(send_fixed_nav_goal
	control_link_contract
	control_link_interfaces
	nav2_msgs
	nav_msgs
	rclcpp
	rclcpp_action
	tf2
	tf2_ros
)
ament_target_dependencies(verify_teleop_takeover
	control_link_contract
	control_link_interfaces
	geometry_msgs
	rclcpp
)
ament_target_dependencies(verify_robot_tf_stale
	control_link_contract
	control_link_interfaces
	geometry_msgs
	lifecycle_msgs
	nav2_msgs
	nav_msgs
	rclcpp
)
ament_target_dependencies(verify_robot_clock_stall
	control_link_contract
	control_link_interfaces
	geometry_msgs
	nav_msgs
	rclcpp
	ros_gz_interfaces
	rosgraph_msgs
)
ament_target_dependencies(verify_robot_controller_loss
	control_link_contract
	control_link_interfaces
	controller_manager_msgs
	geometry_msgs
	nav_msgs
	rclcpp
)
ament_target_dependencies(verify_robot_command_timeout
	control_link_contract
	control_link_interfaces
	geometry_msgs
	nav_msgs
	rclcpp
)
ament_target_dependencies(vcan_vehicle_simulator
	control_link_adapters
	control_link_contract
	rclcpp
)
ament_target_dependencies(measure_runtime
	control_link_adapters
	control_link_contract
	control_link_interfaces
	rclcpp
)
ament_target_dependencies(verify_gateway_lifecycle
	control_link_contract
	control_link_interfaces
	diagnostic_msgs
	lifecycle_msgs
	rclcpp
)
ament_target_dependencies(verify_adas_vcan
	control_link_contract
	control_link_interfaces
	diagnostic_msgs
	rclcpp
)
ament_target_dependencies(replay_input_bridge
	control_link_contract
	control_link_interfaces
	rclcpp
)
ament_target_dependencies(verify_mock_hardware_failure
	control_link_contract
	control_link_interfaces
	controller_manager_msgs
	geometry_msgs
	lifecycle_msgs
	nav_msgs
	rclcpp
	sensor_msgs
)
ament_target_dependencies(replay_decisions
	control_link_contract
	control_link_gateway
)
target_link_libraries(replay_decisions
	nlohmann_json::nlohmann_json
)
ament_target_dependencies(reconfigure_gateway
	control_link_contract
	diagnostic_msgs
	lifecycle_msgs
	rclcpp
)
target_link_libraries(reconfigure_gateway
	control_link_reconfiguration_core
	nlohmann_json::nlohmann_json
)
target_link_libraries(measure_runtime
	nlohmann_json::nlohmann_json
)

install(
	DIRECTORY launch
	DESTINATION share/${PROJECT_NAME}
	PATTERN "__pycache__" EXCLUDE
)
install(
	DIRECTORY maps urdf worlds
	DESTINATION share/${PROJECT_NAME}
)
install(
	DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/../../../config/
	DESTINATION share/${PROJECT_NAME}/config
)
install(
	TARGETS
		robot_readiness_gate
		gateway_lifecycle_activator
		send_fixed_nav_goal
		verify_teleop_takeover
		verify_robot_tf_stale
		verify_robot_clock_stall
		verify_robot_controller_loss
		verify_robot_command_timeout
		vcan_vehicle_simulator
		measure_runtime
		verify_gateway_lifecycle
		verify_adas_vcan
		replay_input_bridge
		verify_mock_hardware_failure
		replay_decisions
		reconfigure_gateway
	RUNTIME DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
	find_package(ament_cmake_pytest REQUIRED)
	ament_add_pytest_test(
		test_replay_launch_config
		test/test_replay_launch_config.py
		ENV
			CONTROL_LINK_REPLAY_LAUNCH_PATH=${CMAKE_CURRENT_SOURCE_DIR}/launch/replay.launch.py
	)
	ament_add_pytest_test(
		test_replay_decisions_cli
		test/test_replay_decisions_cli.py
		ENV
			CONTROL_LINK_REPLAY_DECISIONS=$<TARGET_FILE:replay_decisions>
			CONTROL_LINK_TEST_ROBOT_PROFILE=${CMAKE_CURRENT_SOURCE_DIR}/../../../config/robot/robot_profile.yaml
			CONTROL_LINK_TEST_CONFIG_ROOT=${CMAKE_CURRENT_SOURCE_DIR}/../../../config
			CONTROL_LINK_TEST_DECISION_TRACE=${CMAKE_CURRENT_SOURCE_DIR}/../control_link_gateway/test/fixtures/decision_traces/robot_nominal.jsonl
	)
	add_launch_test(
		test/test_robot_profile_launch.py
		TIMEOUT 360
		ENV ROS_DOMAIN_ID=219 IGN_PARTITION=control_link_robot_profile_219
	)
	add_launch_test(
		test/test_gateway_lifecycle_launch.py
		TARGET test_gateway_lifecycle_default_launch
		TIMEOUT 120
	)
	add_launch_test(
		test/test_gateway_lifecycle_launch.py
		TARGET test_gateway_lifecycle_shm_launch
		TIMEOUT 120
		ENV CONTROL_LINK_TEST_FASTDDS_PROFILE=fastdds_shm_only_profiles.xml
	)
	add_launch_test(
		test/test_mock_execution_launch.py
		TARGET test_mock_execution_command_timeout_launch
		TIMEOUT 150
		ENV
			CONTROL_LINK_MOCK_SCENARIO=command_timeout
			ROS_DOMAIN_ID=220
			IGN_PARTITION=control_link_mock_command_timeout_220
	)
	add_launch_test(
		test/test_mock_execution_launch.py
		TARGET test_mock_execution_read_failure_launch
		TIMEOUT 150
		ENV
			CONTROL_LINK_MOCK_SCENARIO=read_failure
			ROS_DOMAIN_ID=221
			IGN_PARTITION=control_link_mock_read_failure_221
	)
	add_launch_test(
		test/test_mock_execution_launch.py
		TARGET test_mock_execution_write_failure_launch
		TIMEOUT 150
		ENV
			CONTROL_LINK_MOCK_SCENARIO=write_failure
			ROS_DOMAIN_ID=222
			IGN_PARTITION=control_link_mock_write_failure_222
	)
	add_launch_test(
		test/test_reconfiguration_launch.py
		TARGET test_reconfiguration_launch
		TIMEOUT 120
		ENV
			CONTROL_LINK_RECONFIGURE_GATEWAY=$<TARGET_FILE:reconfigure_gateway>
			ROS_DOMAIN_ID=223
	)
	if(CONTROL_LINK_ENABLE_VCAN_TESTS)
		add_launch_test(
			test/test_adas_vcan_launch.py
			TARGET test_adas_vcan_launch
			TIMEOUT 150
			ENV ROS_DOMAIN_ID=218
		)
	endif()
endif()

ament_package()
