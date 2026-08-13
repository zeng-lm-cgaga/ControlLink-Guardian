cmake_minimum_required(VERSION 3.8)
project(control_link_bringup)

if(NOT CMAKE_CXX_STANDARD)
	set(CMAKE_CXX_STANDARD 17)
endif()
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(control_link_contract REQUIRED)
find_package(control_link_adapters REQUIRED)
find_package(control_link_interfaces REQUIRED)
find_package(controller_manager_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(lifecycle_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(nlohmann_json REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(ros_gz_interfaces REQUIRED)
find_package(rosgraph_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)

if(BUILD_TESTING)
	find_package(launch_testing_ament_cmake REQUIRED)
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
	RUNTIME DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
	add_launch_test(
		test/test_robot_profile_launch.py
		TIMEOUT 360
	)
endif()

ament_package()
