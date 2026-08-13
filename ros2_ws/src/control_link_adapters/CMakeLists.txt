cmake_minimum_required(VERSION 3.8)
project(control_link_adapters)

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
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rmw REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(Threads REQUIRED)


add_library(${PROJECT_NAME} SHARED
	src/can_codec.cpp
	src/socketcan_transport.cpp
	src/socketcan_adapter.cpp
	src/local_watchdog.cpp
	src/canonical_input_guard.cpp
	src/canonical_endpoint_tracker.cpp
	src/controller_endpoint_monitor.cpp
	src/tf_health_monitor.cpp
	src/twist_ingress_adapter.cpp
	src/ros2_control_adapter.cpp
)

add_executable(mock_control_source_node
	src/mock_control_source.cpp
)

add_executable(mock_vehicle_adapter_node
	src/mock_vehicle_adapter.cpp
)

add_executable(ros2_control_adapter_node
	src/ros2_control_adapter_main.cpp
)

add_executable(twist_ingress_adapter_node
	src/twist_ingress_adapter_main.cpp
)

add_executable(socketcan_adapter_node
	src/socketcan_adapter_main.cpp
)

target_include_directories(${PROJECT_NAME}
	PUBLIC
		$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
		$<INSTALL_INTERFACE:include>
)
ament_target_dependencies(${PROJECT_NAME}
	PUBLIC
		builtin_interfaces
		control_link_contract
		control_link_interfaces
		diagnostic_msgs
		geometry_msgs
		nav_msgs
		rclcpp
		rmw
		tf2
		tf2_ros
)
target_compile_options(${PROJECT_NAME} PRIVATE -Wall -Wextra -Wpedantic)
target_link_libraries(${PROJECT_NAME} PRIVATE Threads::Threads)

target_compile_options(mock_control_source_node PRIVATE -Wall -Wextra -Wpedantic)
target_compile_options(mock_vehicle_adapter_node PRIVATE -Wall -Wextra -Wpedantic)
target_compile_options(ros2_control_adapter_node PRIVATE -Wall -Wextra -Wpedantic)
target_compile_options(twist_ingress_adapter_node PRIVATE -Wall -Wextra -Wpedantic)
target_compile_options(socketcan_adapter_node PRIVATE -Wall -Wextra -Wpedantic)
target_link_libraries(mock_control_source_node ${PROJECT_NAME})
target_link_libraries(mock_vehicle_adapter_node ${PROJECT_NAME})
target_link_libraries(ros2_control_adapter_node ${PROJECT_NAME})
target_link_libraries(twist_ingress_adapter_node ${PROJECT_NAME})
target_link_libraries(socketcan_adapter_node ${PROJECT_NAME})
ament_target_dependencies(mock_control_source_node
	builtin_interfaces
	control_link_contract
	control_link_interfaces
	rclcpp
	rmw
)
ament_target_dependencies(mock_vehicle_adapter_node
	builtin_interfaces
	control_link_contract
	control_link_interfaces
	rclcpp
	rmw
)
ament_target_dependencies(ros2_control_adapter_node
	rclcpp
)
ament_target_dependencies(twist_ingress_adapter_node
	rclcpp
)
ament_target_dependencies(socketcan_adapter_node
	rclcpp
)

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
	TARGETS mock_control_source_node mock_vehicle_adapter_node ros2_control_adapter_node twist_ingress_adapter_node socketcan_adapter_node
	RUNTIME DESTINATION lib/${PROJECT_NAME}
)

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(
	builtin_interfaces
	control_link_contract
	control_link_interfaces
	diagnostic_msgs
	geometry_msgs
	nav_msgs
	rclcpp
	rmw
	tf2
	tf2_ros
)
ament_package()
