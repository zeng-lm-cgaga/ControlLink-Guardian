cmake_minimum_required(VERSION 3.8)
project(control_link_adapters)

option(
	CONTROL_LINK_ENABLE_VCAN_TESTS
	"Build SocketCAN integration tests that require an existing vcan0"
	OFF
)

option(
	CONTROL_LINK_ENABLE_FUZZ_TARGETS
	"Build bounded CAN codec fuzz-smoke targets with ASan and UBSan"
	OFF
)

option(
	CONTROL_LINK_ENABLE_EDGE_RUNTIME
	"Build the optional EdgeRuntime planning bridge and standalone client"
	OFF
)

if(NOT CMAKE_CXX_STANDARD)
	set(CMAKE_CXX_STANDARD 17)
endif()
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

include(cmake/ControlLinkNetworkDependencies.cmake)
control_link_probe_network_dependencies()

find_package(ament_cmake REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(control_link_contract REQUIRED)
find_package(control_link_interfaces REQUIRED)
find_package(controller_manager_msgs REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(lifecycle_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(rmw REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(Threads REQUIRED)
add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/../../../runtime_timing ${CMAKE_CURRENT_BINARY_DIR}/runtime_timing)

if(CONTROL_LINK_ENABLE_EDGE_RUNTIME OR CONTROL_LINK_ENABLE_SOMEIP OR CONTROL_LINK_ENABLE_GRPC)
	if(CONTROL_LINK_ENABLE_EDGE_RUNTIME)
		find_package(EdgeRuntime CONFIG REQUIRED)
	endif()
	set(CONTROL_LINK_EDGE_PROTOCOL_BUILD_CLIENT OFF CACHE BOOL
		"" FORCE)
	set(CONTROL_LINK_EDGE_PROTOCOL_BUILD_SDK ${CONTROL_LINK_ENABLE_EDGE_RUNTIME} CACHE BOOL
		"" FORCE)
	set(CONTROL_LINK_EDGE_PROTOCOL_BUILD_SUPERVISOR OFF CACHE BOOL
		"" FORCE)
	set(CONTROL_LINK_EDGE_PROTOCOL_BUILD_TESTS OFF CACHE BOOL
		"" FORCE)
	add_subdirectory(
		${CMAKE_CURRENT_SOURCE_DIR}/../../../edge_protocol
		${CMAKE_CURRENT_BINARY_DIR}/edge_protocol
	)
endif()

if(CONTROL_LINK_ENABLE_SOMEIP)
	set(CONTROL_LINK_SOMEIP_BUILD_TESTS OFF)
	add_subdirectory(
		${CMAKE_CURRENT_SOURCE_DIR}/../../../someip_transport
		${CMAKE_CURRENT_BINARY_DIR}/someip_transport
	)
	file(WRITE "${CMAKE_CURRENT_BINARY_DIR}/control_link_someip_runtime.txt"
		"someip.status=RUNTIME_AVAILABLE\nproduction_protocol_targets=control_link_someip_transport;someip_planning_client;someip_configure;someip_planning_bridge_node\nacceptance=NOT_IMPLIED_BY_CONFIGURE\n")
else()
	file(WRITE "${CMAKE_CURRENT_BINARY_DIR}/control_link_someip_runtime.txt"
		"someip.status=DISABLED\nproduction_protocol_targets=NONE\n")
endif()

if(CONTROL_LINK_ENABLE_GRPC)
	set(CONTROL_LINK_GRPC_BUILD_TESTS OFF)
	add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/../../../grpc_transport ${CMAKE_CURRENT_BINARY_DIR}/grpc_transport)
	file(WRITE "${CMAKE_CURRENT_BINARY_DIR}/control_link_grpc_runtime.txt"
		"grpc.status=RUNTIME_AVAILABLE\nproduction_protocol_targets=control_link_grpc_transport;grpc_planning_client;grpc_planning_bridge_node\nacceptance=NOT_IMPLIED_BY_CONFIGURE\n")
else()
	file(WRITE "${CMAKE_CURRENT_BINARY_DIR}/control_link_grpc_runtime.txt" "grpc.status=DISABLED\nproduction_protocol_targets=NONE\n")
endif()


add_library(${PROJECT_NAME} SHARED
	src/can_codec.cpp
	src/socketcan_transport.cpp
	src/socketcan_adapter.cpp
	src/local_watchdog.cpp
	src/canonical_input_guard.cpp
	src/canonical_endpoint_tracker.cpp
	src/controller_endpoint_monitor.cpp
	src/hardware_component_monitor.cpp
	src/tf_health_monitor.cpp
	src/twist_ingress_adapter.cpp
	src/control_command_ingress_adapter.cpp
	src/ros2_control_adapter.cpp
)

add_library(control_link_mock_system SHARED
	src/control_link_mock_system.cpp
)
target_include_directories(control_link_mock_system
	PUBLIC
		$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
		$<INSTALL_INTERFACE:include>
)
ament_target_dependencies(control_link_mock_system
	PUBLIC
		hardware_interface
		pluginlib
		rclcpp
		rclcpp_lifecycle
)
target_compile_options(control_link_mock_system PRIVATE -Wall -Wextra -Wpedantic)
pluginlib_export_plugin_description_file(
	hardware_interface
	control_link_mock_system_plugin.xml
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

add_executable(control_command_ingress_adapter_node
	src/control_command_ingress_adapter_main.cpp
)

if(CONTROL_LINK_ENABLE_EDGE_RUNTIME)
	add_executable(edge_planning_bridge_node
		src/edge_planning_bridge.cpp
	)
	add_executable(edge_planning_client
		${CMAKE_CURRENT_SOURCE_DIR}/../../../edge_protocol/src/edge_planning_client.cpp
	)
	add_executable(edge_planning_supervisor
		${CMAKE_CURRENT_SOURCE_DIR}/../../../edge_protocol/src/edge_planning_supervisor.cpp
	)
endif()

if(CONTROL_LINK_ENABLE_GRPC)
	add_executable(grpc_planning_bridge_node src/grpc_planning_bridge.cpp)
	target_include_directories(grpc_planning_bridge_node PRIVATE include)
	target_compile_options(grpc_planning_bridge_node PRIVATE -Wall -Wextra -Wpedantic)
	target_compile_definitions(grpc_planning_bridge_node PRIVATE CONTROL_LINK_USE_GRPC)
	target_link_libraries(grpc_planning_bridge_node control_link_grpc_transport)
	ament_target_dependencies(grpc_planning_bridge_node builtin_interfaces control_link_contract control_link_interfaces rclcpp rmw)
endif()

if(CONTROL_LINK_ENABLE_SOMEIP)
	add_executable(someip_planning_bridge_node src/someip_planning_bridge.cpp)
	target_include_directories(someip_planning_bridge_node PRIVATE include)
	target_compile_options(someip_planning_bridge_node PRIVATE -Wall -Wextra -Wpedantic)
	target_link_libraries(someip_planning_bridge_node control_link_someip_transport)
	ament_target_dependencies(someip_planning_bridge_node
		builtin_interfaces
		control_link_contract
		control_link_interfaces
		rclcpp
		rmw
	)
endif()

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
		controller_manager_msgs
		diagnostic_msgs
		geometry_msgs
		lifecycle_msgs
		nav_msgs
		rclcpp
		rmw
		tf2
		tf2_ros
)
target_compile_options(${PROJECT_NAME} PRIVATE -Wall -Wextra -Wpedantic)
target_link_libraries(${PROJECT_NAME} PRIVATE Threads::Threads control_link_runtime_timing)

if(CONTROL_LINK_ENABLE_FUZZ_TARGETS)
	target_compile_options(${PROJECT_NAME} PRIVATE
		-fsanitize=address,undefined
		-fno-omit-frame-pointer
	)
	# FUZZ 构建的共享库必须把 sanitizer 运行时传递给所有测试和节点消费者。
	target_link_options(${PROJECT_NAME} PUBLIC -fsanitize=address,undefined)

	add_executable(control_link_fuzz_can_codec
		fuzz/fuzz_can_codec.cpp
	)
	target_link_libraries(control_link_fuzz_can_codec ${PROJECT_NAME})
	target_compile_definitions(control_link_fuzz_can_codec PRIVATE
		CONTROL_LINK_FUZZ_CAN_SIGNAL_MAP_PATH="${CMAKE_CURRENT_SOURCE_DIR}/../../../config/adas/can_signal_map.yaml"
	)
	target_compile_options(control_link_fuzz_can_codec PRIVATE
		-Wall -Wextra -Wpedantic
		-fsanitize=address,undefined
		-fno-omit-frame-pointer
	)
	target_link_options(control_link_fuzz_can_codec PRIVATE -fsanitize=address,undefined)
endif()

target_compile_options(mock_control_source_node PRIVATE -Wall -Wextra -Wpedantic)
target_compile_options(mock_vehicle_adapter_node PRIVATE -Wall -Wextra -Wpedantic)
target_compile_options(ros2_control_adapter_node PRIVATE -Wall -Wextra -Wpedantic)
target_compile_options(twist_ingress_adapter_node PRIVATE -Wall -Wextra -Wpedantic)
target_compile_options(control_command_ingress_adapter_node PRIVATE -Wall -Wextra -Wpedantic)
target_compile_options(socketcan_adapter_node PRIVATE -Wall -Wextra -Wpedantic)
target_link_libraries(mock_control_source_node ${PROJECT_NAME})
target_link_libraries(mock_vehicle_adapter_node ${PROJECT_NAME})
target_link_libraries(ros2_control_adapter_node ${PROJECT_NAME})
target_link_libraries(twist_ingress_adapter_node ${PROJECT_NAME})
target_link_libraries(control_command_ingress_adapter_node ${PROJECT_NAME})
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
ament_target_dependencies(control_command_ingress_adapter_node
	rclcpp
)
ament_target_dependencies(socketcan_adapter_node
	rclcpp
)

if(CONTROL_LINK_ENABLE_EDGE_RUNTIME)
	target_include_directories(edge_planning_bridge_node PRIVATE include)
	target_compile_options(edge_planning_bridge_node PRIVATE -Wall -Wextra -Wpedantic)
	target_compile_options(edge_planning_client PRIVATE -Wall -Wextra -Wpedantic)
	target_compile_options(edge_planning_supervisor PRIVATE -Wall -Wextra -Wpedantic)
	target_link_libraries(edge_planning_bridge_node
		control_link_edge_protocol
		EdgeRuntime::edge_runtime
	)
	target_link_libraries(edge_planning_client
		control_link_edge_client
	)
	target_link_libraries(edge_planning_supervisor
		EdgeRuntime::edge_runtime
	)
	ament_target_dependencies(edge_planning_bridge_node
		builtin_interfaces
		control_link_contract
		control_link_interfaces
		rclcpp
	)
endif()

if(BUILD_TESTING)
	find_package(ament_cmake_gtest REQUIRED)
	find_package(Python3 REQUIRED COMPONENTS Interpreter)
	add_test(NAME test_network_dependency_configuration
		COMMAND ${Python3_EXECUTABLE} -B
			${CMAKE_CURRENT_SOURCE_DIR}/test/test_network_dependency_configuration.py -v
	)
	set_tests_properties(test_network_dependency_configuration PROPERTIES TIMEOUT 300)
	add_test(NAME test_adas_planning_transport_selector
		COMMAND ${Python3_EXECUTABLE} -B
			${CMAKE_CURRENT_SOURCE_DIR}/../control_link_bringup/test/test_adas_planning_transport_selector.py -v
	)
	add_test(NAME test_protocol_transport_contract
		COMMAND ${Python3_EXECUTABLE} -B
			${CMAKE_CURRENT_SOURCE_DIR}/../../../tools/protocol_dependency_probe/test_transport_contract.py -v
	)

	ament_add_gtest(test_adapter_core
		test/test_adapter_core.cpp
	)
	target_link_libraries(test_adapter_core ${PROJECT_NAME})
	target_compile_definitions(test_adapter_core PRIVATE
		CONTROL_LINK_TEST_CONTRACT_PATH="${CMAKE_CURRENT_SOURCE_DIR}/../../../config/common/gateway_contract.yaml"
	)
	target_compile_options(test_adapter_core PRIVATE -Wall -Wextra -Wpedantic)

	ament_add_gtest(test_control_command_ingress test/test_control_command_ingress.cpp)
	target_link_libraries(test_control_command_ingress ${PROJECT_NAME})
	target_compile_definitions(test_control_command_ingress PRIVATE
		CONTROL_LINK_TEST_ADAS_PROFILE="${CMAKE_CURRENT_SOURCE_DIR}/../../../config/adas/adas_profile.yaml"
		CONTROL_LINK_TEST_CONFIG_ROOT="${CMAKE_CURRENT_SOURCE_DIR}/../../../config"
	)
	target_compile_options(test_control_command_ingress PRIVATE -Wall -Wextra -Wpedantic)
	set_tests_properties(test_control_command_ingress PROPERTIES ENVIRONMENT
		"RMW_IMPLEMENTATION=rmw_fastrtps_cpp;RMW_FASTRTPS_USE_QOS_FROM_XML=0;FASTRTPS_DEFAULT_PROFILES_FILE=${CMAKE_CURRENT_SOURCE_DIR}/../../../config/common/fastdds_default_profiles.xml"
	)

	ament_add_gtest(test_can_codec
		test/test_can_codec.cpp
	)
	target_link_libraries(test_can_codec ${PROJECT_NAME})
	target_compile_definitions(test_can_codec PRIVATE
		CONTROL_LINK_TEST_CAN_SIGNAL_MAP_PATH="${CMAKE_CURRENT_SOURCE_DIR}/../../../config/adas/can_signal_map.yaml"
	)
	target_compile_options(test_can_codec PRIVATE -Wall -Wextra -Wpedantic)

	ament_add_gtest(test_control_link_mock_system
		test/test_control_link_mock_system.cpp
	)
	target_link_libraries(test_control_link_mock_system control_link_mock_system)
	ament_target_dependencies(test_control_link_mock_system
		hardware_interface
		pluginlib
		rclcpp
		rclcpp_lifecycle
	)
	target_compile_options(
		test_control_link_mock_system
		PRIVATE -Wall -Wextra -Wpedantic
	)

	ament_add_gtest(test_graph_admission_policy
		test/test_graph_admission_policy.cpp
	)
	target_include_directories(test_graph_admission_policy PRIVATE include)
	target_compile_options(test_graph_admission_policy PRIVATE -Wall -Wextra -Wpedantic)

	if(CONTROL_LINK_ENABLE_EDGE_RUNTIME)
		ament_add_gtest(test_edge_command_wait_policy
			test/test_edge_command_wait_policy.cpp
		)
		target_include_directories(test_edge_command_wait_policy PRIVATE include)
		target_link_libraries(test_edge_command_wait_policy EdgeRuntime::edge_runtime)
		target_compile_options(test_edge_command_wait_policy PRIVATE -Wall -Wextra -Wpedantic)
	endif()
	if(CONTROL_LINK_ENABLE_SOMEIP)
		ament_add_gtest(test_someip_observation test/test_someip_observation.cpp)
		target_include_directories(test_someip_observation PRIVATE include)
		target_compile_definitions(test_someip_observation PRIVATE
			CONTROL_LINK_TEST_CONFIG_ROOT="${CMAKE_CURRENT_SOURCE_DIR}/../../../config")
		target_compile_options(test_someip_observation PRIVATE -Wall -Wextra -Wpedantic)
		target_link_libraries(test_someip_observation control_link_someip_transport)
		ament_target_dependencies(test_someip_observation builtin_interfaces control_link_contract control_link_interfaces)
		foreach(test_name IN ITEMS config ingress application_runtime session_reply_order)
			ament_add_gtest(test_someip_${test_name}
				${CMAKE_CURRENT_SOURCE_DIR}/../../../someip_transport/test/test_${test_name}.cpp
			)
			target_compile_definitions(test_someip_${test_name} PRIVATE
				CONTROL_LINK_PROTOCOL_CONFIG_PATH="${CMAKE_CURRENT_SOURCE_DIR}/../../../config/adas/protocol_transports.yaml")
			target_compile_options(test_someip_${test_name} PRIVATE -Wall -Wextra -Wpedantic)
			target_link_libraries(test_someip_${test_name} control_link_someip_transport)
		endforeach()
	endif()

	if(CONTROL_LINK_ENABLE_VCAN_TESTS)
		ament_add_gtest(test_socketcan_transport
			test/test_socketcan_transport.cpp
		)
		target_link_libraries(test_socketcan_transport ${PROJECT_NAME})
		target_compile_options(
			test_socketcan_transport
			PRIVATE -Wall -Wextra -Wpedantic
		)
	endif()
endif()

install(
	DIRECTORY include/
	DESTINATION include
)
install(FILES
	cmake/ControlLinkNetworkDependencies.cmake
	cmake/ControlLinkNetworkDependencyLock.cmake
	DESTINATION share/${PROJECT_NAME}/cmake
)
install(FILES
	${CMAKE_CURRENT_SOURCE_DIR}/../../../protocols/control_link_network_v1.proto
	${CMAKE_CURRENT_SOURCE_DIR}/../../../config/adas/protocol_transports.yaml
	DESTINATION share/${PROJECT_NAME}/network_contracts
	COMPONENT network_contracts
)
install(
	TARGETS ${PROJECT_NAME} control_link_mock_system
	EXPORT export_${PROJECT_NAME}
	ARCHIVE DESTINATION lib
	LIBRARY DESTINATION lib
	RUNTIME DESTINATION bin
	INCLUDES DESTINATION include
)
install(
	TARGETS mock_control_source_node mock_vehicle_adapter_node ros2_control_adapter_node twist_ingress_adapter_node control_command_ingress_adapter_node socketcan_adapter_node
	RUNTIME DESTINATION lib/${PROJECT_NAME}
)
if(CONTROL_LINK_ENABLE_EDGE_RUNTIME)
		install(
			TARGETS edge_planning_bridge_node edge_planning_client edge_planning_supervisor
			RUNTIME DESTINATION lib/${PROJECT_NAME}
	)
endif()

if(CONTROL_LINK_ENABLE_SOMEIP)
	install(TARGETS someip_planning_bridge_node someip_planning_client someip_configure
		RUNTIME DESTINATION lib/${PROJECT_NAME})
endif()
if(CONTROL_LINK_ENABLE_GRPC)
	install(TARGETS grpc_planning_bridge_node grpc_planning_client
		RUNTIME DESTINATION lib/${PROJECT_NAME} COMPONENT grpc_runtime)
endif()

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(
	builtin_interfaces
	control_link_contract
	control_link_interfaces
	controller_manager_msgs
	diagnostic_msgs
	geometry_msgs
	hardware_interface
	lifecycle_msgs
	nav_msgs
	pluginlib
	rclcpp
	rclcpp_lifecycle
	rmw
	tf2
	tf2_ros
)
ament_package()
