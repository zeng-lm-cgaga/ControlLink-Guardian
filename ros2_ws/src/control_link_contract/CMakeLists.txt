cmake_minimum_required(VERSION 3.8)
project(control_link_contract)

if(NOT CMAKE_CXX_STANDARD)
	set(CMAKE_CXX_STANDARD 17)
endif()
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(ament_cmake REQUIRED)
find_package(OpenSSL REQUIRED)
find_package(yaml_cpp_vendor REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rmw REQUIRED)

add_library(${PROJECT_NAME} SHARED
	src/parser.cpp
	src/can_signal_parser.cpp
	src/qos_factory.cpp
	src/compatibility.cpp
	src/contract_bundle.cpp
	src/config_identity.cpp
	src/reconfiguration.cpp
	src/fastdds_environment.cpp
)
target_include_directories(${PROJECT_NAME}
	PUBLIC
		$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
		$<INSTALL_INTERFACE:include>
)
target_link_libraries(${PROJECT_NAME} PUBLIC yaml-cpp OpenSSL::Crypto)
target_compile_options(${PROJECT_NAME} PRIVATE -Wall -Wextra -Wpedantic)

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
ament_target_dependencies(${PROJECT_NAME} PUBLIC rclcpp rmw)

if(BUILD_TESTING)
	find_package(ament_cmake_gtest REQUIRED)
	find_package(std_msgs REQUIRED)

	ament_add_gtest(test_gateway_contract_parser
		test/test_gateway_contract_parser.cpp
	)
	target_link_libraries(test_gateway_contract_parser ${PROJECT_NAME})
	target_compile_definitions(test_gateway_contract_parser PRIVATE
		CONTROL_LINK_TEST_CONTRACT_PATH="${CMAKE_CURRENT_SOURCE_DIR}/../../../config/common/gateway_contract.yaml"
	)
	target_compile_options(test_gateway_contract_parser PRIVATE -Wall -Wextra -Wpedantic)

	ament_add_gtest(test_contract_components
		test/test_contract_components.cpp
	)
	target_link_libraries(test_contract_components ${PROJECT_NAME})
	ament_target_dependencies(test_contract_components rclcpp std_msgs)
	target_compile_definitions(test_contract_components PRIVATE
		CONTROL_LINK_TEST_CONFIG_ROOT="${CMAKE_CURRENT_SOURCE_DIR}/../../../config"
		CONTROL_LINK_TEST_ROBOT_PROFILE_PATH="${CMAKE_CURRENT_SOURCE_DIR}/../../../config/robot/robot_profile.yaml"
		CONTROL_LINK_TEST_ADAS_PROFILE_PATH="${CMAKE_CURRENT_SOURCE_DIR}/../../../config/adas/adas_profile.yaml"
		CONTROL_LINK_TEST_FASTDDS_PROFILE_PATH="${CMAKE_CURRENT_SOURCE_DIR}/../../../config/common/fastdds_default_profiles.xml"
	)
	target_compile_options(test_contract_components PRIVATE -Wall -Wextra -Wpedantic)

	ament_add_gtest(test_config_identity
		test/test_config_identity.cpp
	)
	target_link_libraries(test_config_identity ${PROJECT_NAME})
	target_compile_definitions(test_config_identity PRIVATE
		CONTROL_LINK_TEST_CONFIG_ROOT="${CMAKE_CURRENT_SOURCE_DIR}/../../../config"
		CONTROL_LINK_TEST_ROBOT_PROFILE_PATH="${CMAKE_CURRENT_SOURCE_DIR}/../../../config/robot/robot_profile.yaml"
		CONTROL_LINK_TEST_ADAS_PROFILE_PATH="${CMAKE_CURRENT_SOURCE_DIR}/../../../config/adas/adas_profile.yaml"
	)
	target_compile_options(test_config_identity PRIVATE -Wall -Wextra -Wpedantic)

	ament_add_gtest(test_reconfiguration
		test/test_reconfiguration.cpp
	)
	target_link_libraries(test_reconfiguration ${PROJECT_NAME})
	target_compile_definitions(test_reconfiguration PRIVATE
		CONTROL_LINK_TEST_CONFIG_ROOT="${CMAKE_CURRENT_SOURCE_DIR}/../../../config"
		CONTROL_LINK_TEST_ROBOT_PROFILE_PATH="${CMAKE_CURRENT_SOURCE_DIR}/../../../config/robot/robot_profile.yaml"
		CONTROL_LINK_TEST_ADAS_PROFILE_PATH="${CMAKE_CURRENT_SOURCE_DIR}/../../../config/adas/adas_profile.yaml"
	)
	target_compile_options(test_reconfiguration PRIVATE -Wall -Wextra -Wpedantic)
endif()

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(OpenSSL rclcpp rmw yaml_cpp_vendor)

ament_package()
