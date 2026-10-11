cmake_minimum_required(VERSION 3.16)
project(control_link_someip_transport VERSION 0.1.0 LANGUAGES CXX)

option(CONTROL_LINK_SOMEIP_BUILD_TESTS
	"Build the ROS-free SOME/IP configuration and ingress tests"
	OFF)

include(${CMAKE_CURRENT_SOURCE_DIR}/../ros2_ws/src/control_link_adapters/cmake/ControlLinkNetworkDependencyLock.cmake)
find_package(vsomeip3 ${CONTROL_LINK_VSOMEIP_VERSION} EXACT CONFIG REQUIRED)
find_package(Boost 1.55 REQUIRED COMPONENTS system thread filesystem)
find_package(Protobuf ${CONTROL_LINK_PROTOBUF_VERSION} EXACT MODULE REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(Threads REQUIRED)
if(NOT TARGET vsomeip3 OR NOT TARGET protobuf::libprotobuf OR NOT TARGET protobuf::protoc)
	message(FATAL_ERROR "ControlLink SOME/IP runtime requires real vsomeip3, protobuf::libprotobuf and protobuf::protoc imported targets")
endif()

get_target_property(control_link_protoc_location protobuf::protoc IMPORTED_LOCATION)
if(NOT control_link_protoc_location OR NOT EXISTS "${control_link_protoc_location}")
	message(FATAL_ERROR "ControlLink SOME/IP runtime requires a real installed protobuf::protoc executable")
endif()
execute_process(COMMAND "${control_link_protoc_location}" --version
	RESULT_VARIABLE control_link_protoc_result
	OUTPUT_VARIABLE control_link_protoc_version
	ERROR_VARIABLE control_link_protoc_error
	OUTPUT_STRIP_TRAILING_WHITESPACE TIMEOUT 10)
if(NOT control_link_protoc_result EQUAL 0 OR
	NOT control_link_protoc_version STREQUAL "libprotoc ${CONTROL_LINK_PROTOBUF_VERSION}")
	message(FATAL_ERROR "ControlLink SOME/IP runtime requires protoc/header/library version ${CONTROL_LINK_PROTOBUF_VERSION}; got=${control_link_protoc_version}; exit=${control_link_protoc_result}; ${control_link_protoc_error}")
endif()

get_target_property(control_link_vsomeip_location vsomeip3 IMPORTED_LOCATION)
if(NOT control_link_vsomeip_location)
	get_target_property(control_link_vsomeip_configurations vsomeip3 IMPORTED_CONFIGURATIONS)
	foreach(configuration IN LISTS control_link_vsomeip_configurations)
		string(TOUPPER "${configuration}" configuration_upper)
		get_target_property(control_link_vsomeip_location vsomeip3 "IMPORTED_LOCATION_${configuration_upper}")
		if(control_link_vsomeip_location)
			break()
		endif()
	endforeach()
endif()
if(NOT control_link_vsomeip_location OR NOT EXISTS "${control_link_vsomeip_location}")
	message(FATAL_ERROR "ControlLink SOME/IP runtime requires an installed vsomeip3 library, not a probe-only target")
endif()
get_filename_component(control_link_vsomeip_location "${control_link_vsomeip_location}" REALPATH)
get_filename_component(control_link_vsomeip_library_dir "${control_link_vsomeip_location}" DIRECTORY)
unset(control_link_vsomeip_sd_library CACHE)
find_library(control_link_vsomeip_sd_library NAMES vsomeip3-sd
	PATHS "${control_link_vsomeip_library_dir}" NO_DEFAULT_PATH)
if(NOT control_link_vsomeip_sd_library)
	message(FATAL_ERROR "ControlLink SOME/IP runtime requires vsomeip3-sd Service Discovery beside ${control_link_vsomeip_location}")
endif()
get_filename_component(control_link_vsomeip_sd_real "${control_link_vsomeip_sd_library}" REALPATH)
get_filename_component(control_link_vsomeip_sd_dir "${control_link_vsomeip_sd_real}" DIRECTORY)
if(NOT control_link_vsomeip_sd_dir STREQUAL control_link_vsomeip_library_dir)
	message(FATAL_ERROR "ControlLink SOME/IP runtime must not mix vsomeip3 and Service Discovery installation prefixes")
endif()

# standalone 使用当前源码 codec；已有安装的同版本旧 archive 不得静默替代布局 API。
if(TARGET control_link_edge_protocol)
	set(control_link_someip_codec_target control_link_edge_protocol)
elseif(TARGET ControlLink::control_link_edge_protocol AND
	NOT CMAKE_SOURCE_DIR STREQUAL CMAKE_CURRENT_SOURCE_DIR)
	set(control_link_someip_codec_target ControlLink::control_link_edge_protocol)
else()
	set(CONTROL_LINK_EDGE_PROTOCOL_BUILD_CLIENT OFF)
	set(CONTROL_LINK_EDGE_PROTOCOL_BUILD_SDK OFF)
	set(CONTROL_LINK_EDGE_PROTOCOL_BUILD_SUPERVISOR OFF)
	set(CONTROL_LINK_EDGE_PROTOCOL_BUILD_TESTS OFF)
	add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/../edge_protocol
		${CMAKE_CURRENT_BINARY_DIR}/edge_protocol)
	set(control_link_someip_codec_target control_link_edge_protocol)
endif()

set(control_link_network_schema "${CMAKE_CURRENT_SOURCE_DIR}/../protocols/control_link_network_v1.proto")
set_property(DIRECTORY APPEND PROPERTY CMAKE_CONFIGURE_DEPENDS "${control_link_network_schema}")
file(SHA256 "${control_link_network_schema}" CONTROL_LINK_NETWORK_SCHEMA_SHA256_HEX)
set(control_link_network_generated_dir "${CMAKE_CURRENT_BINARY_DIR}/generated")
set(control_link_network_generated_source "${control_link_network_generated_dir}/control_link_network_v1.pb.cc")
set(control_link_network_generated_header "${control_link_network_generated_dir}/control_link_network_v1.pb.h")
add_custom_command(
	OUTPUT "${control_link_network_generated_source}" "${control_link_network_generated_header}"
	COMMAND ${CMAKE_COMMAND} -E make_directory "${control_link_network_generated_dir}"
	COMMAND protobuf::protoc
		"--proto_path=${CMAKE_CURRENT_SOURCE_DIR}/../protocols"
		"--cpp_out=${control_link_network_generated_dir}"
		"${control_link_network_schema}"
	DEPENDS "${control_link_network_schema}" protobuf::protoc
	VERBATIM
)

add_library(control_link_someip_transport STATIC
	src/config.cpp
	src/ingress.cpp
	src/server.cpp
	src/client.cpp
	"${control_link_network_generated_source}"
	"${control_link_network_generated_header}"
)
add_library(ControlLink::control_link_someip_transport ALIAS control_link_someip_transport)
set_target_properties(control_link_someip_transport PROPERTIES POSITION_INDEPENDENT_CODE ON)
target_compile_features(control_link_someip_transport PUBLIC cxx_std_17)
target_compile_options(control_link_someip_transport PRIVATE -Wall -Wextra -Wpedantic)
target_compile_definitions(control_link_someip_transport PRIVATE
	CONTROL_LINK_NETWORK_SCHEMA_SHA256_HEX="${CONTROL_LINK_NETWORK_SCHEMA_SHA256_HEX}")
target_include_directories(control_link_someip_transport PUBLIC
	$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
	$<BUILD_INTERFACE:${control_link_network_generated_dir}>
	$<INSTALL_INTERFACE:include>)
target_link_libraries(control_link_someip_transport PUBLIC
	vsomeip3
	protobuf::libprotobuf
	yaml-cpp
	Threads::Threads
	${control_link_someip_codec_target})

add_executable(someip_planning_client src/someip_planning_client.cpp)
target_compile_features(someip_planning_client PRIVATE cxx_std_17)
target_compile_options(someip_planning_client PRIVATE -Wall -Wextra -Wpedantic)
target_link_libraries(someip_planning_client PRIVATE control_link_someip_transport)

add_executable(someip_configure src/someip_configure.cpp)
target_compile_features(someip_configure PRIVATE cxx_std_17)
target_compile_options(someip_configure PRIVATE -Wall -Wextra -Wpedantic)
target_link_libraries(someip_configure PRIVATE control_link_someip_transport)

if(CONTROL_LINK_SOMEIP_BUILD_TESTS)
	include(CTest)
	if(BUILD_TESTING)
		find_package(GTest REQUIRED)
		foreach(test_name IN ITEMS config ingress application_runtime session_reply_order)
			add_executable(test_someip_${test_name} test/test_${test_name}.cpp)
			target_compile_features(test_someip_${test_name} PRIVATE cxx_std_17)
			target_compile_options(test_someip_${test_name} PRIVATE -Wall -Wextra -Wpedantic)
			target_compile_definitions(test_someip_${test_name} PRIVATE
				CONTROL_LINK_PROTOCOL_CONFIG_PATH="${CMAKE_CURRENT_SOURCE_DIR}/../config/adas/protocol_transports.yaml")
			target_link_libraries(test_someip_${test_name} PRIVATE
				control_link_someip_transport GTest::gtest_main)
			add_test(NAME test_someip_${test_name} COMMAND test_someip_${test_name})
		endforeach()
	endif()
endif()

include(GNUInstallDirs)
include(CMakePackageConfigHelpers)
install(TARGETS control_link_someip_transport
	EXPORT control_link_someip_transport_targets
	ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
	LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
	RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
	INCLUDES DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})
install(TARGETS someip_planning_client someip_configure RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR})
install(DIRECTORY include/ DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})
install(FILES "${control_link_network_generated_header}" DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})
install(FILES "${control_link_network_schema}"
	DESTINATION ${CMAKE_INSTALL_DATADIR}/${PROJECT_NAME}/protocols)
install(EXPORT control_link_someip_transport_targets
	FILE control_link_someip_transportTargets.cmake
	NAMESPACE ControlLink::
	DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/control_link_someip_transport)
configure_package_config_file(
	cmake/control_link_someip_transportConfig.cmake.in
	${CMAKE_CURRENT_BINARY_DIR}/control_link_someip_transportConfig.cmake
	INSTALL_DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/control_link_someip_transport)
write_basic_package_version_file(
	${CMAKE_CURRENT_BINARY_DIR}/control_link_someip_transportConfigVersion.cmake
	VERSION ${PROJECT_VERSION} COMPATIBILITY SameMajorVersion)
install(FILES
	${CMAKE_CURRENT_BINARY_DIR}/control_link_someip_transportConfig.cmake
	${CMAKE_CURRENT_BINARY_DIR}/control_link_someip_transportConfigVersion.cmake
	DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/control_link_someip_transport)

message(STATUS "ControlLink SOME/IP RUNTIME_AVAILABLE: control_link_someip_transport, someip_planning_client and someip_configure configured; runtime acceptance still requires actual SD/TCP/Gateway/vcan validation")
