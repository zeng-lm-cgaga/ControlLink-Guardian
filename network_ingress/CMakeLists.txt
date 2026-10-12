cmake_minimum_required(VERSION 3.16)
project(control_link_network_ingress VERSION 0.1.0 LANGUAGES CXX)

option(CONTROL_LINK_NETWORK_INGRESS_BUILD_TESTS
	"Build the ROS-free shared ingress focused tests"
	OFF)

include(${CMAKE_CURRENT_SOURCE_DIR}/../ros2_ws/src/control_link_adapters/cmake/ControlLinkNetworkDependencyLock.cmake)
find_package(Protobuf ${CONTROL_LINK_PROTOBUF_VERSION} EXACT MODULE REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(Boost 1.55 REQUIRED)
find_package(Threads REQUIRED)
if(NOT TARGET protobuf::libprotobuf OR NOT TARGET protobuf::protoc OR NOT TARGET Boost::boost)
	message(FATAL_ERROR "ControlLink network ingress requires real protobuf::libprotobuf, protobuf::protoc and Boost headers targets")
endif()

get_target_property(control_link_protoc_location protobuf::protoc IMPORTED_LOCATION)
if(NOT control_link_protoc_location OR NOT EXISTS "${control_link_protoc_location}")
	message(FATAL_ERROR "ControlLink network ingress requires a real installed protobuf::protoc executable")
endif()
execute_process(COMMAND "${control_link_protoc_location}" --version
	RESULT_VARIABLE control_link_protoc_result
	OUTPUT_VARIABLE control_link_protoc_version
	ERROR_VARIABLE control_link_protoc_error
	OUTPUT_STRIP_TRAILING_WHITESPACE TIMEOUT 10)
if(NOT control_link_protoc_result EQUAL 0 OR
	NOT control_link_protoc_version STREQUAL "libprotoc ${CONTROL_LINK_PROTOBUF_VERSION}")
	message(FATAL_ERROR "ControlLink network ingress requires protoc/header/library version ${CONTROL_LINK_PROTOBUF_VERSION}; got=${control_link_protoc_version}; exit=${control_link_protoc_result}; ${control_link_protoc_error}")
endif()

# 复用唯一 codec target；standalone 只装配 ROS-free codec，绝不查找 EdgeRuntime。
if(TARGET control_link_edge_protocol)
	set(control_link_network_codec_target control_link_edge_protocol)
elseif(TARGET ControlLink::control_link_edge_protocol)
	set(control_link_network_codec_target ControlLink::control_link_edge_protocol)
else()
	set(CONTROL_LINK_EDGE_PROTOCOL_BUILD_CLIENT OFF)
	set(CONTROL_LINK_EDGE_PROTOCOL_BUILD_SDK OFF)
	set(CONTROL_LINK_EDGE_PROTOCOL_BUILD_SUPERVISOR OFF)
	set(CONTROL_LINK_EDGE_PROTOCOL_BUILD_TESTS OFF)
	add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/../edge_protocol
		${CMAKE_CURRENT_BINARY_DIR}/edge_protocol)
	set(control_link_network_codec_target control_link_edge_protocol)
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

add_library(control_link_network_ingress STATIC
	../someip_transport/src/config.cpp
	../someip_transport/src/ingress.cpp
	src/schema.cpp
	"${control_link_network_generated_source}"
	"${control_link_network_generated_header}"
)
add_library(ControlLink::control_link_network_ingress ALIAS control_link_network_ingress)
set_target_properties(control_link_network_ingress PROPERTIES
	POSITION_INDEPENDENT_CODE ON
	CONTROL_LINK_NETWORK_SCHEMA_SHA256_HEX "${CONTROL_LINK_NETWORK_SCHEMA_SHA256_HEX}"
	EXPORT_PROPERTIES CONTROL_LINK_NETWORK_SCHEMA_SHA256_HEX)
target_compile_features(control_link_network_ingress PUBLIC cxx_std_17)
target_compile_options(control_link_network_ingress PRIVATE -Wall -Wextra -Wpedantic)
target_compile_definitions(control_link_network_ingress PRIVATE
	CONTROL_LINK_NETWORK_SCHEMA_SHA256_HEX="${CONTROL_LINK_NETWORK_SCHEMA_SHA256_HEX}")
target_include_directories(control_link_network_ingress PUBLIC
	$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
	$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/../someip_transport/include>
	$<BUILD_INTERFACE:${control_link_network_generated_dir}>
	$<INSTALL_INTERFACE:include>)
target_link_libraries(control_link_network_ingress
	PUBLIC protobuf::libprotobuf yaml-cpp Threads::Threads ${control_link_network_codec_target}
	PRIVATE Boost::boost)

if(CONTROL_LINK_NETWORK_INGRESS_BUILD_TESTS)
	include(CTest)
	if(BUILD_TESTING)
		find_package(GTest REQUIRED)
		add_executable(test_network_ingress test/test_ingress.cpp)
		target_compile_features(test_network_ingress PRIVATE cxx_std_17)
		target_compile_options(test_network_ingress PRIVATE -Wall -Wextra -Wpedantic)
		target_compile_definitions(test_network_ingress PRIVATE
			CONTROL_LINK_PROTOCOL_CONFIG_PATH="${CMAKE_CURRENT_SOURCE_DIR}/../config/adas/protocol_transports.yaml")
		target_link_libraries(test_network_ingress PRIVATE control_link_network_ingress GTest::gtest_main)
		add_test(NAME test_network_ingress COMMAND test_network_ingress)
	endif()
endif()

include(GNUInstallDirs)
include(CMakePackageConfigHelpers)
install(TARGETS control_link_network_ingress
	EXPORT control_link_network_ingress_targets
	ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
	LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
	RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
	INCLUDES DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})
install(DIRECTORY include/ DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})
# common 是这两个历史公共头与 message 头的唯一安装 owner；SOME/IP 只安装 transport API。
install(FILES
	../someip_transport/include/control_link_someip_transport/config.hpp
	../someip_transport/include/control_link_someip_transport/ingress.hpp
	DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/control_link_someip_transport)
install(FILES "${control_link_network_generated_header}" DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})
install(FILES "${control_link_network_schema}"
	DESTINATION ${CMAKE_INSTALL_DATADIR}/${PROJECT_NAME}/protocols)
install(EXPORT control_link_network_ingress_targets
	FILE control_link_network_ingressTargets.cmake
	NAMESPACE ControlLink::
	DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/control_link_network_ingress)
configure_package_config_file(
	cmake/control_link_network_ingressConfig.cmake.in
	${CMAKE_CURRENT_BINARY_DIR}/control_link_network_ingressConfig.cmake
	INSTALL_DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/control_link_network_ingress)
write_basic_package_version_file(
	${CMAKE_CURRENT_BINARY_DIR}/control_link_network_ingressConfigVersion.cmake
	VERSION ${PROJECT_VERSION} COMPATIBILITY SameMajorVersion)
install(FILES
	${CMAKE_CURRENT_BINARY_DIR}/control_link_network_ingressConfig.cmake
	${CMAKE_CURRENT_BINARY_DIR}/control_link_network_ingressConfigVersion.cmake
	DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/control_link_network_ingress)

message(STATUS "ControlLink shared network ingress configured: protobuf/codec/session target only; no SOME/IP/SD/gRPC/ROS/EdgeRuntime runtime dependency")
