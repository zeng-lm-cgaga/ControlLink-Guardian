cmake_minimum_required(VERSION 3.16)
project(control_link_grpc_transport VERSION 0.1.0 LANGUAGES CXX)

option(CONTROL_LINK_GRPC_BUILD_TESTS "Build focused real async gRPC lifecycle tests" OFF)
option(CONTROL_LINK_GRPC_ENABLE_SANITIZERS "Instrument ControlLink async gRPC call ownership" OFF)
include(${CMAKE_CURRENT_SOURCE_DIR}/../ros2_ws/src/control_link_adapters/cmake/ControlLinkNetworkDependencyLock.cmake)
find_package(gRPC ${CONTROL_LINK_GRPC_VERSION} EXACT CONFIG REQUIRED)
find_package(Protobuf ${CONTROL_LINK_PROTOBUF_VERSION} EXACT MODULE REQUIRED)
find_package(Threads REQUIRED)
if(NOT TARGET gRPC::grpc++ OR NOT TARGET gRPC::grpc_cpp_plugin OR NOT TARGET protobuf::protoc)
	message(FATAL_ERROR "ControlLink gRPC runtime requires installed grpc++, grpc_cpp_plugin and locked protoc targets")
endif()
if(NOT TARGET control_link_network_ingress AND NOT TARGET ControlLink::control_link_network_ingress)
	add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/../network_ingress ${CMAKE_CURRENT_BINARY_DIR}/network_ingress)
endif()

set(control_link_grpc_schema ${CMAKE_CURRENT_SOURCE_DIR}/../protocols/control_link_network_v1.proto)
set(control_link_grpc_generated_dir ${CMAKE_CURRENT_BINARY_DIR}/generated)
set(control_link_grpc_generated_source ${control_link_grpc_generated_dir}/control_link_network_v1.grpc.pb.cc)
set(control_link_grpc_generated_header ${control_link_grpc_generated_dir}/control_link_network_v1.grpc.pb.h)
add_custom_command(
	OUTPUT ${control_link_grpc_generated_source} ${control_link_grpc_generated_header}
	COMMAND ${CMAKE_COMMAND} -E make_directory ${control_link_grpc_generated_dir}
	COMMAND protobuf::protoc --proto_path=${CMAKE_CURRENT_SOURCE_DIR}/../protocols
		--grpc_out=${control_link_grpc_generated_dir}
		--plugin=protoc-gen-grpc=$<TARGET_FILE:gRPC::grpc_cpp_plugin> ${control_link_grpc_schema}
	DEPENDS ${control_link_grpc_schema} protobuf::protoc gRPC::grpc_cpp_plugin
	VERBATIM)

add_library(control_link_grpc_transport STATIC src/config.cpp src/client.cpp src/server.cpp
	${control_link_grpc_generated_source} ${control_link_grpc_generated_header})
add_library(ControlLink::control_link_grpc_transport ALIAS control_link_grpc_transport)
set_target_properties(control_link_grpc_transport PROPERTIES POSITION_INDEPENDENT_CODE ON)
target_compile_features(control_link_grpc_transport PUBLIC cxx_std_17)
target_compile_options(control_link_grpc_transport PRIVATE -Wall -Wextra -Wpedantic)
# 固定依赖以非ASan ABI构建；上游宏仅关闭额外gpr_mu leak_checker布局，不关闭本项目ASan。
target_compile_definitions(control_link_grpc_transport PUBLIC GRPC_ASAN_SUPPRESSED)
target_include_directories(control_link_grpc_transport PUBLIC
	$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
	$<BUILD_INTERFACE:${control_link_grpc_generated_dir}>
	$<INSTALL_INTERFACE:include>)
target_link_libraries(control_link_grpc_transport PUBLIC gRPC::grpc++
	ControlLink::control_link_network_ingress Threads::Threads)
if(CONTROL_LINK_GRPC_ENABLE_SANITIZERS)
	target_compile_options(control_link_grpc_transport PRIVATE -fsanitize=address,undefined -fno-omit-frame-pointer)
	target_link_options(control_link_grpc_transport PUBLIC -fsanitize=address,undefined)
endif()

add_executable(grpc_planning_client src/grpc_planning_client.cpp)
target_link_libraries(grpc_planning_client PRIVATE control_link_grpc_transport)
add_executable(grpc_async_planning_client examples/grpc_planning_consumer/async_main.cpp)
target_link_libraries(grpc_async_planning_client PRIVATE control_link_grpc_transport)
if(CONTROL_LINK_GRPC_BUILD_TESTS)
	include(CTest)
	if(BUILD_TESTING)
		find_package(GTest REQUIRED)
		add_executable(test_grpc_lifecycle test/test_grpc_lifecycle.cpp)
		target_include_directories(test_grpc_lifecycle PRIVATE src)
		target_compile_definitions(test_grpc_lifecycle PRIVATE
			CONTROL_LINK_PROTOCOL_CONFIG_PATH="${CMAKE_CURRENT_SOURCE_DIR}/../config/adas/protocol_transports.yaml")
		target_link_libraries(test_grpc_lifecycle PRIVATE control_link_grpc_transport GTest::gtest_main)
		add_test(NAME test_grpc_lifecycle COMMAND test_grpc_lifecycle)
	endif()
endif()

include(GNUInstallDirs)
include(CMakePackageConfigHelpers)
install(TARGETS control_link_grpc_transport EXPORT control_link_grpc_transport_targets
	ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR} LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
	RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR} INCLUDES DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})
install(TARGETS grpc_planning_client grpc_async_planning_client RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR})
install(DIRECTORY include/ DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})
install(FILES ${control_link_grpc_generated_header} DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})
install(EXPORT control_link_grpc_transport_targets FILE control_link_grpc_transportTargets.cmake
	NAMESPACE ControlLink:: DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/control_link_grpc_transport)
configure_package_config_file(cmake/control_link_grpc_transportConfig.cmake.in
	${CMAKE_CURRENT_BINARY_DIR}/control_link_grpc_transportConfig.cmake
	INSTALL_DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/control_link_grpc_transport)
write_basic_package_version_file(${CMAKE_CURRENT_BINARY_DIR}/control_link_grpc_transportConfigVersion.cmake
	VERSION ${PROJECT_VERSION} COMPATIBILITY SameMajorVersion)
install(FILES ${CMAKE_CURRENT_BINARY_DIR}/control_link_grpc_transportConfig.cmake
	${CMAKE_CURRENT_BINARY_DIR}/control_link_grpc_transportConfigVersion.cmake
	DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/control_link_grpc_transport)
message(STATUS "ControlLink gRPC RUNTIME_AVAILABLE targets configured; RPC readiness does not prove Gateway/vcan acceptance")
