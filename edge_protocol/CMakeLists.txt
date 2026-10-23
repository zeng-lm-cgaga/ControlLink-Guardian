cmake_minimum_required(VERSION 3.16)

set(CONTROL_LINK_EDGE_PROTOCOL_VERSION 0.1.0)

if(CMAKE_SOURCE_DIR STREQUAL CMAKE_CURRENT_SOURCE_DIR)
	project(control_link_edge_protocol LANGUAGES CXX)
	set(CONTROL_LINK_EDGE_PROTOCOL_VERSION 0.1.0)
	option(CONTROL_LINK_EDGE_PROTOCOL_BUILD_CLIENT
		"Build the standalone EdgeRuntime planning client"
		ON)
	option(CONTROL_LINK_EDGE_PROTOCOL_BUILD_SDK
		"Build and install the EdgeRuntime-backed C++ client SDK"
		ON)
	option(CONTROL_LINK_EDGE_PROTOCOL_BUILD_SUPERVISOR
		"Build the standalone EdgeRuntime producer supervisor"
		ON)
	option(CONTROL_LINK_EDGE_PROTOCOL_BUILD_TESTS
		"Build the ROS-free protocol tests"
		ON)
	if(CONTROL_LINK_EDGE_PROTOCOL_BUILD_CLIENT OR
		CONTROL_LINK_EDGE_PROTOCOL_BUILD_SDK OR
		CONTROL_LINK_EDGE_PROTOCOL_BUILD_SUPERVISOR)
		find_package(EdgeRuntime CONFIG REQUIRED)
	endif()
else()
	if(NOT DEFINED CONTROL_LINK_EDGE_PROTOCOL_BUILD_CLIENT)
		set(CONTROL_LINK_EDGE_PROTOCOL_BUILD_CLIENT OFF)
	endif()
	if(NOT DEFINED CONTROL_LINK_EDGE_PROTOCOL_BUILD_SDK)
		set(CONTROL_LINK_EDGE_PROTOCOL_BUILD_SDK OFF)
	endif()
	if(NOT DEFINED CONTROL_LINK_EDGE_PROTOCOL_BUILD_SUPERVISOR)
		set(CONTROL_LINK_EDGE_PROTOCOL_BUILD_SUPERVISOR OFF)
	endif()
	if(NOT DEFINED CONTROL_LINK_EDGE_PROTOCOL_BUILD_TESTS)
		set(CONTROL_LINK_EDGE_PROTOCOL_BUILD_TESTS OFF)
	endif()
endif()

add_library(control_link_edge_protocol STATIC
	src/protocol.cpp
)
target_compile_features(control_link_edge_protocol PUBLIC cxx_std_17)
target_include_directories(control_link_edge_protocol
	PUBLIC
		$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
		$<INSTALL_INTERFACE:include>
)
target_compile_options(control_link_edge_protocol PRIVATE -Wall -Wextra -Wpedantic)

if(CONTROL_LINK_EDGE_PROTOCOL_BUILD_SDK)
	include(CheckCXXSourceCompiles)
	set(CMAKE_REQUIRED_LIBRARIES EdgeRuntime::edge_runtime)
	unset(CONTROL_LINK_EDGE_NOTIFICATION_LINKABLE CACHE)
	check_cxx_source_compiles("#include <chrono>
#include <edge_runtime/channel/consumer.hpp>
#include <edge_runtime/channel/producer.hpp>
int main(int argc, char **argv) {
 if(argc > 100) {
  auto *notification=reinterpret_cast<edge_runtime::EventNotification *>(argv);
  auto *producer=reinterpret_cast<edge_runtime::Producer<unsigned long> *>(argv);
  auto *consumer=reinterpret_cast<edge_runtime::Consumer<unsigned long> *>(argv);
  notification->native_handle(); notification->drain();
  producer->enable_event_notifications(); producer->event_notification_error();
  consumer->event_notification(std::chrono::milliseconds(1));
 } return 0; }"
		CONTROL_LINK_EDGE_NOTIFICATION_LINKABLE)
	unset(CMAKE_REQUIRED_LIBRARIES)
	set(CONTROL_LINK_EDGE_DEFAULT_STALE_ONLY 0)
	if(SelfIPCs_VERSION VERSION_GREATER_EQUAL "0.5.0" OR EdgeRuntime_VERSION VERSION_GREATER_EQUAL "0.5.0")
		set(CONTROL_LINK_EDGE_DEFAULT_STALE_ONLY 1)
	endif()
	target_compile_definitions(control_link_edge_protocol PUBLIC
		CONTROL_LINK_EDGE_HAS_NOTIFICATION=$<BOOL:${CONTROL_LINK_EDGE_NOTIFICATION_LINKABLE}>
		CONTROL_LINK_EDGE_DEFAULT_STALE_ONLY=${CONTROL_LINK_EDGE_DEFAULT_STALE_ONLY})
	add_library(control_link_edge_client STATIC
		src/client.cpp
	)
	target_compile_features(control_link_edge_client PUBLIC cxx_std_17)
	target_include_directories(control_link_edge_client
		PUBLIC
			$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
			$<INSTALL_INTERFACE:include>
	)
	target_compile_options(control_link_edge_client PRIVATE -Wall -Wextra -Wpedantic)
	target_link_libraries(control_link_edge_client
		PUBLIC
			control_link_edge_protocol
			EdgeRuntime::edge_runtime
	)
endif()

if(CONTROL_LINK_EDGE_PROTOCOL_BUILD_CLIENT)
	add_executable(edge_planning_client
		src/edge_planning_client.cpp
	)
	target_compile_features(edge_planning_client PRIVATE cxx_std_17)
	target_compile_options(edge_planning_client PRIVATE -Wall -Wextra -Wpedantic)
	target_link_libraries(edge_planning_client
		PRIVATE
		control_link_edge_client
	)
endif()

if(CONTROL_LINK_EDGE_PROTOCOL_BUILD_SUPERVISOR)
	add_executable(edge_planning_supervisor
		src/edge_planning_supervisor.cpp
	)
	target_compile_features(edge_planning_supervisor PRIVATE cxx_std_17)
	target_compile_options(edge_planning_supervisor PRIVATE -Wall -Wextra -Wpedantic)
	target_link_libraries(edge_planning_supervisor
		PRIVATE
		EdgeRuntime::edge_runtime
	)
endif()

if(CONTROL_LINK_EDGE_PROTOCOL_BUILD_TESTS)
	include(CTest)
	if(BUILD_TESTING)
		add_executable(test_edge_protocol
			test/test_protocol.cpp
		)
		target_compile_features(test_edge_protocol PRIVATE cxx_std_17)
		target_compile_options(test_edge_protocol PRIVATE -Wall -Wextra -Wpedantic)
		 target_link_libraries(test_edge_protocol PRIVATE control_link_edge_protocol)
		add_test(NAME test_edge_protocol COMMAND test_edge_protocol)
		if(CONTROL_LINK_EDGE_PROTOCOL_BUILD_SDK)
			add_executable(test_edge_client
				test/test_client.cpp
			)
			target_compile_features(test_edge_client PRIVATE cxx_std_17)
			target_compile_options(test_edge_client PRIVATE -Wall -Wextra -Wpedantic)
			target_link_libraries(test_edge_client PRIVATE control_link_edge_client)
			add_test(NAME test_edge_client COMMAND test_edge_client)
			if(CONTROL_LINK_EDGE_NOTIFICATION_LINKABLE)
				add_test(NAME test_edge_client_eventfd COMMAND test_edge_client eventfd)
			endif()
			find_package(GTest QUIET)
			if(GTest_FOUND)
				add_executable(test_channel_wait test/test_channel_wait.cpp)
				target_link_libraries(test_channel_wait PRIVATE control_link_edge_client GTest::gtest_main)
				target_compile_options(test_channel_wait PRIVATE -Wall -Wextra -Wpedantic)
				if(CONTROL_LINK_EDGE_NOTIFICATION_LINKABLE)
					target_link_options(test_channel_wait PRIVATE -Wl,--wrap=write)
				endif()
				add_test(NAME test_channel_wait COMMAND test_channel_wait)
			endif()
		endif()
	endif()
endif()

install(
	DIRECTORY include/
	DESTINATION include
)
if(CONTROL_LINK_EDGE_PROTOCOL_BUILD_SDK)
	install(
		TARGETS control_link_edge_client
		EXPORT control_link_edge_client_targets
		ARCHIVE DESTINATION lib
		LIBRARY DESTINATION lib
		RUNTIME DESTINATION bin
		INCLUDES DESTINATION include
	)
endif()
install(
	TARGETS control_link_edge_protocol
	EXPORT control_link_edge_protocol_targets
	ARCHIVE DESTINATION lib
	LIBRARY DESTINATION lib
	RUNTIME DESTINATION bin
	INCLUDES DESTINATION include
)
if(CONTROL_LINK_EDGE_PROTOCOL_BUILD_CLIENT)
	install(TARGETS edge_planning_client RUNTIME DESTINATION bin)
endif()
if(CONTROL_LINK_EDGE_PROTOCOL_BUILD_SUPERVISOR)
	install(TARGETS edge_planning_supervisor RUNTIME DESTINATION bin)
endif()
install(
	EXPORT control_link_edge_protocol_targets
	FILE control_link_edge_protocolTargets.cmake
	NAMESPACE ControlLink::
	DESTINATION lib/cmake/control_link_edge_protocol
)
if(CONTROL_LINK_EDGE_PROTOCOL_BUILD_SDK)
	install(
		EXPORT control_link_edge_client_targets
		FILE control_link_edge_clientTargets.cmake
		NAMESPACE ControlLink::
		DESTINATION lib/cmake/control_link_edge_client
	)
endif()
include(CMakePackageConfigHelpers)
configure_package_config_file(
	cmake/control_link_edge_protocolConfig.cmake.in
	${CMAKE_CURRENT_BINARY_DIR}/control_link_edge_protocolConfig.cmake
	INSTALL_DESTINATION lib/cmake/control_link_edge_protocol
)
write_basic_package_version_file(
	${CMAKE_CURRENT_BINARY_DIR}/control_link_edge_protocolConfigVersion.cmake
	VERSION ${CONTROL_LINK_EDGE_PROTOCOL_VERSION}
	COMPATIBILITY SameMajorVersion
)
install(
	FILES
		${CMAKE_CURRENT_BINARY_DIR}/control_link_edge_protocolConfig.cmake
		${CMAKE_CURRENT_BINARY_DIR}/control_link_edge_protocolConfigVersion.cmake
	DESTINATION lib/cmake/control_link_edge_protocol
)
if(CONTROL_LINK_EDGE_PROTOCOL_BUILD_SDK)
	configure_package_config_file(
		cmake/control_link_edge_clientConfig.cmake.in
		${CMAKE_CURRENT_BINARY_DIR}/control_link_edge_clientConfig.cmake
		INSTALL_DESTINATION lib/cmake/control_link_edge_client
	)
	write_basic_package_version_file(
		${CMAKE_CURRENT_BINARY_DIR}/control_link_edge_clientConfigVersion.cmake
		VERSION ${CONTROL_LINK_EDGE_PROTOCOL_VERSION}
		COMPATIBILITY SameMajorVersion
	)
	install(
		FILES
			${CMAKE_CURRENT_BINARY_DIR}/control_link_edge_clientConfig.cmake
			${CMAKE_CURRENT_BINARY_DIR}/control_link_edge_clientConfigVersion.cmake
		DESTINATION lib/cmake/control_link_edge_client
	)
endif()
