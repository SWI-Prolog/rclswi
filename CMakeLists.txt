cmake_minimum_required(VERSION 3.5)

project(rclswi)

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_CURRENT_SOURCE_DIR}/cmake")
find_package(SWI-Prolog REQUIRED)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE "Debug" CACHE STRING
      "Choose the type of build, options are: Debug Release RelWithDebInfo MinSizeRel."
      FORCE)
endif(NOT CMAKE_BUILD_TYPE)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()
# Default to C11
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 11)
endif()
if(CMAKE_COMPILER_IS_GNUCC OR CMAKE_C_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall)
endif()

find_package(ament_cmake REQUIRED)
find_package(rcl REQUIRED)
find_package(rcl_action REQUIRED)
find_package(rcl_yaml_param_parser REQUIRED)
find_package(rcutils REQUIRED)
find_package(rmw REQUIRED)
find_package(rmw_implementation_cmake REQUIRED)

# Only build the library if a C typesupport exists
get_rmw_typesupport(typesupport_impls "rmw_implementation" LANGUAGE "c")
if(typesupport_impls STREQUAL "")
  message(STATUS "Skipping rclswi because no C typesupport library was found.")
  return()
endif()

add_library(rclswi SHARED
  src/rclswi.c
  src/logging.c
  src/pointer.c
  src/qos.c
)
target_link_libraries(rclswi
  SWIPL::LIBSWIPL
)
target_include_directories(rclswi
  PUBLIC
  src
)
ament_target_dependencies(rclswi
  "rcl"
  "rmw"
  "rcl_action"
)

# Causes the visibility macros to use dllexport rather than dllimport,
# which is appropriate when building the dll but not consuming it.
target_compile_definitions(rclswi PRIVATE "RCLPY_COMMON_BUILDING_DLL")

install(TARGETS rclswi
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(PROGRAMS scripts/rclswi
  DESTINATION lib/rclswi
)
install(FILES prolog/ros.pl
  DESTINATION prolog
)
install(FILES prolog/ros/logging.pl
	      prolog/ros/services.pl
	      prolog/ros/actions.pl
	      prolog/ros/clocks.pl
	      prolog/ros/qos.pl
  DESTINATION prolog/ros
)

ament_package()
