cmake_minimum_required(VERSION 3.5)
project(ldlidar_component)

################################################
## Generate symbols for IDE indexer (VSCode)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

#if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
#  add_compile_options(-Wall -Wextra -Wpedantic)
#endif()

# if CMAKE_BUILD_TYPE is not specified, take 'Release' as default
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif()

if(CMAKE_BUILD_TYPE MATCHES Release)
    message(STATUS " * Release Mode")
    add_compile_options(-Wno-deprecated-declarations)
endif()

if(CMAKE_BUILD_TYPE MATCHES RelWithDebInfo)
    message(STATUS " * Release with Debug Info Mode")
    add_compile_options(-Wno-deprecated-declarations)
endif()

if(CMAKE_BUILD_TYPE MATCHES Debug)
    message(STATUS " * Debug Mode")
endif()

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${CMAKE_CURRENT_SOURCE_DIR}/cmake-modules)

################################################
# Check the ROS2 version

set(ROS2_FOUND FALSE)
if(DEFINED ENV{ROS_DISTRO})
  set(FOUND_ROS2_DISTRO $ENV{ROS_DISTRO})
  set(ROS2_FOUND TRUE)
  message(STATUS "* Found ROS2 ${FOUND_ROS2_DISTRO}")
else()
  message(WARNING "* ROS2 distro variable not set. Trying to figure it out...")
  set(ROS2_DISTROS "ardent;crystal;dashing;eloquent;foxy;galactic;humble;iron;jazzy;rolling;kilted")
  set(ROS2_FOUND FALSE)
  foreach(distro ${ROS2_DISTROS})
    if(NOT ROS2_FOUND)
      find_path(RCLCPP_H rclcpp.hpp PATHS /opt/ros/${distro}/include/rclcpp)
      if(RCLCPP_H)
        message(STATUS "* Found ROS2 ${distro}")
        set(FOUND_ROS2_DISTRO ${distro})
        set(ROS2_FOUND TRUE)
      endif()
    endif()
  endforeach()
endif()

if(ROS2_FOUND)
  if(${FOUND_ROS2_DISTRO} STREQUAL "humble")
    message(STATUS "* ROS2 ${FOUND_ROS2_DISTRO} is officially supported by this package.")
    add_definitions(-DFOUND_HUMBLE)
  elseif(${FOUND_ROS2_DISTRO} STREQUAL "jazzy")
    message(STATUS "* ROS2 ${FOUND_ROS2_DISTRO} is officially supported by this package.")
    add_definitions(-DFOUND_JAZZY)
  #elseif(${FOUND_ROS2_DISTRO} STREQUAL "rolling")
  #  message(STATUS "* ROS2 ${FOUND_ROS2_DISTRO} is officially supported by this package.")
  #  add_definitions(-DFOUND_ROLLING)
  elseif(${FOUND_ROS2_DISTRO} STREQUAL "kilted")
    message(STATUS "* ROS2 ${FOUND_ROS2_DISTRO} is officially supported by this package.")
    add_definitions(-DFOUND_KILTED)
  else()
    message(FATAL_ERROR "ROS2 ${FOUND_ROS2_DISTRO} is not officially supported by this package. Correct working is not guarantee.")
  endif()
else()
  message(FATAL_ERROR "*** WARNING *** ROS2 distro is unknown. This package could not work correctly.")
endif()
################################################

#############################################
# Dependencies

set(DEPENDENCIES
    rclcpp_lifecycle
    rclcpp
    rclcpp_components
    lifecycle_msgs
    std_msgs
    sensor_msgs
    diagnostic_msgs
    diagnostic_updater
    nav2_util
    nav2_msgs
)

find_package(ament_cmake REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav2_util REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(diagnostic_updater REQUIRED)

# create ament index resource which references the libraries in the binary dir
set(node_plugins "")

###############################################################################
#Add all files in subdirectories of the project in
# a dummy_target so qtcreator have access to all files
file(GLOB_RECURSE all_files ${CMAKE_SOURCE_DIR}/*)
add_custom_target(all_${PROJECT_NAME}_files SOURCES ${all_files})


###############################################################################
# SOURCES
set(TOOLS_INC
    ${CMAKE_CURRENT_SOURCE_DIR}/tools/include/ldlidar_tools.hpp
)

set(TOOLS_SRC
    ${CMAKE_CURRENT_SOURCE_DIR}/tools/src/ldlidar_tools.cpp
)

set(COMPONENT_INC
    ${CMAKE_CURRENT_SOURCE_DIR}/include/visibility_control.hpp
    ${CMAKE_CURRENT_SOURCE_DIR}/component/include/ldlidar_component.hpp
)

set(COMPONENT_SRC
    ${CMAKE_CURRENT_SOURCE_DIR}/component/src/ldlidar_component.cpp
)

# LDLidar STL SDK (git submodule)
set(LDLIDAR_SDK_DIR ${CMAKE_CURRENT_SOURCE_DIR}/ldlidar_stl_sdk/ldlidar_driver)

file(GLOB_RECURSE LDLIDAR_SDK_HDR
    "${LDLIDAR_SDK_DIR}/include/*.h"
    "${LDLIDAR_SDK_DIR}/include/*.hpp"
)
file(GLOB_RECURSE LDLIDAR_SDK_SRC
    "${LDLIDAR_SDK_DIR}/src/*.c"
    "${LDLIDAR_SDK_DIR}/src/*.cpp"
)

add_library(ldlidar SHARED ${LDLIDAR_SDK_HDR} ${LDLIDAR_SDK_SRC})
set_property(TARGET ldlidar PROPERTY POSITION_INDEPENDENT_CODE ON)
# Upstream SDK omits #include <pthread.h> in its Linux branch — force-include it.
target_compile_options(ldlidar PRIVATE "-include" "pthread.h")
target_include_directories(ldlidar PUBLIC
    ${LDLIDAR_SDK_DIR}/include/core
    ${LDLIDAR_SDK_DIR}/include/dataprocess
    ${LDLIDAR_SDK_DIR}/include/filter
    ${LDLIDAR_SDK_DIR}/include/logger
    ${LDLIDAR_SDK_DIR}/include/networkcom
    ${LDLIDAR_SDK_DIR}/include/serialcom
)

include_directories(
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${CMAKE_CURRENT_SOURCE_DIR}/tools/include
    ${CMAKE_CURRENT_SOURCE_DIR}/component/include
    ${LDLIDAR_SDK_DIR}/include/core
    ${LDLIDAR_SDK_DIR}/include/dataprocess
    ${LDLIDAR_SDK_DIR}/include/filter
    ${LDLIDAR_SDK_DIR}/include/logger
    ${LDLIDAR_SDK_DIR}/include/networkcom
    ${LDLIDAR_SDK_DIR}/include/serialcom
    ${UDEV_INCLUDE_DIRS}
)

###############################################################################
# Bin and Install

add_library(${PROJECT_NAME} SHARED
    ${TOOLS_INC}
    ${TOOLS_SRC}
    ${COMPONENT_INC}
    ${COMPONENT_SRC}
)
target_link_libraries(${PROJECT_NAME}
  ldlidar
)
target_compile_definitions(${PROJECT_NAME}
    PRIVATE "COMPOSITION_BUILDING_DLL"
)
ament_target_dependencies(${PROJECT_NAME}
    ${DEPENDENCIES}
)

rclcpp_components_register_nodes(${PROJECT_NAME} "ldlidar::LdLidarComponent")
set(node_plugins "${node_plugins}ldlidar::LdLidarComponent;$<TARGET_FILE:${PROJECT_NAME}>\n")

# Install components
install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

# Install header files
install(DIRECTORY
    ${CMAKE_CURRENT_SOURCE_DIR}/component/include/
    ${CMAKE_CURRENT_SOURCE_DIR}/tools/include/
    ${CMAKE_CURRENT_SOURCE_DIR}/include/
    ${LDLIDAR_SDK_DIR}/include/core/
    ${LDLIDAR_SDK_DIR}/include/dataprocess/
    ${LDLIDAR_SDK_DIR}/include/filter/
    ${LDLIDAR_SDK_DIR}/include/logger/
    ${LDLIDAR_SDK_DIR}/include/networkcom/
    ${LDLIDAR_SDK_DIR}/include/serialcom/
    DESTINATION include/${PROJECT_NAME}/
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_export_dependencies(${DEPENDENCIES})
ament_package()
