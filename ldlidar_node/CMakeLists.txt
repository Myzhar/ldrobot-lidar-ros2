cmake_minimum_required(VERSION 3.5)
project(ldlidar_node)

################################################
## Generate symbols for IDE indexer (VSCode)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

################################################
# Check the ROS2 version

set(ROS2_FOUND FALSE)
if(DEFINED ENV{ROS_DISTRO})
  set(FOUND_ROS2_DISTRO $ENV{ROS_DISTRO})
  set(ROS2_FOUND TRUE)
  message(STATUS "* Found ROS2 ${FOUND_ROS2_DISTRO}")
else()
  message(WARNING "* ROS2 distro variable not set. Trying to figure it out...")
  set(ROS2_DISTROS "ardent;crystal;dashing;eloquent;foxy;galactic;humble;iron;jazzy;rolling")
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
  else()
    message(FATAL_ERROR "ROS2 ${FOUND_ROS2_DISTRO} is not officially supported by this package. Correct working is not guarantee.")
  endif()
else()
  message(FATAL_ERROR "*** WARNING *** ROS2 distro is unknown. This package could not work correctly.")
endif()
################################################

#############################################
# Dependencies

find_package(ament_cmake REQUIRED)

###############################################################################
# Install

# Install PARAMS files
install(DIRECTORY
  params
  DESTINATION share/${PROJECT_NAME}
)

# Install RVIZ files
install(DIRECTORY
  config
  DESTINATION share/${PROJECT_NAME}
)

# Install URDF files
install(DIRECTORY
  urdf
  DESTINATION share/${PROJECT_NAME}
)

# Install LAUNCH files
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

# Testing
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

ament_package()
