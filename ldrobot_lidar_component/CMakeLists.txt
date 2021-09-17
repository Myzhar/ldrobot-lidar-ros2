cmake_minimum_required(VERSION 3.5)
project(ldrobot_lidar_component)

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
IF(NOT CMAKE_BUILD_TYPE)
    SET(CMAKE_BUILD_TYPE Release)
ENDIF(NOT CMAKE_BUILD_TYPE)

if(CMAKE_BUILD_TYPE MATCHES Release )
    #message(" * Release Mode")
    add_compile_options(-Wno-deprecated-declarations)
endif()

if(CMAKE_BUILD_TYPE MATCHES RelWithDebInfo )
    #message(" * Release with Debug Info Mode")
    add_compile_options(-Wno-deprecated-declarations)
endif()

if(CMAKE_BUILD_TYPE MATCHES Debug )
    message(" * Debug Mode")
endif()

#############################################
# Dependencies

set(DEPENDENCIES
    rclcpp_lifecycle
    lifecycle_msgs
    rclcpp
    rclcpp_components
    std_msgs
    sensor_msgs
)

find_package(ament_cmake REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(lifecycle_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

###############################################################################
#Add all files in subdirectories of the project in
# a dummy_target so qtcreator have access to all files
FILE(GLOB_RECURSE all_files ${CMAKE_SOURCE_DIR}/*)
add_custom_target(all_${PROJECT_NAME}_files SOURCES ${all_files})

###############################################################################
# INCLUDES and LIBS
include_directories(
    ${CMAKE_CURRENT_SOURCE_DIR}/src/include
    ${CMAKE_CURRENT_SOURCE_DIR}/src/driver/include
    ${CMAKE_CURRENT_SOURCE_DIR}/src/tools/include
    ${CMAKE_CURRENT_SOURCE_DIR}/src/lidar_component/include
)

# create ament index resource which references the libraries in the binary dir
set(node_plugins "")

###############################################################################
# SOURCES
set(LIDAR_TOOLS
    ${CMAKE_CURRENT_SOURCE_DIR}/tools/include/ldlidar_tools.hpp
    ${CMAKE_CURRENT_SOURCE_DIR}/tools/src/ldlidar_tools.cpp
)

set(LIDAR_INC
    ${CMAKE_CURRENT_SOURCE_DIR}/include/visibility_control.hpp
    ${CMAKE_CURRENT_SOURCE_DIR}/ldlidar_component/include/ldlidar_component.hpp
)

set(LIDAR_SRC
    ${CMAKE_CURRENT_SOURCE_DIR}/ldlidar_component/src/ldlidar_component.cpp
)

# TODO add driver files
set(LIDAR_DRIVER_INC
    ${CMAKE_CURRENT_SOURCE_DIR}/driver/include/cmd_interface_linux.hpp
    ${CMAKE_CURRENT_SOURCE_DIR}/driver/include/lipkg.hpp
    ${CMAKE_CURRENT_SOURCE_DIR}/driver/include/tofbf.hpp
)

set(LIDAR_DRIVER_SRC
    ${CMAKE_CURRENT_SOURCE_DIR}/driver/include/cmd_interface_linux.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/driver/include/lipkg.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/driver/include/tofbf.cpp
)

###############################################################################
# Bin and Install

add_library(ldrobot_lidar_component SHARED
    ${LIDAR_TOOLS}
    ${LIDAR_INC}
    ${LIDAR_SRC}
    ${LIDAR_DRIVER_INC}
    ${LIDAR_DRIVER_SRC}
)
target_compile_definitions(ldrobot_lidar_component
    PRIVATE "COMPOSITION_BUILDING_DLL"
)
target_link_libraries(ldrobot_lidar_component
)
ament_target_dependencies(ldrobot_lidar_component
    ${DEPENDENCIES}
)

rclcpp_components_register_nodes(ldrobot_lidar_component "ldrobot::LidarLifecycle")
set(node_plugins "${node_plugins}ldrobot::LidarLifecycle;$<TARGET_FILE:ldrobot_lidar_component>\n")

# Install components
install(TARGETS ldrobot_lidar_component
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

# Install header files
install(DIRECTORY 
    ${CMAKE_CURRENT_SOURCE_DIR}/lidar_component/include/
    ${CMAKE_CURRENT_SOURCE_DIR}/tools/include/
    ${CMAKE_CURRENT_SOURCE_DIR}/driver/include/
    ${CMAKE_CURRENT_SOURCE_DIR}/include/
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
ament_export_libraries(ldrobot_lidar_component)
ament_export_dependencies(${DEPENDENCIES})
ament_package()
