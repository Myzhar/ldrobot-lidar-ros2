cmake_minimum_required(VERSION 3.5)
project(ldlidar_component)

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

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${CMAKE_CURRENT_SOURCE_DIR}/cmake-modules)

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
)

find_package(ament_cmake REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(lifecycle_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(diagnostic_updater REQUIRED)

find_package(udev REQUIRED)

# create ament index resource which references the libraries in the binary dir
set(node_plugins "")

###############################################################################
#Add all files in subdirectories of the project in
# a dummy_target so qtcreator have access to all files
FILE(GLOB_RECURSE all_files ${CMAKE_SOURCE_DIR}/*)
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

set(LIDAR_DRIVER_INC
    ${CMAKE_CURRENT_SOURCE_DIR}/driver/include/cmd_interface_linux.hpp
    ${CMAKE_CURRENT_SOURCE_DIR}/driver/include/lipkg.hpp
    ${CMAKE_CURRENT_SOURCE_DIR}/driver/include/tofbf.hpp
)

set(LIDAR_DRIVER_SRC
    ${CMAKE_CURRENT_SOURCE_DIR}/driver/src/cmd_interface_linux.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/driver/src/lipkg.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/driver/src/tofbf.cpp
)

###############################################################################
# Bin and Install

add_library(${PROJECT_NAME} SHARED    
    ${TOOLS_INC}
    ${TOOLS_SRC}
    ${COMPONENT_INC}
    ${COMPONENT_SRC}
    ${LIDAR_DRIVER_INC}
    ${LIDAR_DRIVER_SRC}    
)
target_include_directories(${PROJECT_NAME}
    PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${CMAKE_CURRENT_SOURCE_DIR}/driver/include
    ${CMAKE_CURRENT_SOURCE_DIR}/tools/include
    ${CMAKE_CURRENT_SOURCE_DIR}/component/include
    ${UDEV_INCLUDE_DIRS}
)
target_compile_definitions(${PROJECT_NAME}
    PRIVATE "COMPOSITION_BUILDING_DLL"
)
target_link_libraries(${PROJECT_NAME}
    ${UDEV_LIBRARIES}
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
ament_export_libraries(${PROJECT_NAME})
ament_export_dependencies(${DEPENDENCIES})
ament_package()
