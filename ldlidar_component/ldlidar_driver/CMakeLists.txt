################################################
## Generate symbols for IDE indexer (VSCode)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

################################################
## Add the -fPIC option
set_property(TARGET ldlidar PROPERTY POSITION_INDEPENDENT_CODE ON)

#########################################################
# SOURCES
file(GLOB_RECURSE DRIVER_SRC *.c *.cpp)
# HEADERS
file(GLOB_RECURSE DRIVER_HDR *.h *.hpp)
#########################################################

add_library( ldlidar 
  ${DRIVEDRIVER_HDR_SRC} 
  ${DRIVER_SRC}
)

# Make sure the compiler can find include files for our library
# when other libraries or executables link to it
target_include_directories( ldlidar PUBLIC
  ${CMAKE_CURRENT_SOURCE_DIR}/include
  ${CMAKE_CURRENT_SOURCE_DIR}/include/core
  ${CMAKE_CURRENT_SOURCE_DIR}/include/dataprocess
  ${CMAKE_CURRENT_SOURCE_DIR}/include/filter
  ${CMAKE_CURRENT_SOURCE_DIR}/include/logger
  ${CMAKE_CURRENT_SOURCE_DIR}/include/networkcom
  ${CMAKE_CURRENT_SOURCE_DIR}/include/serialcom
)