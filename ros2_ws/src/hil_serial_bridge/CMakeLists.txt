cmake_minimum_required(VERSION 3.16)
project(hil_serial_bridge)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

add_compile_options(-Wall -Wextra -Wpedantic -Wconversion -Wsign-conversion)

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)

set(COMMON_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../../../common")
add_library(hil_protocol STATIC "${COMMON_DIR}/hil_protocol/src/hil_protocol.c")
target_include_directories(hil_protocol PUBLIC "${COMMON_DIR}/hil_protocol/include")
target_compile_options(hil_protocol PRIVATE -Wall -Wextra -Wpedantic -Wconversion -Wsign-conversion)

add_executable(hil_serial_bridge src/hil_serial_bridge.cpp)
target_include_directories(hil_serial_bridge PRIVATE "${COMMON_DIR}/hil_protocol/include")
target_link_libraries(hil_serial_bridge hil_protocol)
ament_target_dependencies(
  hil_serial_bridge
  geometry_msgs
  rclcpp
  sensor_msgs
  std_msgs
  std_srvs
)

install(TARGETS hil_protocol hil_serial_bridge
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)
ament_package()
