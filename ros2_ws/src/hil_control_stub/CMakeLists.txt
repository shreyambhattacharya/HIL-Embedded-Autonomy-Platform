cmake_minimum_required(VERSION 3.16)
project(hil_control_stub)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

add_compile_options(-Wall -Wextra -Wpedantic -Wconversion -Wsign-conversion)

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)

add_library(hil_control_math src/control_math.cpp)
target_include_directories(
  hil_control_math PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

add_executable(software_mcu_stub src/software_mcu_stub.cpp)
target_link_libraries(software_mcu_stub hil_control_math)
target_include_directories(software_mcu_stub PRIVATE include)
ament_target_dependencies(software_mcu_stub geometry_msgs rclcpp sensor_msgs std_msgs)

add_executable(motion_test_node src/motion_test_node.cpp)
target_include_directories(motion_test_node PRIVATE include)
ament_target_dependencies(motion_test_node geometry_msgs rclcpp)

install(
  TARGETS hil_control_math software_mcu_stub motion_test_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_control_math test/test_control_math.cpp)
  target_link_libraries(test_control_math hil_control_math)
  target_include_directories(test_control_math PRIVATE include)
endif()

ament_export_include_directories(include)
ament_package()
