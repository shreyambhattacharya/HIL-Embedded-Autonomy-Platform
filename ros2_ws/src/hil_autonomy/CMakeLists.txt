cmake_minimum_required(VERSION 3.16)
project(hil_autonomy)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

add_compile_options(-Wall -Wextra -Wpedantic -Wconversion -Wsign-conversion)

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)

add_library(hil_autonomy_core
  src/state_estimator.cpp
  src/waypoint_controller.cpp
  src/collision_filter.cpp
)
target_include_directories(hil_autonomy_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

add_executable(state_estimator_node src/state_estimator_node.cpp)
target_link_libraries(state_estimator_node hil_autonomy_core)
ament_target_dependencies(state_estimator_node nav_msgs rclcpp sensor_msgs std_msgs)

add_executable(waypoint_follower_node src/waypoint_follower_node.cpp)
target_link_libraries(waypoint_follower_node hil_autonomy_core)
ament_target_dependencies(waypoint_follower_node nav_msgs rclcpp geometry_msgs std_msgs std_srvs)

add_executable(collision_filter_node src/collision_filter_node.cpp)
target_link_libraries(collision_filter_node hil_autonomy_core)
ament_target_dependencies(collision_filter_node geometry_msgs rclcpp sensor_msgs std_msgs)

install(TARGETS
  hil_autonomy_core
  state_estimator_node
  waypoint_follower_node
  collision_filter_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY config launch worlds DESTINATION share/${PROJECT_NAME})
install(PROGRAMS scripts/milestone_05a_evaluator.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME milestone_05a_evaluator)
install(PROGRAMS scripts/scan_gate.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME scan_gate)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_autonomy_core test/test_autonomy_core.cpp)
  target_link_libraries(test_autonomy_core hil_autonomy_core)
  target_include_directories(test_autonomy_core PRIVATE include)
endif()

ament_export_include_directories(include)
ament_export_libraries(hil_autonomy_core)
ament_package()
