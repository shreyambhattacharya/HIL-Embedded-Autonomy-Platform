cmake_minimum_required(VERSION 3.16)
project(hil_simulation)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY config launch worlds
  DESTINATION share/${PROJECT_NAME}
)

install(
  PROGRAMS scripts/milestone_01_smoke_test.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME milestone_01_smoke_test
)

ament_package()
