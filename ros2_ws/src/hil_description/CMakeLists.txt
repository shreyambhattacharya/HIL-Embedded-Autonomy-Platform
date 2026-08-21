cmake_minimum_required(VERSION 3.16)
project(hil_description)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY models
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
