cmake_minimum_required(VERSION 3.16)
project(hil_common C)

set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS OFF)

include(CTest)
enable_testing()

add_library(hil_safety STATIC hil_safety/src/hil_safety.c)
target_include_directories(hil_safety PUBLIC hil_safety/include hil_protocol/include)
target_compile_options(hil_safety PRIVATE -Wall -Wextra -Wpedantic -Wconversion -Wsign-conversion)
add_library(hil_protocol STATIC hil_protocol/src/hil_protocol.c)
target_include_directories(hil_protocol PUBLIC hil_protocol/include)
target_compile_options(hil_protocol PRIVATE -Wall -Wextra -Wpedantic -Wconversion -Wsign-conversion)

add_library(hil_control STATIC hil_control/src/hil_control.c)
target_include_directories(hil_control PUBLIC hil_control/include)
target_compile_options(hil_control PRIVATE -Wall -Wextra -Wpedantic -Wconversion -Wsign-conversion)

add_executable(test_protocol hil_protocol/tests/test_protocol.c)
target_link_libraries(test_protocol PRIVATE hil_protocol)
target_compile_options(test_protocol PRIVATE -Wall -Wextra -Wpedantic -Wconversion -Wsign-conversion)
add_test(NAME protocol COMMAND test_protocol)

add_executable(test_control hil_control/tests/test_control.c)
target_link_libraries(test_control PRIVATE hil_control m)
target_compile_options(test_control PRIVATE -Wall -Wextra -Wpedantic -Wconversion -Wsign-conversion)
add_test(NAME control COMMAND test_control)

add_executable(test_safety hil_safety/tests/test_safety.c)
target_link_libraries(test_safety PRIVATE hil_safety hil_protocol)
target_compile_options(test_safety PRIVATE -Wall -Wextra -Wpedantic -Wconversion -Wsign-conversion)
add_test(NAME safety COMMAND test_safety)
