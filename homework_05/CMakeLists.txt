cmake_minimum_required(VERSION 3.20)
project(telemetry CXX)

add_library(telemetry src/telemetry.cpp)
target_include_directories(telemetry PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_compile_options(telemetry PRIVATE -Wall -Wextra -pedantic)

add_executable(telemetry_check src/main.cpp)
target_link_libraries(telemetry_check PRIVATE telemetry)
target_compile_options(telemetry_check PRIVATE -Wall -Wextra -pedantic)
