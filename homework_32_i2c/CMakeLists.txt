cmake_minimum_required(VERSION 3.20)
project(hw32_i2c CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

add_executable(i2c_check
    src/main.cpp
    src/i2c_bus.cpp
    src/mpu6050.cpp
)
target_include_directories(i2c_check PRIVATE include)
target_compile_options(i2c_check PRIVATE -Wall -Wextra -pedantic)

find_package(Threads REQUIRED)
target_link_libraries(i2c_check PRIVATE Threads::Threads)