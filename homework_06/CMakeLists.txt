cmake_minimum_required(VERSION 3.20)
project(balistics CXX)

add_library(balistics src/balistics.cpp)
target_include_directories(balistics PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_compile_options(balistics PRIVATE -Wall -Wextra -pedantic)

add_executable(balistics_check src/main.cpp)
target_link_libraries(balistics_check PRIVATE balistics)
target_compile_options(balistics_check PRIVATE -Wall -Wextra -pedantic)
