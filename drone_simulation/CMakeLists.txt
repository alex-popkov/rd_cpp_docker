cmake_minimum_required(VERSION 3.20)
project(drone_simulation CXX)

add_library(simulation src/simulation.cpp)
target_include_directories(simulation PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_compile_options(simulation PRIVATE -Wall -Wextra -pedantic)

add_library(factory src/factory.cpp)
target_include_directories(factory PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_compile_options(factory PRIVATE -Wall -Wextra -pedantic)

add_executable(drone_simulation_check src/main.cpp)
target_link_libraries(drone_simulation_check PRIVATE simulation)
target_link_libraries(drone_simulation_check PRIVATE factory)
target_compile_options(drone_simulation_check PRIVATE -Wall -Wextra -pedantic)
