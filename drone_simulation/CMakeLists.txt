cmake_minimum_required(VERSION 3.20)
project(drone_simulation CXX)

add_library(drone_simulation_lib
    src/simulation.cpp
    src/solvers/analytical_solver.cpp
    src/providers/json_target_provider.cpp
    src/config_loaders/file_config_loader.cpp
    src/config_loaders/factory.cpp
    src/mission_processor.cpp
)
target_include_directories(drone_simulation_lib PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_compile_options(drone_simulation_lib PRIVATE -Wall -Wextra -pedantic)

add_executable(drone_simulation_check src/main.cpp)
target_link_libraries(drone_simulation_check PRIVATE drone_simulation_lib)
target_compile_options(drone_simulation_check PRIVATE -Wall -Wextra -pedantic)

file(COPY ${CMAKE_CURRENT_SOURCE_DIR}/data/
     DESTINATION ${CMAKE_CURRENT_BINARY_DIR})