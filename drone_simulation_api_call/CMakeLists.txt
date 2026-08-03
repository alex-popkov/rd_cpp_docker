cmake_minimum_required(VERSION 3.20)
project(drone_simulation_api_call CXX)

add_library(drone_simulation_api_lib
    src/simulation.cpp
    src/api/network.cpp
    src/api/api_client.cpp
    src/solvers/analytical_solver.cpp
    src/solvers/table_solver.cpp
    src/providers/json_target_provider.cpp
    src/providers/thread_safe_target_provider.cpp
    src/config_loaders/file_config_loader.cpp
    src/config_loaders/factory.cpp
    src/mission_processor.cpp
    src/drone_states/state_stopped.cpp
    src/drone_states/state_accelerating.cpp
    src/drone_states/state_decelerating.cpp
    src/drone_states/state_turning.cpp
    src/drone_states/state_moving.cpp
    src/drone_physics.cpp
)

find_package(Threads REQUIRED)
target_link_libraries(drone_simulation_api_lib PUBLIC Threads::Threads)

target_include_directories(drone_simulation_api_lib PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_compile_options(drone_simulation_api_lib PRIVATE -Wall -Wextra -pedantic)

add_executable(drone_simulation_api_check src/main.cpp)
target_link_libraries(drone_simulation_api_check PRIVATE drone_simulation_api_lib)
target_compile_options(drone_simulation_api_check PRIVATE -Wall -Wextra -pedantic)

file(COPY ${CMAKE_CURRENT_SOURCE_DIR}/data/
     DESTINATION ${CMAKE_CURRENT_BINARY_DIR})
