cmake_minimum_required(VERSION 3.20)
project(ballistics CXX)

add_library(ballistics src/ballistics.cpp)
target_include_directories(ballistics PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_compile_options(ballistics PRIVATE -Wall -Wextra -pedantic)

add_executable(ballistics_check src/main.cpp)
target_link_libraries(ballistics_check PRIVATE ballistics)
target_compile_options(ballistics_check PRIVATE -Wall -Wextra -pedantic)

if(NOT CMAKE_CROSSCOMPILING)
    # Тест-бінарник для ballistics. gtest_main надає готовий main() з RUN_ALL_TESTS().
    add_executable(ballistics_tests tests/ballistics_tests.cpp)
    target_link_libraries(ballistics_tests PRIVATE ballistics GTest::gtest_main)
    target_compile_options(ballistics_tests PRIVATE -Wall -Wextra -pedantic)
    target_compile_definitions(ballistics_tests
                               PRIVATE TEST_DATA_DIR="${CMAKE_CURRENT_SOURCE_DIR}/tests/data")

    include(GoogleTest)
    gtest_discover_tests(ballistics_tests DISCOVERY_MODE PRE_TEST)
endif()
