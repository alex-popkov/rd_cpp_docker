cmake_minimum_required(VERSION 3.20)
project(balistics CXX)

add_library(balistics src/balistics.cpp)
target_include_directories(balistics PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_compile_options(balistics PRIVATE -Wall -Wextra -pedantic)

add_executable(balistics_check src/main.cpp)
target_link_libraries(balistics_check PRIVATE balistics)
target_compile_options(balistics_check PRIVATE -Wall -Wextra -pedantic)

# Тест-бінарник для balistics. Лінкуємо проти неї самої та
# gtest_main, який надає готовий main() з RUN_ALL_TESTS().
add_executable(balistics_tests tests/balistics_tests.cpp)
target_link_libraries(balistics_tests PRIVATE balistics GTest::gtest_main)
target_compile_options(balistics_tests PRIVATE -Wall -Wextra -pedantic)
target_compile_definitions(balistics_tests PRIVATE TEST_DATA_DIR="${CMAKE_CURRENT_SOURCE_DIR}/tests/data")

# Реєструє кожен TEST(...) як окремий CTest-кейс — після білду CMake
# запускає тест-бінарник з --gtest_list_tests і виймає звідти всі імена.
include(GoogleTest)
gtest_discover_tests(balistics_tests)
