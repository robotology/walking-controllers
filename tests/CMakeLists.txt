cmake_minimum_required(VERSION 3.1)

set (CMAKE_CXX_STANDARD 11)

# YarpUtilities test
find_package(Catch2 REQUIRED)
add_executable(YarpUtilitiesTest YarpHelperTest.cpp)
target_link_libraries(YarpUtilitiesTest YarpUtilities Catch2::Catch2)
add_test(NAME YarpUtilitiesTest COMMAND YarpUtilitiesTest)
