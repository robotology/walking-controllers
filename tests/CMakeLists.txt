cmake_minimum_required(VERSION 3.1)

set (CMAKE_CXX_STANDARD 11)

# YarpHelper test
if(WALKING_CONTROLLERS_COMPILE_libYARP_helper)
  add_executable(YarpHelperTest YarpHelperTest.cpp)
  target_link_libraries(YarpHelperTest YARP_helper Catch2::Catch2)
  add_test(NAME YarpHelperTest COMMAND YarpHelperTest)
endif()


endif()
