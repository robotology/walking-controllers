# Copyright (C) 2017 Fondazione Istituto Italiano di Tecnologia
# Authors: Stefano Dafarra
#          Giulio Romualdi
# CopyPolicy: Released under the terms of the LGPLv2.1 or later

cmake_minimum_required(VERSION 3.1)

set (CMAKE_CXX_STANDARD 11)

project(StepAdaptatorTest)

# StepAdaptation
add_executable(StepAdaptationTest StepAdaptationTest.cpp)
target_link_libraries(StepAdaptationTest StepAdaptation ${iDynTree_LIBRARIES})
add_test(NAME StepAdaptationTest
  COMMAND StepAdaptationTest)


