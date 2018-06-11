# Copyright (C) 2018 Fondazione Istituto Italiano di Tecnologia (IIT)
# All Rights Reserved.
# Authors: Giulio Romualdi <giulio.romualdi@iit.it>

cmake_minimum_required(VERSION 3.0)
set(CMAKE_CXX_STANDARD 14)
project(icubWalkingDCM)

list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake)

find_package(YARP REQUIRED)
find_package(iDynTree REQUIRED)

add_subdirectory(Walking_module)
add_subdirectory(WalkingLogger_module)
add_subdirectory(Joypad_module)

add_subdirectory(app)
