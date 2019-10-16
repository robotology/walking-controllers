# Copyright (C) 2018 Fondazione Istituto Italiano di Tecnologia (IIT)
# All Rights Reserved.
# Authors: Giulio Romualdi <giulio.romualdi@iit.it>

cmake_minimum_required(VERSION 3.5)
set(CMAKE_CXX_STANDARD 14)

## MAIN project
project(WalkingControllers
  VERSION 0.3.0)

list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake)

# Check WalkingControllers dependencies, find necessary libraries.
include(WalkingControllersFindDependencies)

#set default build type to "Release" in single-config generators
if(NOT CMAKE_CONFIGURATION_TYPES)
    if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Release" CACHE STRING
        "Choose the type of build, recommanded options are: Debug or Release" FORCE)
    endif()
    set(ROBOTOLOGY_BUILD_TYPES "Debug" "Release" "MinSizeRel" "RelWithDebInfo")
    set_property(CACHE CMAKE_BUILD_TYPE PROPERTY STRINGS ${ROBOTOLOGY_BUILD_TYPES})
endif()

add_subdirectory(src)

include(InstallBasicPackageFiles)
install_basic_package_files(${PROJECT_NAME}
                            NAMESPACE WalkingControllers::
                            VERSION ${WalkingControllers_VERSION}
                            COMPATIBILITY AnyNewerVersion
                            TARGETS_PROPERTY WalkingControllers_TARGETS
                            VARS_PREFIX ${PROJECT_NAME}
                            NO_CHECK_REQUIRED_COMPONENTS_MACRO)

option(BUILD_TESTING "Create tests using CMake" OFF)
include(CTest)
if(BUILD_TESTING AND WALKING_CONTROLLERS_COMPILE_tests)
  add_subdirectory(tests)
endif()

include(AddUninstallTarget)
