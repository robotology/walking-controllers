# Copyright (C) 2019 Fondazione Istituto Italiano di Tecnologia (IIT)
# All Rights Reserved.
# Authors: Giulio Romualdi <giulio.romualdi@iit.it>

# set target name
set(LIBRARY_TARGET_NAME TimeProfiler)

# set cpp files
set(TimeProfiler_SRC
  src/TimeProfiler.cpp
  )

# set hpp files
set(TimeProfiler_HDR
  include/WalkingControllers/TimeProfiler/TimeProfiler.h
  )

# add an executable to the project using the specified source files.
add_library(${LIBRARY_TARGET_NAME} SHARED ${TimeProfiler_SRC} ${TimeProfiler_HDR})

add_library(WalkingControllers::${LIBRARY_TARGET_NAME} ALIAS ${LIBRARY_TARGET_NAME})
set_target_properties(${LIBRARY_TARGET_NAME} PROPERTIES OUTPUT_NAME "${PROJECT_NAME}${LIBRARY_TARGET_NAME}")

set_target_properties(${LIBRARY_TARGET_NAME} PROPERTIES VERSION ${WalkingControllers_VERSION}
  PUBLIC_HEADER "${TimeProfiler_HDR}")

# Specify include directories for both compilation and installation process.
# The $<INSTALL_PREFIX> generator expression is useful to ensure to create
# relocatable configuration files, see https://cmake.org/cmake/help/latest/manual/cmake-packages.7.html#creating-relocatable-packages
target_include_directories(${LIBRARY_TARGET_NAME} PUBLIC "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:$<INSTALL_PREFIX>/${CMAKE_INSTALL_INCLUDEDIR}>")

# Specify installation targets, typology and destination folders.
install(TARGETS ${LIBRARY_TARGET_NAME}
  EXPORT        ${PROJECT_NAME}
  COMPONENT     runtime
  LIBRARY       DESTINATION "${CMAKE_INSTALL_LIBDIR}"                                     COMPONENT shlib
  ARCHIVE       DESTINATION "${CMAKE_INSTALL_LIBDIR}"                                     COMPONENT lib
  RUNTIME       DESTINATION "${CMAKE_INSTALL_BINDIR}"                                     COMPONENT bin
  PUBLIC_HEADER DESTINATION "${CMAKE_INSTALL_INCLUDEDIR}/WalkingControllers/TimeProfiler" COMPONENT dev)

set_property(GLOBAL APPEND PROPERTY WalkingControllers_TARGETS ${LIBRARY_TARGET_NAME})

message(STATUS "Created target ${LIBRARY_TARGET_NAME} for export ${PROJECT_NAME}.")
