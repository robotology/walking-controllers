# Copyright (C) 2019 Fondazione Istituto Italiano di Tecnologia (IIT)
# All Rights Reserved.
# Authors: Giulio Romualdi <giulio.romualdi@iit.it>

# set target name

set(LIBRARY_TARGET_NAME StdUtilities)

# set hpp files
set(${LIBRARY_TARGET_NAME}_HDR
  include/WalkingControllers/StdUtilities/Helper.h
  include/WalkingControllers/StdUtilities/Helper.tpp
  )

# add an executable to the project using the specified source files.
add_library(${LIBRARY_TARGET_NAME} INTERFACE)

add_library(WalkingControllers::${LIBRARY_TARGET_NAME} ALIAS ${LIBRARY_TARGET_NAME})


# Specify include directories for both compilation and installation process.
# The $<INSTALL_PREFIX> generator expression is useful to ensure to create
# relocatable configuration files, see https://cmake.org/cmake/help/latest/manual/cmake-packages.7.html#creating-relocatable-packages
target_include_directories(${LIBRARY_TARGET_NAME} INTERFACE "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:$<INSTALL_PREFIX>/${CMAKE_INSTALL_INCLUDEDIR}>")

# Specify installation targets, typology and destination folders.
install(TARGETS    ${LIBRARY_TARGET_NAME}
  EXPORT     ${PROJECT_NAME}
  COMPONENT  runtime)

install(FILES ${${LIBRARY_TARGET_NAME}_HDR}
  DESTINATION "${CMAKE_INSTALL_INCLUDEDIR}/WalkingControllers/StdUtilities")

set_property(GLOBAL APPEND PROPERTY WalkingControllers_TARGETS ${LIBRARY_TARGET_NAME})

message(STATUS "Created target ${LIBRARY_TARGET_NAME} for export ${PROJECT_NAME}.")
