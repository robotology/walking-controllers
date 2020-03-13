# Copyright (C) 2019 Fondazione Istituto Italiano di Tecnologia (IIT)
# All Rights Reserved.
# Authors: Giulio Romualdi <giulio.romualdi@iit.it>

# set target name
if(WALKING_CONTROLLERS_COMPILE_YarpUtilities)

  set(LIBRARY_TARGET_NAME YarpUtilities)

  # set cpp files
  set(YARP_helper_SRC
    src/Helper.cpp
    )

  # set hpp files
  set(YARP_helper_HDR
    include/WalkingControllers/YarpUtilities/Helper.h
    include/WalkingControllers/YarpUtilities/Helper.tpp
    )

  # add an executable to the project using the specified source files.
  add_library(${LIBRARY_TARGET_NAME} SHARED ${YARP_helper_SRC} ${YARP_helper_HDR})

  target_link_libraries(${LIBRARY_TARGET_NAME} PUBLIC ${YARP_LIBRARIES})
  add_library(WalkingControllers::${LIBRARY_TARGET_NAME} ALIAS ${LIBRARY_TARGET_NAME})
  set_target_properties(${LIBRARY_TARGET_NAME} PROPERTIES OUTPUT_NAME "${PROJECT_NAME}${LIBRARY_TARGET_NAME}")

  set_target_properties(${LIBRARY_TARGET_NAME} PROPERTIES VERSION ${WalkingControllers_VERSION}
    PUBLIC_HEADER "${YARP_helper_HDR}")

  # Specify include directories for both compilation and installation process.
  # The $<INSTALL_PREFIX> generator expression is useful to ensure to create
  # relocatable configuration files, see https://cmake.org/cmake/help/latest/manual/cmake-packages.7.html#creating-relocatable-packages
  target_include_directories(${LIBRARY_TARGET_NAME} PUBLIC "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
    "$<INSTALL_INTERFACE:$<INSTALL_PREFIX>/${CMAKE_INSTALL_INCLUDEDIR}>")

  # Specify installation targets, typology and destination folders.
  install(TARGETS    ${LIBRARY_TARGET_NAME}
    EXPORT     ${PROJECT_NAME}
    COMPONENT  runtime
    LIBRARY       DESTINATION "${CMAKE_INSTALL_LIBDIR}"                                   COMPONENT shlib
    ARCHIVE       DESTINATION "${CMAKE_INSTALL_LIBDIR}"                                   COMPONENT lib
    RUNTIME       DESTINATION "${CMAKE_INSTALL_BINDIR}"                                   COMPONENT bin
    PUBLIC_HEADER DESTINATION "${CMAKE_INSTALL_INCLUDEDIR}/WalkingControllers/YarpUtilities" COMPONENT dev)

  set_property(GLOBAL APPEND PROPERTY WalkingControllers_TARGETS ${LIBRARY_TARGET_NAME})

  message(STATUS "Created target ${LIBRARY_TARGET_NAME} for export ${PROJECT_NAME}.")


endif()
