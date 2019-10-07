# Copyright (C) 2019 Fondazione Istituto Italiano di Tecnologia (IIT)
# All Rights Reserved.
# Authors: Giulio Romualdi <giulio.romualdi@iit.it>

# set target name
if(WALKING_CONTROLLERS_COMPILE_libYARP_helper)

  set(LIBRARY_TARGET_NAME YARP_helper)

  # set cpp files
  set(YARP_helper_SRC
    src/Helper.cpp
    )

  # set hpp files
  set(YARP_helper_HDR
    include/WalkingControllers/YarpHelper/Helper.h
    include/WalkingControllers/YarpHelper/Helper.tpp
    )

  # add an executable to the project using the specified source files.
  add_library(YARP_helper SHARED ${YARP_helper_SRC} ${YARP_helper_HDR})

  target_link_libraries(YARP_helper PUBLIC ${YARP_LIBRARIES})
  add_library(WalkingControllers::YARP_helper ALIAS YARP_helper)

  set_target_properties(YARP_helper PROPERTIES VERSION ${WalkingControllers_VERSION}
    PUBLIC_HEADER "${YARP_helper_HDR}")

  # Specify include directories for both compilation and installation process.
  # The $<INSTALL_PREFIX> generator expression is useful to ensure to create
  # relocatable configuration files, see https://cmake.org/cmake/help/latest/manual/cmake-packages.7.html#creating-relocatable-packages
  target_include_directories(YARP_helper PUBLIC "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
    "$<INSTALL_INTERFACE:$<INSTALL_PREFIX>/${CMAKE_INSTALL_INCLUDEDIR}>")

  # Specify installation targets, typology and destination folders.
  install(TARGETS    YARP_helper
    EXPORT     ${PROJECT_NAME}
    COMPONENT  runtime
    LIBRARY       DESTINATION "${CMAKE_INSTALL_LIBDIR}"                                   COMPONENT shlib
    ARCHIVE       DESTINATION "${CMAKE_INSTALL_LIBDIR}"                                   COMPONENT lib
    RUNTIME       DESTINATION "${CMAKE_INSTALL_BINDIR}"                                   COMPONENT bin
    PUBLIC_HEADER DESTINATION "${CMAKE_INSTALL_INCLUDEDIR}/WalkingControllers/YarpHelper" COMPONENT dev)

  set_property(GLOBAL APPEND PROPERTY WalkingControllers_TARGETS ${LIBRARY_TARGET_NAME})

  message(STATUS "Created target ${LIBRARY_TARGET_NAME} for export ${PROJECT_NAME}.")


endif()
