# Copyright (C) 2019 Fondazione Istituto Italiano di Tecnologia (IIT)
# All Rights Reserved.
# Authors: Giulio Romualdi <giulio.romualdi@iit.it>

# set target name
if(WALKING_CONTROLLERS_COMPILE_libiDynTree_helper)

  set(LIBRARY_TARGET_NAME iDynTree_helper)

  # set cpp files
  set(iDynTree_helper_SRC
    src/Helper.cpp
    )

  # set hpp files
  set(iDynTree_helper_HDR
    include/WalkingControllers/iDynTreeHelper/Helper.h
    )

  # add an executable to the project using the specified source files.
  add_library(iDynTree_helper SHARED ${iDynTree_helper_SRC} ${iDynTree_helper_HDR})

  target_link_libraries(iDynTree_helper PUBLIC ${iDynTree_LIBRARIES} YARP::YARP_OS)

  add_library(WalkingControllers::iDynTree_helper ALIAS iDynTree_helper)

  set_target_properties(iDynTree_helper PROPERTIES VERSION ${WalkingControllers_VERSION}
    PUBLIC_HEADER "${iDynTree_helper_HDR}")

  # Specify include directories for both compilation and installation process.
  # The $<INSTALL_PREFIX> generator expression is useful to ensure to create
  # relocatable configuration files, see https://cmake.org/cmake/help/latest/manual/cmake-packages.7.html#creating-relocatable-packages
  target_include_directories(iDynTree_helper PUBLIC "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
    "$<INSTALL_INTERFACE:$<INSTALL_PREFIX>/${CMAKE_INSTALL_INCLUDEDIR}>")

  # Specify installation targets, typology and destination folders.
  install(TARGETS    iDynTree_helper
    EXPORT     ${PROJECT_NAME}
    COMPONENT  runtime
    LIBRARY       DESTINATION "${CMAKE_INSTALL_LIBDIR}"                                       COMPONENT shlib
    ARCHIVE       DESTINATION "${CMAKE_INSTALL_LIBDIR}"                                       COMPONENT lib
    RUNTIME       DESTINATION "${CMAKE_INSTALL_BINDIR}"                                       COMPONENT bin
    PUBLIC_HEADER DESTINATION "${CMAKE_INSTALL_INCLUDEDIR}/WalkingControllers/iDynTreeHelper" COMPONENT dev)

  set_property(GLOBAL APPEND PROPERTY WalkingControllers_TARGETS ${LIBRARY_TARGET_NAME})

  message(STATUS "Created target ${LIBRARY_TARGET_NAME} for export ${PROJECT_NAME}.")

endif()
