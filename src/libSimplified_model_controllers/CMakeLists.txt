# Copyright (C) 2019 Fondazione Istituto Italiano di Tecnologia (IIT)
# All Rights Reserved.
# Authors: Giulio Romualdi <giulio.romualdi@iit.it>

# set target name
if(WALKING_CONTROLLERS_COMPILE_libSimplified_model_controllers)

  set(LIBRARY_TARGET_NAME Simplified_model_controllers)

  # set cpp files
  set(${LIBRARY_TARGET_NAME}_SRC
    src/DCMModelPredictiveController.cpp
    src/DCMReactiveController.cpp
    src/MPCSolver.cpp
    src/ZMPController.cpp
    )

  # set hpp files
  set(${LIBRARY_TARGET_NAME}_HDR
    include/WalkingControllers/SimplifiedModelControllers/DCMModelPredictiveController.h
    include/WalkingControllers/SimplifiedModelControllers/DCMReactiveController.h
    include/WalkingControllers/SimplifiedModelControllers/MPCSolver.h
    include/WalkingControllers/SimplifiedModelControllers/ZMPController.h
    )

  # add an executable to the project using the specified source files.
  add_library(${LIBRARY_TARGET_NAME} SHARED ${${LIBRARY_TARGET_NAME}_SRC} ${${LIBRARY_TARGET_NAME}_HDR})

  target_link_libraries(${LIBRARY_TARGET_NAME} PUBLIC
    WalkingControllers::YARP_helper
    WalkingControllers::iDynTree_helper
    osqp::osqp
    OsqpEigen::OsqpEigen)

  add_library(WalkingControllers::${LIBRARY_TARGET_NAME} ALIAS ${LIBRARY_TARGET_NAME})

  set_target_properties(${LIBRARY_TARGET_NAME} PROPERTIES VERSION ${WalkingControllers_VERSION}
    PUBLIC_HEADER "${${LIBRARY_TARGET_NAME}_HDR}")

  # Specify include directories for both compilation and installation process.
  # The $<INSTALL_PREFIX> generator expression is useful to ensure to create
  # relocatable configuration files, see https://cmake.org/cmake/help/latest/manual/cmake-packages.7.html#creating-relocatable-packages
  target_include_directories(${LIBRARY_TARGET_NAME} PUBLIC "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
    "$<INSTALL_INTERFACE:$<INSTALL_PREFIX>/${CMAKE_INSTALL_INCLUDEDIR}>")

  # Specify installation targets, typology and destination folders.
  install(TARGETS    ${LIBRARY_TARGET_NAME}
    EXPORT     ${PROJECT_NAME}
    COMPONENT  runtime
    LIBRARY       DESTINATION "${CMAKE_INSTALL_LIBDIR}"                                                   COMPONENT shlib
    ARCHIVE       DESTINATION "${CMAKE_INSTALL_LIBDIR}"                                                   COMPONENT lib
    RUNTIME       DESTINATION "${CMAKE_INSTALL_BINDIR}"                                                   COMPONENT bin
    PUBLIC_HEADER DESTINATION "${CMAKE_INSTALL_INCLUDEDIR}/WalkingControllers/SimplifiedModelControllers" COMPONENT dev)

  message(STATUS "Created target ${LIBRARY_TARGET_NAME} for export ${PROJECT_NAME}.")

  set_property(GLOBAL APPEND PROPERTY WalkingControllers_TARGETS ${LIBRARY_TARGET_NAME})

endif()
