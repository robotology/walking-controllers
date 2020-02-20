# Copyright (C) 2019 Fondazione Istituto Italiano di Tecnologia (IIT)
# All Rights Reserved.
# Authors: Giulio Romualdi <giulio.romualdi@iit.it>

# set target name
if(WALKING_CONTROLLERS_COMPILE_WholeBodyControllers)

  set(LIBRARY_TARGET_NAME WholeBodyControllers)

  # set cpp files
  set(${LIBRARY_TARGET_NAME}_SRC
    src/InverseKinematics.cpp
    src/QPInverseKinematics.cpp
    src/QPInverseKinematics_osqp.cpp
    src/QPInverseKinematics_qpOASES.cpp
    )

  # set hpp files
  set(${LIBRARY_TARGET_NAME}_HDR
    include/WalkingControllers/WholeBodyControllers/InverseKinematics.h
    include/WalkingControllers/WholeBodyControllers/QPInverseKinematics.h
    include/WalkingControllers/WholeBodyControllers/QPInverseKinematics_osqp.h
    include/WalkingControllers/WholeBodyControllers/QPInverseKinematics_qpOASES.h
    )

  # add an executable to the project using the specified source files.
  add_library(${LIBRARY_TARGET_NAME} SHARED ${${LIBRARY_TARGET_NAME}_SRC} ${${LIBRARY_TARGET_NAME}_HDR})

  # Workaround for the bug in Eigen3 3.3 with MSVC
  # See https://github.com/robotology/walking-controllers/issues/47
  if (MSVC AND (${EIGEN3_VERSION} VERSION_LESS 3.4))
    target_compile_definitions(${LIBRARY_TARGET_NAME} PRIVATE WALKING_CONTROLLERS_EIGEN_3_3_WORKAROUND)
  endif()

  set_target_properties(${LIBRARY_TARGET_NAME} PROPERTIES VERSION ${WalkingControllers_VERSION}
    PUBLIC_HEADER "${YARP_helper_HDR}")

  target_link_libraries(${LIBRARY_TARGET_NAME} PUBLIC
    WalkingControllers::YarpUtilities
    WalkingControllers::iDynTreeUtilities
    osqp::osqp
    OsqpEigen::OsqpEigen
    ${qpOASES_LIBRARIES}
    ctrlLib)

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
    PUBLIC_HEADER DESTINATION "${CMAKE_INSTALL_INCLUDEDIR}/WalkingControllers/WholeBodyControllers"       COMPONENT dev)

  set_property(GLOBAL APPEND PROPERTY WalkingControllers_TARGETS ${LIBRARY_TARGET_NAME})

  message(STATUS "Created target ${LIBRARY_TARGET_NAME} for export ${PROJECT_NAME}.")

endif()
