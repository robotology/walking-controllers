# Copyright (C) 2018 Fondazione Istituto Italiano di Tecnologia (IIT)
# All Rights Reserved.
# Authors: Giulio Romualdi <giulio.romualdi@iit.it>

# set target name
set(EXE_TARGET_NAME WalkingModule)

option(ENABLE_RPATH "Enable RPATH for this library" ON)
mark_as_advanced(ENABLE_RPATH)
include(AddInstallRPATHSupport)
add_install_rpath_support(BIN_DIRS "${CMAKE_INSTALL_PREFIX}/${CMAKE_INSTALL_BINDIR}"
  LIB_DIRS "${CMAKE_INSTALL_PREFIX}/${CMAKE_INSTALL_LIBDIR}"
  INSTALL_NAME_DIR "${CMAKE_INSTALL_PREFIX}"
  DEPENDS ENABLE_RPATH
  USE_LINK_PATH)

# Find required package
find_package(ICUB REQUIRED)
find_package(UnicyclePlanner REQUIRED)
find_package(OsqpEigen REQUIRED)
find_package(qpOASES REQUIRED)
find_package(Threads REQUIRED)

include(FindPackageHandleStandardArgs)

# set cpp files
set(${EXE_TARGET_NAME}_SRC
  src/main.cpp
  src/TrajectoryGenerator.cpp
  src/MPCSolver.cpp
  src/WalkingController.cpp
  src/WalkingDCMReactiveController.cpp
  src/WalkingModule.cpp
  src/WalkingInverseKinematics.cpp
  src/WalkingQPInverseKinematics_osqp.cpp
  src/WalkingQPInverseKinematics_qpOASES.cpp
  src/WalkingForwardKinematics.cpp
  src/WalkingZMPController.cpp
  src/StableDCMModel.cpp
  src/WalkingPIDHandler.cpp
  src/WalkingLogger.cpp
  src/TimeProfiler.cpp
  )

# set hpp files
set(${EXE_TARGET_NAME}_HDR
  include/TrajectoryGenerator.hpp
  include/MPCSolver.hpp
  include/WalkingController.hpp
  include/WalkingDCMReactiveController.hpp
  include/WalkingModule.hpp
  include/WalkingInverseKinematics.hpp
  include/WalkingQPInverseKinematics_osqp.hpp
  include/WalkingQPInverseKinematics_qpOASES.hpp
  include/WalkingForwardKinematics.hpp
  include/WalkingZMPController.hpp
  include/StableDCMModel.hpp
  include/WalkingPIDHandler.hpp
  include/WalkingLogger.hpp
  include/WalkingLogger.tpp
  include/TimeProfiler.hpp
  )

# add include directories to the build.
include_directories(
  ${CMAKE_CURRENT_SOURCE_DIR}/include
  ${qpOASES_INCLUDEDIR}
)

# Thrift for the RPC commands
yarp_add_idl(THRIFT "${CMAKE_CURRENT_SOURCE_DIR}/thrifts/WalkingCommands.thrift")
add_library(icubWalking-service STATIC ${THRIFT})
target_include_directories(icubWalking-service SYSTEM PUBLIC ${YARP_INCLUDE_DIRS})
target_link_libraries(icubWalking-service YARP::YARP_init YARP::YARP_OS)

# add an executable to the project using the specified source files.
add_executable(${EXE_TARGET_NAME} ${${EXE_TARGET_NAME}_SRC} ${${EXE_TARGET_NAME}_HDR})

target_link_libraries(${EXE_TARGET_NAME}
  ${YARP_LIBRARIES}
  ${iDynTree_LIBRARIES}
  ctrlLib
  UnicyclePlanner
  UtilityLibrary
  OsqpEigen::OsqpEigen
  osqp::osqp
  icubWalking-service
  Threads::Threads
  ${qpOASES_LIBRARIES})

install(TARGETS ${EXE_TARGET_NAME} DESTINATION bin)
