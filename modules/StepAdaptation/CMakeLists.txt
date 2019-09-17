# Copyright (C) 2018 Fondazione Istituto Italiano di Tecnologia (IIT)
# All Rights Reserved.
# Authors: Giulio Romualdi <giulio.romualdi@iit.it>

# set target name
set(STEPADAPTATION_LIBRARY StepAdaptation)
message(${STEPADAPTATION_LIBRARY})
find_package(OsqpEigen REQUIRED)
find_package(qpOASES REQUIRED)
option(ENABLE_RPATH "Enable RPATH for this library" ON)
mark_as_advanced(ENABLE_RPATH)
include(AddInstallRPATHSupport)
add_install_rpath_support(BIN_DIRS "${CMAKE_INSTALL_PREFIX}/${CMAKE_INSTALL_BINDIR}"
  LIB_DIRS "${CMAKE_INSTALL_PREFIX}/${CMAKE_INSTALL_LIBDIR}"
  INSTALL_NAME_DIR "${CMAKE_INSTALL_PREFIX}"
  DEPENDS ENABLE_RPATH
  USE_LINK_PATH)

# set cpp files
set(${STEPADAPTATION_LIBRARY}_SRC
  src/StepAdaptator.cpp
  src/QPSolver.cpp
  src/WalkingLogger.cpp
  src/DCMPlanning.cpp
  )

# set hpp files
set(${STEPADAPTATION_LIBRARY}_HDR
  include/StepAdaptator.hpp
  include/QPSolver.hpp
  include/WalkingLogger.hpp
  include/WalkingLogger.tpp
  include/DCMPlanning.hpp
  )

# add an executable to the project using the specified source files.
add_library(${STEPADAPTATION_LIBRARY} ${${STEPADAPTATION_LIBRARY}_SRC} ${${STEPADAPTATION_LIBRARY}_HDR})

# add include directories to the build.
target_include_directories(${STEPADAPTATION_LIBRARY} PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include
      ${qpOASES_INCLUDEDIR})

target_link_libraries(${STEPADAPTATION_LIBRARY}
  ${YARP_LIBRARIES}
  ${iDynTree_LIBRARIES}
  UtilityLibrary
  OsqpEigen::OsqpEigen
  osqp::osqp
    ${qpOASES_LIBRARIES})

include(CTest)
if(BUILD_TESTING)
   add_subdirectory(test)
endif()
