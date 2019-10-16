# Copyright (C) 2018 Fondazione Istituto Italiano di Tecnologia (IIT)
# All Rights Reserved.
# Authors: Giulio Romualdi <giulio.romualdi@iit.it>

# List the subdirectory
# http://stackoverflow.com/questions/7787823/cmake-how-to-get-the-name-of-all-subdirectories-of-a-directory
macro(SUBDIRLIST result curdir)
  file(GLOB children RELATIVE ${curdir} ${curdir}/*)
  set(dirlist "")
  foreach(child ${children})
    if(IS_DIRECTORY ${curdir}/${child})
      list(APPEND dirlist ${child})
    endif()
  endforeach()
  set(${result} ${dirlist})
endmacro()

option(INSTALL_ALL_ROBOTS "Enable installation of all robots" ON)
set(ROBOT_NAME "$ENV{YARP_ROBOT_NAME}" CACHE PATH "Name of your robot")
set(ROBOT_DIR "${CMAKE_CURRENT_SOURCE_DIR}/robots/${ROBOT_NAME}")

# Get list of models
if(INSTALL_ALL_ROBOTS)
  subdirlist(robots ${CMAKE_CURRENT_SOURCE_DIR}/robots/)

  # Install each model
  foreach (robot ${robots})
    file(GLOB scripts robots/${robot}/*.ini)
    yarp_install(FILES ${scripts} DESTINATION ${ICUBCONTRIB_ROBOTS_INSTALL_DIR}/${robot})

    subdirlist(subdirs ${CMAKE_CURRENT_SOURCE_DIR}/robots/${robot}/)
    foreach (subdir ${subdirs})
      yarp_install(DIRECTORY robots/${robot}/${subdir} DESTINATION ${ICUBCONTRIB_ROBOTS_INSTALL_DIR}/${robot})
    endforeach ()
  endforeach ()
else()
  if(ROBOT_NAME)
    if(IS_DIRECTORY ${ROBOT_DIR})

      file(GLOB scripts robots/${ROBOT_NAME}/*.ini)
      yarp_install(FILES ${scripts} DESTINATION ${ICUBCONTRIB_ROBOTS_INSTALL_DIR}/${ROBOT_NAME})

      subdirlist(subdirs ${CMAKE_CURRENT_SOURCE_DIR}/robots/${ROBOT_NAME}/)
      foreach (subdir ${subdirs})
        yarp_install(DIRECTORY robots/${ROBOT_NAME}/${subdir} DESTINATION ${ICUBCONTRIB_ROBOTS_INSTALL_DIR}/${ROBOT_NAME})
      endforeach ()
    endif()
  endif()
endif()

# Install the application
file(GLOB scripts scripts/*.xml)
yarp_install(FILES ${scripts} DESTINATION ${ICUBCONTRIB_APPLICATIONS_INSTALL_DIR})

