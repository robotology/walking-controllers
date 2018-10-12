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

# Get list of models
subdirlist(subdirs ${CMAKE_CURRENT_SOURCE_DIR}/robots/)

# Install each model
foreach (dir ${subdirs})
  file(GLOB scripts robots/${dir}/*.ini)
  yarp_install(FILES ${scripts} DESTINATION ${YARP_ROBOTS_INSTALL_DIR}/${dir})
  
  file(GLOB scripts robots/${dir}/*.urdf)
  yarp_install(FILES ${scripts} DESTINATION ${YARP_ROBOTS_INSTALL_DIR}/${dir})

  file(GLOB scripts robots/${dir}/*.xml)
  yarp_install(FILES ${scripts} DESTINATION ${YARP_APPLICATIONS_INSTALL_DIR}/${dir})
endforeach ()

yarp_install(DIRECTORY dcmWalkingLogger DESTINATION ${YARP_CONTEXTS_INSTALL_DIR})
yarp_install(DIRECTORY dcmWalkingJoypad DESTINATION ${YARP_CONTEXTS_INSTALL_DIR})
