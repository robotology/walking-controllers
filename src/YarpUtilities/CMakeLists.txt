# Copyright (C) 2019 Fondazione Istituto Italiano di Tecnologia (IIT)
# All Rights Reserved.
# Authors: Giulio Romualdi <giulio.romualdi@iit.it>

set(YARP_UTILITIES_HDR include/WalkingControllers/YarpUtilities/Helper.h
                       include/WalkingControllers/YarpUtilities/Helper.tpp
                       include/WalkingControllers/YarpUtilities/TransformServerHelper.h)

set(YARP_UTILITIES_SRC src/Helper.cpp
                       src/TransformServerHelper.cpp)

add_walking_controllers_library(
  NAME YarpUtilities
  SOURCES ${YARP_UTILITIES_SRC}
  PUBLIC_HEADERS ${YARP_UTILITIES_HDR}
  PUBLIC_LINK_LIBRARIES ${YARP_LIBRARIES} ${iDynTree_LIBRARIES})

add_walking_controllers_yarp_thrift(
  NAME HumanState
  THRIFT thrifts/WalkingControllers/YarpUtilities/HumanState.thrift
  INSTALLATION_FOLDER YarpUtilities)
