# Copyright (C) 2019 Fondazione Istituto Italiano di Tecnologia (IIT)
# All Rights Reserved.
# Authors: Giulio Romualdi <giulio.romualdi@iit.it>

add_walking_controllers_library(
  NAME YarpUtilities
  SOURCES src/Helper.cpp
  PUBLIC_HEADERS include/WalkingControllers/YarpUtilities/Helper.h include/WalkingControllers/YarpUtilities/Helper.tpp
  PUBLIC_LINK_LIBRARIES ${YARP_LIBRARIES})
