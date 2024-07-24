# Copyright (C) 2019 Fondazione Istituto Italiano di Tecnologia (IIT)
# All Rights Reserved.
# Authors: Giulio Romualdi <giulio.romualdi@iit.it>

add_walking_controllers_library(
  NAME iDynTreeUtilities
  SOURCES src/Helper.cpp
  PUBLIC_HEADERS include/WalkingControllers/iDynTreeUtilities/Helper.h
  PUBLIC_LINK_LIBRARIES YARP::YARP_os ${iDynTree_LIBRARIES}
  PRIVATE_LINK_LIBRARIES Eigen3::Eigen)
