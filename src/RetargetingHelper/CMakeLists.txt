# Copyright (C) 2019 Fondazione Istituto Italiano di Tecnologia (IIT)
# All Rights Reserved.
# Authors: Giulio Romualdi <giulio.romualdi@iit.it>

add_walking_controllers_library(
  NAME RetargetingHelper
  SOURCES src/Helper.cpp
  PUBLIC_HEADERS include/WalkingControllers/RetargetingHelper/Helper.h
  PUBLIC_LINK_LIBRARIES WalkingControllers::YarpUtilities WalkingControllers::KinDynWrapper ${iDynTree_LIBRARIES} ctrlLib
  PRIVATE_LINK_LIBRARIES Eigen3::Eigen)
