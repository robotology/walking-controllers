# Copyright (C) 2019 Fondazione Istituto Italiano di Tecnologia (IIT)
# All Rights Reserved.
# Authors: Giulio Romualdi <giulio.romualdi@iit.it>

add_walking_controllers_library(
  NAME NavigationHelper
  SOURCES src/NavigationHelper.cpp
  PUBLIC_HEADERS include/WalkingControllers/NavigationHelper/NavigationHelper.h
  PUBLIC_LINK_LIBRARIES WalkingControllers::YarpUtilities WalkingControllers::iDynTreeUtilities WalkingControllers::KinDynWrapper WalkingControllers::TrajectoryPlanner WalkingControllers::StdUtilities ctrlLib
  PRIVATE_LINK_LIBRARIES Eigen3::Eigen)
