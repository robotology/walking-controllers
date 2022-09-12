# Copyright (C) 2019 Fondazione Istituto Italiano di Tecnologia (IIT)
# All Rights Reserved.
# Authors: Giulio Romualdi <giulio.romualdi@iit.it>

add_walking_controllers_library(
  NAME RobotInterface
  SOURCES src/Helper.cpp src/PIDHandler.cpp
  PUBLIC_HEADERS include/WalkingControllers/RobotInterface/Helper.h include/WalkingControllers/RobotInterface/PIDHandler.h
  PUBLIC_LINK_LIBRARIES WalkingControllers::YarpUtilities WalkingControllers::iDynTreeUtilities ctrlLib
  PRIVATE_LINK_LIBRARIES Eigen3::Eigen)
