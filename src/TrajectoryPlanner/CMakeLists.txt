# Copyright (C) 2019 Fondazione Istituto Italiano di Tecnologia (IIT)
# All Rights Reserved.
# Authors: Giulio Romualdi <giulio.romualdi@iit.it>

add_walking_controllers_library(
  NAME TrajectoryPlanner
  SOURCES src/StableDCMModel.cpp src/TrajectoryGenerator.cpp src/FreeSpaceEllipseManager.cpp
  PUBLIC_HEADERS include/WalkingControllers/TrajectoryPlanner/StableDCMModel.h include/WalkingControllers/TrajectoryPlanner/TrajectoryGenerator.h include/WalkingControllers/TrajectoryPlanner/FreeSpaceEllipseManager.h
  PUBLIC_LINK_LIBRARIES Threads::Threads WalkingControllers::YarpUtilities UnicyclePlanner ctrlLib
  PRIVATE_LINK_LIBRARIES Eigen3::Eigen)
