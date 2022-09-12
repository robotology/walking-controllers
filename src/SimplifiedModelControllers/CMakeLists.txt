# Copyright (C) 2019 Fondazione Istituto Italiano di Tecnologia (IIT)
# All Rights Reserved.
# Authors: Giulio Romualdi <giulio.romualdi@iit.it>

add_walking_controllers_library(
  NAME SimplifiedModelControllers
  SOURCES src/DCMModelPredictiveController.cpp src/DCMReactiveController.cpp src/MPCSolver.cpp src/ZMPController.cpp
  PUBLIC_HEADERS include/WalkingControllers/SimplifiedModelControllers/DCMModelPredictiveController.h include/WalkingControllers/SimplifiedModelControllers/DCMReactiveController.h include/WalkingControllers/SimplifiedModelControllers/MPCSolver.h include/WalkingControllers/SimplifiedModelControllers/ZMPController.h
  PUBLIC_LINK_LIBRARIES WalkingControllers::YarpUtilities WalkingControllers::iDynTreeUtilities OsqpEigen::OsqpEigen Eigen3::Eigen ctrlLib)
