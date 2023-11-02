# Copyright (C) 2019 Fondazione Istituto Italiano di Tecnologia (IIT)
# All Rights Reserved.
# Authors: Giulio Romualdi <giulio.romualdi@iit.it>

add_walking_controllers_library(
  NAME WholeBodyControllers
  SOURCES src/InverseKinematics.cpp src/BLFIK.cpp
  PUBLIC_HEADERS include/WalkingControllers/WholeBodyControllers/InverseKinematics.h
                 include/WalkingControllers/WholeBodyControllers/BLFIK.h
  PUBLIC_LINK_LIBRARIES BipedalLocomotion::IK BipedalLocomotion::ContinuousDynamicalSystem WalkingControllers::YarpUtilities WalkingControllers::iDynTreeUtilities WalkingControllers::KinDynWrapper OsqpEigen::OsqpEigen ${qpOASES_LIBRARIES} Eigen3::Eigen ctrlLib)
