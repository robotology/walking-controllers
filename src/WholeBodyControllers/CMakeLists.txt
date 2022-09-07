# Copyright (C) 2019 Fondazione Istituto Italiano di Tecnologia (IIT)
# All Rights Reserved.
# Authors: Giulio Romualdi <giulio.romualdi@iit.it>

add_walking_controllers_library(
  NAME WholeBodyControllers
  SOURCES src/InverseKinematics.cpp src/QPInverseKinematics.cpp src/QPInverseKinematics_osqp.cpp src/QPInverseKinematics_qpOASES.cpp
  PUBLIC_HEADERS include/WalkingControllers/WholeBodyControllers/InverseKinematics.h include/WalkingControllers/WholeBodyControllers/QPInverseKinematics.h include/WalkingControllers/WholeBodyControllers/QPInverseKinematics_osqp.h include/WalkingControllers/WholeBodyControllers/QPInverseKinematics_qpOASES.h
  PUBLIC_LINK_LIBRARIES BipedalLocomotion::IK BipedalLocomotion::ContinuousDynamicalSystem WalkingControllers::YarpUtilities WalkingControllers::iDynTreeUtilities WalkingControllers::KinDynWrapper OsqpEigen::OsqpEigen ${qpOASES_LIBRARIES} Eigen3::Eigen ctrlLib)
