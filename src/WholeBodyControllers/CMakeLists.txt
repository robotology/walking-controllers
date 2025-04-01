# Copyright (C) 2019 Fondazione Istituto Italiano di Tecnologia (IIT)
# All Rights Reserved.
# Authors: Giulio Romualdi <giulio.romualdi@iit.it>

add_walking_controllers_library(
  NAME WholeBodyControllers
  SOURCES src/InverseKinematics.cpp src/BLFIK.cpp src/BLFTSID.cpp
  PUBLIC_HEADERS include/WalkingControllers/WholeBodyControllers/InverseKinematics.h
                 include/WalkingControllers/WholeBodyControllers/BLFIK.h
                 include/WalkingControllers/WholeBodyControllers/BLFTSID.h
  PUBLIC_LINK_LIBRARIES BipedalLocomotion::IK
                        BipedalLocomotion::TSID
                        BipedalLocomotion::ContinuousDynamicalSystem
                        WalkingControllers::YarpUtilities
                        WalkingControllers::iDynTreeUtilities
                        WalkingControllers::KinDynWrapper
                        Eigen3::Eigen
                        ctrlLib)
