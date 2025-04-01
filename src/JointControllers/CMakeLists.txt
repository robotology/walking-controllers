# Copyright (C) 2025 Fondazione Istituto Italiano di Tecnologia (IIT)
# All Rights Reserved.
# Authors: Lorenzo Moretti <lorenzo.moretti@iit.it>

add_walking_controllers_library(
  NAME JointControllers
  SOURCES src/AdmittanceController.cpp
  PUBLIC_HEADERS include/WalkingControllers/JointControllers/AdmittanceController.h
  PUBLIC_LINK_LIBRARIES BipedalLocomotion::ParametersHandler
                        BipedalLocomotion::TextLogging
                        Eigen3::Eigen) 
