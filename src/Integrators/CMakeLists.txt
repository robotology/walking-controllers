# Copyright (C) 2025 Fondazione Istituto Italiano di Tecnologia (IIT)
# All Rights Reserved.
# Authors: Lorenzo Moretti <lorenzo.moretti@iit.it>

add_walking_controllers_library(
  NAME Integrators
  SOURCES src/jointIntegrators.cpp
  PUBLIC_HEADERS include/WalkingControllers/Integrators/jointIntegrators.h
  PUBLIC_LINK_LIBRARIES BipedalLocomotion::ContinuousDynamicalSystem
                        BipedalLocomotion::TextLogging
                        Eigen3::Eigen) 