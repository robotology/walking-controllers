# Copyright (C) 2019 Fondazione Istituto Italiano di Tecnologia (IIT)
# All Rights Reserved.
# Authors: Giulio Romualdi <giulio.romualdi@iit.it>

add_walking_controllers_library(
  NAME KinDynWrapper
  SOURCES src/Wrapper.cpp
  PUBLIC_HEADERS include/WalkingControllers/KinDynWrapper/Wrapper.h
  PUBLIC_LINK_LIBRARIES WalkingControllers::YarpUtilities ${iDynTree_LIBRARIES} ctrlLib
  PRIVATE_LINK_LIBRARIES Eigen3::Eigen)
