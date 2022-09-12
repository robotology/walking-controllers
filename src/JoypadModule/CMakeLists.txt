# Copyright (C) 2018 Fondazione Istituto Italiano di Tecnologia (IIT)
# All Rights Reserved.
# Authors: Giulio Romualdi <giulio.romualdi@iit.it>

add_walking_controllers_application(
  NAME WalkingJoypadModule
  SOURCES src/main.cpp src/Module.cpp
  HEADERS include/WalkingControllers/JoypadModule/Module.h
  LINK_LIBRARIES WalkingControllers::YarpUtilities
  SUBDIRECTORIES app)
