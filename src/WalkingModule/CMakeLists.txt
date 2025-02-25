# Copyright (C) 2018 Fondazione Istituto Italiano di Tecnologia (IIT)
# All Rights Reserved.
# Authors: Giulio Romualdi <giulio.romualdi@iit.it>

set(WalkingModule_THRIFT_HDR thrifts/WalkingCommands.thrift)

yarp_add_idl(WalkingModule_THRIFT_GEN_FILES ${WalkingModule_THRIFT_HDR})

add_walking_controllers_application(
  NAME WalkingModule
  SOURCES src/main.cpp src/Module.cpp ${WalkingModule_THRIFT_GEN_FILES}
  HEADERS include/WalkingControllers/WalkingModule/Module.h
  LINK_LIBRARIES WalkingControllers::YarpUtilities
                 WalkingControllers::iDynTreeUtilities
                 WalkingControllers::StdUtilities
                 WalkingControllers::RobotInterface
                 WalkingControllers::KinDynWrapper
                 WalkingControllers::TrajectoryPlanner
                 WalkingControllers::SimplifiedModelControllers
                 WalkingControllers::WholeBodyControllers
                 WalkingControllers::JointControllers
                 WalkingControllers::Integrators
                 WalkingControllers::RetargetingHelper
                 BipedalLocomotion::VectorsCollection
                 BipedalLocomotion::ParametersHandlerYarpImplementation
                 BipedalLocomotion::Contacts
                 BipedalLocomotion::ManifConversions
                 BipedalLocomotion::System
                 ctrlLib
  SUBDIRECTORIES app)

