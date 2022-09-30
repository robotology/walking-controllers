################################################################################
# Find all packages

find_package(Threads REQUIRED)
find_package(YARP 3.6.0 REQUIRED)
find_package(ICUB REQUIRED)
find_package(ICUBcontrib REQUIRED)
find_package(iDynTree REQUIRED)
find_package(Eigen3 3.2.92 REQUIRED)
find_package(UnicyclePlanner 0.5.2 REQUIRED)
find_package(OsqpEigen 0.4.0 REQUIRED)
find_package(qpOASES REQUIRED)
find_package(BipedalLocomotionFramework 0.10.0
  COMPONENTS VectorsCollection IK ParametersHandlerYarpImplementation
             ContinuousDynamicalSystem ManifConversions
             ParametersHandlerYarpImplementation
             RobotInterfaceYarpImplementation REQUIRED)
