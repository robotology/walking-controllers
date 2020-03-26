# Changelog
All notable changes to this project are documented in this file.

## [Unreleased]
### Added

### Changed

## [0.3.2] - 2020-03-21
### Changed
- Add missing includes in `TimeProfiler` (https://github.com/robotology/walking-controllers/pull/60)

## [0.3.1] - 2020-03-18
### Changed
- Fix the windows compilation (https://github.com/robotology/walking-controllers/pull/59)

## [0.3.0] - 2020-03-16
### Added
- The `CHANGELOG.md` file
- Implement the `WalkingControllersFindDepencies.cmake`
- Adding the possibility of selecting Stiff/Compliant mode in joint level.

### Changed
- General refactoring of the library. The WalkingModule is now split in several library. Namelly:
   - `YarpUtilities`: utilities for using `YARP`
   - `StdUtilities`: utilities related to `std` library
   - `iDynTreeUtilities`: utilities related to `iDynTree`
   - `TimeProfiler`: library for time profiling
   - `SimplifiedModelControllers` library related to simplified model controllers
   - `WholeBodyControllers` library related to controller based on the entire robot model
   - `TrajectoryPlanner` library related to trajectory planner
   - `KinDynWrapper` iDynTree `KinDynComputation` wrapper.
   - `RetargetingClient` client for the retargeting

## [0.2.0] - 2019-10-24
### Added
- Implement the first version of the hand retargeting (i.e. `RetargetingClient`)
- Is it possible running different mode of the `WalkingModules` by changing the main configuration file
- Add versioning to the project (0.2.0)

### Changed
- The `time profiler` is moved to the `utilities` library
- The required version of `osqp-eigen` the  is now `0.4.0` (before was `0.3.0`)
- Update the `MPCSolver` class to be compatible with `osqp-eigen v0.4.0`
- Update the `WalkingQPInverseKinematics` to take into account the hand retargeting (the
`setPhase` method is implemented)
- WalkingLogger is now in a separate library
- Parameters on `iCubGenova04` are tuned
- Parameters on `icubGazeboSim` are tuned
- Set default build type to `release`
- Avoid using `Eigen::VectorXd` in the evaluate gradient and bounds

###  Fixed
- Initialize the `zmpLeft` and `zmpRight` vectors in the `evaluateZMP` method
- Fix the initialization of the `inverseKinematicsQPSolverOptions` bottle in the `WalkingModule.cpp`

### Removed
- Remove `Stance` state from `WalkingFSM`

## [0.1.0] -  2019-10-21
### Added
- `cmake/FindEigen3.cmake` file in order to improve the compatibility with `Windows` operating system
- implement `getNumberFromSearchable` in `Utils.cpp` file

### Changed
- Improve the description of the errors in the `WalkingLoggerModule`
- Fix typos in `README.md`
- Implement the `stopWalking` and the `pauseWalking` commands
- Parameters on `iCubGazeboV2_5` are tuned
- Parameters on `iCubGenova04` are tuned
- The required version of the unicycle planner is now `0.1.2` (before was `0.1.102`)
- the `WalkingQPIK_qpOASES` and `WalkingQPIK_osqp` now inherits  from the `WalkingQPIK` class

###  Fixed
- Fix the `close` function in the `WalkingLoggerModule`
- Add missing includes in `TimeProfiler.cpp` and WaldkingDCMModelPredictiveController.cpp`
- The joypad control mapping is now fixed

### Removed
- `onTheFly` feature for the `WalkingModule` application
- `iCubGenova02` is no more supported

## [0.0.1] - 2018-12-18
### Added
- Implement the first version of the `WalkingModule`
- Implement the first version of the `WalkingLoggerModule`
- Implement the first version of the `WalkingJoypadModule`

[Unreleased]: https://github.com/robotology/walking-controllers/compare/v0.3.2...devel
[0.3.2]: https://github.com/robotology/walking-controllers/compare/v0.3.1...v0.3.2
[0.3.1]: https://github.com/robotology/walking-controllers/compare/v0.3.0...v0.3.1
[0.3.0]: https://github.com/robotology/walking-controllers/compare/v0.2.0...v0.3.0
[0.2.0]: https://github.com/robotology/walking-controllers/compare/v0.1.0...v0.2.0
[0.1.0]: https://github.com/robotology/walking-controllers/compare/v0.0.1...v0.1.0
[0.0.1]: https://github.com/robotology/walking-controllers/releases/tag/v0.0.1
