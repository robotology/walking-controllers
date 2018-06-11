# DCM Walking Module
This module allows iCub walking using the position control interface.
It implements the following architecture
![controller_architecture](https://user-images.githubusercontent.com/16744101/37352869-757896b4-26de-11e8-97bc-10700add7759.jpg)
where two controller loops can be distinguished.
* An inner ZMP-CoM control loop http://ieeexplore.ieee.org/stamp/stamp.jsp?tp=&arnumber=4359259&tag=1;
* an outer DCM control loop (model predictive controller).

## Dependeces
* [YARP](http://www.yarp.it/): to handle the comunication with the robot;
* [OSQP-Wraper](https://github.com/GiulioRomualdi/OSQP-Wrapper/tree/devel_update_matrices): to solve the MPC problem;
* [Unicycle footstep planner](https://github.com/robotology/unicycle-footstep-planner/tree/dcmTrajectoryGenerator): to generate a trajectory for the DCM;
* [Gazebo](http://gazebosim.org/): for the simulation (tested Gazebo 8).


## How to install the Module
### Linux/macOs

```sh
git clone https://github.com/loc2/element_capture-point-walking
cd element_capture-point-walking
git checkout devel_cp_mpc_architecture3
cd code/cpp
mkdir build && cd build
cmake ../
make
[sudo] make install
```

Notice: `sudo` is not necessary if you specify the `CMAKE_INSTALL_PREFIX`. In this case it is necessary to add in the `.bashrc` or `.bash_profile` the following lines:
```
export dcmIcubWalking_DIR=/path/where/you/installed/
export PATH=$PATH:dcmIcubWalking_DIR/bin
export YARP_DATA_DIRS=${YARP_DATA_DIRS}:$dcmIcubWalking_DIR/share/yarp
```

## How to use the Module in Simulation
1. Run `yarpserver`
   ```
   yarpserver --write
   ```
2. run `yarpmanager` and open `DCM_walking_coordinator` application
3. run all the modules:
   * `gzserver` and `gzclient` run Gazebo;
   * `yarprobotinteface` starts all the devices required;
   * `WalkingModule` is the main module;
   * `WalkingLoggerModule` a logger that allows you to collect the data.
4. connect all the ports;
5. set the `YARP_ROBOT_NAME` environment variable according to the chosen Gazebo model:
   ```sh
   export YARP_ROBOT_NAME="icubGazeboSim"
   ```
5. reset the offset of the FT sensors. Open a terminal and write

   ```
   yarp rpc /wholeBodyDynamics/rpc
   >> resetOffset all
   ```
6. communicate with the `WalkingModule`:
   ```
   yarp rpc /walking-coordinator/rpc
   ```
   the following commands are allowed:
   * `prepareRobot`: put iCub in the home position;
   * `startWalking`: run the controller;
   * `setGoal x y`: send the desired final position, `x` and `y` are expressed in iCub fixed frame.
   
   
**Notice**: 
1. you can find the recorded dataset in the folder where `yarpmanager` was runned;
2. if you want to check the data during the experiment please run the [simulink model](../MATLAB/Logger).
