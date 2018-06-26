# walking-controllers
The **walking-controllers** project is a _suite_ of modules for bipedal locomotion for the humanoid robot platform iCub.

The suite includes:

* **Walking_module**: this is the main module and it implements all the controller architecture that allows iCub to walk.
* **Joypad_module**: this module allows using the Joypad as reference input for the trajectory generated by the Walking Module
* **WalkingLogger_module**: an module that can be useful to dump data coming from the Walking Module

# Overview
 - [:orange_book: Some theory behind the code](#orange_book-some-theory-behind-the-code)
 - [:page_facing_up: Dependencies](#page_facing_up-dependencies)
 - [:hammer: Build the suite](#hammer-build-the-suite)
 - [:computer: How to run the simulation](#computer-how-to-run-the-simulation)
 - [:running: How to test on iCub](#running-how-to-test-on-icub)

# :orange_book: Some theory behind the code
This module allows iCub walking using the position control interface.
It implements the following architecture
![controller_architecture](https://user-images.githubusercontent.com/16744101/37352869-757896b4-26de-11e8-97bc-10700add7759.jpg)
where two controller loops can be distinguished:
* an inner ZMP-CoM control loop http://ieeexplore.ieee.org/stamp/stamp.jsp?tp=&arnumber=4359259&tag=1;
* an outer DCM control loop:
   * model predictive controller;
   * reactive controller.

Two different inverse kinematics solver are implemented:
* a standard non-linear IK solver;
* a standard QP Jacobian based IK solver.

## Reference paper
A paper describing some of the algorithms implemented in this repository  can be downloaded [here](https://arxiv.org/abs/1809.02167).
If you're going to use the code developed for your work, please quote it within any resulting publication:
~~~
G. Romualdi, S. Dafarra, Y. Hu, D. Pucci "A Benchmarking of DCM Based Architectures for Position and Velocity Controlled Walking of Humanoid Robots", 2018
~~~

The bibtex code for including this citation is provided:
~~~
@misc{1809.02167,
Author = {Giulio Romualdi and Stefano Dafarra and Yue Hu and Daniele Pucci},
Title = {A Benchmarking of DCM Based Architectures for Position and Velocity Controlled Walking of Humanoid Robots},
Year = {2018},
Eprint = {arXiv:1809.02167},
}
~~~

# :page_facing_up: Dependencies
* [YARP](http://www.yarp.it/): to handle the comunication with the robot;
* [osqp-eigen](https://github.com/robotology/osqp-eigen): to solve the MPC problem;
* [qpOASES](https://github.com/robotology-dependencies/qpOASES): to solve the IK problem;
* [Unicycle footstep planner](https://github.com/robotology/unicycle-footstep-planner/tree/dcmTrajectoryGenerator): to generate a trajectory for the DCM;
* [Gazebo](http://gazebosim.org/): for the simulation (tested Gazebo 8 and 9).

# :hammer: Build the suite
## Linux/macOs

```sh
git clone https://github.com/robotology/walking-controllers.git
cd walking-controllers
mkdir build && cd build
cmake ../
make
[sudo] make install
```
Notice: `sudo` is not necessary if you specify the `CMAKE_INSTALL_PREFIX`. In this case it is necessary to add in the `.bashrc` or `.bash_profile` the following lines:
```sh
export WalkingControllers_INSTALL_DIR=/path/where/you/installed/
export PATH=$PATH:$WalkingControllers_INSTALL_DIR/bin
export YARP_DATA_DIRS=$YARP_DATA_DIRS:$WalkingControllers_INSTALL_DIR/share/yarp
```

# :computer: How to run the simulation
1. Set the `YARP_ROBOT_NAME` environment variable according to the chosen Gazebo model:
   ```sh
   export YARP_ROBOT_NAME="icubGazeboSim"
   ```
2. Run `yarpserver`
   ``` sh
   yarpserver --write
   ```
3. Run gazebo and drag and drop iCub (e.g. icubGazeboSim or iCubGazeboV2_5):

    ``` sh
    gazebo -slibgazebo_yarp_clock.so
    ```
4. Run `yarprobotinterface`

    ``` sh
     YARP_CLOCK=/clock yarprobotinterface --config launch-wholebodydynamics.xml
    ```
5. Reset the offset of the FT sensors. Open a terminal and write

   ```
   yarp rpc /wholeBodyDynamics/rpc
   >> resetOffset all
   ```
6. Run the walking module
    ``` sh
    YARP_CLOCK=/clock WalkingModule
    ```
7. communicate with the `WalkingModule`:
   ```
   yarp rpc /walking-coordinator/rpc
   ```
   the following commands are allowed:
   * `prepareRobot`: put iCub in the home position;
   * `startWalking`: run the controller;
   * `setGoal x y`: send the desired final position, `x` and `y` are expressed in iCub fixed frame.

## How to run the Joypad Module
If you want to control iCub with the Joypad:
1. Run the Joypad device:

    ``` sh
    yarpdev --device JoypadControlServer --use_separate_ports 1 --period 10 --name /joypadDevice/xbox --subdevice SDLJoypad --sticks 0

    ```
2. Run the Joypad Module

    ``` sh
    YARP_CLOCK=/clock WalkingJoypadModule
    ```

3. Enjoy

## How to dump data
Before run `WalkingModule` check if [`dump_data`](app/robots/iCubGazeboV2_5/dcmWalkingCoordinator.ini#L33) is set to 1

Run the Logger Module
``` sh
YARP_CLOCK=/clock WalkingLoggerModule
```

All the data will be saved in the current folder inside a `txt` named `Dataset_YYYY_MM_DD_HH_MM_SS.txt`

## Some interesting parameters
You can change the DCM controller and the inverse kinematics solver by edit [these parameters](app/robots/iCubGazeboV2_5/dcmWalkingCoordinator.ini#L22-L30)

# :running: How to test on iCub
You can follows the same instructions of the simulation section without using `YARP_CLOCK=/clock`. Make sure your `YARP_ROBOT_NAME` is coherent with the name of the robot (e.g. iCubGenova04)
## :warning: Warning
Currently the supported robots are only:
- ``iCubGenova04``
- ``iCubGenova02``
- ``icubGazeboSim``
- ``icubGazeboV2_5``

Yet, it is possible to use these controllers provided that the robot has V2.5 legs. In this case, the user should define the robot specific configuration files (those of ``iCubGenova04`` are a good starting point). 

:warning: The STRAIN F/T sensors normally mounted on iCub may suffer from saturations due to the strong impacts the robot has with the ground, which may lead to a failure of the controller. It is suggested to use these controllers with STRAIN2 sensors only (as in ``iCubGenova04``) to avoid such saturations.

