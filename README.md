# element_capture-point-walking

## Responsible
|                Giulio Romualdi                              |
:------------------------------------------------------------:|
<img src="https://github.com/GiulioRomualdi.png" width="180"> |

## Background
The Capture Point (CP) was a well known concept in robot locomotion. It was firstly introduced by [Pratt et al.](http://ieeexplore.ieee.org/document/4115602/) using the linear inverted pendulum orbital energy. Then it was formalized by [Englsberger et al.](http://ieeexplore.ieee.org/document/6094435/). The CP is defined as the point on the ground which, when the ZMP is at this position, ensures that the robot comes to rest in a vertical position. You can find further information [here](../technical_reports/technical-reports/introductory-report/introductory-report.pdf) and [here](../technical_reports/technical-reports/trajectory_generator-report/trajectory_generator-report.pdf).

## Objectives
The objectives of the element are:
* **Implement the locomotion task on iCub using the MPC approach**: this controller architecture is mainly based on the [Englsberger's  work](https://www.sciencedirect.com/science/article/pii/S1474667016336059);
* **Implement the locomotion task on iCub using the standard reactive control**: a simple controller architecture will be implemented in order to verify if the MPC approach is strictly necessary;
* **Compare the results obtained with the controllers above**: a techincal report will be produced in order to shown the results obtained with both of the controller architectures.

## Outcomes
The possible outcomes can be summarized in the following list:
* **A C++ software that implements a trajectory generator for the CP**
* **A C++ software that implements a MPC for the Capture Point**
* **A C++ software that implements a standard reactive controller for the Capture Point**

## Task break-down
The following epic-tasks can been identified:
* [**Implement a CP trajectory generator from the desired footstep positions provided from a kinematic footstep planning algorithm**](https://github.com/loc2/element_capture-point-walking/issues/6)
* [**Make iCub walk with the MPC-CP approach using the IK framework and Position Control**](https://github.com/loc2/element_capture-point-walking/issues/3)
* [**Make iCub walk with the reactive control approach using the IK framework and Position Control**](https://github.com/loc2/element_capture-point-walking/issues/4)
