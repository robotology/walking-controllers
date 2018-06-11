%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% /**
%  * Copyright (C): copyright 2018 iCub Facility - Istituto Italiano di Tecnologia
%  * @author: Giulio Romualdi
%  * Permission is granted to copy, distribute, and/or modify this program
%  * under the terms of the GNU General Public License, version 2 or any
%  * later version published by the Free Software Foundation.
%  *
%  * A copy of the license can be found at
%  * http://www.robotcub.org/icub/license/gpl.txt
%  *
%  * This program is distributed in the hope that it will be useful, but
%  * WITHOUT ANY WARRANTY; without even the implied warranty of
%  * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General
%  * Public License for more details
%  */
%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


%% GENERAL SIMULATION INFO
clear all;
clc;
% Simulation time in seconds
Config.SIMULATION_TIME = inf;  

% set equal to false if you use the real robot
Config.ON_GAZEBO = true;
% Config.ON_GAZEBO = false;

% sampling time
Config.Ts = 0.01; 

% setenv('YARP_ROBOT_NAME','iCubGenova04');
  setenv('YARP_ROBOT_NAME','icubGazeboSim');
% setenv('YARP_ROBOT_NAME','iCubGenova02');
% setenv('YARP_ROBOT_NAME','iCubGazeboV2_5');  

run(strcat('app/robots/',getenv('YARP_ROBOT_NAME'),'/configRobot.m')); 