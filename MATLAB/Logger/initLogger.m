%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% /**
%  * Copyright (C): copyright 2018 iCub Facility - Istituto Italiano di Tecnologia
%  * @author: Giulio Romualdi
%  * SPDX-FileCopyrightText: Fondazione Istituto Italiano di Tecnologia (IIT)
%  * SPDX-License-Identifier: BSD-3-Clause
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