%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% /**
%  * Copyright (C) 2016 CoDyCo
%  * @author: Daniele Pucci
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
clear variables
close all
clc

%% GENERAL SIMULATION INFO
%
% Set the environmental variable YARP_ROBOT_NAME = icubGazeboSim.
%
% To do this, you can uncomment the following lines.
%
setenv('YARP_ROBOT_NAME','icubGazeboSim');

% simulation info
Config.simulationTime                 = 20;
Config.Ts                             = 0.2e-1;

% set local paths
addpath(genpath('./src/'));
% addpath(genpath('../../../library'));

% DEMO CONFIGURATION PARAMETERS

Config.USE_MIN_JERK_FOR_COM_REF       = true;
Config.testRobustness                 = false;
Config.robustnessFactor               = 1.1;

Config.USE_EQUALITY_CONSTRAINTS       = 0;                

Config.visualiser.timeStep            = 0.1;
Config.visualiser.useSavedData        = false;

Config.controlType                    = 0; % 0: CONTROL WITH NO ASSURANCE OF POSITIVE THRUST
                                           % 1: CONTROL WITH RELATIVE DEGREE AGMENTATION 
                                                 
Config.orientationControl             = false;
Config.orientationCorrection          = false;

Config.CONSIDER_DYNAMICS              = true;

%% Run configurations files

% Run robot-specific configuration parameters
run(strcat('app/robots/',getenv('YARP_ROBOT_NAME'),'/configRobot.m')); 
run(strcat('app/robots/',getenv('YARP_ROBOT_NAME'),'/initFlyingControl.m'));

