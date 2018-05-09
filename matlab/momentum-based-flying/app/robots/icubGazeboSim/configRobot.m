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

% Robot configuration for WBT3.0
WBTConfigRobot           = WBToolbox.Configuration;
WBTConfigRobot.RobotName = 'icubSim';
WBTConfigRobot.UrdfFile  = 'model.urdf';
WBTConfigRobot.LocalName = 'WBT';

WBTConfigRobot.ControlBoardsNames = {'torso','left_arm','right_arm','left_leg','right_leg'};
WBTConfigRobot.ControlledJoints   = {'torso_pitch','torso_roll','torso_yaw', ...
                                     'l_shoulder_pitch','l_shoulder_roll','l_shoulder_yaw','l_elbow', 'l_wrist_prosup', ...
                                     'r_shoulder_pitch','r_shoulder_roll','r_shoulder_yaw','r_elbow', 'r_wrist_prosup',...
                                     'l_hip_pitch','l_hip_roll','l_hip_yaw','l_knee','l_ankle_pitch','l_ankle_roll', ...
                                     'r_hip_pitch','r_hip_roll','r_hip_yaw','r_knee','r_ankle_pitch','r_ankle_roll'};
% General robot parameters
Config.N_DOF         = 25;
Config.N_DOF_MATRIX  = eye(Config.N_DOF);
Config.GRAVITY_ACC   = 9.81;

% Saturations
Config.maxJetsIntVar   = [10 10 10 10]'*10;
Config.maxJointVelDes  = 45*pi/180*ones(Config.N_DOF,1);

% 4 element list identifying jets'axes: The value can be either 1,2,3 and 
% identifies the axes x,y,z of the associated end effector frame. The sign
% identifies the direction.
Config.jets.axes     = zeros(4,1);
Config.jets.axes(1)  = -1;
Config.jets.axes(2)  = -1;
Config.jets.axes(3)  =  1;
Config.jets.axes(4)  =  1; 

Config.jets.choiceOfJetsIntAt0        = 2; % 0: choose the inital jet intensities equal to zero
                                           % 1: choose the inital jet intensities so as to make them as symmetric as possible, i.e.
                                           %    left hand jet = right hand jet;     left foot jet = right foot jet
                                           % 2: choose jet intensities so as to make them symmetric, but hand
                                           %    jets intensities are alpha times as much the  foot jet intensities, i.e.
                                           %
                                           %    left-hand-jet = right-hand-jet = alpha*left-foot-jet = alpha*right-foot-hand   
% In this case, set config.jets.alpha
Config.jets.alpha                     = 1;

% Frames list
Frames.L_HAND_FRAME  = 'l_hand_dh_frame';
Frames.R_HAND_FRAME  = 'r_hand_dh_frame';
Frames.L_FOOT_FRAME  = 'l_foot_dh_frame';
Frames.R_FOOT_FRAME  = 'r_foot_dh_frame';
Frames.COM_FRAME     = 'com';
