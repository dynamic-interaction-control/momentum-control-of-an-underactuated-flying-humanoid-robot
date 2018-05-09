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

%% INITIAL CONDITIONS

state_0                                = zeros(13+2*Config.N_DOF,1);
state_0(4)                             = 1;

% initial joints position (deg)      
torso_0                                = [  0     0    0]';           
leftArm_0                              = [  0    25    0   0  0]';          
rightArm_0                             = leftArm_0;
leftLeg_0                              = [  0   0   0   0   0   0]';
rightLeg_0                             = leftLeg_0;

% initial joints position (rad)
state_0(8:8+Config.N_DOF-1)            = [torso_0;leftArm_0;rightArm_0;leftLeg_0;rightLeg_0]*(pi/180);

% initial state
Config.initState                       = state_0;

% initial CoM position (integrator)
Config.posCoM_des_0                    = [0;0;0.1];

%% REFERENCES

% desired joint positions
Config.jointPos_des                            = [zeros(13,1);leftLeg_0;rightLeg_0]*(pi/180);

% time at which the robot stops to spin around the yaw angle, and spin velocity
Config.references.orientation.spinVel          = 0.0;
Config.references.orientation.stopSpin         = 120;
Config.references.orientation.constantRdes     = true;
Config.references.orientation.constantRdes_rpy = [0;0;60]*pi/180;

% ELICOIDAL  FLIGHT REFERENCE
Config.references.elicoidalFlight     = false;
Config.references.elicoidal.frequency = 0.15;
Config.references.elicoidal.amplitude = 2;
Config.references.elicoidal.dir       = [0;0;1];
Config.references.elicoidal.dir       = Config.references.elicoidal.dir/norm(Config.references.elicoidal.dir);
Config.references.elicoidal.vel       = 1;
Config.references.elicoidal.timeReady = 10;
Config.references.elicoidal.constCom  = [0;0;0];

% SINUSOIDAL FLIGHT REFERENCE
Config.references.sinuisoidalFlight   = false;
Config.references.sin.dirIncVel       = [0;1;1];
Config.references.sin.dirIncVel       = Config.references.sin.dirIncVel/norm(Config.references.sin.dirIncVel);
Config.references.sin.vel             = 0.0;
Config.references.sin.tChangeRef      = 10;
Config.references.sin.frequency       = 0.2;
Config.references.sin.amplitude       = 0.0;
Config.references.sin.dir             = [1;-1;1];
Config.references.sin.dir             = Config.references.sin.dir/norm(Config.references.sin.dir);
Config.references.sin.constCom        = [0;0;1];

% VARIOUS DIRECTION FLIGHT
Config.references.variousDirFlight    = true;
%                                        Time  %  normVel % vel direction
Config.references.varDir.series       = [10          1       0   0   1;
                                         20          1       0   1   0
                                         30          1       1   0   0
                                         40          1       0  -1   0
                                         50          1       0   0  -1
                                         60          0       1   1   1];
                                     
Config.references.useRefVelAccJerk    = true;

for i = 1:size(Config.references.varDir.series,1)
    Config.references.varDir.series(i,3:end) = Config.references.varDir.series(i,3:end)/norm(Config.references.varDir.series(i,3:end));
end

%                                        Time      % settlingTime % normVel % vel direction
Config.references.minJerkVel.series   = [ 5        5              0.5       0   0   1;
                                         10        10             1         1   0   0;
                                         20        10             1         0   1   0;
                                         30        10             1        -1   0   0;
                                         40        10             1         0  -1   0;
                                         45        10             1         0   0  -1;
                                         55        10             1         0   0  -1];
                                     
for i = 1:size(Config.references.minJerkVel.series,1)
    Config.references.minJerkVel.series(i,4:end) = Config.references.minJerkVel.series(i,4:end)/norm(Config.references.minJerkVel.series(i,4:end));
end                                     

%% GAINS
Config.gains.quaternionIntegration    = 1;

Config.gains.com.KP                   = blkdiag(5*eye(3),50*eye(3));
Config.gains.com.KD                   = 2*sqrt(Config.gains.com.KP);
Config.gains.com.KO                   = 10*eye(6);
Config.gains.com.KI                   = 0*eye(6);

posturalGainTorso                     = 10   *ones(1,3);
posturalGainArms                      = [ 5  5  5 10  1];
posturalGainLegs                      = [10 10 10 10 10 10];

Config.gains.postural.KP              = 0.1*diag([posturalGainTorso,posturalGainArms,posturalGainArms,posturalGainLegs,posturalGainLegs]);
Config.gains.torqueControl.KI         = 1000*eye(25);
Config.gains.torqueControl.KP         = 2*sqrt(Config.gains.torqueControl.KI);

Config.aux0                           = zeros(Config.N_DOF+4,1);

if Config.controlType == 1
    alpha                                 = 4;
    Config.gains.com.KD                   = 3*alpha*eye(6);
    Config.gains.com.KP                   = 3*alpha^2*eye(6);
    Config.gains.com.KI                   = alpha^3*eye(6);
    posturalGainTorso                     = 1   *ones(1,3);
    posturalGainArms                      = 0.05*ones(1,5);
    posturalGainLegs                      = 10  *ones(1,6);
    Config.gains.postural.KP              = diag([posturalGainTorso,posturalGainArms,posturalGainArms,posturalGainLegs,posturalGainLegs]);
end

%% WEIGHTS
Config.weights.threshEqCons           = 1e-2;
Config.weights.momentum               = 50;
Config.weights.postural               = 1;
Config.weights.minHandsThrustDot      = 0.01;
Config.weights.minFeetThrustDot       = 0.01;
Config.weights.symFeetThrusts         = false;
Config.weights.symHandsThrusts        = false;
Config.weights.symHandsAndFeetThrusts = false;
Config.weights.symLeftAndRightThrusts = false;
Config.weights.symAllThrusts          = true;
Config.weights.symmetryThrust         = 0.1; 
Config.weights.minJointVel            = 50;

%% REGULARIZATION TERMS
Config.reg.pinvTol                    = 1e-5;
Config.reg.pinvDamp                   = 1e-5;
Config.reg.thessianQp                 = 1e-6;

