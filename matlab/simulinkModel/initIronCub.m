%clear all;
close all;
clc;
setenv('YARP_ROBOT_NAME','icubGazeboSim');
CODYCO_DIR = getenv('CODYCO_SUPERBUILD_ROOT');

addpath([CODYCO_DIR '/build/install/mex/mexwbi-utilities']);
addpath([CODYCO_DIR '/main/WBIToolboxControllers/controllers/utilityMatlabFunctions']);

addpath('src')


config                                = struct; 

config.testRobustness                 = false;
config.robustnessFactor               = 1.1;

config.simulationTime                 = 20;
config.Ts                             = 0.2e-1;

config.ndof                           = 25;
config.ndofM                          = eye(config.ndof);
config.gravityAcc                     = 9.81;


config.maxJetsIntVar                  = [10 10 10 10]'*10;
config.maxJointVelDes                 = 45*pi/180*ones(config.ndof,1);

config.useEqualityConstraints         = 0;
config.weights.threshEqCons           = 1e-2;

config.weights.momentum               = 50;

config.weights.postural               = 1;

config.weights.minHandsThrustDot      = 0.01;
config.weights.minFeetThrustDot       = 0.01;

config.weights.symFeetThrusts         = false;
config.weights.symHandsThrusts        = false;
config.weights.symHandsAndFeetThrusts = false;
config.weights.symLeftAndRightThrusts = false;
config.weights.symAllThrusts          = true;
config.weights.symmetryThrust         = 0.1; 

config.weights.minJointVel            = 50;

config.jets.choiceOfJetsIntAt0        = 2; % 0: choose the inital jet intensities equal to zero
                                           % 1: choose the inital jet intensities so as to make them as symmetric as possible, i.e.
                                           %    left hand jet = right hand jet;     left foot jet = right foot jet
                                           % 2: choose jet intensities so as to make them symmetric, but hand
                                           %    jets intensities are alpha times as much the  foot jet intensities, i.e.
                                           %
                                           %    left-hand-jet = right-hand-jet = alpha*left-foot-jet = alpha*right-foot-hand
                                           %    
                                           %    In this case, set config.jets.alpha
                                           
config.jets.alpha                     = 1;


config.visualiser.timeStep            = 0.1;
config.visualiser.useSavedData        = false;

config.controlType                    = 0; % 0: CONTROL WITH NO ASSURANCE OF POSITIVE THRUST
                                           % 1: CONTROL WITH RELATIVE DEGREE AGMENTATION 
                                                 
config.orientationControl             = true;
config.orientationCorrection          = false;

config.references.orientation.constantRdes = true;
config.references.orientation.constantRdesrpy = [0;0;60]*pi/180;
config.references.orientation.spinVel = 0.0;
config.references.orientation.stopSpin= 120;

%% GAIN TUNING
config.gains.com.KP                   = blkdiag(5*eye(3),50*eye(3));
config.gains.com.KD                   = 2*sqrt(config.gains.com.KP);

if config.orientationControl
%    config.gains.com.KP                   = blkdiag(40*eye(3),30*eye(3));
%    config.gains.com.KD                   = 2*sqrt(config.gains.com.KP);
end

config.gains.com.KO                   = 10*eye(6);

posturalGainTorso                     = 10   *ones(1,3);
posturalGainArms                      = [ 5  5  5 10  1];
posturalGainLegs                      = [10 10 10 10 10 10];
config.gains.postural.KP              = 0.1*diag([posturalGainTorso,posturalGainArms,posturalGainArms,posturalGainLegs,posturalGainLegs]);

if config.orientationControl
%    config.gains.postural.KP              = 1*diag([posturalGainTorso,posturalGainArms,posturalGainArms,posturalGainLegs,posturalGainLegs]);
%    config.weights.momentum               = 10;
%    config.weights.postural               = 1;
%    config.weights.minHandsThrustDot      = 0.01;
%    config.weights.minFeetThrustDot       = 0.01;
%    config.weights.minJointVel            = 20;
%    config.gains.com.KO                   = 0.1*eye(6);
end
config.gains.torqueControl.KI          = 1000*eye(25);
config.gains.torqueControl.KP          = 2*sqrt(config.gains.torqueControl.KI);

%% REFERENCE TUNING 
config.references.useRefVelAccJerk    = true;

% ELICOIDAL  FLIGHT REFERENCE
config.references.elicoidalFlight     = false;
config.references.elicoidal.frequency = 0.15;
config.references.elicoidal.amplitude = 2;
config.references.elicoidal.dir       = [0;0;1];
config.references.elicoidal.dir       = config.references.elicoidal.dir/norm(config.references.elicoidal.dir);
config.references.elicoidal.vel       = 1;
config.references.elicoidal.timeReady = 10;
config.references.elicoidal.constCom  = [0;0;0];

% SINUSOIDAL FLIGHT REFERENCE
config.references.sinuisoidalFlight   = false;
config.references.sin.dirIncVel       = [0;1;1];
config.references.sin.dirIncVel       = config.references.sin.dirIncVel/norm(config.references.sin.dirIncVel);
config.references.sin.vel             = 0.0;
config.references.sin.tChangeRef      = 10;
config.references.sin.frequency       = 0.2;
config.references.sin.amplitude       = 0.0;
config.references.sin.dir             = [1;-1;1];
config.references.sin.dir             = config.references.sin.dir/norm(config.references.sin.dir);
config.references.sin.constCom        = [0;0;1];

% VARIOUS DIRECTION FLIGHT
config.references.variousDirFlight    = true;
%                                        Time  %  normVel % vel direction
config.references.varDir.series       = [10          1       0   0   1;
                                         20          1       0   1   0
                                         30          1       1   0   0
                                         40          1       0  -1   0
                                         50          1       0   0  -1
                                         60          0       1   1   1];
%if config.orientationControl && config.references.variousDirFlight
%    config.references.varDir.series       = [ 5          1       0   0   1;
%                                             10          0       0   0   1];
%end
for i = 1:size(config.references.varDir.series,1)
    config.references.varDir.series(i,3:end) = config.references.varDir.series(i,3:end)/norm(config.references.varDir.series(i,3:end));
end

% VARIOUS MINIMUM JERK VELOCITY FILGHT
config.references.minJerkVelFligt     = true;
%                                        Time  % settlingTime % normVel % vel direction
config.references.minJerkVel.series   = [ 5          5            0.5         0   0   1;
%                                         10          5            0         0   0   1
                                         10         10             1         1   0   0
                                         20         10             1         0   1   0
                                         30         10             1        -1   0   0
                                         40         10             1         0  -1   0
                                         45         10             1         0   0  -1
                                         55         10             1         0   0  -1];
                                     
config.references.minJerkVel.comDes0  = [0;0;0.1];


for i = 1:size(config.references.minJerkVel.series,1)
    config.references.minJerkVel.series(i,4:end) = config.references.minJerkVel.series(i,4:end)/norm(config.references.minJerkVel.series(i,4:end));
end                                     


%% INITIAL CONDITIONS

state0                                = zeros(13+2*config.ndof,1);
state0(4)                             = 1;

% initial joints position (deg)      
torso0                                = [  0     0    0]';           
leftArm0                              = [  0    25    0   0  0]';          
rightArm0                             = leftArm0;
      
% initial conditions for balancing on two feet  
leftLeg0                              = [  0   0   0  0  0  0]';
rightLeg0                             = leftLeg0;

 
state0(8:8+config.ndof-1)             = [torso0;leftArm0;rightArm0;leftLeg0;rightLeg0]*(pi/180);

config.qjDes                          = [zeros(13,1);leftLeg0;rightLeg0]*(pi/180);
config.state0                         = state0;
                                    
config.aux0                           = zeros(config.ndof+4,1);


%% OTHER PARAMETERS

config.demux.baseOrientationType      = 0; % 0: Rotation matrix
                                           % 1: Quaternion

config.reg.pinvTol                    = 1e-5;
config.reg.pinvDamp                   = 1e-5;
config.reg.thessianQp                 = 1e-6;

config.considerDynamics               = true;

% 4 element list identifying jets'axes: 1,2,3 identify the axes x,y,z of the 
%  above frames
config.jets.axes     = zeros(4,1);
config.jets.axes(1)  = -1;
config.jets.axes(2)  = -1;
config.jets.axes(3)  =  1;
config.jets.axes(4)  =  1; 


ROBOT_DOF = config.ndof ;


config.gains.quaternionIntegration    = 1;
config.gains.com.KI                   = 0*eye(6);

if config.controlType == 1
    alpha                                 = 4;
    config.gains.com.KD                   = 3*alpha*eye(6);
    config.gains.com.KP                   = 3*alpha^2*eye(6);
    config.gains.com.KI                   = alpha^3*eye(6);
    posturalGainTorso                     = 1   *ones(1,3);
    posturalGainArms                      = 0.05*ones(1,5);
    posturalGainLegs                      = 10  *ones(1,6);
    config.gains.postural.KP              = diag([posturalGainTorso,posturalGainArms,posturalGainArms,posturalGainLegs,posturalGainLegs]);
end