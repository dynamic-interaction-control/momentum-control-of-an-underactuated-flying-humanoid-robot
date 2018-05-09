# Simulation for "Momentum Control of an Underactuated Flying Humanoid Robot"

## Dependencies and installation
To run the simulation scripts available in this repository, you need to have installed on your computer:
* [Matlab/Simulink](https://www.mathworks.com/products/simulink.html), at least version R2015b or following.
* [WB-Toolbox](https://github.com/robotology/WB-Toolbox)

These dependencies (in particular `WB-Toolbox`) have their own dependencies, so the most convenient way to install them is through the [`robotology-superbuild`](https://github.com/robotology/robotology-superbuild). 
You can follow the instructions in https://github.com/robotology/robotology-superbuild/blob/master/README.md, following in particular the steps in https://github.com/robotology/robotology-superbuild/blob/master/README.md#dynamics to install the software that depends on Matlab/Simulink. Regarding the visualization of the robot movements, it is currently dependent on the [iDyntree visualizer](https://github.com/dynamic-interaction-control/momentum-control-of-an-underactuated-flying-humanoid-robot/tree/master/matlab/momentum-based-flying/src/simulation). In order to be able to use it, enable the options `IDYNTREE_USES_MATLAB` and `IDYNTREE_USES_IRRLICHT` by running `ccmake ./` in iDyntree build directory. If the irrlicht library is not installed on your computer, you can install it by running `sudo apt install libirrlicht-dev`.

## Running the simulation
Once you followed the installation instructions, you can replicate the experiments present in the paper by running the script. In particular, you can run the simulation of the underactuated flying humanoid robot by simulating the [`/matlab/momentum-based-flying/momentumBasedFlying.mdl`](https://github.com/dynamic-interaction-control/momentum-control-of-an-underactuated-flying-humanoid-robot/blob/master/matlab/momentum-based-flying/momentumBasedFlying.mdl) Simulink model. The parameters, configuration options and reference trajectories for the simulation are specified in the [`matlab/simulinkModel/initMomentumBasedFlying.m`](https://github.com/dynamic-interaction-control/momentum-control-of-an-underactuated-flying-humanoid-robot/blob/master/matlab/momentum-based-flying/initMomentumBasedFlying.m) matlab script, and in the scripts inside the [app/robots/icubGazeboSim](https://github.com/dynamic-interaction-control/momentum-control-of-an-underactuated-flying-humanoid-robot/tree/master/matlab/momentum-based-flying/app/robots/icubGazeboSim) folder. By changing these script, you can replicate the results contained in the paper. In the next subsection we document how to run the different simulation experiments contained in the paper, by just changing a limited set of parameters. 

### Simulation 1a : piece-wise constant velocity trajectory without orientation control
* Set [`Config.references.variousDirFlight`](https://github.com/dynamic-interaction-control/momentum-control-of-an-underactuated-flying-humanoid-robot/blob/master/matlab/momentum-based-flying/app/robots/icubGazeboSim/initFlyingControl.m#L74) to `true`.
* Set [`Config.references.elicoidalFlight`](https://github.com/dynamic-interaction-control/momentum-control-of-an-underactuated-flying-humanoid-robot/blob/master/matlab/momentum-based-flying/app/robots/icubGazeboSim/initFlyingControl.m#L52) to `false`.
* Set [`Config.orientationControl`](https://github.com/dynamic-interaction-control/momentum-control-of-an-underactuated-flying-humanoid-robot/blob/master/matlab/momentum-based-flying/initMomentumBasedFlying.m#L52) to `false`.
* Set [`Config.testRobustness`](https://github.com/dynamic-interaction-control/momentum-control-of-an-underactuated-flying-humanoid-robot/blob/master/matlab/momentum-based-flying/initMomentumBasedFlying.m#L41) to `false`.

### Simulation 1b : piece-wise constant velocity trajectory with orientation control
* Set [`Config.references.variousDirFlight`](https://github.com/dynamic-interaction-control/momentum-control-of-an-underactuated-flying-humanoid-robot/blob/master/matlab/momentum-based-flying/app/robots/icubGazeboSim/initFlyingControl.m#L74) to `true`.
* Set [`Config.references.elicoidalFlight`](https://github.com/dynamic-interaction-control/momentum-control-of-an-underactuated-flying-humanoid-robot/blob/master/matlab/momentum-based-flying/app/robots/icubGazeboSim/initFlyingControl.m#L52) to `false`.
* Set [`Config.orientationControl`](https://github.com/dynamic-interaction-control/momentum-control-of-an-underactuated-flying-humanoid-robot/blob/master/matlab/momentum-based-flying/initMomentumBasedFlying.m#L52) to `true`.
* Set [`Config.testRobustness`](https://github.com/dynamic-interaction-control/momentum-control-of-an-underactuated-flying-humanoid-robot/blob/master/matlab/momentum-based-flying/initMomentumBasedFlying.m#L41) to `false`.

### Simulation 2 : helicoidal flight with orientation control and non-perfect control model to test controller robustness
* Set [`Config.references.variousDirFlight`](https://github.com/dynamic-interaction-control/momentum-control-of-an-underactuated-flying-humanoid-robot/blob/master/matlab/momentum-based-flying/app/robots/icubGazeboSim/initFlyingControl.m#L74) to `false`.
* Set [`Config.references.elicoidalFlight`](https://github.com/dynamic-interaction-control/momentum-control-of-an-underactuated-flying-humanoid-robot/blob/master/matlab/momentum-based-flying/app/robots/icubGazeboSim/initFlyingControl.m#L52) to `true`.
* Set [`Config.orientationControl`](https://github.com/dynamic-interaction-control/momentum-control-of-an-underactuated-flying-humanoid-robot/blob/master/matlab/momentum-based-flying/initMomentumBasedFlying.m#L52) to `true`.
* Set [`Config.testRobustness`](https://github.com/dynamic-interaction-control/momentum-control-of-an-underactuated-flying-humanoid-robot/blob/master/matlab/momentum-based-flying/initMomentumBasedFlying.m#L41) to `true`.
