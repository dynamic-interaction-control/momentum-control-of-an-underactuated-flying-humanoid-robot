# Simulation for "Momentum Control of an Underactuated Flying Humanoid Robot"

## Dependencies and installation
To run the simulation scripts available in this repository, you need to have installed on your computer:
* [Matlab/Simulink](https://www.mathworks.com/products/simulink.html), at least version R2015b or following.
* [WB-Toolbox](https://github.com/robotology/WB-Toolbox)
* [mex-wholebodymodel](https://github.com/robotology/mex-wholebodymodel)

These dependencies (in particular `WB-Toolbox` and `mex-wholebodymodel`) have their own dependencies, so the most convenient way 
to install them is through the [`codyco-superbuild`](https://github.com/robotology/codyco-superbuild). 
You can follow the instructions in https://github.com/robotology/codyco-superbuild#installation, following in particular 
the steps in https://github.com/robotology/codyco-superbuild#matlab-software to install the software that depends on Matlab/Simulink.  

## Running the simulation
Once you followed the installation instructions, you can replicate the experiments present in the paper by running the script. 
In particular, you can run the simulation of the underactuated flying humanoid robot by simulating the [`matlab/simulinkModel/ironCub.mdl`](matlab/simulinkModel/ironCub.mdl)
Simulink model. The parameters, configuration options and reference trajectories for the simulation are specified in the in the 
[`matlab/simulinkModel/initIronCub.m`](matlab/simulinkModel/initIronCub.m) matlab script. By changing these script, you can replicate the results contained in the paper.
In the next subsection we document how to run the different simulation experiments contained in the paper, by just 
changing a limited set of parameters. 

### Simulation 1a : piece-wise constant velocity trajectory without orientation control
* Set [`config.references.variousDirFlight`](https://github.com/dynamic-interaction-control/momentum-control-of-an-underactuated-flying-humanoid-robot/blob/master/matlab/simulinkModel/initIronCub.m#L129) to `true`.
* Set [`config.references.elicoidalFlight`](https://github.com/dynamic-interaction-control/momentum-control-of-an-underactuated-flying-humanoid-robot/blob/master/matlab/simulinkModel/initIronCub.m#L107) to `false`.
* Set [`config.orientationControl`](https://github.com/dynamic-interaction-control/momentum-control-of-an-underactuated-flying-humanoid-robot/blob/master/matlab/simulinkModel/initIronCub.m#L67) to `false`.
* Set [`config.testRobustness`](https://github.com/dynamic-interaction-control/momentum-control-of-an-underactuated-flying-humanoid-robot/blob/master/matlab/simulinkModel/initIronCub.m#L15) to `false`.

### Simulation 1b : piece-wise constant velocity trajectory with orientation control
* Set [`config.references.variousDirFlight`](https://github.com/dynamic-interaction-control/momentum-control-of-an-underactuated-flying-humanoid-robot/blob/master/matlab/simulinkModel/initIronCub.m#L129) to `true`.
* Set [`config.references.elicoidalFlight`](https://github.com/dynamic-interaction-control/momentum-control-of-an-underactuated-flying-humanoid-robot/blob/master/matlab/simulinkModel/initIronCub.m#L107) to `false`.
* Set [`config.orientationControl`](https://github.com/dynamic-interaction-control/momentum-control-of-an-underactuated-flying-humanoid-robot/blob/master/matlab/simulinkModel/initIronCub.m#L67) to `true`.
* Set [`config.testRobustness`](https://github.com/dynamic-interaction-control/momentum-control-of-an-underactuated-flying-humanoid-robot/blob/master/matlab/simulinkModel/initIronCub.m#L15) to `false`.

### Simulation 2 : helicoidal flight with orientation control and non-perfect control model to test controller robustness
* Set [`config.references.variousDirFlight`](https://github.com/dynamic-interaction-control/momentum-control-of-an-underactuated-flying-humanoid-robot/blob/master/matlab/simulinkModel/initIronCub.m#L129) to `false`.
* Set [`config.references.elicoidalFlight`](https://github.com/dynamic-interaction-control/momentum-control-of-an-underactuated-flying-humanoid-robot/blob/master/matlab/simulinkModel/initIronCub.m#L107) to `true`.
* Set [`config.orientationControl`](https://github.com/dynamic-interaction-control/momentum-control-of-an-underactuated-flying-humanoid-robot/blob/master/matlab/simulinkModel/initIronCub.m#L67) to `true`.
* Set [`config.testRobustness`](https://github.com/dynamic-interaction-control/momentum-control-of-an-underactuated-flying-humanoid-robot/blob/master/matlab/simulinkModel/initIronCub.m#L15) to `true`.
