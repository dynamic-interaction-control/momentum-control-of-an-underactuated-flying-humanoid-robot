load('../../data/RALSIM17_squareFlight.mat')
visualizer_demo_idyntree(time.Data,state.Data,config,desired_x_dx_ddx_dddx_CoM,jetsIntensitiesByWeight);