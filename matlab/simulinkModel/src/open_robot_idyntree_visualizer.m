function [viz,model] = open_robot_idyntree_visualizer()
    % Load model
    mdlLoader = iDynTree.ModelLoader();
    consideredJoints = iDynTree.StringVector();
    consideredJoints.push_back('torso_pitch');
    consideredJoints.push_back('torso_roll');
    consideredJoints.push_back('torso_yaw');
    consideredJoints.push_back('l_shoulder_pitch');
    consideredJoints.push_back('l_shoulder_roll');
    consideredJoints.push_back('l_shoulder_yaw');
    consideredJoints.push_back('l_elbow');
    consideredJoints.push_back('l_wrist_prosup');
    consideredJoints.push_back('r_shoulder_pitch');
    consideredJoints.push_back('r_shoulder_roll');
    consideredJoints.push_back('r_shoulder_yaw');
    consideredJoints.push_back('r_elbow');
    consideredJoints.push_back('r_wrist_prosup');
    consideredJoints.push_back('l_hip_pitch');
    consideredJoints.push_back('l_hip_roll');
    consideredJoints.push_back('l_hip_yaw');
    consideredJoints.push_back('l_knee');
    consideredJoints.push_back('l_ankle_pitch');
    consideredJoints.push_back('l_ankle_roll');
    consideredJoints.push_back('r_hip_pitch');
    consideredJoints.push_back('r_hip_roll');
    consideredJoints.push_back('r_hip_yaw');
    consideredJoints.push_back('r_knee');
    consideredJoints.push_back('r_ankle_pitch');
    consideredJoints.push_back('r_ankle_roll');
    mdlLoader.loadReducedModelFromFile('../../data/urdf/icub/model.urdf',consideredJoints);
    
    % Create visualizer 
    viz = iDynTree.Visualizer();
    vizOpt = iDynTree.VisualizerOptions();
    vizOpt.winWidth = 500;
    vizOpt.winHeight = 500;
    viz.init(vizOpt);
    
    % Add models to visualizer 
    viz.addModel(mdlLoader.model(),'icub');
    
    % Setup environment and lights 
    % Disable all environmental features 
    env = viz.enviroment();
    env.setElementVisibility('floor_grid',true);
    env.setElementVisibility('root_frame',true);
    
    sun = viz.enviroment().lightViz('sun');
    lightDir = iDynTree.Direction();
    lightDir.fromMatlab([0 -1 0]);
    sun.setDirection(lightDir);
    
    viz.enviroment().addLight('secondSun');
    secondSun = viz.enviroment().lightViz('secondSun');
    secondSun.setType(iDynTree.DIRECTIONAL_LIGHT);
    lightDir = iDynTree.Direction();
    lightDir.fromMatlab([-1 0 0]);
    secondSun.setDirection(lightDir);
    
    viz.enviroment().addLight('thirdSun');
    thirdSun = viz.enviroment().lightViz('thirdSun');
    thirdSun.setType(iDynTree.DIRECTIONAL_LIGHT);
    lightDir = iDynTree.Direction();
    lightDir.fromMatlab([1 0 0]);
    thirdSun.setDirection(lightDir);
    
    viz.enviroment().addLight('fourthSun');
    forthSun = viz.enviroment().lightViz('fourthSun');
    forthSun.setType(iDynTree.DIRECTIONAL_LIGHT);
    lightDir = iDynTree.Direction();
    lightDir.fromMatlab([1 0 0]);
    forthSun.setDirection(lightDir);
    
    viz.enviroment().setBackgroundColor(iDynTree.ColorViz(0.2,0.2,0.2,1.0));
    
    % Set camera 
    cam = viz.camera();
    cam.setPosition(iDynTree.Position(-0.1,0.9,0.5));
    cam.setTarget(iDynTree.Position(0.4,0,0.5));
    
    % Setup jets 
    modelViz = viz.modelViz('icub');

    jetsFrames = iDynTree.StringVector();
    jetsFrames.push_back('l_hand_dh_frame');
    jetsFrames.push_back('r_hand_dh_frame');
    jetsFrames.push_back('l_foot_dh_frame');
    jetsFrames.push_back('r_foot_dh_frame');
    modelViz.jets().setJetsFrames(jetsFrames);
    modelViz.jets().setJetsDimensions(0.02,0.1,0.3);
    modelViz.jets().setJetDirection(0,iDynTree.Direction(1.0,0,0));   
    modelViz.jets().setJetDirection(1,iDynTree.Direction(1.0,0,0));   
    modelViz.jets().setJetDirection(2,iDynTree.Direction(-1.0,0,0));   
    modelViz.jets().setJetDirection(3,iDynTree.Direction(-1.0,0,0));  
    orangeJet = iDynTree.ColorViz(1.0,0.6,0.1,0.0);
    modelViz.jets().setJetColor(0,orangeJet);   
    modelViz.jets().setJetColor(1,orangeJet);   
    modelViz.jets().setJetColor(2,orangeJet);   
    modelViz.jets().setJetColor(3,orangeJet);   
 

    % Return a copy of the model 
    model = mdlLoader.model().copy();
    
    return
