function [] = visualizer_demo_idyntree(t,state,Config,references,jetsIntensitiesByWeight)

Config.visualiser.makeVideo         = false;

if Config.visualiser.makeVideo
   Config.visualiser.video.filename = 'helicoidal';  
end

Config.visualiser.timeStep = 0.05;

BackGroundColor = [0 0 0];
GridColor       = [1 1 1];

% Create two iCub visualizers: one for the inertial view and one for 
% the body fixed view  
[vizInertial,model] = open_robot_idyntree_visualizer('icub1');
[vizBodyFixed,~]    = open_robot_idyntree_visualizer('icub2');

% set camera only once for the inertial camera 
camInertial = vizInertial.camera();     
camInertial.setPosition(iDynTree.Position(-5,-5,5));     
camInertial.setTarget(iDynTree.Position(0,0,0));     

camBodyFixed = vizBodyFixed.camera();     

% Normalize thurst for enhanced simulation 
thrustIntensitiesNormalized = jetsIntensitiesByWeight.Data./max(max(jetsIntensitiesByWeight.Data));

%% Robot simulation
jointPos                         = iDynTree.JointPosDoubleArray(model);
thrustIntensitiesNormalized_idyn = iDynTree.VectorDynSize(4);
thrustIntensitiesNormalized_idyn.zero();
   
% Compute visualization timestamps 
viz_ts  = t(1):Config.visualiser.timeStep:t(end);
counter = 0;

for ts=viz_ts 

        tic;
      
        % Find closest timestamp 
        [~,idx] = min(abs(ts-t));
        
        % Extract chi 
        chi = state(idx,:)';
        
        % Extract iDynTree model position from chi 
        [basePos,shape] = iDynTreeModelPositionFromChi(chi);
        
        % Update icub pos in both viz 
        vizInertial.modelViz('icub1').setPositions(basePos,shape);
        vizBodyFixed.modelViz('icub2').setPositions(basePos,shape);
        
        % Update camera for the body camera 
        camBodyFixed.setTarget(basePos.getPosition()); 
        cameraPos = basePos.getPosition().toMatlab() + [-0.6;-0.6;0.6];
        cameraPos_idyn = iDynTree.Position();
        cameraPos_idyn.fromMatlab(cameraPos);
        camBodyFixed.setPosition(cameraPos_idyn);     
        
        % Update thrust intensities 
        thrustIntensitiesNormalized_idyn.fromMatlab(thrustIntensitiesNormalized(idx,:)');
        vizInertial.modelViz('icub1').jets().setJetsIntensity(thrustIntensitiesNormalized_idyn);
        vizBodyFixed.modelViz('icub2').jets().setJetsIntensity(thrustIntensitiesNormalized_idyn);
        
        % Draw visualizer 
        vizInertial.draw();
        vizBodyFixed.draw();
        
        % Draw frame to file 
        if( Config.visualiser.makeVideo )
            filenameInertial = strcat('../../data/video-inertial',sprintf('%04d',counter),'.png');
            filenameBodyFixed = strcat('../../data/video-body',sprintf('%04d',counter),'.png');
            vizInertial.drawToFile(filenameInertial);
            vizBody.drawToFile(filenameBodyFixed);
        end
        
        % Pause for the right amount of time
        elapsedTime = toc;
        
        % If we still need to wait, wait
        if( elapsedTime <= Config.visualiser.timeStep )
            pause(elapsedTime-Config.visualiser.timeStep);
        end
        
        % Increase counter 
        counter = counter+1;
end

end
