function [jointTorques,jointVelError] = torqueControl(jetsIntensities,matrixOfJetsAxes,jointVelDes,Min,hin,Jin,nuin,intsTildeDot,Config)
%#codegen 
    ndof = size(Config.N_DOF_MATRIX,1);
    M    = Min(:,1:ndof+6);
    h    = hin(:,1);
    J    = Jin(1:24,:);
    nu   = nuin(:,1);
    
    jointVelError   = nu(7:end) - jointVelDes;

    f               = fromJetsIntensitiesToForces(matrixOfJetsAxes,jetsIntensities);
    jointVelocityControl = - Config.gains.torqueControl.KP*jointVelError - Config.gains.torqueControl.KI*intsTildeDot;

    Jf              = [J(1:3,:);J(7:9,:);J(13:15,:);J(19:21,:)];        
    Jfb             = Jf(:,1:6);
    Jfj             = Jf(:,7:end);
    jointTorques    = (M(7:end,7:end)-M(7:end,1:6)/M(1:6,1:6)*M(1:6,7:end))*jointVelocityControl ...
                    + h(7:end)-Jfj'*f + M(7:end,1:6)/M(1:6,1:6)*(Jfb'*f-h(1:6));    
end