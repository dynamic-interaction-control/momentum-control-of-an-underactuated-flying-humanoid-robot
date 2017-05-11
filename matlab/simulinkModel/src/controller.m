function [hessianMatrixQP,gVectorQP,constraintMatA,lowerBoundQPMatA,upperBoundQPMatA,lowerBoundQP,upperBoundQP,aux] = controller(jetsIntensities,matrixOfJetsAxes,matrixOfJetsArms,w_H_b,s,sDes,Min,hin,Jin,Jcom,JDotNu,nuin,xcom,H,intH,CMMin,comRefs,baseRotRefs,config)
%#codegen 
    ndof = size(config.ndofM,1);
    
    robustness = 1;
    if config.testRobustness
        robustness = config.robustnessFactor;
    end
    M    = Min(:,1:ndof+6)*robustness;
    Mc   = Min(:,ndof+7:end)*robustness;
    h    = hin(:,1)*robustness;
    hc   = hin(:,2)*robustness;
    J    = Jin(1:24,:);
    Jc   = Jin(25:end,:);
    nu   = nuin(:,1);
    nuc  = nuin(:,2);
    CMM  = CMMin(1:6,:)*robustness;
    CMMc = CMMin(7:end,:)*robustness;
    
    mg              = M(1,1)*config.gravityAcc*[0;0;1;zeros(3,1)];

    [r,HdDDot,HdDot,Hd] = computeDesiredValues(comRefs,M(1,1));
    
    [rtLHand,rtRHand,rtLFoot,rtRFoot,jLHand,jRHand,jLFoot,jRFoot] = ...
        computeJetAxesAndArms(matrixOfJetsAxes,matrixOfJetsArms);
    
    angMomentumMat  = [skew(rtLHand)*jLHand,...
                       skew(rtRHand)*jRHand,...
                       skew(rtLFoot)*jLFoot,...
                       skew(rtRFoot)*jRFoot];
                   
    A               = [matrixOfJetsAxes;angMomentumMat];
    
    HDot            = A*jetsIntensities-mg;
    HTildeDot       = HDot-HdDot;
    intHTilde       = [M(1,1)*(xcom-r);intH(4:end)];
        
    BTilde          = zeros(6,4+ndof);
    deltaTilde      = zeros(6,1);
    
    aux             = zeros(2,1);
    
    Lambda =-[jetsIntensities(1)*SZBar(jLHand),...
              jetsIntensities(1)*SBar(rtLHand)*Sf(jLHand),...
              jetsIntensities(2)*SZBar(jRHand),...
              jetsIntensities(2)*SBar(rtRHand)*Sf(jRHand),...
              jetsIntensities(3)*SZBar(jLFoot),...
              jetsIntensities(3)*SBar(rtLFoot)*Sf(jLFoot),...
              jetsIntensities(4)*SZBar(jRFoot),...
              jetsIntensities(4)*SBar(rtRFoot)*Sf(jRFoot)];
       
    JcomExt = [Jcom;zeros(3,ndof+6);Jcom;zeros(3,ndof+6);Jcom;zeros(3,ndof+6);Jcom;zeros(3,ndof+6)];

    %% CONTROL WITH NO ASSURANCE OF POSITIVE THRUST
    if config.orientationControl
        A_R_B                = w_H_b(1:3,1:3);
        intHTilde(4:6)       = Mc(4:6,4:6)*skewVee(baseRotRefs(1:3,1:3)'*A_R_B); %  + skew(nu(4:6))*H(4:6);
    end
    
    if config.controlType == 0
        Lambda          = Lambda*(J-JcomExt);
        Lambda_b        = Lambda(:,1:6);
        Lambda_s        = Lambda(:,7:end);
        JH_b            = CMM(:,1:6); 
        JH_s            = CMM(:,7:end); 
        KTilde          = config.gains.com.KP + inv(config.gains.com.KO) + config.gains.com.KD;
        
        if config.orientationControl && config.orientationCorrection
            KTilde      = blkdiag(config.gains.com.KP(1:3,1:3),zeros(3)) + inv(config.gains.com.KO) + config.gains.com.KD;
        end
        BTilde          = [A, Lambda_s+KTilde*JH_s];
        
        deltaTilde      = (Lambda_b + KTilde *JH_b)*nu(1:6) - KTilde * Hd ...
                          + (config.gains.com.KD + eye(6))*HTildeDot -HdDDot +config.gains.com.KP*intHTilde; 
          
        if config.orientationControl && config.orientationCorrection
            deltaTilde  = deltaTilde + [zeros(3,1);config.gains.com.KP(4:6,4:6)*Mc(4:6,4:6)*skewVee(baseRotRefs(1:3,1:3)'*skew(nu(4:6))*w_H_b(1:3,1:3))];
        end              
                      
    elseif config.controlType == 1
        %% CONTROL WITH RELATIVE DEGREE AGMENTATION 
        
        Lambda          = Lambda*(J-JcomExt);
        Lambda_b        = Lambda(:,1:6);
        Lambda_s        = Lambda(:,7:end);
        JH_b            = CMM(:,1:6); 
        JH_s            = CMM(:,7:end); 
        BTilde          = [A, Lambda_s + config.gains.com.KP*JH_s];
        
        deltaTilde      = (Lambda_b + config.gains.com.KP *JH_b)*nu(1:6) -HdDDot ...
                          - config.gains.com.KP * HdDot + config.gains.com.KD*HTildeDot + config.gains.com.KI*intHTilde;
                                                   
    end
    
    postural        = -config.gains.postural.KP*(s-sDes);        
    


    %% Hessian construction
    % Postural thrust variation minimizations, joint velocity minimsation,
    % and postural task
    hessianMatrixQP = blkdiag(config.weights.minHandsThrustDot*eye(2),config.weights.minFeetThrustDot*eye(2),eye(ndof)*(config.weights.postural+config.weights.minJointVel)); 
    % Symmetry thrust task
    eS1             = [0;0;0;0];
    eS2             = [0;0;0;0];
    eS3             = [0;0;0;0];
      
    if config.weights.symLeftAndRightThrusts
        eS1             = [1;0;-1;0];
        eS2             = [0;1;0;-1];
    elseif config.weights.symHandsAndFeetThrusts
        eS1             = [1;-1;0;0];
        eS2             = [0;0;1;-1];
    elseif config.weights.symFeetThrusts
        eS1             = [0;0;1;-1];
    elseif config.weights.symHandsThrusts  
        eS1             = [1;-1;0;0];
    elseif config.weights.symAllThrusts 
        eS1             = [1;-1;0;0];
        eS2             = [0;0;1;-1];  
        eS3             = [0;1;-1;0];  
    end
    hessianMatrixQP =  hessianMatrixQP + blkdiag((eS1*eS1'+eS2*eS2'+eS3*eS3')*config.weights.symmetryThrust,zeros(ndof));                                                  

    % Momentum task
    % hessianMatrixQP =  hessianMatrixQP + BTilde'*BTilde*config.weights.momentum;                                                  
    
    % Regularize hessian to impose symmetry and positive definiteness
    hessianMatrixQP = (hessianMatrixQP + hessianMatrixQP')/2 + eye(size(hessianMatrixQP,1))*config.reg.thessianQp;
    
    %% gVector Contrstuction
    % Postural task
    gVectorQP       = [zeros(4,1);-postural]*config.weights.postural;
    % Momentum task
    %gVectorQP       = gVectorQP  + BTilde'*deltaTilde*config.weights.momentum;
    
    constraintMatA  =  BTilde;
    lowerBoundQPMatA= -deltaTilde - config.weights.threshEqCons*ones(size(deltaTilde,1),1);
    upperBoundQPMatA= -deltaTilde + config.weights.threshEqCons*ones(size(deltaTilde,1),1);
    lowerBoundQP    = -[config.maxJetsIntVar;config.maxJointVelDes];
    upperBoundQP    = -lowerBoundQP;
                    
    
    x               = -pinv(BTilde,config.reg.pinvTol)*deltaTilde;
    aux(2)          = norm(BTilde*x + deltaTilde);
end






















%% AUXILIARY FUNCTIONS

function [r,HdDDot,HdDot,Hd] = computeDesiredValues(comRefs,m)
    rDDD            = comRefs(:,4);
    rDD             = comRefs(:,3);
    rD              = comRefs(:,2);
    r               = comRefs(:,1);
    HdDDot          = [m*rDDD;zeros(3,1)];
    HdDot           = [m*rDD;zeros(3,1)];
    Hd              = [m*rD;zeros(3,1)];
end

function [rtLHand,rtRHand,rtLFoot,rtRFoot,jLHand,jRHand,jLFoot,jRFoot] = computeJetAxesAndArms(matrixOfJetsAxes,matrixOfJetsArms)
    rtLHand         = matrixOfJetsArms(:,1);
    rtRHand         = matrixOfJetsArms(:,2);
    rtLFoot         = matrixOfJetsArms(:,3);
    rtRFoot         = matrixOfJetsArms(:,4);
    
    jLHand          = matrixOfJetsAxes(:,1);
    jRHand          = matrixOfJetsAxes(:,2);
    jLFoot          = matrixOfJetsAxes(:,3);
    jRFoot          = matrixOfJetsAxes(:,4);
end