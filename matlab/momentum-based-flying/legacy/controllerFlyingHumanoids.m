function [jetsIntensitiesDot,jointTorquesJointVels,jointVelocitiesDes,sTildeDot,aux] = controllerFlyingHumanoids(jetsIntensities,matrixOfJetsAxes,matrixOfJetsArms,w_H_b,s,sDes,Min,hin,Jin,JDotNu,nuin,xcom,H,intH,CMMin,comRefs,baseRotRefs,intsTildeDot,config)
%#codegen 
    ndof = size(config.ndofM,1);
    M    = Min(:,1:ndof+6);
    Mc   = Min(:,ndof+7:end);
    h    = hin(:,1);
    hc   = hin(:,2);
    J    = Jin(1:24,:);
    Jc   = Jin(25:end,:);
    nu   = nuin(:,1);
    nuc  = nuin(:,2);
    CMM  = CMMin(1:6,:);
    CMMc = CMMin(7:end,:);
    
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
        
    
    jetsIntensitiesDot = zeros(4,1);
    jointTorquesJointVels    = zeros(ndof,1); 
    sDotDes         = zeros(ndof,1); 
    sTildeDot       = zeros(ndof,1); 
    aux             = zeros(2,1);

    
    Lambda =-[jetsIntensities(1)*SZBar(jLHand),...
              jetsIntensities(1)*SBar(rtLHand)*Sf(jLHand),...
              jetsIntensities(2)*SZBar(jRHand),...
              jetsIntensities(2)*SBar(rtRHand)*Sf(jRHand),...
              jetsIntensities(3)*SZBar(jLFoot),...
              jetsIntensities(3)*SBar(rtLFoot)*Sf(jLFoot),...
              jetsIntensities(4)*SZBar(jRFoot),...
              jetsIntensities(4)*SBar(rtRFoot)*Sf(jRFoot)];

    %% CONTROL WITH NO ASSURANCE OF POSITIVE THRUST
    if config.orientationControl
        A_R_B                = w_H_b(1:3,1:3);
        intHTilde(4:6)       = Mc(4:6,4:6)*skewVee(baseRotRefs(1:3,1:3)'*A_R_B); %  + skew(nu(4:6))*H(4:6);
    end
    
    if config.controlType == 0
        if config.useBaseCoordinates
            Jx              = J;
            CMMx            = CMM;
            nux             = nu;
        else
            Jx              = Jc;
            CMMx            = CMMc;
            nux             = nuc;
        end
        
        Lambda          = Lambda*Jx;
        Lambda_b        = Lambda(:,1:6);
        Lambda_s        = Lambda(:,7:end);
        JH_b            = CMMx(:,1:6); 
        JH_s            = CMMx(:,7:end); 
        KTilde          = config.gains.com.KP + inv(config.gains.com.K3) + config.gains.com.KD;
        
        BTilde          = [A, Lambda_s+KTilde*JH_s];

        pinvBTilde      = pinvDamped(BTilde,config.reg.pinvDamp);
        nullB           = null(BTilde);
        nullB           = nullB*nullB';
        
        deltaTilde      = (Lambda_b + KTilde *JH_b)*nux(1:6) - KTilde * Hd ...
                          + (config.gains.com.KD + eye(6))*HTildeDot -HdDDot +config.gains.com.KP*intHTilde;
                      
        postural        = -config.gains.postural.KP*(s-sDes);        

        W0              = zeros(ndof+4,1);
        
        if config.posturalType == 0
            W0              = [zeros(4,1);
                               postural]; 
                           
        elseif config.posturalType == 1
            Ss              = [zeros(ndof,4),eye(ndof)];
            pinvSsN         = pinvDamped(Ss*nullB,config.reg.pinvDamp);
            W0              = pinvSsN*(Ss*pinvBTilde*deltaTilde+postural);
            
        elseif config.posturalType == 2
            symmetJets      = [1 0;1 0;0 1;0 1];
            sym             = [symmetJets,zeros(4,ndof)
                               zeros(ndof,2),eye(ndof)];
                           
            Ss              = [zeros(ndof,4),eye(ndof)];
            pinvSsN         = pinvDamped(Ss*nullB*sym,config.reg.pinvDamp);
            W0Sym           = pinvSsN*(Ss*pinvBTilde*deltaTilde+postural);
            W0              = sym*W0Sym;
        end

        W               = -pinvBTilde*deltaTilde + nullB*W0;
        
        jetsIntensitiesDot ...
                        = W(1:4);
        sDotDes         = W(5:end);
        sTildeDot       = nu(7:end) - sDotDes;

        f               = fromJetsIntensitiesToForces(matrixOfJetsAxes,jetsIntensities);
        jointVelocityControl = - config.gains.jointVelCtrl.KP*sTildeDot - config.gains.jointVelCtrl.KI*intsTildeDot;

        if config.useBaseCoordinates
            Jf              = [J(1:3,:);J(7:9,:);J(13:15,:);J(19:21,:)];        
            Jfb             = Jf(:,1:6);
            Jfj             = Jf(:,7:end);
            jointTorquesJointVels             = (M(7:end,7:end)-M(7:end,1:6)/M(1:6,1:6)*M(1:6,7:end))*jointVelocityControl ...
                            + h(7:end)-Jfj'*f + M(7:end,1:6)/M(1:6,1:6)*(Jfb'*f-h(1:6));
        else
            Jcf             = [Jc(1:3,:);Jc(7:9,:);Jc(13:15,:);Jc(19:21,:)];        
            Jcfj            = Jcf(:,7:end);
            jointTorquesJointVels             = Mc(7:end,7:end)*jointVelocityControl ...
                            + hc(7:end)-Jcfj'*f;
        end
                             
        aux(1)          = norm(sTildeDot);
        aux(2)          = norm(deltaTilde + BTilde*W);
        
    elseif config.controlType == 1
        %% CONTROL WITH RELATIVE DEGREE AGMENTATION 
        
        Lambda          = Lambda*J;
        Lambda_b        = Lambda(:,1:6);
        Lambda_s        = Lambda(:,7:end);
        JH_b            = CMM(:,1:6); 
        JH_s            = CMM(:,7:end); 
        BTilde          = [A, Lambda_s + config.gains.com.KP*JH_s];
        pinvBTilde      = pinvDamped(BTilde,config.reg.pinvDamp);
        nullB           = null(BTilde);
        nullB           = nullB*nullB';
        
        deltaTilde      = (Lambda_b + config.gains.com.KP *JH_b)*nu(1:6) -HdDDot ...
                          - config.gains.com.KP * HdDot + config.gains.com.KD*HTildeDot + config.gains.com.KI*intHTilde;
                      
        postural        = -config.gains.postural.KP*(s-sDes);       
        
        W0              = zeros(ndof+4,1);

        if config.posturalType == 0
            W0              = [zeros(4,1);
                               postural]; 
                           
        elseif config.posturalType == 1
            Ss              = [zeros(ndof,4),eye(ndof)];
            pinvSsN         = pinvDamped(Ss*nullB,config.reg.pinvDamp);
            W0              = pinvSsN*(Ss*pinvBTilde*deltaTilde+postural);
            
        elseif config.posturalType == 2
            symmetJets      = [1 0;1 0;0 1;0 1];
            sym             = [symmetJets,zeros(4,ndof)
                               zeros(ndof,2),eye(ndof)];
                           
            Ss              = [zeros(ndof,4),eye(ndof)];
            pinvSsN         = pinvDamped(Ss*nullB*sym,config.reg.pinvDamp);
            W0Sym           = pinvSsN*(Ss*pinvBTilde*deltaTilde+postural);
            W0              = sym*W0Sym;
        end

        W               = -pinvBTilde*deltaTilde + nullB*W0;
        
        jetsIntensitiesDot ...
                        = W(1:4);
                
        sDotDes         = W(5:end);
        sTildeDot       = nu(7:end) - sDotDes;

        f               = fromJetsIntensitiesToForces(matrixOfJetsAxes,jetsIntensities);
        jointVelocityControl = - config.gains.jointVelCtrl.KP*sTildeDot - config.gains.jointVelCtrl.KI*intsTildeDot;

        if config.useBaseCoordinates
            Jf              = [J(1:3,:);J(7:9,:);J(13:15,:);J(19:21,:)];        
            Jfb             = Jf(:,1:6);
            Jfj             = Jf(:,7:end);
            jointTorquesJointVels             = (M(7:end,7:end)-M(7:end,1:6)/M(1:6,1:6)*M(1:6,7:end))*jointVelocityControl ...
                            + h(7:end)-Jfj'*f + M(7:end,1:6)/M(1:6,1:6)*(Jfb'*f-h(1:6));
        else
            Jcf             = [Jc(1:3,:);Jc(7:9,:);Jc(13:15,:);Jc(19:21,:)];        
            Jcfj            = Jcf(:,7:end);
            jointTorquesJointVels             = Mc(7:end,7:end)*jointVelocityControl ...
                            + hc(7:end)-Jcfj'*f;
        end
                             
        aux(1)          = norm(sTildeDot);
        aux(2)          = norm(deltaTilde + BTilde*W);
    end

    jointVelocitiesDes = sDotDes;
end


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