function [jetsIntensitiesDot,W0Dot,tau,jointVelocitiesDes,aux] = controllerFlyingHumanoids(jetsIntensities,W0,matrixOfJetsAxes,matrixOfJetsPos,s,sDes,M,h,J,JDotNu,nu,xcom,H,intH,CMM,comRefs,config)
%#codegen 
    ndof            = size(config.ndofM,1);
    if config.controlType == 2
        jetsIntensities = exp(jetsIntensities) ;
    end
    S               = [zeros(6,ndof);eye(ndof)];
    mg              = M(1,1)*config.gravityAcc*[0;0;1;zeros(3,1)];
    rDDD            = comRefs(:,4);
    rDD             = comRefs(:,3);
    rD              = comRefs(:,2);
    r               = comRefs(:,1);
    angMomentumMat  = J(:,4:6)';

    Am              = matrixOfJetsAxes;
    Mm              = [angMomentumMat(:, 1: 3)*Am(:,1),...
                       angMomentumMat(:, 7: 9)*Am(:,2),...
                       angMomentumMat(:,13:15)*Am(:,3),...
                       angMomentumMat(:,19:21)*Am(:,4)];
                   
    A               = [Am;Mm];
    
    HdDDot          = [M(1,1)*rDDD;zeros(3,1)];
    HdDot           = [M(1,1)*rDD;zeros(3,1)];
    Hd              = [M(1,1)*rD;zeros(3,1)];
    HTildeDot       = A*jetsIntensities-mg-HdDot;
    HTilde          = H-Hd;
    intHTilde       = [M(1,1)*(xcom-r);intH(4:end)];
        

    jetsIntensitiesDot = zeros(4,1);
    tau             = zeros(ndof,1); 
    jointVelocitiesDes ...
                    = zeros(ndof,1); 
    aux             = zeros(2,1);
    W0Dot           = zeros(ndof+4,1);

    rtLHand         = fromSkew2Vector(angMomentumMat(:, 1: 3));
    rtRHand         = fromSkew2Vector(angMomentumMat(:, 7: 9));
    rtLFoot         = fromSkew2Vector(angMomentumMat(:,13:15));
    rtRFoot         = fromSkew2Vector(angMomentumMat(:,19:21));

    jLHand          = Am(:,1);
    jRHand          = Am(:,2);
    jLFoot          = Am(:,3);
    jRFoot          = Am(:,4);

    Lambda =-[jetsIntensities(1)*SZBar(jLHand),...
              jetsIntensities(1)*SBar(rtLHand)*Sf(jLHand),...
              jetsIntensities(2)*SZBar(jRHand),...
              jetsIntensities(2)*SBar(rtRHand)*Sf(jRHand),...
              jetsIntensities(3)*SZBar(jLFoot),...
              jetsIntensities(3)*SBar(rtLFoot)*Sf(jLFoot),...
              jetsIntensities(4)*SZBar(jRFoot),...
              jetsIntensities(4)*SBar(rtRFoot)*Sf(jRFoot)];
          
    if config.controlType == 0
        BTilde      = [A,Lambda];
        pinvBTilde  = pinv(BTilde,config.reg.pinvTol);
%         pinvB  = pinvDamped(B,config.reg.pinvDamp);

        FDot   = config.gains.KD*(-mg-HdDot+A*jetsIntensities) ...
               + config.gains.KP*intHTilde -HdDDot;
        F      = -mg + config.gains.KD*HTilde + config.gains.KP*intHTilde;
        W      = -pinvBTilde*(F + A*jetsIntensities + FDot + config.gains.K3\HTilde);
        
        jetsIntensitiesDot = W(1:4);
        vDes   = W(5:end);
        
        v               = J*nu;
        vTilde          = v - vDes;
        Jf              = [J(1:3,:);J(7:9,:);J(13:15,:);J(19:21,:)];
        JinvM           = J/M;
        
%         JinvMSPinv     = pinv(JinvM*S,config.reg.pinvTol);
        JinvMSPinv      = pinvDamped(JinvM*S,config.reg.pinvDamp);
        
        NlambdaS        = eye(size(JinvMSPinv,1)) - pinv(JinvM*S,config.reg.pinvTol)*JinvM*S;
        
        f               = fromJetsIntensitiesToForces(matrixOfJetsAxes,jetsIntensities);
        tau0            = - Jf(:,7:end)'*f + h(7:end) ...
                          - M(7:end,7:end)*(config.gains.qj.kp*(s-sDes) ...
                          + config.gains.qj.kd*nu(7:end));

        vDesDot         = zeros(24,1);
                
        tau             = - JinvMSPinv*(JinvM*(h-Jf'*f) + JDotNu - vDesDot ...
                          + config.gains.K4C0*vTilde )...
                          + NlambdaS*tau0;
    elseif config.controlType == 1
        %% CONTROL WITH NO ASSURANCE OF POSITIVE THRUST
        Jb              = J(:,1:6); 
        Jj              = J(:,7:end); 
        JH_b             = CMM(:,1:6); 
        JH_s             = CMM(:,7:end); 
        KTilde            = config.gains.KP + inv(config.gains.K3);
        LambdaBar       = (KTilde + config.gains.KD)*JH_s + Lambda*Jj;
        BTilde               = [A,LambdaBar];
        pinvBTilde           = pinv(BTilde,config.reg.pinvTol);
        nullB           = null(BTilde);
        nullB           = nullB*nullB';
%          pinvB  = pinvDamp(B,config.reg.pinvDamp);
        
        deltaTilde        = config.gains.KD*HTildeDot + ...
                         (KTilde + config.gains.KD)*(JH_b*nu(1:6)-Hd) + ...
                          Lambda*Jb*nu(1:6) -HdDDot -mg +config.gains.KP*intHTilde ...
                          -HdDot + A*jetsIntensities;

        W               = -pinvBTilde*deltaTilde + nullB*W0;
        
        jetsIntensitiesDot = W(1:4);
        sDotDes        = W(5:end);
        postural        = -config.gains.postural.KP*(s-sDes) -config.gains.postural.KD*nu(7:end);        
        
        W0Dot           = pinv(nullB(5:end,:),config.reg.pinvTol)*postural;
        qjDDdes         = nullB(5:end,:)*W0Dot;
        
        sDotTilde      = nu(7:end) - sDotDes;
        Jf              = [J(1:3,:);J(7:9,:);J(13:15,:);J(19:21,:)];
        Jfj             = Jf(:,7:end);
                
        f               = fromJetsIntensitiesToForces(matrixOfJetsAxes,jetsIntensities);

        tau             = M(7:end,7:end)*(qjDDdes - config.gains.K4*sDotTilde ) ...
                        + h(7:end)-Jfj'*f;
                             
        aux(1)          = norm(sDotTilde);
     elseif config.controlType == 2
        %% CONTROL WITH NO ASSURANCE OF POSITIVE THRUST AND DIFFERENT POSTURAL
        
        if config.orientationControl
        	intHTilde   = [intHTilde(1:3);intH(4:end)];
        end
        
        Lambda          = Lambda*J;
        Lambda_b        = Lambda(:,1:6);
        Lambda_s        = Lambda(:,7:end);
        JH_b            = CMM(:,1:6); 
        JH_s            = CMM(:,7:end); 
        KTilde          = config.gains.KP + inv(config.gains.K3) + config.gains.KD;
        BTilde          = [A, Lambda_s + KTilde*JH_s];
        pinvBTilde      = pinv(BTilde,config.reg.pinvTol);
        nullB           = null(BTilde);
        nullB           = nullB*nullB';
        
        deltaTilde      = (Lambda_b + KTilde *JH_b)*nu(1:6) - KTilde * Hd ...
                          + config.gains.KD*HTildeDot -HdDDot -mg +config.gains.KP*intHTilde ...
                          -HdDot + A*jetsIntensities;
                      
        postural        = -config.gains.postural.KP*(s-sDes);        

        W0              = [zeros(4,1);
                           postural]; 

        W               = -pinvBTilde*deltaTilde + nullB*W0;
        
        jetsIntensitiesDot ...
                        = W(1:4);
        sDotDes        = W(5:end);

        sDotTilde      = nu(7:end) - sDotDes;
        Jf              = [J(1:3,:);J(7:9,:);J(13:15,:);J(19:21,:)];        
        Jfb             = Jf(:,1:6);
        Jfj             = Jf(:,7:end);
                
        f               = fromJetsIntensitiesToForces(matrixOfJetsAxes,jetsIntensities);

        if config.useBaseCoordinates
            tau             = (M(7:end,7:end)-M(7:end,1:6)/M(1:6,1:6)*M(1:6,7:end))*( - config.gains.K4*sDotTilde ) ...
                            + h(7:end)-Jfj'*f + M(7:end,1:6)/M(1:6,1:6)*(Jfb'*f-h(1:6));
        else
            tau             = M(7:end,7:end)*( - config.gains.K4*sDotTilde ) ...
                            + h(7:end)-Jfj'*f;
        end
                             
        aux(1)          = norm(sDotTilde);
    elseif config.controlType == 3
        %% CONTROL WITH POSITIVE THRUST

        Jb              = J(:,1:6); 
        Jj              = J(:,7:end); 
        JH_b             = CMM(:,1:6); 
        JH_s             = CMM(:,7:end); 
        KTilde            = config.gains.KP + inv(config.gains.K3);
        LambdaBar       = (KTilde + config.gains.KD)*JH_s + Lambda*Jj;
        BTilde          = [A*diag(jetsIntensities),LambdaBar];
        pinvBTilde      = pinv(BTilde,config.reg.pinvTol);
        nullB           = null(BTilde);
        nullB           = nullB*nullB';
        
        deltaTilde        = config.gains.KD*HTildeDot + ...
                         (KTilde + config.gains.KD)*(JH_b*nu(1:6)-Hd) + ...
                          Lambda*Jb*nu(1:6) -HdDDot -mg +config.gains.KP*intHTilde ...
                          -HdDot + A*jetsIntensities;

        W               = -pinvBTilde*deltaTilde + nullB*W0;
        
        jetsIntensitiesDot = W(1:4);
        sDotDes        = W(5:end);
        postural        = -config.gains.postural.KP*(s-sDes) -config.gains.postural.KD*nu(7:end);        
        
        W0Dot(:)        = pinv(nullB(5:end,:),config.reg.pinvTol)*postural;
        qjDDdes         = nullB(5:end,:)*W0Dot;
        
        sDotTilde      = nu(7:end) - sDotDes;
        Jf              = [J(1:3,:);J(7:9,:);J(13:15,:);J(19:21,:)];
        Jfj             = Jf(:,7:end);
                
        f               = fromJetsIntensitiesToForces(matrixOfJetsAxes,jetsIntensities);

        tau             = M(7:end,7:end)*(qjDDdes - config.gains.K4*sDotTilde ) ...
                        + h(7:end)-Jfj'*f;
                             
        aux(1)          = norm(sDotTilde);
    elseif config.controlType == 4
        %% CONTROL WITH ALMOST-EQUAL THRUST

        Jb              = J(:,1:6); 
        Jj              = J(:,7:end); 
        JH_b             = CMM(:,1:6); 
        JH_s             = CMM(:,7:end); 
        KTilde            = config.gains.KP + inv(config.gains.K3);
        LambdaBar       = (KTilde + config.gains.KD)*JH_s + Lambda*Jj;
        BTilde               = [A*diag(jetsIntensities),LambdaBar];
        pinvBTilde           = pinv(BTilde,config.reg.pinvTol);
        nullB           = null(BTilde);
        nullB           = nullB*nullB';
%          pinvB  = pinvDamp(B,config.reg.pinvDamp);
        
        deltaTilde        = config.gains.KD*HTildeDot + ...
                         (KTilde + config.gains.KD)*(JH_b*nu(1:6)-Hd) + ...
                          Lambda*Jb*nu(1:6) -HdDDot -mg +config.gains.KP*intHTilde ...
                          -HdDot + A*jetsIntensities;

        W               = -pinvBTilde*deltaTilde + nullB*W0;  % W = [jetsIntensitiesDot
                                                       %      qjDotDes          ]
        
        jetsIntensitiesDot = W(1:4);
        sDotDes        = W(5:end);
        postural        = -config.gains.postural.KP*(s-sDes) -config.gains.postural.KD*nu(7:end);        
        
        W0Dot(:)        = pinv(nullB(5:end,:),config.reg.pinvTol)*postural;
        qjDDdes         = nullB(5:end,:)*W0Dot;
        
        sDotTilde      = nu(7:end) - sDotDes;
        Jf              = [J(1:3,:);J(7:9,:);J(13:15,:);J(19:21,:)];
        Jfj             = Jf(:,7:end);
                
        f               = fromJetsIntensitiesToForces(matrixOfJetsAxes,jetsIntensities);

        tau             = M(7:end,7:end)*(qjDDdes - config.gains.K4*sDotTilde ) ...
                        + h(7:end)-Jfj'*f;
                      
        F               = -mg + config.gains.KD*HTilde + config.gains.KP*intHTilde;
       
        aux(1)          = norm(sDotTilde);
    elseif config.controlType == 5
        %% CONTROL WITH RELATIVE DEGREE AGMENTATION 
        
        if config.orientationControl
        	intHTilde   = [intHTilde(1:3);intH(4:end)];
        end
        
        Lambda          = Lambda*J;
        Lambda_b        = Lambda(:,1:6);
        Lambda_s        = Lambda(:,7:end);
        JH_b            = CMM(:,1:6); 
        JH_s            = CMM(:,7:end); 
        BTilde          = [A, Lambda_s + config.gains.KP*JH_s];
        pinvBTilde      = pinv(BTilde,config.reg.pinvTol);
        nullB           = null(BTilde);
        nullB           = nullB*nullB';
        
        deltaTilde      = (Lambda_b + config.gains.KP *JH_b)*nu(1:6) -HdDDot ...
                          - config.gains.KP * HdDot + config.gains.KD*HTildeDot + config.gains.KI*intHTilde;
                      
        postural        = -config.gains.postural.KP*(s-sDes);        

        W0              = [zeros(4,1);
                           postural]; 

        W               = -pinvBTilde*deltaTilde + nullB*W0;
        
        jetsIntensitiesDot ...
                        = W(1:4);
        sDotDes         = W(5:end);

        sDotTilde       = nu(7:end) - sDotDes;
        Jf              = [J(1:3,:);J(7:9,:);J(13:15,:);J(19:21,:)];
        Jfb             = Jf(:,1:6);
        Jfj             = Jf(:,7:end);
                
        f               = fromJetsIntensitiesToForces(matrixOfJetsAxes,jetsIntensities);

        if config.useBaseCoordinates
            tau             = (M(7:end,7:end)-M(7:end,1:6)/M(1:6,1:6)*M(1:6,7:end))*( - config.gains.K4*sDotTilde ) ...
                            + h(7:end)-Jfj'*f + M(7:end,1:6)/M(1:6,1:6)*(Jfb'*f-h(1:6));
        else
            tau             = M(7:end,7:end)*( - config.gains.K4*sDotTilde ) ...
                            + h(7:end)-Jfj'*f;
        end
        jointVelocitiesDes ...
                        = sDotDes;
        aux(1)          = norm(sDotTilde);
    end

end


