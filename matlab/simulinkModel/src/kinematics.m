function stateDot = kinematics(jointVels,state,config)
    %#codegen

    % state  
    % state(1:3)               =  I_p_b,     i.e. base position w.r.t. inertial frame
    % state(4:7)               =  Q_b,       i.e. quaternion representing base orientation w.r.t. base frame. More precisely, dot(Q_b) = -0.5 SB(b_omega_b)Q_b
    % state(      8: 8+nDof-1) =  qj,        i.e. joint angles
    % state( 8+nDof:10+nDof)   =  I_v_b,     i.e. base linear velocity w.r.t. inertial frame
    % state(11+nDof:13+nDof)   =  I_omega_b, i.e. base angular velocity w.r.t. inertial frame
    % state(14+nDof:13+2*nDof) =  qjDot,     i.e. joint velocity
    
    nDof          = size(config.ndofM,1);
    stateDot      = zeros(13+2*nDof,1);
    S             = [zeros(6,nDof); eye(nDof)];
    
    [basePose,~,~,~] = stateDemux(state,config);
    
    b_omega_b = basePose(1:3,1:3)\state(11+nDof:13+nDof);
    
    stateDot(1:3)        = state( 8+nDof:10+nDof)';      % I_v_b
    stateDot(4:7)        = quaternionDerivative(b_omega_b,state(4:7));
    stateDot(8:8+nDof-1) = jointVels;                    % qjDot
    stateDot(8+nDof:end) = zeros(1,nDof+6);              % qjDDot forwardDynamics

end