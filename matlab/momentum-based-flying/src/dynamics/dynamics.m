function stateDot = dynamics(M, h, J, f, tau, state, Config)

    % state 
    %
    % state(1:3)               =  I_p_b,     i.e. base position w.r.t. inertial frame
    % state(4:7)               =  Q_b,       i.e. quaternion representing base orientation w.r.t. base frame. More precisely, dot(Q_b) = -0.5 SB(b_omega_b)Q_b
    % state(8:8+nDof-1)        =  qj,        i.e. joint angles
    % state(8+nDof:10+nDof)    =  I_v_b,     i.e. base linear velocity w.r.t. inertial frame
    % state(11+nDof:13+nDof)   =  I_omega_b, i.e. base angular velocity w.r.t. inertial frame
    % state(14+nDof:13+2*nDof) =  qjDot,     i.e. joint velocity
    
    nDof             = size(Config.N_DOF_MATRIX,1);
    stateDot         = zeros(13+2*nDof,1);
    S                = [zeros(6,nDof); eye(nDof)];
    
    [w_H_b,~,~,~] = stateDemux(state,Config);
    
    b_omega_b = w_H_b(1:3,1:3)\state(11+nDof:13+nDof);
    
    stateDot(1:3)        = state( 8+nDof:10+nDof)';    % I_v_b
    stateDot(4:7)        = dquat(state(4:7), b_omega_b, Config.gains.quaternionIntegration);
    stateDot(8:8+nDof-1) = state(14+nDof:13+2*nDof)';  % qjDot
    stateDot(8+nDof:end) = M\(J'*f + S*tau-h);         % forwardDynamics

end