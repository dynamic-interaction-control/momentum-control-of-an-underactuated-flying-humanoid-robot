function [w_H_b, jointAngles, baseVelocity, jointsVelocity] = stateDemux(state, Config)

    % state 
    %
    % state(1:3)               =  I_p_b,     i.e. base position w.r.t. inertial frame
    % state(4:7)               =  Q_b,       i.e. quaternion representing base orientation w.r.t. base frame. More precisely, dot(Q_b) = -0.5 SB(b_omega_b)Q_b
    % state(8: 8+nDof-1)       =  qj,        i.e. joint angles
    % state(8+nDof:10+nDof)    =  I_v_b,     i.e. base linear velocity w.r.t. inertial frame
    % state(11+nDof:13+nDof)   =  I_omega_b, i.e. base angular velocity w.r.t. inertial frame
    % state(14+nDof:13+2*nDof) =  qjDot,     i.e. joint angles
    
    nDof            = size(Config.N_DOF_MATRIX,1);
    I_p_b           = state(1:3);               
    Q_b             = state(4:7)';
    jointAngles     = state(8:8+nDof-1);
    baseVelocity    = state(8+nDof:13+nDof)';
    jointsVelocity  = state(14+nDof:13+2*nDof)';
    
    % base pose as transformation matrix
    I_R_b           = quat2rotm(Q_b);
    w_H_b           = [I_R_b, I_p_b; zeros(1,3), 1];
end