function I_desRotAcc = rotationalPID(rotMatrix,I_angVel,I_desRotMatAngVelAcc,gainsPD)
    
    % rotMatrix = I_R_B
    %             3x3 rotation matrix transforming a vector expressed w.r.t. a body frame B, e.g. p_B, into 
    %             a vector expressed in the inertial frame I, e.g. p_A, i.e.
    %            
    %                                   p_I = I_R_B * p_B
    %
    % I_angVel  = omega_I 
    %             3x1 vector representing the angular velocity expressed w.r.t. the inertial frame I, i.e.
    %
    %                                   dot(I_R_B) = S(I_angVel) * I_R_B
    %         
    %             with S(x) the skew-symmetric operator associated with the cross product in R^3.
    %
    % I_desRotMatAngVelAcc = [I_RDes_B,omegaDes_I,omegaDotDes_I]
    %             3x5 matrix containing the desired rotation matrix I_RDes_B, and the desired angular velocity omegaDes_I and 
    %             the desired angular acceleration omegaDotDes_I, both expressed w.r.t. the inertial frame.
    %   
    % gainsPD = [Kp,Kd]
    %             1x2 vector containing proportional and derivative gains. 
    %
    
    B_angVel      = rotMatrix'*I_angVel;
    desRotMat     = I_desRotMatAngVelAcc(1:3,1:3);
    B_desAngVel   = desRotMat'*I_desRotMatAngVelAcc(:,4);
    
    B_desRotAcc   = - gainsPD(:,1)*gainsPD(:,2)*skewVee(desRotMat'*rotMatrix)  ...
                    - gainsPD(:,2)*(B_angVel-B_desAngVel)...
                    - gainsPD(:,1)*skewVee(desRotMat'*rotMatrix*skew(B_angVel) - skew(B_desAngVel)*desRotMat'*rotMatrix);
    
    I_desRotAcc   = rotMatrix*B_desRotAcc + I_desRotMatAngVelAcc(:,5);
end
