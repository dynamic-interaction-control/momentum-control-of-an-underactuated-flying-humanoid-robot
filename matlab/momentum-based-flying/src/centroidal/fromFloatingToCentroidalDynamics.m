function [M_c,h_c, Jc_c, Jc_cDNu_c, Nu_c,CMM,CMMc] = fromFloatingToCentroidalDynamics(M, h, g, J, dJcNu, Nu, c_T_b, dT)
% centroidalConversion 
% converts dynamic equation parameters to the
% corresponding values in centroidal frame of reference
ndof           = size(g,1)-6;

invT           = eye(ndof+6)/c_T_b;
invTt          = eye(ndof+6)/(c_T_b');

%% control terms conversion
M_c            = invTt*M*invT;

M_c(1:6,7:end) = zeros(6,ndof);
M_c(7:end,1:6) = zeros(ndof,6);

M_c(1:3,1:3)   = M(1,1)*eye(3);
M_c(1:3,4:6)   = zeros(3);
M_c(4:6,1:3)   = zeros(3);

Nu_c          = c_T_b*Nu;
gravAcc       = norm(invTt*g)/M(1,1);

e3            = zeros(ndof+6,1);
e3(3)         = 1;
g_c           = M(1,1)*gravAcc*e3;

CNu           = h - g;

Mb            =   M(1:6,1:6);
Mbj           =   M(1:6,7:end);

C_cNu_c_dT    = invTt*CNu - M_c*dT*Nu;

C_cNu_c       = [ zeros(3,1); 
                  C_cNu_c_dT(4:6); 
                  CNu(7:end)-(Mbj')*(Mb\CNu(1:6))];

%% new dT*Nu computation for Jacobian
dTNu          = M_c\(CNu-C_cNu_c);  

Jc_c          = J*invT;
Jc_cDNu_c     = dJcNu - J*invT*dTNu;

CMMc          = M_c(1:6,:);

CMM           = CMMc*c_T_b;

h_c           = C_cNu_c + g_c;

end