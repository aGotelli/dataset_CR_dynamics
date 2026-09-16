function dy = Forward_eta_dot_IDM_ORIN(X,y,t,Const,Config)
% Forward_eta_dot_IDM_ORIN  Third spatial ODE of the recursive sweep:
%                            forward acceleration/actuation sweep,
%                            integrated base -> tip (X: 0 -> Config.L).
%
% Re-derives the same forward kinematics and S,s recursion as
% Forward_eta.m / Backward_Sweep_IDM.m along the way (so this ODE can be
% integrated independently, using the already-known S_0, s_0 as part of
% its initial condition), and additionally propagates the twist rate
% eta_dot and the generalised actuation force Q_a.
%
% State
%   q       = y(1:4)              : section orientation (quaternion)
%   r       = y(5:7)               : section position
%   eta     = y(8:13)              : section twist
%   F_tilde = y(14:19)             : section inertial wrench term
%   S       = y(20:55)             : articulated-body-style S matrix (vectorised)
%   s       = y(56:61)             : articulated-body-style s vector
%   eta_dot = y(62:67)             : section twist rate
%   Q_a     = y(68:67+dim_base)    : generalised actuation force

Q       = y(1:4);
r       = y(5:7);
eta     = y(8:13);
F_tilde = y(14:19);
S_vec   = y(20:55);
s       = y(56:61);
eta_dot = y(62:67);

S = reshape(S_vec,[6 6]);

%-------------- forcing (modal strain field) -----------------%
Phi = Base_Phi(X,t,Const,Config);

Xi_a         = Phi'*Const.q;
Xi_dot_a     = Phi'*Const.q_dot;
Xi_dot_dot_a = Phi'*Const.q_dot_dot;

Xi         = Const.B*Xi_a + Const.B_bar*Const.Xi_c + Const.B*Const.Xi_0;
Xi_dot     = Const.B*Xi_dot_a;
Xi_dot_dot = Const.B*Xi_dot_dot_a;

%-------------- recursion -----------------%
B     = Const.B;
M_cal       = Const.M_cal;
M_cal_prime = Const.M_cal_prime;

[~,~,F_bar,F_bar_prime] = Forces_exterieur(X,Q,r,eta,Const,Config);

Pxx = -ad_(Xi);
Pxy = zeros(6,6);
Pyx = M_cal;
Pyy = ad_(Xi)';

px = Xi_dot_dot - ad_(Xi_dot)*eta;
py = -F_tilde - F_bar;

q_prime   = quaternion_dot(Q,Xi(1:3));
r_prime   = r_dot(Q,Xi(4:6));
eta_prime = -ad_(Xi)*eta + Xi_dot;

F_tilde_prime = F_bar_prime + ad_(eta_prime)'*M_cal*eta + ad_(eta)'*(M_cal_prime*eta + M_cal*eta_prime);

S_prime = Pyx - S*Pxx + Pyy*S - S*Pxy*S;
s_prime = Pyy*s + py - S*Pxy*s - S*px;

eta_dot_prime = (Pxx + Pxy*S)*eta_dot + Pxy*s + px;

Q_a_prime = -Phi*B'*(S*eta_dot + s);

%-------------- outputs -----------------%
S_prime_vec = reshape(S_prime,[36,1]);

dy(1:4,1)   = q_prime;
dy(5:7,1)   = r_prime;
dy(8:13,1)  = eta_prime;
dy(14:19,1) = F_tilde_prime;
dy(20:55,1) = S_prime_vec;
dy(56:61,1) = s_prime;
dy(62:67,1) = eta_dot_prime;
dy(68:67+Const.dim_base) = Q_a_prime;

end
