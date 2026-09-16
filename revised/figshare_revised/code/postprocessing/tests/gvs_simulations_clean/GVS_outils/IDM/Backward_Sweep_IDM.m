function dy = Backward_Sweep_IDM(X,y,t,Const,Config)
% Backward_Sweep_IDM  Second spatial ODE of the recursive sweep: backward
%                     wrench sweep (articulated-body-style S,s
%                     recursion), integrated tip -> base.
%
% Integrated with the change of variable X <- Config.L - X, so that the
% ODE is solved forward in its own integration coordinate while sweeping
% the physical rod backward from the tip (X=0 here, i.e. physical
% X=Config.L) to the base (X=Config.L here, i.e. physical X=0). Since
% x' = f(x) becomes x' = -f(x) under this reflection, every physical-X
% derivative computed below is negated before being returned.
%
% State
%   q       = y(1:4)   : section orientation (quaternion)
%   r       = y(5:7)   : section position
%   eta     = y(8:13)  : section twist
%   F_tilde = y(14:19) : section inertial wrench term
%   S       = y(20:55) : articulated-body-style S matrix (vectorised)
%   s       = y(56:61) : articulated-body-style s vector

Q       = y(1:4);
r       = y(5:7);
eta     = y(8:13);
F_tilde = y(14:19);
S_vec   = y(20:55);
s       = y(56:61);

S = reshape(S_vec,[6 6]);

%-------------- backward change of variable: X -> Config.L - X -----------------%
X = Config.L - X;

%-------------- forcing (modal strain field) -----------------%
Phi = Base_Phi(X,t,Const,Config);

Xi_a         = Phi'*Const.q;
Xi_dot_a     = Phi'*Const.q_dot;
Xi_dot_dot_a = Phi'*Const.q_dot_dot;

Xi         = Const.B*Xi_a + Const.B_bar*Const.Xi_c + Const.B*Const.Xi_0;
Xi_dot     = Const.B*Xi_dot_a;
Xi_dot_dot = Const.B*Xi_dot_dot_a;

%-------------- recursion -----------------%
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

%-------------- negate for the backward change of variable -----------------%
S_prime_vec = reshape(S_prime,[36,1]);

dy(1:4,1)   = -q_prime;
dy(5:7,1)   = -r_prime;
dy(8:13,1)  = -eta_prime;
dy(14:19,1) = -F_tilde_prime;
dy(20:55,1) = -S_prime_vec;
dy(56:61,1) = -s_prime;

end
