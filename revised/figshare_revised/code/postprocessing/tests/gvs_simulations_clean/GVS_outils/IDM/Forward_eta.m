function dy = Forward_eta(X,y,t,Const,Config)
% Forward_eta  First spatial ODE of the recursive sweep: forward
%              kinematics, integrated base -> tip (X: 0 -> Config.L).
%
% State
%   q   = y(1:4)  : section orientation (quaternion)
%   r   = y(5:7)  : section position
%   eta = y(8:13) : section twist
%
% Derivative
%   dy(1:4)  = q_prime   : orientation rate
%   dy(5:7)  = r_prime   : position rate
%   dy(8:13) = eta_prime : twist rate

Q   = y(1:4);
r   = y(5:7); %#ok<NASGU> % kept in the state for a uniform signature; unused here
eta = y(8:13);

%-------------- forcing (modal strain field) -----------------%
Phi = Base_Phi(X,t,Const,Config);

Xi_a     = Phi'*Const.q;
Xi_dot_a = Phi'*Const.q_dot;

% NOTE: the reference strain offset Const.Xi_0 is multiplied by zero
% here, unlike in the (retired) spectral implementation. For this
% robot's actuation pattern Const.Xi_0 evaluates to zero regardless
% (the actuated components B select are disjoint from the one nonzero
% entry of the reference strain), so this has no numerical effect on
% the results in this dataset -- but the two expressions are not
% equivalent in general, and this is a pre-existing inconsistency of
% the original implementation kept as-is rather than silently resolved.
Xi     = Const.B*Xi_a + Const.B_bar*Const.Xi_c + 0*Const.B*Const.Xi_0;
Xi_dot = Const.B*Xi_dot_a;

%-------------- kinematics -----------------%
q_prime   = quaternion_dot(Q,Xi(1:3));
r_prime   = r_dot(Q,Xi(4:6));
eta_prime = -ad_(Xi)*eta + Xi_dot;

dy(1:4,1)  = q_prime;
dy(5:7,1)  = r_prime;
dy(8:13,1) = eta_prime;

end
