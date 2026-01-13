function [dydx] = BackwardDynamics(X, y, q, dot_q, ddot_q, Config, Const)

% The state has the form
%     | Q |   w, x, y, z                  1-4
%     | r |   x, y, z                     5-7
%     | η |   Ω1 , Ω2 , Ω3 , V1 , V2 , V3 8-13
%     | η̇ |   Ω1 , Ω2 , Ω3 , V1 , V2 , V3 14-19
%     | Λ |   C1, C2, C3, N1, N2, N3      20-25
%     | Qa|                               26-end


%   Obtain the need variables
B     = Const.B;
Xi_c  = Const.Xi_c;
L     = Const.L;
M     = Const.M_cal;



%   Compute strains
Phi = getPhi(X, Const);

Xi  = B*Phi*q + Xi_c;


%  Unpack state vector
Q       = y(1:4);
eta     = y(8:13);
dot_eta = y(14:19);
Lambda  = y(20:25);

%   Get the external force
F_bar = F_gravity(Q, Const);

%   Compute the derivatives
dydx_kinematics = ForwardKinematics(X, y, q, dot_q, ddot_q, Config, Const);
Lambda_prime    = ad_(Xi)'*Lambda + M*dot_eta - ad_(eta)'*M*eta - F_bar;
Qa_prime        = -Phi'*B'*Lambda;

%  Packing state vector derivative
dydx = L*[dydx_kinematics;
          Lambda_prime;
          Qa_prime];


end