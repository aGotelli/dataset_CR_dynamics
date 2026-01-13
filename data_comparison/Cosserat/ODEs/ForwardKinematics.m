function [dydx] = ForwardKinematics(X, y, q, dot_q, ddot_q, Config, Const)

% The state has the form
%     | Q |   w, x, y, z                  1-4
%     | r |   x, y, z                     5-7
%     | η |   Ω1 , Ω2 , Ω3 , V1 , V2 , V3 8-13
%     | η̇ |   Ω1 , Ω2 , Ω3 , V1 , V2 , V3 14-19

%   Obtain the need variables
B     = Const.B;
Xi_c  = Const.Xi_c;
L     = Const.L;


%   Compute strains
Phi = getPhi(X, Const);

Xi      = B*Phi*q + Xi_c;
dot_Xi  = B*Phi*dot_q;
ddot_Xi = B*Phi*ddot_q;

K     = Xi(1:3);
Gamma = Xi(4:6);

%  Unpack state vector
Q       = y(1:4);
eta     = y(8:13);
dot_eta = y(14:19);

%   Compute the derivatives
Q_prime       =   1/2*A(K)*Q;
r_prime       =   getR(Q)*Gamma;
eta_prime     = - ad_(Xi)*eta + dot_Xi;
dot_eta_prime = - ad_(Xi)*dot_eta - ad_(dot_Xi)*eta + ddot_Xi;


%  Packing state vector derivative
dydx = L*[Q_prime;
          r_prime;
          eta_prime;
          dot_eta_prime];
end