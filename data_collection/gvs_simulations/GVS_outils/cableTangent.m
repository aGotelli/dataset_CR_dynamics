function dydx = cableTangent(X, y, q, D, D_prime, Const)


% The state has the form
%     |  Q  |   w, x, y, z                  1-4
%     |  r  |   x, y, z                     5-7
%     | phi |   L                           8




%   Obtain the need variables
B     = Const.B;
Xi_c  = Const.B_bar*Const.Xi_c;

%   Compute strains
Phi = Base_Phi(X, 0, Const, Config)';
% Phi = getPhi(X, Const);
Xi      = B*Phi*q + Xi_c;
K     = Xi(1:3);
Gamma = Xi(4:6);



%  Unpack state vector
Q = y(1:4);
Q_norm = Q/norm(Q);
R = quaternion_to_matrice(Q_norm);

%   Compute the derivatives
Q_prime       =   1/2*getA(K)*Q;
r_prime       =   R*Gamma;

R_prime = R*hat(K);


%   Derivative position of the tendon
phi_prime = r_prime + R_prime*D + R*D_prime;

norm_phi_prime = norm(phi_prime);


%  Packing state vector derivative
dydx = [Q_prime;
          r_prime;
          norm_phi_prime];
end