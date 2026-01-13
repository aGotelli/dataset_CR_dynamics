function [dydx] = ProjectInitialConditions(X, y, q, eta_0_bar, Config, Const)

% The state has the form
%     | Q |   w, x, y, z                  1-4
%     | r |   x, y, z                     5-7
%     | q |   

%   Obtain the need variables
B     = Const.B;
Xi_c  = Const.Xi_c;
L     = Const.L;


%   Compute strains
Phi = getPhi(X, Const);

Xi      = B*Phi*q + Xi_c;

K     = Xi(1:3);
Gamma = Xi(4:6);

%  Unpack state vector
Q = y(1:4);
R = getR(Q);
r = y(5:7);

g_X_inv = [
        R'     -R'*r
    0   0   0    1
];

eta_0_X = Ad(g_X_inv)*eta_0_bar;

Lambda_coriolis = - ad(eta_0_X)'*eta_0_X;

%   Compute the derivatives
Q_prime            = 1/2*A(K)*Q;
r_prime            = R*Gamma;
% overline_Q_a_prime = Phi'*B'*Lambda_coriolis;


overline_Q_a_prime = Phi'*B'*eta_0_X;


%  Packing state vector derivative
dydx = L*[Q_prime;
          r_prime;
          overline_Q_a_prime
          ];
end


