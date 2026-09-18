function [pulled_length, cable_length] = getCablesLength(Const, Config)

%   Initialization
y0 = [Const.q_0;
      Const.r_0;
      0];


cable_length = zeros(2, 1);

cable_length_rest = [
  Config.L
  Config.L
];

q = Const.q;


% Définition des noeuds
N_noeuds = 30; % nombre de noeuds

% % Calcul des Noeuds sur la grille [0,L] legendre
[~,X_grille]=cheb(N_noeuds-1,Config.L);  % sur une grille [0,L]


D_prime = zeros(3, 1);
D = Const.D1;

%   Integrate the cableTangent function to get the lenght
[~, Y] = ode45(@(X, y) cableTangent(X, y, q, D, D_prime, Const, Config), X_grille', y0);

cable_length(1) = Y(end, end);


D = Const.D2;

%   Integrate the cableTangent function to get the lenght
[~, Y] = ode45(@(X, y) cableTangent(X, y, q, D, D_prime, Const, Config), X_grille', y0);

cable_length(2) = Y(end, end);

pulled_length = cable_length - cable_length_rest;

end


function dydx = cableTangent(X, y, q, D, D_prime, Const, Config)


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
Q_prime       =   quaternion_dot(Q,Xi(1:3));
r_prime       =   R*Gamma;

R_prime = R*hat_(K);


%   Derivative position of the tendon
phi_prime = r_prime + R_prime*D + R*D_prime;

norm_phi_prime = norm(phi_prime);


%  Packing state vector derivative
dydx = [Q_prime;
          r_prime;
          norm_phi_prime];
end