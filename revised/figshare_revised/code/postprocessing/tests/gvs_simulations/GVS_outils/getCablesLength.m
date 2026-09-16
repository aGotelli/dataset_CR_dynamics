function [pulled_length,cable_length] = getCablesLength(Const,Config)
% getCablesLength  Current length of each tendon, obtained by integrating
%                   its tangent vector's norm along the rod, and the
%                   corresponding pulled length relative to the rod's
%                   rest length.

y0 = [Const.q_0;
      Const.r_0;
      0];

cable_length = zeros(2,1);

cable_length_rest = [
  Config.L
  Config.L
];

q = Const.q;

N_nodes = 30;
[~,X_grid] = cheb(N_nodes-1,Config.L);

D_prime = zeros(3,1);
D = Const.D1;

%   Integrate the cable tangent norm to get the length
[~,Y] = ode45(@(X,y) cableTangent(X,y,q,D,D_prime,Const,Config),X_grid',y0);
cable_length(1) = Y(end,end);

D = Const.D2;

[~,Y] = ode45(@(X,y) cableTangent(X,y,q,D,D_prime,Const,Config),X_grid',y0);
cable_length(2) = Y(end,end);

pulled_length = cable_length - cable_length_rest;

end

function dydx = cableTangent(X,y,q,D,D_prime,Const,Config)
% State: [Q(1:4); r(5:7); phi(8)], where phi is the running arc length of
% the tendon path and D is its (constant) material-frame offset.

B    = Const.B;
Xi_c = Const.B_bar*Const.Xi_c;

Phi = Base_Phi(X,0,Const,Config)';
Xi  = B*Phi*q + Xi_c;

K     = Xi(1:3);
Gamma = Xi(4:6);

Q      = y(1:4);
Q_norm = Q/norm(Q);
R      = quaternion_to_matrice(Q_norm);

Q_prime = quaternion_dot(Q,Xi(1:3));
r_prime = R*Gamma;
R_prime = R*hat_(K);

phi_prime      = r_prime + R_prime*D + R*D_prime;
norm_phi_prime = norm(phi_prime);

dydx = [Q_prime;
        r_prime;
        norm_phi_prime];

end
