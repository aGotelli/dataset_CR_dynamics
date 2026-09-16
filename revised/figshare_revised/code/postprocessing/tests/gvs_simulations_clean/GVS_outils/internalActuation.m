function [Qad,L1,L2] = internalActuation(tau,Const,Config)
% internalActuation  Generalised actuation force produced by the two
%                     tendons, projected onto the actuated modal
%                     coordinates through the virtual-work moment arms
%                     L1, L2 (one per tendon).
%
% tau : [tau1;tau2] tendon tensions
%
% L1, L2 are obtained by integrating the tendon's strain-map contribution
% along the rod (actuationOdes), then scaled by the corresponding tension
% and summed to give the net generalised actuation force Qad.

L0 = zeros(Const.dim_base,1);
q  = Const.q;

N_nodes = 30;
[~,X_grid] = cheb(N_nodes-1,Config.L);

D_prime = zeros(3,1);

D = Const.D1;
[~,L] = ode45(@(X,y) actuationOdes(X,y,q,D,D_prime,Const,Config),X_grid',L0);
L1 = L(end,:)';
Qad_1 = L1*tau(1);

D = Const.D2;
[~,L] = ode45(@(X,y) actuationOdes(X,y,q,D,D_prime,Const,Config),X_grid',L0);
L2 = L(end,:)';
Qad_2 = L2*tau(2);

Qad = Qad_1 + Qad_2;

end

function L_prime = actuationOdes(X,~,q,D,D_prime,Const,Config)
% Spatial derivative of the tendon moment-arm integral at X, for a tendon
% routed at material offset D (constant here, so D_prime = 0).

B    = Const.B;
Xi_c = Const.B_bar*Const.Xi_c;

Phi = Base_Phi(X,0,Const,Config)';
Xi  = B*Phi*q + Xi_c;

K     = Xi(1:3);
Gamma = Xi(4:6);

Gamma_i = Gamma + cross(K,D) + D_prime;

strain_map = [cross(D,Gamma_i);
              Gamma_i];

Tnorm = norm(Gamma_i);

L_prime = Phi'*B'*strain_map/Tnorm;

end
