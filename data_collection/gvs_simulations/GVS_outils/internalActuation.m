function [Qad, L1, L2] = internalActuation(tau, Const, Config)


L0 = zeros(Const.dim_base, 1);

q = Const.q;


% Define nodes
N_nodes = 30; 

[~,X_grid]=cheb(N_nodes-1,Config.L); 


D_prime = zeros(3, 1);

D = Const.D1;
[~, L] = ode45(@(X, y) actuationOdes(X, y, q, D, D_prime, Const, Config), X_grid', L0);
L1 = L(end,:)';

Qad_1 = L1*tau(1);

D = Const.D2;
[~, L] = ode45(@(X, y) actuationOdes(X, y, q, D, D_prime, Const, Config), X_grid', L0);
L2 = L(end,:)';

Qad_2 = L2*tau(2);

Qad = Qad_1 + Qad_2;


end



function L_prime = actuationOdes(X, ~, q, D, D_prime, Const, Config)




    %   Obtain the need variables
    B     = Const.B;
    Xi_c  = Const.B_bar*Const.Xi_c;

    %   Compute strains
    Phi = Base_Phi(X, 0, Const, Config)';
    % Phi = getPhi(X, Const);
    Xi      = B*Phi*q + Xi_c;
    K     = Xi(1:3);
    Gamma = Xi(4:6);


    %   Compute the tension
    Gamma_i = Gamma + cross(K, D) + D_prime;

    %   Get strain map
    strain_map = [ cross(D, Gamma_i); 
                      Gamma_i   ];

 

    %   The norm
    Tnorm = norm(Gamma_i);

    L_prime = Phi'*B'*strain_map/ Tnorm;


end