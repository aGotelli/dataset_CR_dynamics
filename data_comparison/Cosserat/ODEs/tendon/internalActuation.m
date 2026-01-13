function Qad = internalActuation(q, tau, Const, Config)


L0 = zeros(Const.dim_base, 1);


[~, L] = ode45(@(X, y) actuationOdes(X, y, q, Const), Config.forward_integration_domain, L0);

L = L(end,:)';

Qad = L*tau;


end



function L_prime = actuationOdes(X, ~, q, Const)

    %   Position of the tendon
    D = Const.D;
    D_prime = Const.D_prime;


    %   Obtain the need variables
    B     = Const.B;
    Xi_c  = Const.Xi_c;
    l     = Const.L;

    %   Compute strains
    Phi = getPhi(X, Const);
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

    L_prime = l*Phi'*B'*strain_map/ Tnorm;


end