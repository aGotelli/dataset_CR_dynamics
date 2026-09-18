function Qad = internalActuation(q, Const, Config)


tau = Const.tau;

L0 = zeros(Const.dim_base, 1);
D_prime = zeros(3,1);

D = Const.D1;
[~, L] = ode45(@(X, y) actuationOdes(X, y, q, D, D_prime,Const), Config.forward_integration_domain, L0);
L1 = L(end,:)';
Qad_1 = L1*tau(1);


D = Const.D2;
[~, L] = ode45(@(X, y) actuationOdes(X, y, q, D, D_prime,Const), Config.forward_integration_domain, L0);
L2 = L(end,:)';
Qad_2 = L2*tau(2);


Qad = Qad_1 + Qad_2;


end


function L_prime = actuationOdes(X, ~, q, D, D_prime, Const)


    %   Obtain the need variables
    B     = Const.B;
    Xi_c  = Const.Xi_c;
    l     = Const.L;

    %   Compute strains
    Phi = getPhi(X, Const);
    Xi    = B*Phi*q + Xi_c;
    K     = Xi(1:3);
    Gamma = Xi(4:6);

    %   Compute the tension
    Gamma_i = Gamma + cross(K, D) + D_prime;

    %   Get strain map
    strain_map = [ cross(D, Gamma_i); 
                      Gamma_i   ];

    %   Compute derivative of internal stress
    L_prime = l*Phi'*B'*strain_map/ norm(Gamma_i);

end