function [Kee, Dee] = computeGeneralisedStiffnessDampingMatrices(Const, Config)

Kee_0 = zeros(Const.dim_base*Const.dim_base,1);
Dee_0 = zeros(Const.dim_base*Const.dim_base,1);

initial_conditions = [Kee_0; 
                       Dee_0];
option=odeset('RelTol',10^(-8),'AbsTol',10^(-8));

%   Integrate Kee and Dee
[~, Y1] = ode45(@(X,y) elastic_gain(X,y,Const),...
                Config.forward_integration_domain, initial_conditions, option);

% ------- Affectation des résultats -----------%
Kee_vec = Y1(end,1:Const.dim_base*Const.dim_base);
Dee_vec = Y1(end,1+Const.dim_base*Const.dim_base:2*Const.dim_base*Const.dim_base);

Kee = reshape(Kee_vec, [Const.dim_base Const.dim_base]);
Dee = reshape(Dee_vec, [Const.dim_base Const.dim_base]);


end