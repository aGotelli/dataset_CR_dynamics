function [Kee,Dee] = computeGeneralisedStiffnessDampingMatrices(Const,Config)
% computeGeneralisedStiffnessDampingMatrices  Generalised (modal)
%   stiffness and damping matrices, obtained by integrating the
%   cross-sectional stiffness/damping projected onto the modal basis
%   Phi over the rod length.

Kee_0 = zeros(Const.dim_base*Const.dim_base,1);
Dee_0 = zeros(Const.dim_base*Const.dim_base,1);

initial_conditions = [Kee_0;
                       Dee_0];
option = odeset('RelTol',1e-8,'AbsTol',1e-8);

[~,Y1] = ode45(@(X,y) integral_K_D(X,y,Const,Config),[0,Config.L],initial_conditions,option);

Kee_vec = Y1(end,1:Const.dim_base*Const.dim_base);
Dee_vec = Y1(end,1+Const.dim_base*Const.dim_base:2*Const.dim_base*Const.dim_base);

Kee = reshape(Kee_vec,[Const.dim_base Const.dim_base]);
Dee = reshape(Dee_vec,[Const.dim_base Const.dim_base]);

end
