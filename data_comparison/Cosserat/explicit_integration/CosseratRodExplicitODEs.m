function dydx = CosseratRodExplicitODEs(t, y, Const, Config)


%   Get the right end side Qad - ( Qc + Qv + Qe + Ce )
Q_tot = getGeneralizedForcesLagrangian(t, y, Const, Config);

%   Get the mass matrix
M = getMLagrangian(t, y, Const, Config);

%   Compute the DDM
ddot_q = M\Q_tot;

%   Get dimension of the base
ne = Const.dim_base;
   
%   Get value of dot_q
dot_q = y(ne+1:end);

%   Compose derivative of state
dydx = [dot_q; ddot_q];

end