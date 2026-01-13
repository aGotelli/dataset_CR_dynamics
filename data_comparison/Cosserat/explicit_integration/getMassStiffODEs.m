function M_ode = getMassStiffODEs(t, y, Const, Config)

M = getMLagrangian(t, y, Const, Config);

%   Get dimension of the base
ne = Const.dim_base;

M_ode = eye(2*ne, 2*ne);

M_ode(ne+1:end, ne+1:end) = M;


end