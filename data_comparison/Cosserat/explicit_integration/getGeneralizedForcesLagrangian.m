function Q_tot = getGeneralizedForcesLagrangian(t, y, Const, Config)


%   Get dimension of the base
ne = Const.dim_base;

q     = y(1:ne);
dot_q = y(ne+1:end);
zeroq = zeros(ne, 1);

%   Compute with only q
[~, Qa_X0] = IDM(t, q, zeroq, zeroq, Config, Const);

%   Get configuration dependent efforts
Qc = Qa_X0;


%   Compute with veclocities
[~, Qa_X0] = IDM(t, q, dot_q, zeroq, Config, Const);

%   Get Coriolis and centrifugal efforts
Qv = Qa_X0 - Qc;


%   Now compute the forces from internal stiffness and damping
Qe = Const.Kee*q;
Ce = Const.Dee*dot_q;


%   Internal actuation
Qad = zeros(ne, 1);

Q_tot = Qad - ( Qc + Qv + Qe + Ce );
   
end