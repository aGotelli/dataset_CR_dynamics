function [q_sol, Residual] = computeErrorPulledCable(q_init, Const, Config)


options = optimoptions("fsolve", "Display","none");

%   Use convex optimization
q_sol = fsolve(@(q) getResidual(q, Const, Config), q_init, options);

cable_length = getCableLength(q_sol, Const, Config);


pulled = cable_length - Const.cable_length_rest;
Residual = pulled + Const.u;

end