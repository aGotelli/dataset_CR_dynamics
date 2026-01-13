function R = getResidual(t, q, dot_q, ddot_q, Const, Config)


[~, Qa_X0, ~] = IDM(t, q, dot_q, ddot_q, Config, Const);

Qe = Const.Kee*q;
Ce = Const.Dee*dot_q;

% tau = Const.cable_tension(t);
tau = Const.T;
Qad = internalActuation(q, tau, Const, Config);

R = Qad + Qe + Ce - Qa_X0;

end