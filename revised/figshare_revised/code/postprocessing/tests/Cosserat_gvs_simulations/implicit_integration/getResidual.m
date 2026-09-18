function R = getResidual(t, q, dot_q, ddot_q, Const, Config)


[~, Qa_X0] = IDM(t, q, dot_q, ddot_q, Config, Const);

Qe = Const.Kee*q;
Ce = Const.Dee*dot_q;

%   Actuation
Q_ad = internalActuation(q, Const, Config);

R = Qa_X0 + Qe + Ce - Q_ad;

end