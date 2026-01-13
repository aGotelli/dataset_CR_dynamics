function [Lambda_X0, R] = getResidualWithStateEstimation(t, q, dot_q, ddot_q, Const, Config, Measures)

[Lambda_X0, Qa_X0, overline_Qa, ~] = IDMWithStateEstimation(t, q, dot_q, ddot_q, Config, Const, Measures);



Qe = Const.Kee*q;
Ce = Const.Dee*dot_q;

tau = Const.cable_tension(t);
Qad = internalActuation(q, tau, Const, Config);

R = Qad + Qe + Ce - Qa_X0 - overline_Qa;

end


