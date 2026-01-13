function Jacobian = cableLengthControl(q, Const, Config)

%   Get variation in the tension
[~, Residual] = computeErrorPulledCable(q, Const, Config);

%   Get delta
deltaT = Const.deltaT;

%   Perturb the tension
Const.T = Const.T + deltaT;

%   Get variation in the tension
[~, delta_Residual] = computeErrorPulledCable(q, Const, Config);

Jacobian = (delta_Residual - Residual)/1e-3;

end