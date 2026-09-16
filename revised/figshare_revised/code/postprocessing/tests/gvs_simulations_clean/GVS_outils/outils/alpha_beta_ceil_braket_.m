function [alpha,beta,ceil_braket] = alpha_beta_ceil_braket_(U,V)
% alpha_beta_ceil_braket_  Scalar coefficients alpha, beta of the SO(3)
%   exponential/tangent-operator series expansions, and the symmetrised
%   bracket ceil_braket = hat_(U)*hat_(V) + hat_(V)*hat_(U), used by
%   T_SO3_m1_ and T_SE3_m1_. The small-angle branch uses the Taylor
%   series of alpha = sin(x)/x and beta = (2-2cos(x))/x^2 to avoid the
%   0/0 indeterminacy as norm(U) -> 0.

norm_U = norm(U);

if abs(norm_U) <= 1e-5
    alpha = 1 - (norm_U^2/6);
    beta  = 1 - (norm_U^2/12);
else
    alpha = sin(norm_U)/norm_U;
    beta  = (2-2*cos(norm_U))/norm_U^2;
end

ceil_braket = hat_(U)*hat_(V) + hat_(V)*hat_(U);

end
