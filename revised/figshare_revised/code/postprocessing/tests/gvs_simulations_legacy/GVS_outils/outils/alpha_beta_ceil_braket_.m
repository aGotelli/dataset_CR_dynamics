function [alpha,beta,ceil_braket] = alpha_beta_ceil_braket_(U,V)

norm_U = norm(U);


if abs(norm_U) <= 1e-5
    alpha = 1-(norm_U^2/6);
    beta = (1-norm_U^2/12);
else
    alpha = sin(norm_U)/norm_U;
    beta = (2-2*cos(norm_U))/norm_U^2;
end


ceil_braket = hat_(U)*hat_(V) + hat_(V)*hat_(U);
