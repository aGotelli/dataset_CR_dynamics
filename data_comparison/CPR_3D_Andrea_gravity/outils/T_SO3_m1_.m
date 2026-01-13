function TSO3_m1 = T_SO3_m1_(Theta)

[alpha,beta,~] = alpha_beta_ceil_braket_(Theta,Theta);

if abs(norm(Theta)) <= 1e-5
    TSO3_m1 = eye(3);
else
    TSO3_m1 = eye(3) + (1/2)*hat_(Theta) + (1/norm(Theta)^2)*(1-(alpha/beta))*hat_(Theta)*hat_(Theta);
end


