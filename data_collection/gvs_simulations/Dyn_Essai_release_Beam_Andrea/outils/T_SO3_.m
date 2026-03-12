function TSO3 = T_SO3_(Theta)

[alpha,beta,~] = alpha_beta_ceil_braket_(Theta,Theta);

if abs(norm(Theta)) <= 1e-5
    TSO3 = eye(3,3) - (1/2)*hat_(Theta) + (1/6)*hat_(Theta)*hat_(Theta);
else
    TSO3 = eye(3,3) - (beta/2)*hat_(Theta) + ((1-alpha)/norm(Theta)^2)*hat_(Theta)*hat_(Theta);
end



