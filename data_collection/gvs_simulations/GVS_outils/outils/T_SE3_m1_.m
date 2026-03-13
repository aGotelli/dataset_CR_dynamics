function TSE3_m1 = T_SE3_m1_(Theta,D)

[alpha,beta,ceil_braket] = alpha_beta_ceil_braket_(Theta,D);

if abs(norm(Theta)) <= 1e-5
%     Tic = - (1/2)*hat_(D) + zeros(3,3);
    Tic = + (1/2)*hat_(D) + zeros(3,3);
else
%     Tic = - (1/2)*hat_(D) + ((beta-alpha)/(beta*norm(Theta)^2))*ceil_braket + ...
%         ((1+alpha-2*beta)/(beta*norm(Theta)^4))*(Theta'*D)*hat_(Theta)*hat_(Theta);
    Tic = + (1/2)*hat_(D) + ((beta-alpha)/(beta*norm(Theta)^2))*ceil_braket + ...
        ((1+alpha-2*beta)/(beta*norm(Theta)^4))*(Theta'*D)*hat_(Theta)*hat_(Theta);
end

T_SO3_m1 = T_SO3_m1_(Theta);

TSE3_m1 = [T_SO3_m1, zeros(3,3); 
           Tic  , T_SO3_m1];