function exp_SE3 = Exp_SE3_(Theta,D)

exp_SO3 = Exp_SO3_(Theta);

[alpha,beta,ceil_braket] = alpha_beta_ceil_braket_(Theta,D);

if abs(norm(Theta)) <= 1e-5
    Tc = - (beta/2)*hat_(D) + zeros(3,3);
else
    Tc = - (beta/2)*hat_(D) + ((1-alpha)/norm(Theta)^2)*ceil_braket + ...
        ((Theta'*D)/norm(Theta)^2)*((beta-alpha)*hat_(Theta) + ((beta/2) - (3*(1-alpha)/norm(Theta)^2))*hat_(Theta)*hat_(Theta));
end

% exp_SE3 = [exp_SO3,Tc'*D; 0 0 0 1];
exp_SE3 = [exp_SO3,T_SO3_(Theta)'*D; 0 0 0 1];