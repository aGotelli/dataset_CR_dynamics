function exp_SO3 = Exp_SO3_(Theta)

[alpha,beta,~] = alpha_beta_ceil_braket_(Theta,Theta);

exp_SO3 = eye(3,3) + alpha*hat_(Theta) + (beta/2)*hat_(Theta)*hat_(Theta);
