function   dy = integral_K_D(X,~,Const, Config)

B = Const.B;

H = Const.H_cal;
D = Const.mu*H;



Ha  = B'*H*B;
Da  = B'*D*B;

Phi = Base_Phi(X, 0, Const, Config)';

Krr_prime = Phi'*Ha*Phi;
Drr_prime = Phi'*Da*Phi;

Krr_prime_vec = reshape(Krr_prime,[Const.dim_base*Const.dim_base, 1]);
Drr_prime_vec = reshape(Drr_prime,[Const.dim_base*Const.dim_base, 1]);


dy  = [
    Krr_prime_vec;
    Drr_prime_vec
];

end