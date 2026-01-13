function   dy = elastic_gain(X,y,t,Const,Config)

% calcul la première ode spaciale 
% Integration spaciale de 0->1
% entrées
Krr   = y(1:Const.dim_base*Const.dim_base); 
Drr   = y(1+Const.dim_base*Const.dim_base:2*Const.dim_base*Const.dim_base); 
% sorties
% dy(1:3,1)  = Krr_prime; 
% dy(4:6,1)  = Drr_prime;

B = Const.B;

% H = diag([Gj*I1,E*I2,E*I3,E*Aire,Gj*Aire,Gj*Aire]);
% D = diag([mu*I1,mu*I2,mu*I3,mu*Aire,mu*Aire,mu*Aire]);

H = diag([Const.GI,Const.EI,Const.EI,Const.EA,Const.GA,Const.GA]);
D = Const.mu*H;


Ha  = B'*H*B;
Da  = B'*D*B;

Phi = Base_Phi(X,t,Const,Config);

Krr_prime = Phi*Ha*Phi'; 
Drr_prime = Phi*Da*Phi';

Krr_prime_vec = reshape(Krr_prime,[Const.dim_base*Const.dim_base, 1]);
Drr_prime_vec = reshape(Drr_prime,[Const.dim_base*Const.dim_base, 1]);


% sorties
dy  = [Krr_prime_vec; Drr_prime_vec];