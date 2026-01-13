function   dy = Delta_Psy(X,y,t,Const,Config)


%-------------- entrées -----------------%
delta_psi = y(1:6);

%-------------- forçage -----------------%

Phi = Base_Phi(X,t,Const,Config);

Xi_a = Phi'*Const.q;
Xi_dot_a = Phi'*Const.q_dot;

Xi = Const.B*Xi_a +  Const.B_bar*Const.Xi_c + Const.B*Const.Xi_0;
Xi_dot = Const.B*Xi_dot_a;

%-------------- calculs -----------------%

delta_psi_prime = -ad_(Xi)*delta_psi + Xi_dot;

%-------------- sorties -----------------%

dy(1:6,1) = delta_psi_prime; 


