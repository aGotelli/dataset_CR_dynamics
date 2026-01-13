function [q,q_dot,q_dot_dot] = Correction_Limb(Config,Var,Delta_xi_bar)

% Newmark
a = Config.Integrateur.newmark.a;
b = Config.Integrateur.newmark.b;

Delta_q_k = Delta_xi_bar(7:6+length(Var.q));

q = Var.q + Delta_q_k;
q_dot = Config.simu.dyn*(Var.q_dot + a*Delta_q_k);
q_dot_dot = Config.simu.dyn*(Var.q_dot_dot + b*Delta_q_k);


% alpha Methode
h = Config.dt;
rho_inf = Config.Integrateur.alpha_methode.rho_inf;

alpha_m = (2*rho_inf-1)/(rho_inf+1);
alpha_f = (rho_inf)/(rho_inf+1);

Gamma = 1/2+alpha_f-alpha_m;
Beta = (Gamma+1/2)^2/4;

a = Gamma/(h*Beta);
b = (1-alpha_m)/(h^2*Beta*(1-alpha_f));

Delta_q_k = Delta_xi_bar(7:6+length(Var.q));

q = Var.q + Delta_q_k;
q_dot = Config.simu.dyn*(Var.q_dot + a*Delta_q_k);
q_dot_dot = Config.simu.dyn*(Var.q_dot_dot + b*Delta_q_k);
