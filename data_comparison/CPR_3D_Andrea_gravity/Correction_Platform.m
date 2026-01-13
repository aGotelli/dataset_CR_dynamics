function [g_0,eta_0,eta_0_dot,mu] = Correction_Platform(Config,Var,Delta_xi_bar)

% %%%%%%% SO(3)xR^3 %%%%%%%%

a = Config.Integrateur.newmark.a;
b = Config.Integrateur.newmark.b;

Delta_nu_k = Delta_xi_bar(1:6);
Theta_k = Var.mu_p(1:3);


R_p_n = Var.R_p_n;
r_p_n = Var.r_p_n;

R_k = R_p_n*Exp_SO3_(Var.mu_p(1:3));
r_k = r_p_n + Var.mu_p(4:6);

% R_k = Var.g_0(1:3,1:3);
% r_k = Var.g_0(1:3,4);
R_n = Var.R_p_n;

v_k = R_k * Var.eta_0(4:6);
v_k_dot = R_k * (Var.eta_0_dot(4:6)+hat_(Var.eta_0(1:3))*Var.eta_0(4:6));

% Tetha_k = Log_SO3_(R_n'*R_k);

% Theta_0=T_SO3_(Tetha_k)*Delta_nu_k(1:3);
% R_0 = R_k*Exp_SO3_(Theta_0);

Theta_0=Theta_k + Delta_nu_k(1:3);
R_0 = R_n*Exp_SO3_(Theta_0);
    
Omega_k = Var.eta_0(1:3);
Omega_k_dot = Var.eta_0_dot(1:3);

Omega_0 = Omega_k + a*Delta_nu_k(1:3);
Omega_0_dot = Omega_k_dot + b*Delta_nu_k(1:3);


r_0 = r_k + Delta_nu_k(4:6);
v_0 = v_k +  a*Delta_nu_k(4:6);
v_0_dot = v_k_dot +  b*Delta_nu_k(4:6);
    
   
V_0 = R_0'*v_0;
V_0_dot = R_0'*v_0_dot - hat_(Omega_0)*V_0;

g_0 = [R_0,r_0;0 0 0 1];
eta_0 = Config.simu.dyn*[Omega_0;V_0];
eta_0_dot = Config.simu.dyn*[Omega_0_dot;V_0_dot];

mu = [Theta_0;Var.mu_p(4:6) + Delta_nu_k(4:6)];
