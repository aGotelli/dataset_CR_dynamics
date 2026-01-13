function [q_0,q_0_dot,q_0_dot_dot,a_0] = Prediction_Limb(q_n,q_n_dot,q_n_dot_dot,a_n,Config)

h = Config.dt;

% Newmark
Beta = Config.Integrateur.newmark.Beta;
Gamma = Config.Integrateur.newmark.Gamma;


q_0 = q_n + Config.simu.dyn*(h*q_n_dot+ (1/2-Beta)*h^2*q_n_dot_dot);
q_0_dot = Config.simu.dyn*(q_n_dot + (1-Gamma)*h*q_n_dot_dot);
q_0_dot_dot = 0*q_n_dot_dot;

% alpha Methode
rho_inf = Config.Integrateur.alpha_methode.rho_inf;

alpha_m = (2*rho_inf-1)/(rho_inf+1);
alpha_f = (rho_inf)/(rho_inf+1);

Gamma = 1/2+alpha_f-alpha_m;
Beta = (Gamma+1/2)^2/4;


a_0 = (alpha_f*q_n_dot_dot - alpha_m*a_n)/(1-alpha_m);

q_0 = q_n + Config.simu.dyn*(h*q_n_dot+ (1/2-Beta)*h^2*a_n + (Beta*h^2)/(1-alpha_m)*(alpha_f*q_n_dot_dot - alpha_m*a_n));
q_0_dot = Config.simu.dyn*(q_n_dot + (1-Gamma)*h*a_n + Gamma*h/(1-alpha_m)*(alpha_f*q_n_dot_dot - alpha_m*a_n));
q_0_dot_dot = 0*q_n_dot_dot;

