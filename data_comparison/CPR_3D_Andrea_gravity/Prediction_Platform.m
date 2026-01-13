function  [g_0,eta_0,eta_0_dot,mu] = Prediction_Platform(eta_p_n,dot_eta_p_n,Config,Var)

h = Config.dt;
Beta = Config.Integrateur.newmark.Beta;
Gamma = Config.Integrateur.newmark.Gamma;
a = Config.Integrateur.newmark.a;

R_p_n = Var.R_p_n;
r_p_n = Var.r_p_n;

Omega_p_n = eta_p_n(1:3);
Omega_p_n_dot = dot_eta_p_n(1:3);

V_p_n =  eta_p_n(4:6);
V_p_n_dot =  eta_p_n(4:6);
v_p_n = R_p_n*V_p_n;
v_p_n_dot = R_p_n*(V_p_n_dot + hat_(Omega_p_n)*V_p_n);

% Prediction

Theta_0 = Config.simu.dyn *(h*Omega_p_n + (1/2-Beta)*h^2*Omega_p_n_dot);
d_0 = Config.simu.dyn *(h*v_p_n + (1/2-Beta)*h^2*v_p_n_dot);
   
Omega_0 = Omega_p_n+h*(1-Gamma)*Omega_p_n_dot;
Omega_0_dot = zeros(3,1);
    
R_0 = R_p_n*Exp_SO3_(Theta_0);
r_0 = r_p_n + d_0;

g_0 = [R_0,r_0;0 0 0 1];

v_0 = v_p_n + (1-Gamma)*h*v_p_n_dot;
v_0_dot = zeros(3,1);

V_0 = R_0'*v_0;
V_0_dot = R_0'*v_0_dot + hat_(V_0)*Omega_0;

eta_0 = Config.simu.dyn *[Omega_0;V_0];
eta_0_dot = Config.simu.dyn *[Omega_0_dot;V_0_dot];

mu = [Theta_0;d_0];

