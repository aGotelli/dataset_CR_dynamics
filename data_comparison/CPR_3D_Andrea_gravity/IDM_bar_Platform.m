function [Var,F0] = IDM_bar_Platform(Config,Var)


% gk = Var.g_0;
R_p_n = Var.R_p_n;
r_p_n = Var.r_p_n;

R_0 = R_p_n*Exp_SO3_(Var.mu_p(1:3));
r_0 = r_p_n + Var.mu_p(4:6);

gk = [R_0,r_0;0 0 0 1];

% Forward

for it_limb = 1 :Config.robot.nl
    
    gpj_k{it_limb} = [Config.robot.plaform.Rpi_p{it_limb},Config.robot.plaform.rpi_p{it_limb};0 0 0 1];
    gj{it_limb} = gk*gpj_k{it_limb};

end

% Backward
Fp_c = zeros(6,1);
for it_limb = Config.robot.nl:-1:1
    
    gp_pi = inv(gpj_k{it_limb});
    Fp_c = Fp_c + Ad_g_(gp_pi(1:3,1:3),gp_pi(1:3,4))'*Var.Fpi{it_limb};
end

M_cal_p = Config.robot.plaform.M_cal_p;

eta_p = Config.simu.dyn*Var.eta_0;
eta_dot_p = Config.simu.dyn*Var.eta_0_dot;

Fext_p = Forces_exterieur_platform(Config,Var);

F0 = Fp_c + M_cal_p*eta_dot_p - ad_(eta_p)'*M_cal_p*eta_p - Fext_p;
