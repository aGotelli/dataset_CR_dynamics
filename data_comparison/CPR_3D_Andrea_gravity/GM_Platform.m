function   [gpi] = GM_Platform(i_limb,Config,Var)
%%
% entrées

% gp = Var.g_0;
R_p_n = Var.R_p_n;
r_p_n = Var.r_p_n;

R_0 = R_p_n*Exp_SO3_(Var.mu_p(1:3));
r_0 = r_p_n + Var.mu_p(4:6);

gp = [R_0,r_0;0 0 0 1];


gpi_p = [Config.robot.plaform.Rpi_p{i_limb},Config.robot.plaform.rpi_p{i_limb};0 0 0 1];

gpi = gp *gpi_p;
