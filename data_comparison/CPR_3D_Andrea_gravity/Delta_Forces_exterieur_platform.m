function Delta_Fext_p  = Delta_Forces_exterieur_platform(Config,Var)


Fext_p_materielle = [0;0;0;0;0;0];
% Fext_p_materielle = [0*2e-1*(Var.t-Config.dt);0;10e-1*(Var.t-Config.dt);0;0*1.5*(Var.t-Config.dt);0];
        if Config.simu.dyn==0
            % Fext_p_spaciale =  [0;0;0;0.3*Var.t;0;-1*Config.robot.plaform.M_platform*Config.Gamma_g];
            Fext_p_spaciale =  [
                0;
                0;
                0;
                0.3*Var.t;
                0.1*Var.t;
                -1*Config.robot.plaform.M_platform*Config.Gamma_g
            ];
        else
            Fext_p_spaciale =  [0;0;0;0*Var.t;0;-1*Config.robot.plaform.M_platform*Config.Gamma_g];        
        end
        % Fext_p_spaciale =  [0.01;0.0;0;0;0;0.1];
% Fext_p_spaciale =  [0;0;-4e-2*(Var.t-Config.dt);0;0;-Config.robot.plaform.M_platform*Config.Gamma_g];
% Fext_p_spaciale =  [0;0;0;-1*(Var.t-Config.dt);0;-Config.robot.plaform.M_platform*Config.Gamma_g];
% Fext_p_spaciale =  [0;0;0;0;-0.5*(Var.t-Config.dt);-Config.robot.plaform.M_platform*Config.Gamma_g];
Delta_Fext_p_materielle = zeros(6,1);
Delta_Fext_p_spaciale = zeros(6,1);

R = Var.g_0(1:3,1:3);
R_p_n = Var.R_p_n;

R = R_p_n*Exp_SO3_(Var.mu_p(1:3));

Delta_Fext_p = Delta_Fext_p_materielle + [R',zeros(3,3);zeros(3,3),R']*Delta_Fext_p_spaciale +  [hat_(Var.Delta_zeta_k)'*R',zeros(3,3);zeros(3,3),hat_(Var.Delta_zeta_k)'*R']*Fext_p_spaciale;
