function J = Calcul_Jacobian(Config,Var)

for it_J = 1 :Var.size.mu_p+Var.size.q+Var.size.lambda
    
    % Affectation de Delta_Xi

    Delta_Xi = zeros(Var.size.mu_p+Var.size.q+Var.size.lambda,1);
    Delta_Xi(it_J) = 1;

    Var.Delta_mu_p = Delta_Xi(1:6);
    Var.Delta_q = Delta_Xi(Var.size.mu_p+1:Var.size.mu_p+Var.size.q);
    
    for it_nl = 1 :Config.robot.nl
        dim_A_bar = size(Config.robot.limb.bar_Api{it_nl});
        Var.Delta_lambda{it_nl}  = Delta_Xi(Var.size.mu_p+Var.size.q+1+dim_A_bar(1)*(it_nl-1):Var.size.mu_p+Var.size.q+dim_A_bar(1)*it_nl);
    end
    

    % ---------------------- Res_Gamma_bar, J et Fpi Fi_p -------------------------ù
    Delta_Bar_Res_Gamma = [];
    
    for it_limb = 1 :Config.robot.nl
        [Delta_zeta_pi] =  TGM_Platform(it_limb,Config,Var);
        % Config.robot.plaform.Delta_zeta_pi{it_limb} = Delta_zeta_pi;
        
        [Delta_zeta_i_p] = TGM_Limb(it_limb,Config,Var);
        
        gpi_i_p = inv(Config.robot.limb.gi_p{it_limb})*Config.robot.plaform.gpi{it_limb};
    
        Delta_zeta_i_p_pi = Delta_zeta_i_p -Ad_g_(gpi_i_p(1:3,1:3),gpi_i_p(1:3,4))*Delta_zeta_pi;
    
        bar_Ai = Config.robot.limb.bar_Api{it_limb};
    
        Ri_p_pi = Config.robot.plaform.Rpi{it_limb}'*Config.robot.limb.Ri_p{it_limb};
        Theta{it_limb} = Log_SO3_(Ri_p_pi);
    
        Delta_Bar_Res_Gamma_i{it_limb} = bar_Ai*[T_SO3_m1_(Theta{it_limb}),zeros(3,3)
                                                 zeros(3,3),Ri_p_pi]*Delta_zeta_i_p_pi;
        % Delta_Res_Gamma_bar{it_limb}
        Delta_Bar_Res_Gamma = [Delta_Bar_Res_Gamma;Delta_Bar_Res_Gamma_i{it_limb}];
    
        % Delta_Calcul de Ji+

        % Delta_Ji_p_T{it_limb} = [inv(Delta_T_SO3_(Theta{it_limb},Var.Delta_mu_p(1:3)))',zeros(3,3);zeros(3,3),T_SO3_(Theta{it_limb})'*Ri_p_pi']*bar_Ai';
        Delta_Ji_p_T{it_limb} = [zeros(3,3),zeros(3,3);zeros(3,3),hat_(Delta_zeta_i_p_pi)'*Ri_p_pi']*bar_Ai';
        % Delta_Ji_p_T{it_limb} = zeros(6,6)*bar_Ai';
    
        % Calcul de Delta_Fi+
    
        Var.Delta_Fi_p{it_limb} =  Delta_Ji_p_T{it_limb}*Var.lambda{it_limb} + Var.Ji_p_T{it_limb}*Var.Delta_lambda{it_limb};
    
        % Calcul de Delta_Fpi
    
        Var.Delta_Fpi{it_limb} = -Ad_g_(gpi_i_p(1:3,1:3),0*gpi_i_p(1:3,4))'*(Var.Delta_Fi_p{it_limb} - ad_(Delta_zeta_i_p_pi)'*Var.Fi_p{it_limb});
    end
    
    
    [Var,Delta_Bar_Res_p] = TIDM_bar_Platform(Config,Var);

    Delta_Bar_Res_r = [];
    Delta_Bar_Res_e = [];

    for it_limb = 1 :Config.robot.nl

        [Var,Delta_F,Delta_Lambda,Delta_Bar_Res_r_i,Delta_Bar_Res_e_i] = TIDM_bar_Limb(Config,Var,it_limb);

        Delta_Bar_Res_r = [Delta_Bar_Res_r;Delta_Bar_Res_r_i];
        Delta_Bar_Res_e = [Delta_Bar_Res_e;Delta_Bar_Res_e_i];

    end
    Delta_Bar_Res_q = [Delta_Bar_Res_r;Delta_Bar_Res_e];

    Delta_Bar_Res = [Delta_Bar_Res_p;Delta_Bar_Res_q;Delta_Bar_Res_Gamma];

    J(:,it_J) = Delta_Bar_Res;
end