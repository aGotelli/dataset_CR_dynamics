function [Var,Config,Bar_Res] = Calcul_Residu(Config,Var)


% ---------------------- Res_Gamma_bar, J et Fpi Fi_p -------------------------ù
Bar_Res_Gamma = [];

for it_limb = 1 :Config.robot.nl
    [gpi] = GM_Platform(it_limb,Config,Var);
    Config.robot.plaform.gpi{it_limb} = gpi;
    Config.robot.plaform.Rpi{it_limb} = gpi(1:3,1:3);
    Config.robot.plaform.rpi{it_limb} = gpi(1:3,4);
    
    [g1,g2,g3,Q2X,r2X] = GM_Limb(it_limb,Config,Var);
    Config.robot.limb.gi_p{it_limb} = g3;
    Config.robot.limb.Ri_p{it_limb} = g3(1:3,1:3);
    Config.robot.limb.ri_p{it_limb} = g3(1:3,4);

    
    bar_Ai = Config.robot.limb.bar_Api{it_limb};

    Bar_Res_Gamma_i{it_limb} = -bar_Ai*[Log_SO3_(Config.robot.plaform.Rpi{it_limb}'*Config.robot.limb.Ri_p{it_limb});
                              Config.robot.plaform.Rpi{it_limb}'*(Config.robot.plaform.rpi{it_limb}-Config.robot.limb.ri_p{it_limb})];
    % Res_Gamma_bar{it_limb}
    Bar_Res_Gamma = [Bar_Res_Gamma;Bar_Res_Gamma_i{it_limb}];
    %Calcul de Ji+
    Theta_i = Log_SO3_(Config.robot.plaform.Rpi{it_limb}'*Config.robot.limb.Ri_p{it_limb});
    Ri_p_pi = Config.robot.plaform.Rpi{it_limb}'*Config.robot.limb.Ri_p{it_limb};

    Var.Ji_p_T{it_limb} = [inv(T_SO3_(Theta_i))',zeros(3,3);zeros(3,3),Ri_p_pi']*bar_Ai';

    % Calcul de Fi+

    Var.Fi_p{it_limb} =  Var.Ji_p_T{it_limb}*Var.lambda{it_limb};

    % Calcul de Fpi

    gpi_i_p = inv(Config.robot.limb.gi_p{it_limb})*Config.robot.plaform.gpi{it_limb};
    Var.Fpi{it_limb} = -Ad_g_(gpi_i_p(1:3,1:3),0*gpi_i_p(1:3,4))'*Var.Ji_p_T{it_limb}*Var.lambda{it_limb};
end


[Var,Bar_Res_p] = IDM_bar_Platform(Config,Var);
Bar_Res_r = [];
Bar_Res_e = [];

for it_limb = 1 :Config.robot.nl
    
    [Var,F,Lambda,Bar_Res_r_i,Bar_Res_e_i] = IDM_bar_Limb(Config,Var,it_limb);

    Bar_Res_r = [Bar_Res_r;Bar_Res_r_i];
    Bar_Res_e = [Bar_Res_e;Bar_Res_e_i];

end
Bar_Res_q = [Bar_Res_r;Bar_Res_e];

Bar_Res = [Bar_Res_p;Bar_Res_q;Bar_Res_Gamma];