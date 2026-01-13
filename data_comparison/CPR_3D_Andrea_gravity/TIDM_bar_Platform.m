function [Var,Delta_F0] = TIDM_bar_Platform(Config,Var)

% function calcul CI en function de delta_mu
[Delta_zeta_k,Delta_eta_p,Delta_eta_dot_p] = Variation_Prediction_Platform(Config,Var);


Delta_eta_p = Config.simu.dyn*Delta_eta_p;
Delta_eta_dot_p = Config.simu.dyn*Delta_eta_dot_p;

Var.Delta_zeta_k = Delta_zeta_k;

Delta_zeta_j_k = zeros(6,1);    


% Forward

for it_limb = 1 :Config.robot.nl

    gpj_k{it_limb} = [Config.robot.plaform.Rpi_p{it_limb},Config.robot.plaform.rpi_p{it_limb};0 0 0 1];
    gk_j = inv(gpj_k{it_limb});
    
    Delta_zeta_j(:,it_limb) = Ad_g_(gk_j(1:3,1:3),gk_j(1:3,4))*Delta_zeta_k+Delta_zeta_j_k;
end

% Forward

% Backward
Delta_Fp_c = zeros(6,1);

for it_limb = Config.robot.nl:-1:1
    
    gp_pi = inv(gpj_k{it_limb});
    % Delta_Fp_c = Delta_Fp_c + Ad_g_(gp_pi(1:3,1:3),gp_pi(1:3,4))'*(Var.Delta_Fpi{it_limb}-ad_(Delta_zeta_j(:,it_limb))'*Var.Fpi{it_limb});
    Delta_Fp_c = Delta_Fp_c + Ad_g_(gp_pi(1:3,1:3),gp_pi(1:3,4))'*(Var.Delta_Fpi{it_limb}-ad_(Delta_zeta_j_k)'*Var.Fpi{it_limb});
end

M_cal_p = Config.robot.plaform.M_cal_p;

eta_p = Config.simu.dyn*Var.eta_0;

Delta_Fext_p  = Delta_Forces_exterieur_platform(Config,Var) ;

Delta_F0 = Delta_Fp_c + M_cal_p*Delta_eta_dot_p - ad_(Delta_eta_p)'*M_cal_p*eta_p  - ad_(eta_p)'*M_cal_p*Delta_eta_p - Delta_Fext_p;
