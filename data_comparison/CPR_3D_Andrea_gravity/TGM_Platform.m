function [Delta_zeta_pi] =  TGM_Platform(i_limb,Config,Var)

[Delta_zeta_k,~,~] = Variation_Prediction_Platform(Config,Var);

Delta_zeta_j_k = zeros(6,1);

gj_k = [Config.robot.plaform.Rpi_p{i_limb},Config.robot.plaform.rpi_p{i_limb};0 0 0 1];

gk_j = inv(gj_k);

Delta_zeta_pi = Ad_g_(gk_j(1:3,1:3),gk_j(1:3,4))*Delta_zeta_k+Delta_zeta_j_k;