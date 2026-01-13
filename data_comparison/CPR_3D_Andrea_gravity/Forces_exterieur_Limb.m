function [F0,F1,F_Bar] = Forces_exterieur_Limb(Config,Var,i_limb)

Fm_materielle = zeros(6,1);
Fm_spaciale = zeros(6,1);

R = quaternion_to_matrice(Var.Quat_X{i_limb}(:,1));
F0 = Fm_materielle + [R',zeros(3,3);zeros(3,3),R']*Fm_spaciale;

Fp_materielle = zeros(6,1);
Fp_spaciale = zeros(6,1);

R = quaternion_to_matrice(Var.Quat_X{i_limb}(:,end));
F1 = Fp_materielle + [R',zeros(3,3);zeros(3,3),R']*Fp_spaciale;

F_Bar_materielle = zeros(6,1);
F_Bar_spaciale = 0*[0;0;0;0;0;Config.Aire*Config.Gamma_g];

for it_grille = 1:Config.N_noeuds
    R = quaternion_to_matrice(Var.Quat_X{i_limb}(:,it_grille));
    F_Bar(:,it_grille) = F_Bar_materielle + [R',zeros(3,3);zeros(3,3),R']*F_Bar_spaciale;
end

