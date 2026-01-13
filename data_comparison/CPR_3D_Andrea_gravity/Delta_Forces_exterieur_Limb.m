function [Delta_F0,Delta_F1,Delta_F_Bar] = Delta_Forces_exterieur_Limb(Config,Var,i_limb)

Fm_materielle = zeros(6,1);
Fm_spaciale = zeros(6,1);
Delta_Fm_materielle = zeros(6,1);
Delta_Fm_spaciale = zeros(6,1);

R = quaternion_to_matrice(Var.Quat_X{i_limb}(:,1));
Delta_F0 = Delta_Fm_materielle + [R',zeros(3,3);zeros(3,3),R']*Delta_Fm_spaciale +  [hat_(Var.Delta_eta_j{i_limb}{2}(:,1))'*R',zeros(3,3);zeros(3,3),hat_(Var.Delta_eta_j{i_limb}{2}(:,1))'*R']*Fm_spaciale;

Fp_materielle = zeros(6,1);
Fp_spaciale = zeros(6,1);
Delta_Fp_materielle = zeros(6,1);
Delta_Fp_spaciale = zeros(6,1);

R = quaternion_to_matrice(Var.Quat_X{i_limb}(:,end));
Delta_F1 = Delta_Fp_materielle + [R',zeros(3,3);zeros(3,3),R']*Delta_Fp_spaciale +  [hat_(Var.Delta_eta_j{i_limb}{2}(:,end))'*R',zeros(3,3);zeros(3,3),hat_(Var.Delta_eta_j{i_limb}{2}(:,end))'*R']*Fp_spaciale;

F_Bar_materielle = zeros(6,1);
F_Bar_spaciale = 0*[0;0;0;0;0;Config.Aire*Config.Gamma_g];
Delta_F_Bar_materielle = zeros(6,1);
Delta_F_Bar_spaciale = zeros(6,1);

for it_grille = 1:Config.N_noeuds
    R = quaternion_to_matrice(Var.Quat_X{i_limb}(:,it_grille));
    Delta_F_Bar(:,it_grille) = Delta_F_Bar_materielle + [R',zeros(3,3);zeros(3,3),R']*Delta_F_Bar_spaciale +  [hat_(Var.Delta_eta_j{i_limb}{2}(:,it_grille))'*R',zeros(3,3);zeros(3,3),hat_(Var.Delta_eta_j{i_limb}{2}(:,it_grille))'*R']*F_Bar_spaciale;
end

