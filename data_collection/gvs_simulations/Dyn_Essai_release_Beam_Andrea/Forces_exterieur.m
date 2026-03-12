function [F_0,F_1,F_bar,F_bar_prime] = Forces_exterieur(X,q,r,eta,Const,Config)


R = quaternion_to_matrice(q);
g = [R,r;0 0 0 1];

Aire = pi*Const.Rc^2;
Gamma_g = Const.Gamma_g;
t = 0;


F_0 = zeros(6,1);
F_1 = [R',zeros(3,3);zeros(3,3),R']*Const.F1;
F_bar = [zeros(3,1);R'*[0;   0; -Const.rho*Aire*Gamma_g]];
% F_bar = [zeros(3,1);R'*[0;   0; 0]];
F_bar_prime = zeros(6,1);

if X <= 0
    F_bar       = zeros(6,1);
    F_bar_prime = zeros(6,1);
end
if X >= Config.L
    F_bar       = zeros(6,1);
    F_bar_prime = zeros(6,1);
end
