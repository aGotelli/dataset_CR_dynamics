function [F_0,F_1,F_bar,F_bar_prime] = Forces_exterieur(X,q,r,eta,Const,Config)
% Forces_exterieur  External load acting on the rod cross-section at X.
%
% F_0        : externally-applied base wrench (always zero: the base is
%              actuated through the boundary condition, not a load)
% F_1        : tip wrench Const.F1, expressed in the material frame
% F_bar      : distributed load (self-weight), zero outside the open
%              interval (0,Config.L); the tip load is applied as the
%              boundary condition F_1 instead of as a distributed term
% F_bar_prime: spatial derivative of F_bar (zero, since the cross-section
%              area, density and gravity are all spatially constant)
%
% r and eta are accepted for a uniform calling convention with other
% force laws but are not used by this one.

R = quaternion_to_matrice(q);

Aire = pi*Const.Rc^2;
Gamma_g = Const.Gamma_g;

F_0 = zeros(6,1);
F_1 = [R',zeros(3,3);zeros(3,3),R']*Const.F1;
F_bar = [zeros(3,1);R'*[0;0;-Const.rho*Aire*Gamma_g]];
F_bar_prime = zeros(6,1);

if X <= 0 || X >= Config.L
    F_bar       = zeros(6,1);
    F_bar_prime = zeros(6,1);
end

end
