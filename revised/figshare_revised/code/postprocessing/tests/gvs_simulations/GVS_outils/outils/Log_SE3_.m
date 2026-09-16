function log_SE3 = Log_SE3_(R,r)
% Log_SE3_  SE(3) logarithm of the rigid transform g = [R,r;0,1],
%           returning the twist coordinates [Theta;V] (Theta: SO(3) log
%           of R, V: translational part mapped through the inverse
%           SO(3) tangent operator).

Theta = Log_SO3_(R);

TSO3_m1 = T_SO3_m1_(Theta);

log_SE3 = [Theta;TSO3_m1'*r];

end
