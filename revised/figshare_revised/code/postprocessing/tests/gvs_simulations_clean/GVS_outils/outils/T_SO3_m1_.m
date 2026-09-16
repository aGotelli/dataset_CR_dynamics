function TSO3_m1 = T_SO3_m1_(Theta)
% T_SO3_m1_  Inverse of the SO(3) (left) tangent operator at Theta, i.e.
%            the matrix mapping a rotation-vector rate to the
%            corresponding body angular velocity. Falls back to the
%            identity for small Theta, where the closed form is a 0/0
%            indeterminacy.

[alpha,beta,~] = alpha_beta_ceil_braket_(Theta,Theta);

if abs(norm(Theta)) <= 1e-5
    TSO3_m1 = eye(3);
else
    TSO3_m1 = eye(3) + (1/2)*hat_(Theta) + (1/norm(Theta)^2)*(1-(alpha/beta))*hat_(Theta)*hat_(Theta);
end

end
