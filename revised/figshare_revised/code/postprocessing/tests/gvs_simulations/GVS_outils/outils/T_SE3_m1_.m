function TSE3_m1 = T_SE3_m1_(Theta,D)
% T_SE3_m1_  Inverse of the SE(3) (left) tangent operator at [Theta;D],
%            i.e. the matrix mapping a twist-coordinate rate to the
%            corresponding body twist. Falls back to the small-angle
%            approximation for Theta near zero, where the closed form is
%            a 0/0 indeterminacy.

[alpha,beta,ceil_braket] = alpha_beta_ceil_braket_(Theta,D);

if abs(norm(Theta)) <= 1e-5
    Tic = (1/2)*hat_(D) + zeros(3,3);
else
    Tic = (1/2)*hat_(D) + ((beta-alpha)/(beta*norm(Theta)^2))*ceil_braket + ...
        ((1+alpha-2*beta)/(beta*norm(Theta)^4))*(Theta'*D)*hat_(Theta)*hat_(Theta);
end

T_SO3_m1 = T_SO3_m1_(Theta);

TSE3_m1 = [T_SO3_m1, zeros(3,3);
           Tic,      T_SO3_m1];

end
