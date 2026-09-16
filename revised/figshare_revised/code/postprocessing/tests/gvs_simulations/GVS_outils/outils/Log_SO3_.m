function log_SO3 = Log_SO3_(R)
% Log_SO3_  SO(3) logarithm of a rotation matrix R, returning the
%           rotation vector Theta (axis * angle). Falls back to zero for
%           angles near zero, where the closed form is a 0/0
%           indeterminacy.

theta = acos(0.5*(trace(R)-1));

if abs(theta) <= 1e-5
    log_SO3 = zeros(3,1);
else
    log_SO3 = (theta/(2*sin(theta)))*anti_hat_(R-R');
end

end
