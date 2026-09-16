function V = anti_hat_(M)
% anti_hat_  Inverse of hat_: extracts the axis vector V such that
%            hat_(V) equals the skew-symmetric part of M.

M = (M-M')/2;

V = [-M(2,3);M(1,3);-M(1,2)];

end
