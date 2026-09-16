function dot_Q = quaternion_dot2(Xi)
% quaternion_dot2  Linear operator A(Omega) such that dQ/dt = A(Omega)*Q
%                  (see quaternion_dot.m), returned directly as a
%                  4x4 matrix rather than applied to Q. Xi(1:3) is the
%                  angular-rate part of the strain/twist vector Xi.

Omega = Xi(1:3);

A_Omega = [       0, -Omega(1), -Omega(2), -Omega(3)
            Omega(1),         0,  Omega(3), -Omega(2)
            Omega(2), -Omega(3),         0,  Omega(1)
            Omega(3),  Omega(2), -Omega(1),        0];

dot_Q = 1/2*A_Omega;

end
