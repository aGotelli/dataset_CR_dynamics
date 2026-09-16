function dq = quaternion_dot(Q,Omega)
% quaternion_dot  Time (or spatial) derivative of a unit quaternion Q
%                 under the angular rate Omega, dQ/dt = (1/2)*A(Omega)*Q.

norm_Q = sqrt(Q(1)^2+Q(2)^2+Q(3)^2+Q(4)^2);
Q(1) = Q(1)/norm_Q;
Q(2) = Q(2)/norm_Q;
Q(3) = Q(3)/norm_Q;
Q(4) = Q(4)/norm_Q;

A_Omega = [       0, -Omega(1), -Omega(2), -Omega(3)
            Omega(1),         0,  Omega(3), -Omega(2)
            Omega(2), -Omega(3),         0,  Omega(1)
            Omega(3),  Omega(2), -Omega(1),        0];

dq = 1/2*A_Omega*Q;

end
