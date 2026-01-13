function Delta_TSO3 = Delta_T_SO3_(Theta,Delta_Theta)

theta = norm(Theta);
U = Theta/theta;

if abs(norm(Theta)) <= 1e-5
        Delta_TSO3 = -(1/2)*hat_(Delta_Theta);
else
        Delta_TSO3 = (cos(theta)-sin(theta)/theta)*(U'*Delta_Theta/theta)*eye(3) + ...
                 (1-sin(theta)/theta)*[Delta_Theta*U'-U*Delta_Theta']+ ...
                 (3*sin(theta)/theta-cos(theta)-2)*(U'*Delta_Theta/theta)*(U*U')+...
                 ((sin(theta/2)/(theta/2))^2-sin(theta)/theta)*(U'*Delta_Theta/theta)*hat_(Theta)-...
                 (1/2)*((sin(theta/2)/(theta/2))^2)*hat_(Delta_Theta);
end



