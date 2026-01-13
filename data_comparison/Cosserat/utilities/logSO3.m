function Theta = logSO3(R)


theta = acos(0.5*(trace(R) - 1));

if abs(theta) <= 1e-5
    Theta = zeros(3,1);
else
    Theta = (theta/(2*sin(theta)))*anti_hat(R-R');
end
