function log_SO3 = Log_SO3_(R)


theta = acos(0.5*(trace(R) - 1));

if abs(theta) <= 1e-5
    log_SO3 = zeros(3,1);
else
    log_SO3 = (theta/(2*sin(theta)))*anti_hat_(R-R');
end
