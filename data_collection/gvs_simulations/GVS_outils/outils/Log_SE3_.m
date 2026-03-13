function log_SE3 = Log_SE3_(R,r)

Theta   = Log_SO3_(R);

TSO3_m1 = T_SO3_m1_(Theta);

log_SE3 = [Theta;TSO3_m1'*r];
