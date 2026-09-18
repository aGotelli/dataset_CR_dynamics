function [q_kp1, dot_q_kp1, ddot_q_kp1] = correction(q_k, dot_q_k, ddot_q_k, Delta_q_k, Config)

h = Config.dt;
Beta = Config.Beta;
Gamma = Config.Gamma;

q_kp1      = q_k      + Delta_q_k;
dot_q_kp1  = dot_q_k  + (Gamma/(Beta*h))*Delta_q_k;
ddot_q_kp1 = ddot_q_k + (1/(Beta*h^2))*Delta_q_k;

end