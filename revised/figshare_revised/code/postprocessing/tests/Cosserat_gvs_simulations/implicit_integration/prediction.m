function [q_np1,dot_q_np1, ddot_q_np1] = prediction(q_n, dot_q_n, ddot_q_n, Config)

h = Config.dt;
Beta = Config.Beta;
Gamma = Config.Gamma;

q_np1      = q_n + h*dot_q_n + (1/2 - Beta)*h^2*ddot_q_n;    
dot_q_np1  = dot_q_n         + ( 1 - Gamma)*h*ddot_q_n;  
ddot_q_np1 = 0*ddot_q_n;


end