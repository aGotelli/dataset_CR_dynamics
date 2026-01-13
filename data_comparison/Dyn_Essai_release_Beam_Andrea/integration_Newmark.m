function [a,b,q_dot_np1,q_dot_dot_np1] = integration_Newmark(q_np1,q_n,q_dot_n,q_dot_dot_n,h,Beta,Gamma)

a = (Gamma)/(h*Beta);
b = (Gamma)/(h^2*Beta);

q_dot_np1     =   Gamma/(h*Beta)*q_np1 -   Gamma/(h*Beta)*q_n + (1 - Gamma/Beta)*q_dot_n +  (1 - Gamma/(2*Beta))*q_dot_dot_n;
q_dot_dot_np1 = Gamma/(h^2*Beta)*q_np1 - Gamma/(h^2*Beta)*q_n -       1/(h*Beta)*q_dot_n - (1/Beta)*(1/2 - Beta)*q_dot_dot_n;
