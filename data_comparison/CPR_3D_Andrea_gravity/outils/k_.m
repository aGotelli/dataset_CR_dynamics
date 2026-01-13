function [res]=k_(beta,gamma,dt,Omega,Omega_dot);
res=(beta-gamma)/beta*Omega+dt*(2*beta-gamma)/(2*beta)*Omega_dot;
end 