function [res]=l_(beta,dt,Omega,Omega_dot)
%res=-1/(beta*dt)*Omega+(1-2*beta)/(2*beta)*Omega_dot;
res=-1/(beta*dt)*Omega-(1-2*beta)/(2*beta)*Omega_dot;
end 