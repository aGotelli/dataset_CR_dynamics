function [eta_X0_bar, Lambda_X1_bar] = injection(eta_X1_star, Lambda_X0_star, eta_X1, Lambda_X0, Config)

Gamma_w = Config.Gamma_wrench;
Gamma_t = Config.Gamma_twist;

%   Compute twsit and wrench to inject
eta_X0_bar    =   Gamma_w*( Lambda_X0 - Lambda_X0_star );
Lambda_X1_bar = - Gamma_t*( eta_X1    -    eta_X1_star );

end