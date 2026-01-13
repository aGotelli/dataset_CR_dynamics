function eta_reconstructed = reconstruction(observed_Q_n, observed_Q_np1, observed_r_n, observed_r_np1, observed_eta_n, observed_deta_n, Config)

observation_points = length(Config.backward_integration_domain);

eta_reconstructed = zeros(observation_points, 6);

a = Config.a;

for it=1:observation_points

    %   Extract quaternions
    Q_n   = observed_Q_n(it, :)';
    Q_np1 = observed_Q_np1(it, :)';

    %   Rotation matrices
    R_n   = getR(Q_n);
    R_np1 = getR(Q_np1);

    %   Extract positions
    r_n   = observed_r_n(it, :)';
    r_np1 = observed_r_np1(it, :)';
    
    %   Compute rotations/translations
    Theta_np1 = logSO3(R_n'*R_np1); 
    d_np1 = r_np1 - r_n;

    %   Get velocities (angular/linear)
    Omega_n = observed_eta_n(it, 1:3)';
    V_n     = observed_eta_n(it, 4:6)';

    %   Get velocities (angular/linear)
    dot_Omega_n = observed_deta_n(it, 1:3)';
    dot_V_n     = observed_deta_n(it, 4:6)';

    %   Project in world frame
    v_n = R_n*V_n;
    dot_v_n = R_n*(dot_V_n + hat(Omega_n)*V_n);


    
    eta_np1 = [  a*Theta_np1 + fn(Omega_n, dot_Omega_n, Config)
                R_np1'*(a*d_np1 + fn(v_n, dot_v_n, Config))
             ];
    
        
    eta_reconstructed(it, :) = eta_np1';

end






end 


function res = fn(dx, ddx, Config)
gamma = Config.Gamma;
beta  = Config.Beta;

dt = Config.dt;

res = (1.0 - gamma/beta)*dx + dt*(1.0 - gamma/(2*beta))*ddx;
end