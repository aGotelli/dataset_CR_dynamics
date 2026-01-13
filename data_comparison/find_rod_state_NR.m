function positions_simu = find_rod_state_NR(q, Const, Config)
    
    time = 0;
    dot_q = 0*q;
    ddot_q = 0*q;

    %   Compute Hooke matrix (zero entries for inactive modes)
    Const.H_cal = diag([0, Const.EI, Const.EI, 0, 0, 0]);
    [Kee, Dee] = computeGeneralisedStiffnessDampingMatrices(Const, Config);
    Const.Kee = Kee;
    Const.Dee = Dee;
    
    %   Compute Residual
    Residual = getResidual(time, q, dot_q, ddot_q, Const, Config);
    
    
    
    %   Newton-Raphson loop to find equilibrium
    while norm(Residual) > Config.r_min
        %   Compute Jacobian
        J = getJacobian(time, q, dot_q, ddot_q, Const, Config);
           
        %   Compute update
        Delta_q = J\Residual;
        q = q - Delta_q;
    
        %   Compute Residual
        Residual = getResidual(time, q, dot_q, ddot_q, Const, Config);
    end
    
    %   Once equilibrium found, compare the obtained position
    positions_simu = rod_shape(time, q, Const, Config)';

end