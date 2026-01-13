function simulation_results = generateGroundTruth(q, dot_q, ddot_q, Const,Config)

%   initialize the stack
[simulation_results, it_t, time] = initializeStack(q, dot_q, ddot_q, Config, Const);

simulation_results.Config = Config;
simulation_results.Const = Const;

%%   Time loop
it_t_init = it_t + 1;
it_t_end = Config.t_end/Config.dt + 1;

for it_t=it_t_init:it_t_end
    display(time)

    %   Compute prediction
    [q_k, dot_q_k, ddot_q_k] = prediction(q, dot_q, ddot_q, Config);

    %   Compute residual
    Residual = getResidual(time, q_k, dot_q_k, ddot_q_k, Const, Config);

    %   Iterate to cancel residual norm
    iter = 0;
    while norm(Residual) > Config.r_min      
        
        %   Compute Jacobian
        J = getJacobian(time, q_k, dot_q_k, ddot_q_k, Const, Config);
           
        %   Compute update
        Delta_q_k = - J\Residual;
                
        %   Correction
        [q_kp1, dot_q_kp1, ddot_q_kp1] = correction(q_k, dot_q_k, ddot_q_k, Delta_q_k, Config);

        %   Compute residual
        Residual = getResidual(time, q_kp1, dot_q_kp1, ddot_q_kp1, Const, Config);

        %   Update values
        q_k      = q_kp1;
        dot_q_k  = dot_q_kp1;
        ddot_q_k = ddot_q_kp1;

        iter = iter +1;
    end

    display(iter)

    %   Update values
    q      = q_k;
    dot_q  = dot_q_k;
    ddot_q = ddot_q_k;
       
    %%  Save results into stacks 
    simulation_results = addToStack(it_t, q, dot_q, ddot_q, time, simulation_results, Config, Const);


    %%  Time update
    time = time + Config.dt;
end




end