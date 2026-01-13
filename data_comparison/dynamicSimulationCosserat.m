function simulation_results = dynamicSimulationCosserat(q_init, Const,Config)

q = q_init;
dot_q = 0*q_init;
ddot_q = 0*q_init;

%   initialize the stack
simulation_results = initializeStack(q, Config);

simulation_results.Config = Config;
simulation_results.Const = Const;

%   Make sure no tension (dynamic release)
Const.T = 0;

%%   Time loop
time = 0;
N = Config.t_end/Config.dt;
for it_t=1:N
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

    figure(1)
    plot3(simulation_results.r_stack(:, 1, it_t), simulation_results.r_stack(:, 2, it_t), simulation_results.r_stack(:, 3, it_t));
    grid on
    xlim([-.5, .5]);
    ylim([-.5, .5]);
    zlim([-.6, .0]);
    drawnow;


    %%  Time update
    time = time + Config.dt;
end




end