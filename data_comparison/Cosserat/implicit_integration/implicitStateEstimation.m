function simulation_results = implicitStateEstimation(ground_truth, q, dot_q, ddot_q, Const, Config)

%   initialize the stack
[simulation_results, it_t, time] = initializeStack(q, dot_q, ddot_q, Config, Const);


simulation_results.Config = Config;
simulation_results.Const = Const;

%%   Time loop
it_t_init = it_t + 1;
it_t_end = Config.t_end/Config.dt + 1;



%   Get measurements
eta_world_X1_star = ground_truth.eta_world_stack( end, :, it_t_init-1)';
Lambda_X0_star    = ground_truth.Lambda_stack(      1, :, it_t_init-1)';

Measures.eta_world_X1_star    = eta_world_X1_star;
Measures.Lambda_X0_star = Lambda_X0_star;
% Measures.eta_X0_bar = zeros(6, 1);

% figure
% hold on
for it_t=it_t_init:it_t_end
    display(time)

    %   Compute prediction
    [q_k, dot_q_k, ddot_q_k] = prediction(q, dot_q, ddot_q, Config);
 
    %   Compute residual
    [~, Residual] = getResidualWithStateEstimation(time, q_k, dot_q_k, ddot_q_k, Const, Config, Measures);
    % Measures.eta_X0_bar = Config.Gamma_wrench*(Lambda_X0 - Measures.Lambda_X0_star);

    %   Iterate to cancel residual norm
    iter = 0;
    while norm(Residual) > Config.r_min      
        
        %   Compute Jacobian
        J = getJacobianNumericWithStateEstimation(time, q_k, dot_q_k, ddot_q_k, Const, Config, Measures);
           
        %   Compute update
        Delta_q_k = - J\Residual;
                
        %   Correction
        [q_kp1, dot_q_kp1, ddot_q_kp1] = correction(q_k, dot_q_k, ddot_q_k, Delta_q_k, Config);

        %   Compute residual
        [~, Residual] = getResidualWithStateEstimation(time, q_kp1, dot_q_kp1, ddot_q_kp1, Const, Config, Measures);
        % Measures.eta_X0_bar = Config.Gamma_wrench*(Lambda_X0 - Measures.Lambda_X0_star);

        %   Update values
        q_k      = q_kp1;
        dot_q_k  = dot_q_kp1;
        ddot_q_k = ddot_q_kp1;

        iter = iter + 1;
    end
    display(iter)

    %   Update values
    q      = q_k;
    dot_q  = dot_q_k;
    ddot_q = ddot_q_k;

    %%  Save results into stacks 
    simulation_results = addToStack(it_t, q, dot_q, ddot_q, time, simulation_results, Config, Const);
    simulation_results = addToStackStateEstimation(it_t, q, dot_q, ddot_q, time, simulation_results, Measures, Config, Const);
    
    %%  Time update
    time = time + Config.dt;


    %%  Measurements happens now
    eta_world_X1_star = ground_truth.eta_world_stack( end, :, it_t)';
    Lambda_X0_star    = ground_truth.Lambda_stack(      1, :, it_t)';
    
    Measures.eta_world_X1_star    = eta_world_X1_star;
    Measures.Lambda_X0_star = Lambda_X0_star;


end



simulation_results.t_end = Config.t_end;
simulation_results.dt = Config.dt;
simulation_results.Gamma_wrench = Config.Gamma_wrench;
simulation_results.Gamma_twist = Config.Gamma_twist;


end

function simulation_results = addToStackStateEstimation(it_t, q, dot_q, ddot_q, time, simulation_results, Measures, Config, Const)

eta_world_X1_star = Measures.eta_world_X1_star;


[~, ~, ~, dynamics_states] = IDMWithStateEstimation(time, q, dot_q, ddot_q, Config, Const, Measures);

simulation_results.eta_X1_star(it_t, :) = eta_world_X1_star';
simulation_results.eta_X1(it_t, :)      = dynamics_states.eta_X(end, :)';
simulation_results.Lambda_X1_bar(it_t, :) = dynamics_states.Lambda_X1_bar';


% simulation_results.Lambda_X0_star(it_t, :) = Lambda_X0_star';
% simulation_results.Lambda_X0(it_t, :)      = Lambda_X0';
% simulation_results.eta_X0_bar(it_t, :)     = eta_X0_bar';

end
% 
% 
% function Measures=getMeasurements(ground_truth, it_t)
% 
% eta_X1_star       = ground_truth.eta_stack( end, :, it_t)';
% Measures.eta_X1 = eta_X1_star;
% 
% eta_world_X1_star = ground_truth.eta_world_stack( end, :, it_t)';
% Measures.eta_world_X1 = eta_world_X1_star;
% 
% 
% Lambda_X0_star = ground_truth.Lambda_stack( 1, :, it_t)';
% Measures.Lambda_X0 = Lambda_X0_star;
% end
