function [t_stack, states_stack] = CosseratRodImplicitSimulation(q, dot_q, ddot_q, tau, Const, Config)

%   Prepare stacks for data
t = 0;
t_stack = [t];
states_stack = [q', dot_q', ddot_q'];



%   Time loop
idx_t = 1;
while t<Config.t_end

    Const.tau = tau(:, idx_t);


    %   Compute prediction
    [q_k, dot_q_k, ddot_q_k] = prediction(q, dot_q, ddot_q, Config);

    %   Compute residual
    Residual = getResidual(t, q_k, dot_q_k, ddot_q_k, Const, Config);

    %   Iterate to cancel residual norm
    while norm(Residual) > Config.r_min      
        
        %   Compute Jacobian
        J = getJacobian(t, q_k, dot_q_k, ddot_q_k, Const, Config);
           
        %   Compute update
        Delta_q_k = - J\Residual;
                
        %   Correction
        [q_kp1, dot_q_kp1, ddot_q_kp1] = correction(q_k, dot_q_k, ddot_q_k, Delta_q_k, Config);

        %   Compute residual
        Residual = getResidual(t, q_kp1, dot_q_kp1, ddot_q_kp1, Const, Config);

        %   Update values
        q_k      = q_kp1;
        dot_q_k  = dot_q_kp1;
        ddot_q_k = ddot_q_kp1;
    end

    %   Update values
    q         = q_k;
    dot_q     = dot_q_k;
    ddot_q = ddot_q_k;
    t = t + Config.dt;

    
    %   Update iter count
    idx_t = idx_t + 1;


    %   Stack current state
    t_stack = [t_stack; t];
    states_stack = [states_stack;
                    q', dot_q', ddot_q'];

    display(t)

    % if Config.plot_simu plotRod(t, q, Const, Config, 'b'); end
    
end


end