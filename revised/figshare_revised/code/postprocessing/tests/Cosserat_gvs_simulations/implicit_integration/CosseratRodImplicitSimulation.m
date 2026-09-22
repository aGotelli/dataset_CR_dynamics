function [t_stack, states_stack] = CosseratRodImplicitSimulation(q, dot_q, ddot_q, tau, Const, Config)

%   Prepare stacks for data
N_time = round( Config.t_end/Config.dt );
t_stack = zeros(1, N_time);
states_stack.q   = zeros(Const.dim_base, N_time);
states_stack.dq  = zeros(Const.dim_base, N_time);
states_stack.ddq = zeros(Const.dim_base, N_time);

states_stack.Lambda_X0 = zeros(6, N_time);
states_stack.tip_position = zeros(3, N_time);


t = 0;


%   Time loop
idx_t = 1;
while t<=Config.t_end

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

    %   Stack current state
    t_stack(1, idx_t) = t;
    states_stack.q(:, idx_t)   = q;
    states_stack.dq(:, idx_t)  = dot_q;
    states_stack.ddq(:, idx_t) = ddot_q;
    
    [Lambda_X0, ~, ~, r_X] = IDM(t, q, dot_q, ddot_q, Config, Const);
    states_stack.Lambda_X0(:, idx_t) = Lambda_X0;
    states_stack.tip_position(:, idx_t) = r_X(:, end);


    display(t)

    %   Update counts
    t = t + Config.dt;
    idx_t = idx_t + 1;

    
end


end