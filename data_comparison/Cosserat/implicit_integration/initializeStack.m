function [simulation_results, it_t, time] = initializeStack(q, dot_q, ddot_q, Config, Const)

%%   Retrieve time variables
t_start = Config.t_start;
t_end   = Config.t_end;
dt      = Config.dt;
time    = 0;

time_steps    = t_end/dt;
steps_skipped = t_start/dt + 1;


%%   Prepare stacks for data

%   Points
observation_points = length(Config.forward_integration_domain);

%   Time steps
simulation_results.t_stack = zeros(time_steps, 1);

%   Generalized coordinates
simulation_results.q_stack      = zeros(time_steps, length(q));
simulation_results.dot_q_stack  = zeros(time_steps, length(q));
simulation_results.ddot_q_stack = zeros(time_steps, length(q));

%   Kinematics (local frame)
simulation_results.Q_stack    = zeros(observation_points, 4, time_steps);
simulation_results.r_stack    = zeros(observation_points, 3, time_steps);
simulation_results.eta_stack  = zeros(observation_points, 6, time_steps);
simulation_results.deta_stack = zeros(observation_points, 6, time_steps);

%   Wrenches (local frame)
simulation_results.Lambda_stack = zeros(observation_points, 6, time_steps);

%   Kinematics (global frame)
simulation_results.eta_world_stack= zeros(observation_points, 6, time_steps);

%  Additional stuff related to state estimation
simulation_results.Lambda_X0_star = zeros(time_steps, 6);
simulation_results.Lambda_X0      = zeros(time_steps, 6);
simulation_results.eta_X0_bar     = zeros(time_steps, 6);

simulation_results.eta_X1_star   = zeros(time_steps, 6);
simulation_results.eta_X1        = zeros(time_steps, 6);
simulation_results.Lambda_X1_bar = zeros(time_steps, 6);




%%  Initialize stack
for it_t=1:steps_skipped    
    simulation_results = addToStack(it_t, q, dot_q, ddot_q, time, simulation_results, Config, Const);

    time = time + dt;
end






end