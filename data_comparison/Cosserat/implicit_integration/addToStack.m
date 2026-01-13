function simulation_results = addToStack(it_t, q, dot_q, ddot_q, time, simulation_results, Config, Const)

simulation_results.t_stack(it_t, :) = time;

simulation_results.q_stack(it_t, :)      = q';
simulation_results.dot_q_stack(it_t, :)  = dot_q';
simulation_results.ddot_q_stack(it_t, :) = ddot_q';


[~, ~, dynamics_states] = IDM(time, q, dot_q, ddot_q, Config, Const);

%   Unpack kinematics
simulation_results.Q_stack(:,:, it_t) = dynamics_states.Q_X;
simulation_results.r_stack(:,:, it_t) = dynamics_states.r_X;
simulation_results.eta_stack(:,:, it_t)  = dynamics_states.eta_X;
simulation_results.deta_stack(:,:, it_t) = dynamics_states.deta_X;

%   Unpack dynamics
simulation_results.Lambda_stack(:,:, it_t) = dynamics_states.Lambda_X;

%% project the twist into the local frame
Q_stack   = dynamics_states.Q_X;
eta_X     = dynamics_states.eta_X;
dot_eta_X = dynamics_states.deta_X;

% number of points 
N = length(Config.forward_integration_domain);

% Get rotation mtrix at those points
R_X = quat2rotm(Q_stack);

% Convert in S matrix S = diag(R, R)
S_X = zeros(6, 6, N);
S_X(1:3, 1:3, :) = R_X;
S_X(4:6, 4:6, :) = R_X;

% % from local frame to global frame
% ST_X = pagetranspose(S_X);

%   Make it a 6x1xN dimentional array
eta_X     = reshape(    eta_X', [6, 1, N]);
dot_eta_X = reshape(dot_eta_X', [6, 1, N]);

%   One shot projection
eta_world_X     = pagemtimes(S_X,     eta_X);
dot_eta_world_X = pagemtimes(S_X, dot_eta_X);

% convert it back to Nx6
simulation_results.eta_world_stack(:,:, it_t)     = squeeze(eta_world_X)';
simulation_results.dot_eta_world_stack(:,:, it_t) = squeeze(dot_eta_world_X)';


end


