%% run_legacy_pipeline.m
% Runs gvs_simulations_legacy on the SAME fixed test state as
% run_new_pipeline.m and dumps the matching quantities:
%   Qad    - actuation generalized force (internalActuation.m)
%   F0     - internal wrench at the base (TIDM_spectral.m)
%   Q_a    - dynamic/elastic generalized force at the base (TIDM_spectral.m)
%   Q_X,r_X,eta_X - forward-pass profile (Forward_eta.m), on a grid shared
%                   with the new pipeline for a like-for-like diff
%
% NOTE: parameters_dataset_robot.m reads Const.EA/Const.GA but never
% defines them. They do not affect the result for this DoFs config
% (B only selects the K2/K3 columns of H_cal), but the script would
% error without them, so they are stubbed here.
clear; clc;
restoredefaultpath; rehash;

here     = fileparts(mfilename('fullpath'));
leg_root = fullfile(here, '..', 'gvs_simulations_legacy', 'GVS_outils');
addpath(genpath(leg_root));

Config.option = odeset('RelTol', 1e-8, 'AbsTol', 1e-8);
Config.plot   = false;
Config.V_a    = [0, 1, 1, 0, 0, 0];

Const.dim_base_k = [0, 3, 3, 0, 0, 0];
Const.dim_base   = Config.V_a*Const.dim_base_k';

Const.EA = 1e6;   % unused by H_cal after B-projection; stubbed, see NOTE
Const.GA = 1e6;

run(fullfile(leg_root, '..', 'parameters_dataset_robot.m'));

r_0 = [0; 0; 0];
Q_0 = [0.7071068 0 0.7071068 0]';

Const.r_0 = r_0;
Const.q_0 = Q_0;
Const.F1  = zeros(6, 1);

%   Fixed test state, shared verbatim with run_new_pipeline.m
q      = [ 0.05; -0.02;  0.01; -0.03;  0.02; -0.01];
dot_q  = [ 0.10; -0.05;  0.02; -0.02;  0.03; -0.01];
ddot_q = [ 0.20; -0.10;  0.05; -0.05;  0.02; -0.02];
tau    = [3; -2];

Const.q         = q;
Const.q_dot     = dot_q;
Const.q_dot_dot = ddot_q;

Qad = internalActuation(tau, Const, Config);

%   Newmark coefficients: unused by F0/Q_a themselves (only by the
%   Jacobian's variation block), a fixed dt is fine here.
Beta = 1/4; Gamma = 1/2; dt = 1e-2;
a = Gamma/(Beta*dt);
b = 1/(Beta*dt^2);

eta_0     = zeros(6, 1);   % clamped base: no rigid-body rate/accel
eta_dot_0 = zeros(6, 1);

[F0, Q_a] = TIDM_spectral(Q_0, r_0, Q_0, r_0, eta_0, eta_dot_0, a, b, 0, Const, Config);

%   Forward-pass profile, evaluated on a grid shared with the new
%   pipeline (see compare_pipelines.m) rather than legacy's internal
%   30-node Chebyshev grid, so the two profiles line up point-for-point.
X_common = linspace(0, Config.L, 20)';
CI0 = [Q_0; r_0; eta_0];
[~, Yf] = ode45(@(X, y) Forward_eta(X, y, 0, Const, Config), X_common, CI0, Config.option);
Q_X   = Yf(:, 1:4)';
r_X   = Yf(:, 5:7)';
eta_X = Yf(:, 8:13)';

save(fullfile(here, 'legacy_results.mat'), ...
     'q', 'dot_q', 'ddot_q', 'tau', 'Qad', 'F0', 'Q_a', ...
     'X_common', 'Q_X', 'r_X', 'eta_X');

fprintf('--- legacy pipeline ---\n');
fprintf('Qad = %s\n', mat2str(Qad', 4));
fprintf('F0  = %s\n', mat2str(F0', 4));
fprintf('Q_a = %s\n', mat2str(Q_a', 4));
