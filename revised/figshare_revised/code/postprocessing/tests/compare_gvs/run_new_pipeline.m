%% run_new_pipeline.m
% Runs Cosserat_gvs_simulations on a fixed test state and dumps the
% quantities compared against the legacy pipeline in compare_pipelines.m:
%   Qad       - actuation generalized force (internalActuation.m)
%   Lambda_X0 - internal wrench at the base (IDM.m)
%   Qa_X0     - dynamic/elastic generalized force at the base (IDM.m)
%   Q_X,r_X,eta_X,deta_X - forward-pass profiles on the ODE grid
clear; clc;

here    = fileparts(mfilename('fullpath'));
new_root = fullfile(here, '../', 'Cosserat_gvs_simulations');
addpath(fullfile(new_root, 'ODEs'));
addpath(fullfile(new_root, 'utilities'));
addpath(fullfile(new_root, 'rod_properties'));
addpath(fullfile(new_root, 'implicit_integration'));

DoFs = [0, 3, 3, 0, 0, 0];
[Const, Config] = simulationConfigurations(DoFs, 1, 1e-2);

Const.r_X0 = [0; 0; 0];
Const.Q_X0 = [0.7071068 0 0.7071068 0]';

%   Fixed test state, shared verbatim with run_legacy_pipeline.m
q      = [ 0.05; -0.02;  0.01; -0.03;  0.02; -0.01];
dot_q  = [ 0.10; -0.05;  0.02; -0.02;  0.03; -0.01];
ddot_q = [ 0.20; -0.10;  0.05; -0.05;  0.02; -0.02];
tau    = [3; -2];

Const.tau = tau;

[Lambda_X0, Qa_X0, Q_X, r_X, eta_X, deta_X] = IDM(0, q, dot_q, ddot_q, Config, Const);
Qad = internalActuation(q, Const, Config);

X_grid = Config.forward_integration_domain;   % increasing X, 0 -> L

%   Forward-pass profile on a grid shared with run_legacy_pipeline.m
%   (that one is stuck on its own internal 30/31-node Chebyshev grid;
%   evaluating both on the same X points makes the diff meaningful).
X_common = linspace(0, Const.L, 20)';
CI0 = zeros(19, 1);
CI0(1:7) = [Const.Q_X0; Const.r_X0];
[~, Yf] = ode45(@(X, y) ForwardKinematics(X, y, q, dot_q, ddot_q, Config, Const), X_common, CI0);
Q_X_common   = Yf(:, 1:4)';
r_X_common   = Yf(:, 5:7)';
eta_X_common = Yf(:, 8:13)';

save(fullfile(here, 'new_results.mat'), ...
     'q', 'dot_q', 'ddot_q', 'tau', 'Qad', 'Lambda_X0', 'Qa_X0', ...
     'X_grid', 'Q_X', 'r_X', 'eta_X', 'deta_X', ...
     'X_common', 'Q_X_common', 'r_X_common', 'eta_X_common');

fprintf('--- new pipeline ---\n');
fprintf('Qad       = %s\n', mat2str(Qad', 4));
fprintf('Lambda_X0 = %s\n', mat2str(Lambda_X0', 4));
fprintf('Qa_X0     = %s\n', mat2str(Qa_X0', 4));
