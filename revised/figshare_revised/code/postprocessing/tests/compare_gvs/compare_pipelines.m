%% compare_pipelines.m
% Loads new_results.mat and legacy_results.mat (run run_new_pipeline.m
% and run_legacy_pipeline.m first, in separate MATLAB sessions -- both
% addpath a different codebase and some filenames collide, e.g. A.m,
% getR.m, r_dot.m, internalActuation.m, cheb.m) and diffs the shared
% quantities, base wrench/force first since that is what showed the
% 5x symptom.
clear; clc;

here = fileparts(mfilename('fullpath'));
N = load(fullfile(here, 'new_results.mat'));
L = load(fullfile(here, 'legacy_results.mat'));

assert(isequal(N.q, L.q) && isequal(N.dot_q, L.dot_q) && ...
       isequal(N.ddot_q, L.ddot_q) && isequal(N.tau, L.tau), ...
       'Test states differ between the two .mat files -- rerun both pipelines.');

report('Qad (actuation, N)',        N.Qad,       L.Qad);
report('Lambda_X0 / F0 (base wrench, N)', N.Lambda_X0, L.F0);
report('Qa_X0 / Q_a (base gen. force, N)', N.Qa_X0,     L.Q_a);

%   Forward-pass profile, same X grid on both sides
fprintf('\n--- forward-pass profile (max abs diff over X_common) ---\n');
fprintf('Q   : %.3e\n', max(abs(N.Q_X_common(:)   - L.Q_X(:))));
fprintf('r   : %.3e\n', max(abs(N.r_X_common(:)   - L.r_X(:))));
fprintf('eta : %.3e\n', max(abs(N.eta_X_common(:) - L.eta_X(:))));

function report(name, new_val, legacy_val)
fprintf('\n--- %s ---\n', name);
fprintf('new    = %s\n', mat2str(new_val(:)', 4));
fprintf('legacy = %s\n', mat2str(legacy_val(:)', 4));
ratio = legacy_val(:) ./ new_val(:);
ratio(abs(new_val(:)) < 1e-10) = NaN;
fprintf('legacy/new ratio per component = %s\n', mat2str(ratio', 4));
end
