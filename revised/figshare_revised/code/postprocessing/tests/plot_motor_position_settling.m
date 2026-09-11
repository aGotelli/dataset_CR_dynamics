%% plot_motor_position_settling.m
% Plots Motor 1's commanded vs. achieved angle, and the resulting
% tracking error, for a representative quasi-static hold
% (static_bend_x_180), and reports the settled-hold error once the
% tracking error stays inside a chosen tolerance for the rest of the
% recording.
%
% Run this script directly; it locates the dataset relative to its own
% file location (see `data_folder` below), so MATLAB's current folder
% does not matter.

close all;
clear;
clc;

%% ====== SETTINGS ======

% Located from this script's own file location (.../code/postprocessing/
% tests/) rather than a path relative to MATLAB's current folder.
this_script_folder     = fileparts(mfilename('fullpath'));
postprocessing_folder  = fileparts(this_script_folder);
code_folder             = fileparts(postprocessing_folder);
figshare_revised_folder = fileparts(code_folder);
data_root = fullfile(figshare_revised_folder, "data");

data_folder   = fullfile(data_root, "quasi_static", "static_bend_x_180");
tol_deg       = 0.5;      % position tolerance used to detect "settled"
motor_index   = 1;        % Motor 1 is the primary actuator for this bend
saving_folder = fullfile(this_script_folder, "figures");
fig_name      = "motor_settling";

if ~isfolder(saving_folder)
    mkdir(saving_folder);
end

%% ====== LOAD DATA ======
motor = readtable(fullfile(data_folder, "dataMotor.csv"));

t0     = motor.timestamp(1);
time   = motor.timestamp - t0;

target_col = sprintf('target%d_rad', motor_index);
abs_col    = sprintf('abs_angle%d_rad', motor_index);

target_deg = rad2deg(motor.(target_col));
abs_deg    = rad2deg(motor.(abs_col));
err_deg    = abs_deg - target_deg;

%% ====== FIND SETTLED WINDOW ======
% First sample index after which |error| stays below tol_deg for the
% remainder of the recording.
n = numel(err_deg);
settled_idx = n;
for i = 1:n
    if all(abs(err_deg(i:end)) < tol_deg)
        settled_idx = i;
        break
    end
end
settle_t = time(settled_idx);

tail_err = err_deg(settled_idx:end);
fprintf('Settled-hold error (Motor %d): %.2f deg to %.2f deg (n=%d samples)\n', ...
    motor_index, min(tail_err), max(tail_err), numel(tail_err));

%% ====== PLOT ======
BLUE   = [0.1647 0.4706 0.8392];   % #2a78d6
ORANGE = [0.9216 0.4078 0.2039];   % #eb6834
GREY   = [0.890 0.886 0.863];      % #e3e2dc

fig = figure('Name', fig_name, 'Color', 'w', 'Position', [100 100 700 560]);

% --- Panel 1: angle vs time ---
ax1 = subplot(2,1,1);
hold(ax1, 'on');
xline_patch = patch(ax1, [settle_t time(end) time(end) settle_t], ...
    [ax1.YLim(1) ax1.YLim(1) ax1.YLim(2) ax1.YLim(2)], GREY, ...
    'EdgeColor', 'none', 'FaceAlpha', 0.6);
plot(ax1, time, target_deg, '--', 'Color', ORANGE, 'LineWidth', 1.6, ...
    'DisplayName', 'Commanded target');
plot(ax1, time, abs_deg, '-', 'Color', BLUE, 'LineWidth', 1.6, ...
    'DisplayName', 'Achieved (motor encoder)');
ylabel(ax1, 'Motor 1 angle [deg]');
grid(ax1, 'on');
box(ax1, 'off');
legend(ax1, 'Location', 'best', 'Box', 'off');
uistack(xline_patch, 'bottom');

% --- Panel 2: tracking error vs time ---
ax2 = subplot(2,1,2);
hold(ax2, 'on');
patch(ax2, [time(1) time(end) time(end) time(1)], ...
    [-tol_deg -tol_deg tol_deg tol_deg], ORANGE, ...
    'EdgeColor', 'none', 'FaceAlpha', 0.10);
yline(ax2, tol_deg, ':', 'Color', ORANGE, 'LineWidth', 0.9);
yline(ax2, -tol_deg, ':', 'Color', ORANGE, 'LineWidth', 0.9);
yline(ax2, 0, '-', 'Color', [0.32 0.32 0.30], 'LineWidth', 0.6);
patch(ax2, [settle_t time(end) time(end) settle_t], ...
    [ax2.YLim(1) ax2.YLim(1) ax2.YLim(2) ax2.YLim(2)], GREY, ...
    'EdgeColor', 'none', 'FaceAlpha', 0.6);
plot(ax2, time, err_deg, '-', 'Color', BLUE, 'LineWidth', 1.4);
ylabel(ax2, {'Tracking error [deg]', '(achieved - target)'});
xlabel(ax2, 'Time [s]');
grid(ax2, 'on');
box(ax2, 'off');
text(ax2, 0.985, 0.93, sprintf('intended tolerance \\pm%.1f°', tol_deg), ...
    'Units', 'normalized', 'HorizontalAlignment', 'right', ...
    'VerticalAlignment', 'top', 'Color', ORANGE, 'FontSize', 9);
text(ax2, 0.985, 0.20, ...
    sprintf('settled-hold error:\n%+.2f° to %+.2f°', min(tail_err), max(tail_err)), ...
    'Units', 'normalized', 'HorizontalAlignment', 'right', ...
    'VerticalAlignment', 'top', 'FontSize', 9, ...
    'BackgroundColor', 'w', 'EdgeColor', GREY);

linkaxes([ax1 ax2], 'x');

%% ====== SAVE ======
savefig(fig, fullfile(saving_folder, fig_name));
saveas(fig, fullfile(saving_folder, fig_name + ".png"));
exportgraphics(fig, fullfile(saving_folder, fig_name + ".pdf"), 'ContentType', 'vector');
