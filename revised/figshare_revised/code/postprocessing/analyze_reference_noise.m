close all; clear; clc;

addpath("outils\")

%% ====== PATHS ======
folder = fullfile("..", "dataCollectionPack/data/references/released_config");

%% ====== LOAD DATA ======
mk_px = readtable(fullfile(folder, "dataMark10_+x.csv"));
mk_py = readtable(fullfile(folder, "dataMark10_+y.csv"));
mk_nx = readtable(fullfile(folder, "dataMark10_-x.csv"));
mk_ny = readtable(fullfile(folder, "dataMark10_-y.csv"));

ati = readtable(fullfile(folder, "dataATIFT.csv"));

[~, ~, ~, ~, rel_kinematics_disks] = data_optitrack(fullfile(folder, "dataOptiTrack.csv"), false);

[~, fbgs_shapes, curvatures, angles] = data_fbgs(fullfile(folder, "dataFBGS.csv"));

%% ====== MARK10: std of tension_N_ ======
std_mk = [
    std(mk_px.("tension_N_"))
    std(mk_py.("tension_N_"))
    std(mk_nx.("tension_N_"))
    std(mk_ny.("tension_N_"))
];

%% ====== ATI-FT: std per channel ======
std_ati = [
    std(ati.("Fx_N_"))
    std(ati.("Fy_N_"))
    std(ati.("Fz_N_"))
    std(ati.("Tx_Nm_"))
    std(ati.("Ty_Nm_"))
    std(ati.("Tz_Nm_"))
];

%% ====== OPTITRACK: std of position (mm) and orientation (deg) per disk ======
% rel_kinematics_disks is [N_time x 6 x N_disks]
% cols 1-3: XYZ Euler angles [rad], cols 4-6: position [m]
N_disks = size(rel_kinematics_disks, 3);
std_pos_mm  = zeros(N_disks, 3);   % position std [mm]
std_ori_deg = zeros(N_disks, 3);   % orientation std [deg]
for d = 1:N_disks
    data = rel_kinematics_disks(:, :, d);  % [N x 6]
    std_ori_deg(d, :) = std(data(:, 1:3)) * (180/pi);  % rad -> deg
    std_pos_mm(d, :)  = std(data(:, 4:6)) * 1000;      % m   -> mm
end

%% ====== FBGS: avg std of curvature along arc length, tip position noise ======
% fbgs_shapes is [3 x 502 x N_time]
N_time = size(fbgs_shapes, 3);

% Std of position at each arc-length point across time -> [3 x 502]
shape_std = std(fbgs_shapes, 0, 3);         % std over time dimension
curvatures_std = std(curvatures, 1);
angles_std = std(angles, 1);

% Average std along arc length (mm)
avg_shape_std = mean(shape_std, 2) * 1000;  % [3 x 1], mm
avg_curvatures_std = mean(curvatures_std');
avg_angles_std = mean(angles_std');

% Tip (last point)
tip_std = shape_std(:, end) * 1000;         % mm



%% ====== FBGS TIP NOISE: HISTOGRAM (GAUSSIANITY CHECK) ======
tip_data = squeeze(fbgs_shapes(:, end, :))' * 1000;  % [N_time x 3], mm
% Remove mean (center the noise)
%tip_data = tip_data - mean(tip_data, 1);

labels = {'x', 'y', 'z'};
colors = {'#0072BD', '#D95319', '#77AC30'};

figure;
for i = 1:3
    subplot(1, 3, i);
    histogram(tip_data(:, i), 'Normalization', 'pdf', ...
              'FaceColor', colors{i}, 'EdgeColor', 'none', 'FaceAlpha', 0.7);
    hold on;
    % Overlay fitted Gaussian
    mu    = mean(tip_data(:, i));
    sigma = std(tip_data(:, i));
    x_range = linspace(min(tip_data(:, i)), max(tip_data(:, i)), 200);
    plot(x_range, normpdf(x_range, mu, sigma), 'k-', 'LineWidth', 1.5);
    xlabel(sprintf('Tip %s noise [mm]', labels{i}));
    ylabel('Probability density');
    title(sprintf('%s  (\\sigma = %.3f mm)', labels{i}, sigma));
    legend('Measured', 'Gaussian fit', 'Location', 'best');
    grid on; box on;
end
sgtitle('FBGS tip position noise — gaussianity check');



%% ====== FBGS ANGLE & CURVATURE NOISE: HISTOGRAM AT SELECTED GRATINGS ======
grating_idx = [8, 13, 18];
grating_labels = arrayfun(@(g) sprintf('grating %d', g), grating_idx, 'UniformOutput', false);

% --- Bending angles ---
figure;
for i = 1:numel(grating_idx)
    g = grating_idx(i);
    data = angles(:, g);
    %data = data - mean(data);   % centre

    subplot(1, numel(grating_idx), i);
    histogram(data, 'Normalization', 'pdf', ...
              'FaceColor', '#0072BD', 'EdgeColor', 'none', 'FaceAlpha', 0.7);
    hold on;
    mu    = mean(data);
    sigma = std(data);
    x_range = linspace(min(data), max(data), 200);
    plot(x_range, normpdf(x_range, mu, sigma), 'k-', 'LineWidth', 1.5);
    xlabel(sprintf('Angle noise [rad] — %s', grating_labels{i}));
    ylabel('Probability density');
    title(sprintf('%s  (\\sigma = %.4f rad)', grating_labels{i}, sigma));
    legend('Measured', 'Gaussian fit', 'Location', 'best');
    grid on; box on;
end
sgtitle('FBGS bending angle noise — gaussianity check');

% --- Curvatures ---
figure;
for i = 1:numel(grating_idx)
    g = grating_idx(i);
    data = curvatures(:, g);
    %data = data - mean(data);   % centre

    subplot(1, numel(grating_idx), i);
    histogram(data, 'Normalization', 'pdf', ...
              'FaceColor', '#D95319', 'EdgeColor', 'none', 'FaceAlpha', 0.7);
    hold on;
    mu    = mean(data);
    sigma = std(data);
    x_range = linspace(min(data), max(data), 200);
    plot(x_range, normpdf(x_range, mu, sigma), 'k-', 'LineWidth', 1.5);
    xlabel(sprintf('Curvature noise [1/mm] — %s', grating_labels{i}));
    ylabel('Probability density');
    title(sprintf('%s  (\\sigma = %.4f 1/mm)', grating_labels{i}, sigma));
    legend('Measured', 'Gaussian fit', 'Location', 'best');
    grid on; box on;
end
sgtitle('FBGS curvature noise — gaussianity check');


%% ====== PRINT FOR LATEX TABLE ======
fprintf('\n%% --- Paste into LaTeX sensor noise table ---\n\n');

fprintf('Mark-10 ($+x$)   & Cable tension & $%.3f$ \\\\\n', std_mk(1));
fprintf('Mark-10 ($+y$)   & Cable tension & $%.3f$ \\\\\n', std_mk(2));
fprintf('Mark-10 ($-x$)   & Cable tension & $%.3f$ \\\\\n', std_mk(3));
fprintf('Mark-10 ($-y$)   & Cable tension & $%.3f$ \\\\\n', std_mk(4));
fprintf('\\midrule\n');
fprintf('ATI-FT & $F_x$ & $%.4f$ \\\\\n', std_ati(1));
fprintf('       & $F_y$ & $%.4f$ \\\\\n', std_ati(2));
fprintf('       & $F_z$ & $%.4f$ \\\\\n', std_ati(3));
fprintf('       & $T_x$ & $%.5f$ \\\\\n', std_ati(4));
fprintf('       & $T_y$ & $%.5f$ \\\\\n', std_ati(5));
fprintf('       & $T_z$ & $%.5f$ \\\\\n', std_ati(6));
fprintf('\\midrule\n');
for d = 1:N_disks
    fprintf('OptiTrack disk~%d & Position ($x,y,z$) [mm]    & $(%.3f,\\;%.3f,\\;%.3f)$ \\\\\n', ...
        d-1, std_pos_mm(d,1), std_pos_mm(d,2), std_pos_mm(d,3));
    fprintf('                  & Orientation ($X,Y,Z$) [deg] & $(%.4f,\\;%.4f,\\;%.4f)$ \\\\\n', ...
        std_ori_deg(d,1), std_ori_deg(d,2), std_ori_deg(d,3));
end
fprintf('\\midrule\n');
fprintf('FBGS & Avg shape std ($x,y,z$) & $(%.3f,\\;%.3f,\\;%.3f)$ mm \\\\\n', ...
    avg_shape_std(1), avg_shape_std(2), avg_shape_std(3));
fprintf('     & Tip position std ($x,y,z$) & $(%.3f,\\;%.3f,\\;%.3f)$ mm \\\\\n', ...
    tip_std(1), tip_std(2), tip_std(3));
fprintf('     & AVg curvature std & $%.3f$ \\\\\n', avg_curvatures_std);
fprintf('     & AVg angles std & $%.3f$ \\\\\n', avg_angles_std);