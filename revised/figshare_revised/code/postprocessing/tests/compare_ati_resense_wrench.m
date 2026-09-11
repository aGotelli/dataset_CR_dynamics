%% compare_ati_resense_wrench.m
%
% Compares the ATI base force/torque sensor against the Resense contact
% wand, for the contact_motion/push_retract recording. The Resense
% wand's measured wrench is transported from its own sensor frame to the
% robot base frame (via its mocap pose and a fixed sensor-to-mocap
% offset, Ad_g), and plotted against the ATI sensor's own base-frame
% reading for direct comparison.
%
% Pipeline: load ATI, Resense and OptiTrack data for the recording;
% compute the per-disk mocap pose correction from this recording's own
% first 3 seconds (the wand is disk 6); low-pass filter every signal;
% resample everything onto a common 100 Hz grid; transport the Resense
% wrench to the base frame; plot the comparison; save the processed
% wrenches to the recording's processed/ folder.
%
% Note: the mocap correction here is computed from this recording's own
% first 3 seconds, independently of outils/compute_mocap_correction.m
% (which the rest of the pipeline uses, computed once from the dedicated
% straight_config reference recording instead).
%
% Run this script directly; it locates the dataset relative to its own
% file location, so MATLAB's current folder does not matter.

close all;
clear;
clc;

addpath("outils\")

%% ====== PATHS / SETTINGS ======

% Located from this script's own file location rather than a path
% relative to MATLAB's current folder: this script lives inside
% code/postprocessing/tests/, and data/ is code/'s sibling folder, so
% climb up to code/ and step across into data/.
this_script_folder = fileparts(mfilename('fullpath'));
code_folder = fileparts(fileparts(this_script_folder));
data_root = fullfile(fileparts(code_folder), "data");

folder = fullfile(data_root, "contact_motion", "push_retract");

cutoffHz    = 30;   % Butterworth cutoff
butterOrder = 4;

samplingHz = 100;

saving_fig_folder = fullfile(this_script_folder, "figures") + filesep;


%% ====== LOAD DATA ======

ati = readtable(fullfile(folder, "dataATIFT.csv"));

use_resense = true;
resense = readtable(fullfile(folder, "dataResenseFT.csv"));

filename = fullfile(folder, "dataOptiTrack.csv");
[N_disks, mocap_timestamps, poses_disks, rel_poses_disks, rel_kinematics_disks] = data_optitrack(filename, true);

%% ====== EXTRACT ATI AND RESENSE, ON THEIR OWN RAW TIMESTAMPS ======
tA = ati.timestamp;

pose_wand = rel_poses_disks(:, :, 6, :);

time_resense = resense.timestamp_s_;
wrench_wand = [resense.Fx resense.Fy resense.Fz resense.Tx/1000 resense.Ty/1000 resense.Tz/1000];

% Common time origin: the later of the two sensors' own start times.
t_0 = max(tA(1), time_resense(1));

tA_rel = tA - t_0;
tresense_rel = time_resense - t_0;
relative_time_mocap = mocap_timestamps - t_0;


%%  Correct mocap disk poses (per-disk residual-offset correction)
idx_init = relative_time_mocap <= 3.0;

rel_kinematics_disks_init = rel_kinematics_disks(idx_init, :, :);
mocap_time_rel_init = relative_time_mocap(idx_init);

% Nominal z-position (m) of the 5 robot disks in the undeformed/straight
% configuration.
pos_disks = [
    0    0    0    0    0
    0    0    0    0    0
    0    0.12 0.24 0.36 0.48
];

N_disks_robot = 5;

g_correction = zeros(4, 4, N_disks_robot);
rel_kinematics_disks_corr = zeros(size(rel_kinematics_disks));
for it=1:N_disks_robot

    g_disk_ref = eye(4);
    g_disk_ref(1:3, 4) = pos_disks(:, it);

    EUL_disk_t = rel_kinematics_disks_init(:, 1:3, it)';
    r_disk_t = rel_kinematics_disks_init(:, 4:6, it)';

    EUL_disk = mean(EUL_disk_t, 2);
    r_disk = mean(r_disk_t, 2);

    R_disk = eul2rotm(EUL_disk', 'XYZ');

    g_meas_m1 = [
        R_disk' -R_disk'*r_disk
        0   0   0   1
    ];

    g_correction(:,:, it) = g_meas_m1*g_disk_ref;

    rel_poses_disk = rel_poses_disks(:, :, it, :);
    rel_poses_disk_corr = pagemtimes(rel_poses_disk, g_correction(:,:, it));

    r_disk_corr = squeeze( rel_poses_disk_corr(1:3,   4, :, :) );
    R_disk_corr = squeeze( rel_poses_disk_corr(1:3, 1:3, :, :) );
    XYZ_disk_corr = rotm2eul(R_disk_corr, 'XYZ');

    rel_kinematics_disks_corr(:, :, it) = [
      XYZ_disk_corr   r_disk_corr'
    ];

end

rel_kinematics_disks_corr_init = rel_kinematics_disks_corr(idx_init, :, :);


%% ====== FILTER (BUTTERWORTH, ZERO-PHASE) ======

ATI_F = [ati.Fx_N_, ati.Fy_N_, ati.Fz_N_];
ATI_T = [ati.Tx_Nm_, ati.Ty_Nm_, ati.Tz_Nm_];

ATI_F_f = zeros(size(ATI_F));
ATI_T_f = zeros(size(ATI_T));
for k = 1:3
    ATI_F_f(:,k) = butter_filtfilt(tA, ATI_F(:,k), cutoffHz, butterOrder);
    ATI_T_f(:,k) = butter_filtfilt(tA, ATI_T(:,k), cutoffHz, butterOrder);
end

ATI_FT_f = [ATI_F_f ATI_T_f];
ATI_FT = [ATI_F ATI_T];

rel_kinematics_disks_f = zeros(size(rel_kinematics_disks));
for it=1:N_disks
    for k=1:6
        rel_kinematics_disks_f(:, k, it) = butter_filtfilt(mocap_timestamps, rel_kinematics_disks(:, k, it), cutoffHz, butterOrder);
    end
end

wrench_wand_f = zeros(size(wrench_wand));
for k = 1:6
    wrench_wand_f(:,k) = butter_filtfilt(time_resense, wrench_wand(:,k), cutoffHz, butterOrder);
end


%%  Resample everything onto a common 100 Hz grid

if tA_rel(1) < tresense_rel(1)
    time_end = tA_rel(end);
else
    time_end = tresense_rel(end);
end

N_samples = floor(samplingHz*time_end);
sampling_dt = 1/samplingHz;
sampling_time = (0:sampling_dt:sampling_dt*(N_samples-1))';

interp_base_wrench = zeros(N_samples, 6);
interp_base_wrench_raw = zeros(N_samples, 6);
for it=1:6
    interp_base_wrench(:, it) = interp1(tA_rel, ATI_FT_f(:, it), sampling_time)';
    interp_base_wrench_raw(:, it) = interp1(tA_rel, ATI_FT(:, it), sampling_time)';
end

interp_rel_kinematics_disks = zeros(N_samples, 6, N_disks);
for it=1:N_disks
    for k=1:6
        interp_rel_kinematics_disks(:, k, it) = interp1(relative_time_mocap, rel_kinematics_disks_f(:, k, it), sampling_time);
    end
end

interp_wrench_wand = zeros(N_samples, 6);
for it=1:6
    interp_wrench_wand(:, it) = interp1(tresense_rel, wrench_wand_f(:, it), sampling_time)';
end


%%  Transport the Resense wrench to the robot base frame (Ad_g)

% Fixed offset between the wand's mocap frame and its sensor frame.
R_fix_x = axang2rotm([1 0 0 pi/2]);
R_fix_z = axang2rotm([0 0 1 pi/6]);
R_fix = R_fix_x*R_fix_z;
r_fix = [
    0
   -0.1137
    0
];
g_fix = [
        R_fix r_fix
        0 0 0   1
    ];

wrench_at_base = zeros(6, N_samples);
pos_sensor = zeros(3, N_samples);
for it_t=1:length(sampling_time)
    wand_XYZ_xyz = interp_rel_kinematics_disks(it_t, :, 6);

    R = eul2rotm(wand_XYZ_xyz(1:3), 'XYZ');
    r = wand_XYZ_xyz(4:6)';

    g = [
      R     r
      0 0 0 1
    ];

    g_s = g*g_fix;
    R_s = g_s(1:3, 1:3);
    r_s = g_s(1:3, 4);
    pos_sensor(:, it_t) = r_s;
    wrench_wand_it_t = interp_wrench_wand(it_t, :)';

    Ad_g_=[R_s zeros(3,3)
            hat_(r_s)*R_s R_s];

    wrench_at_base(:, it_t) = -Ad_g_*wrench_wand_it_t;
end


%% ====== PLOTS ======

% ATI vs Resense, raw signals, first 20 s (forces only).
fig = figure("Name", "FT_Forces");
subplot(2, 1, 1)
plot(tA_rel, ati.Fx_N_, 'b')
hold on
plot(tresense_rel, -resense.Fx, 'r')
set(gca,"FontSize",30)
grid on
xlim([0, 20])
ylabel("F_x [N]", "FontSize", 30)

subplot(2, 1, 2)
plot(tA_rel, ati.Fy_N_, 'b')
hold on
plot(tresense_rel, -resense.Fz, 'r')
xlim([0, 20])
set(gca,"FontSize",30)
grid on
ylabel("F_y [N]", "FontSize", 30)
xlabel("Time [s]", "FontSize", 30)


% ATI vs Resense-transported-to-base (Ad_g), forces, on the common grid.
fig = figure("Name", "Forces (Ad_g)");
subplot(2, 1, 1)
plot(sampling_time, interp_base_wrench_raw(:, 1), 'b')
hold on
plot(sampling_time, wrench_at_base(1, :), 'r')
set(gca,"FontSize",30)
grid on
ylabel("F_x [N]", "FontSize", 30)

subplot(2, 1, 2)
plot(sampling_time, interp_base_wrench_raw(:, 2), 'b')
hold on
plot(sampling_time, wrench_at_base(2, :), 'r')
set(gca,"FontSize",30)
grid on
ylabel("F_y [N]", "FontSize", 30)
xlabel("Time [s]", "FontSize", 30)

savefig(saving_fig_folder + fig.Name)
saveas(fig, saving_fig_folder + fig.Name, 'png')


% ATI vs Resense-transported-to-base (Ad_g), torques, on the common grid.
figure("Name", "Torques (Ad_g)")
subplot(2, 1, 1)
plot(sampling_time, interp_base_wrench_raw(:, 4), 'b')
hold on
plot(sampling_time, wrench_at_base(4, :), 'r')
set(gca,"FontSize",30)
grid on
ylabel("T_x [N]", "FontSize", 30)

subplot(2, 1, 2)
plot(sampling_time, interp_base_wrench_raw(:, 5), 'b')
hold on
plot(sampling_time, wrench_at_base(5, :), 'r')
set(gca,"FontSize",30)
grid on
ylabel("T_y [N]", "FontSize", 30)
xlabel("Time [s]", "FontSize", 30)


% Same forces/torques comparison, compact 2x2 layout.
figure("Name", "Wrench")

subplot(2, 2, 1)
plot(sampling_time, interp_base_wrench_raw(:, 1), 'b')
hold on
plot(sampling_time, wrench_at_base(1, :), 'r')
grid on
ylabel("F_x [N]")

subplot(2, 2, 3)
plot(sampling_time, interp_base_wrench_raw(:, 2), 'b')
hold on
plot(sampling_time, wrench_at_base(2, :), 'r')
grid on
ylabel("F_y [N]")
xlabel("Time [s]")

subplot(2, 2, 2)
plot(sampling_time, interp_base_wrench_raw(:, 4), 'b')
hold on
plot(sampling_time, wrench_at_base(4, :), 'r')
grid on
ylabel("T_x [Nm]")

subplot(2, 2, 4)
plot(sampling_time, interp_base_wrench_raw(:, 5), 'b')
hold on
plot(sampling_time, wrench_at_base(5, :), 'r')
grid on
ylabel("T_y [Nm]")
xlabel("Time [s]")


% Disk 1 (base) position, on the common grid.
figure("Name", "Position base")
subplot(3, 1, 1)
plot(sampling_time, interp_rel_kinematics_disks(:, 4, 1), 'b', 'LineWidth', 1)
ylabel('p_x [m]')
grid on

subplot(3, 1, 2)
plot(sampling_time, interp_rel_kinematics_disks(:, 5, 1), 'b', 'LineWidth', 1)
ylabel('p_y [m]')
grid on

subplot(3, 1, 3)
plot(sampling_time, interp_rel_kinematics_disks(:, 6, 1), 'b', 'LineWidth', 1)
ylabel('p_z [m]')
grid on
xlabel("Time [s]")


% Robot tip (disk 5) vs wand sensor position, on the common grid.
figure("Name", "Position disk and wand")
subplot(3, 1, 1)
plot(sampling_time, interp_rel_kinematics_disks(:, 4, 5), 'b', 'LineWidth', 1)
hold on
plot(sampling_time, pos_sensor(1, :), 'r', 'LineWidth', 1)
ylabel('p_x [m]')
grid on

subplot(3, 1, 2)
plot(sampling_time, interp_rel_kinematics_disks(:, 5, 5), 'b', 'LineWidth', 1)
hold on
plot(sampling_time, pos_sensor(2, :), 'r', 'LineWidth', 1)
ylabel('p_y [m]')
grid on

subplot(3, 1, 3)
plot(sampling_time, interp_rel_kinematics_disks(:, 6, 5), 'b', 'LineWidth', 1)
hold on
plot(sampling_time, pos_sensor(3, :), 'r', 'LineWidth', 1)
ylabel('p_z [m]')
grid on
xlabel("Time [s]")
legend('robot tip', 'wand')


% In-plane (xy) distance between the robot tip and the wand sensor.
disk_pos = interp_rel_kinematics_disks(:, 4:6, 5);
sensor_pos = pos_sensor(1:3, :)';

diff_3D = disk_pos - sensor_pos;
dist_3D = sqrt( diff_3D(:, 1).^2 + diff_3D(:, 2).^2 );

figure("Name", "Distance sensor from disk")
plot(sampling_time, dist_3D, 'b', 'LineWidth', 1)
ylabel('Distance [m]')
grid on
xlabel("Time [s]")


%% ====== SAVE RESULTS ======
processed_folder = fullfile(folder, "processed");
if ~isfolder(processed_folder)
    mkdir(processed_folder);
end

%   ATI-FT base wrench (filtered, resampled at 100 Hz)
T_ati = array2table([sampling_time, interp_base_wrench], ...
    'VariableNames', {'time','Fx','Fy','Fz','Tx','Ty','Tz'});
writetable(T_ati, fullfile(processed_folder, "base_wrench.csv"));

%   ATI-FT base wrench (raw, resampled at 100 Hz -- no low-pass filter)
T_ati_raw = array2table([sampling_time, interp_base_wrench_raw], ...
    'VariableNames', {'time','Fx','Fy','Fz','Tx','Ty','Tz'});
writetable(T_ati_raw, fullfile(processed_folder, "base_wrench_raw.csv"));

%   Resense contact wrench at sensor frame (filtered, resampled at 100 Hz)
T_resense = array2table([sampling_time, interp_wrench_wand], ...
    'VariableNames', {'time','Fx','Fy','Fz','Tx','Ty','Tz'});
writetable(T_resense, fullfile(processed_folder, "contact_wrench.csv"));

%   Resense contact wrench transported to robot base via Ad_g (resampled at 100 Hz)
T_adg = array2table([sampling_time, wrench_at_base'], ...
    'VariableNames', {'time','Fx','Fy','Fz','Tx','Ty','Tz'});
writetable(T_adg, fullfile(processed_folder, "contact_wrench_at_base.csv"));


%% ====== LOCAL FUNCTIONS ======

function y = butter_filtfilt(t, x, fc, n)
    %   Zero-phase Butterworth low-pass filtering. Estimates the
    %   sampling rate as 1/median(diff(t)) and filters x at cutoff
    %   frequency fc (Hz) with a filter of order n.
    Fs = 1/median(diff(t));
    [b,a] = butter(n, fc/(Fs/2), "low");
    y = filtfilt(b,a, x);
end

function [A] = hat_(x)
    %   Skew-symmetric cross-product matrix of a 3-vector x, such that
    %   hat_(x)*v == cross(x, v).
    A=zeros(3,3);

    A(1,2)=-x(3);
    A(1,3)=x(2);
    A(2,3)=-x(1);

    A(2,1)=x(3);
    A(3,1)=-x(2);
    A(3,2)=x(1);
end
