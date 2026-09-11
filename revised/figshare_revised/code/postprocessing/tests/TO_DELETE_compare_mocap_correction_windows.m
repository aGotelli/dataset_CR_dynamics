%% compare_mocap_correction_windows.m
%
% Compares the per-disk mocap residual-offset correction computed from
% different time windows of different recordings, to check how sensitive
% the correction is to which window it's estimated from.
%
% For each of three representative recordings (Lissajous_fast,
% circle_fast, static_bend_x_100) and for each of the 5 tracked robot
% disks, this reports the translation and rotation difference between:
%   (a) the correction that recording's own first 3 seconds would give,
%   (b) the correction references/straight_config/ gives, computed both
%       from its own first 3 seconds and from its full duration.
% The straight_config first-3s-vs-full-duration difference is also
% printed on its own, as a check on how stable that recording's estimate
% is.
%
% This script does not call outils/compute_mocap_correction.m or modify
% any pipeline file. mean_disk_pose below independently reimplements the
% same window-averaging computation, so this comparison stays valid even
% if compute_mocap_correction.m's own implementation changes.
%
% Run this script directly; it locates the dataset and the postprocessing
% code relative to its own file location, so MATLAB's current folder
% does not matter.

close all;
clear;
clc;


%% ====================================================================
%%  SETTINGS
%% ====================================================================

% Located from this script's own file location rather than a path
% relative to MATLAB's current folder: this script lives inside
% code/postprocessing/tests/, and data/ is code/'s sibling folder, so
% climb up to code/ and step across into data/.
this_script_folder    = fileparts(mfilename('fullpath'));
postprocessing_folder = fileparts(this_script_folder);
code_folder            = fileparts(postprocessing_folder);

data_root = fullfile(fileparts(code_folder), "data");

outils_folder = fullfile(postprocessing_folder, "outils");
addpath(outils_folder);

correction_window_s = 3.0;
N_disks_robot = 5;

% The dedicated straight/reference recording.
straight_config_folder = fullfile(data_root, "references", "straight_config");

% A small, representative spread of released recordings to compare
% against: two dynamic_motion trajectories and one quasi_static
% recording.
test_recordings = {
    fullfile(data_root, "dynamic_motion", "Lissajous_fast")
    fullfile(data_root, "dynamic_motion", "circle_fast")
    fullfile(data_root, "quasi_static",   "static_bend_x_100")
};
test_labels = {"Lissajous_fast", "circle_fast", "static_bend_x_100"};


%% ====================================================================
%%  LOCAL HELPERS
%% ====================================================================

function [r_mean, R_mean] = mean_disk_pose(folder, window_s, N_disks_robot)
    % Averages each disk's pose over the first window_s seconds of the
    % recording at folder (pass inf for the whole recording). Returns
    % the mean position (3 x N_disks_robot) and mean orientation as a
    % rotation matrix per disk (3 x 3 x N_disks_robot).
    filename = fullfile(folder, "dataOptiTrack.csv");
    [~, mocap_timestamps, ~, ~, rel_kinematics_disks] = data_optitrack(filename, false);

    mocap_time_rel = mocap_timestamps - mocap_timestamps(1);
    idx = mocap_time_rel <= window_s;

    r_mean = zeros(3, N_disks_robot);
    R_mean = zeros(3, 3, N_disks_robot);
    for it = 1:N_disks_robot
        EUL_t = rel_kinematics_disks(idx, 1:3, it)';
        r_t   = rel_kinematics_disks(idx, 4:6, it)';

        EUL_mean = mean(EUL_t, 2);
        r_mean(:, it) = mean(r_t, 2);
        R_mean(:, :, it) = eul2rotm(EUL_mean', 'XYZ');
    end
end

function [transl_mm, rot_deg] = pose_difference(r_A, R_A, r_B, R_B, N_disks_robot)
    % Per-disk translation (mm) and rotation (deg) difference between two
    % mean poses, both expressed in the same (per-recording) base frame.
    transl_mm = zeros(N_disks_robot, 1);
    rot_deg = zeros(N_disks_robot, 1);
    for it = 1:N_disks_robot
        transl_mm(it) = norm(r_A(:, it) - r_B(:, it)) * 1000;

        R_diff = R_A(:, :, it)' * R_B(:, :, it);
        axang = rotm2axang(R_diff);
        rot_deg(it) = rad2deg(abs(axang(4)));
    end
end


%% ====================================================================
%%  COMPUTE
%% ====================================================================

fprintf("Loading references/straight_config ...\n");
[r_straight_3s, R_straight_3s] = mean_disk_pose(straight_config_folder, correction_window_s, N_disks_robot);
[r_straight_full, R_straight_full] = mean_disk_pose(straight_config_folder, inf, N_disks_robot);

fprintf("\n=== straight_config: first %.0fs window vs full recording ===\n", correction_window_s);
[transl_mm, rot_deg] = pose_difference(r_straight_3s, R_straight_3s, r_straight_full, R_straight_full, N_disks_robot);
for it = 1:N_disks_robot
    fprintf("  disk %d: translation = %.3f mm, rotation = %.3f deg\n", it-1, transl_mm(it), rot_deg(it));
end

for k = 1:numel(test_recordings)
    fprintf("\nLoading %s ...\n", test_labels{k});
    [r_test, R_test] = mean_disk_pose(test_recordings{k}, correction_window_s, N_disks_robot);

    fprintf("=== %s (own first %.0fs) vs straight_config (first %.0fs) ===\n", ...
        test_labels{k}, correction_window_s, correction_window_s);
    [transl_mm, rot_deg] = pose_difference(r_test, R_test, r_straight_3s, R_straight_3s, N_disks_robot);
    for it = 1:N_disks_robot
        fprintf("  disk %d: translation = %.3f mm, rotation = %.3f deg\n", it-1, transl_mm(it), rot_deg(it));
    end

    fprintf("=== %s (own first %.0fs) vs straight_config (FULL recording) ===\n", ...
        test_labels{k}, correction_window_s);
    [transl_mm, rot_deg] = pose_difference(r_test, R_test, r_straight_full, R_straight_full, N_disks_robot);
    for it = 1:N_disks_robot
        fprintf("  disk %d: translation = %.3f mm, rotation = %.3f deg\n", it-1, transl_mm(it), rot_deg(it));
    end
end
