%% compare_optitrack_calibration_5_22.m
%
% This script supports the discussion of Reviewer 5, Comment 5.22 (the
% OptiTrack calibration discrepancy between the manuscript and the code).
%
% THE QUESTION THIS SCRIPT ANSWERS
% ---------------------------------
% Section 2.6.3 of the manuscript ("Coordinate frame alignment") states
% that the per-disk residual-offset correction (removing static
% assembly/marker-placement error) is computed ONCE from
% references/straight_config/ (robot laser-verified straight) and then
% applied to every recording. The actual code (align_mocap_and_fbgs.m,
% "Correct pose mocap" block) instead recomputes this correction
% independently for every recording, from that recording's own first 3
% seconds -- references/straight_config/ is never read anywhere in this
% codebase. This is a real discrepancy between what is documented and
% what generated the released data (grep for "straight_config" across
% every .m file in this repository: zero matches outside this script).
%
% This script does NOT decide which procedure is "correct". It only asks
% a narrower, purely numerical question: how different would the
% per-disk correction be if it HAD been computed once from
% straight_config, compared to what each recording's own first-3-seconds
% estimate actually gives? If the difference is small compared to the
% robot's actual motion, a text-only fix to the manuscript (describe what
% the code does) is clearly adequate. If it is large, that would argue
% for actually changing the code to match the manuscript instead (a much
% bigger change: regenerating all 30 released recordings).
%
% THIS SCRIPT DOES NOT MODIFY process_data.m OR align_mocap_and_fbgs.m.
% The averaging step below is a DUPLICATE of the relevant ~15 lines of
% align_mocap_and_fbgs.m's "Correct pose mocap" block, copied here (not
% called from that file) specifically so this analysis cannot accidentally
% change pipeline behaviour. If that block is ever edited, this copy must
% be updated to match, or the comparison below stops being meaningful.
%
% WHAT THE SCRIPT PRODUCES
% -------------------------
% For each of three representative recordings (Lissajous_fast, circle_fast
% -- both already used in the Comment 5.30 analysis -- and
% static_bend_x_100, a quasi_static recording for variety), and for each
% of the 5 tracked robot disks: the translation and rotation difference
% between (a) the correction that recording's own first-3-seconds data
% would give (i.e. exactly what the released data actually used) and
% (b) the correction references/straight_config/ would give, computed two
% ways -- using only its own first 3 seconds (apples-to-apples with every
% other recording) and using its FULL duration (it is a dedicated static
% reference, so more data should only help). The straight_config
% first-3s-vs-full-duration comparison is also printed, as a sanity check
% on how stable straight_config's own estimate is.
%
% HOW TO RUN IT
% --------------
% Just run this script -- it locates the dataset and the postprocessing
% code relative to its own file location, so it does not matter what
% MATLAB's current folder happens to be when you press Run.

close all;
clear;
clc;


%% ====================================================================
%%  SETTINGS
%% ====================================================================

this_script_folder = fileparts(mfilename('fullpath'));
reviews_folder      = fileparts(this_script_folder);
repository_root     = fileparts(reviews_folder);

data_root = fullfile(repository_root, "revised", "figshare_revised", "data");

postprocessing_folder = fullfile(repository_root, "revised", ...
    "figshare_revised", "code", "postprocessing");
outils_folder = fullfile(postprocessing_folder, "outils");
addpath(outils_folder);

% Matches align_mocap_and_fbgs.m's hardcoded "idx_init = mocap_time_rel
% <= 3.0" window and its N_disks_robot = 5.
correction_window_s = 3.0;
N_disks_robot = 5;

% The reference recording the manuscript says the correction should come
% from.
straight_config_folder = fullfile(data_root, "references", "straight_config");

% A small, representative spread of released recordings to compare
% against: two dynamic_motion trajectories already used for Comment 5.30,
% plus one quasi_static recording for variety.
test_recordings = {
    fullfile(data_root, "dynamic_motion", "Lissajous_fast")
    fullfile(data_root, "dynamic_motion", "circle_fast")
    fullfile(data_root, "quasi_static",   "static_bend_x_100")
};
test_labels = {"Lissajous_fast", "circle_fast", "static_bend_x_100"};


%% ====================================================================
%%  LOCAL HELPER: average disk pose over a time window
%%  (duplicate of align_mocap_and_fbgs.m's "Correct pose mocap" block --
%%  see header comment above)
%% ====================================================================

function [r_mean, R_mean] = mean_disk_pose(folder, window_s, N_disks_robot)
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
    % mean poses, both already expressed in the same (per-recording) base
    % frame.
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

fprintf("\n=== SANITY CHECK: straight_config, first %.0fs window vs full recording ===\n", correction_window_s);
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
