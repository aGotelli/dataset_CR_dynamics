%% compare_ati_resense_wrench.m
%
% Compares the ATI base wrench (mini40) against the Resense contact wand
% wrench, transported to the base frame via Ad_g.
%
% Reads process_data.m's saved base_wrench.csv, wrench_wand.csv and
% mocap_frames.csv for each recording. An extra low-pass filter is applied 
% on top of process_data.m's own filtering, for this comparison only.
%
% Contact gating: for most of a recording the wand isn't touching
% anything, where both sensors trivially agree near zero. Contact
% samples are identified from the 3D distance between the wand sensor
% and its contact reference (tip disk for push_retract, base-frame
% origin for touching_base), thresholded at CONTACT_THRESHOLDS below.
%
% Reported metrics: RMSE, bias, max abs error and Pearson r, for Fx/Fy only, 
% contact samples only. Fz is excluded.
%
% Outputs: wrench_RMSEs.txt per recording (in its own processed/
% folder), plus contact_force_validation_summary.txt (in this script's
% folder, forces/contact-only.
%
% Requires process_data.m to have already been run for every recording
% in RECORDINGS. Run directly; paths are relative to this file, not
% MATLAB's current folder.

close all;
clear;
clc;

%% ====== PATHS / SETTINGS ======

% This script lives in code/postprocessing/tests/; data/ is code/'s
% sibling folder.
this_script_folder = fileparts(mfilename('fullpath'));
code_folder = fileparts(fileparts(this_script_folder));
data_root = fullfile(fileparts(code_folder), "data");

% Load the two recordings of interest
recordings = ["push_retract", "touching_base"];

% Contact reference point and distance threshold (m) per recording.
% push_retract: wand presses the tip disk directly (40 mm radius +
% 10 mm margin = 0.05 m). touching_base: wand presses against the base
% mocap y bracket, empirically-checked distance threshold of 0.10 m
contact_refs       = ["tip",  "base"];
contact_thresholds = [0.05,   0.10 ];

% 5 robot disks; the Resense wand pose is loaded separately from its own
% wand_pose.csv (see LOAD process_data.m's SAVED OUTPUT below) rather
% than as a 6th disk block in mocap_frames.csv.
N_disks_robot = 5;
tip_disk_index = 5;

% Extra low-pass filter on top of process_data.m's own (15 Hz), for
% this comparison only
compare_cutoffHz = 2;
compare_butterOrder = 4;

wrench_labels = ["Fx", "Fy", "Fz", "Tx", "Ty", "Tz"];
wrench_units  = ["N", "N", "N", "Nm", "Nm", "Nm"];
force_idx = 1:3;

summary_fid = fopen(fullfile(this_script_folder, "contact_force_validation_summary.txt"), 'w');
fprintf(summary_fid, "ATI vs Ad_g-transported Resense -- forces only, contact samples only (Comment 5.4)\n");
fprintf(summary_fid, "Comparison low-pass filter: %d Hz, order %d\n\n", compare_cutoffHz, compare_butterOrder);


%% ====== PER-RECORDING LOOP ======

for ir = 1:numel(recordings)

    recording = recordings(ir);
    contact_ref = contact_refs(ir);
    contact_threshold = contact_thresholds(ir);

    folder = fullfile(data_root, "contact_motion", recording);
    processed_folder = fullfile(folder, "processed");

    saving_fig_folder = fullfile(this_script_folder, "figures", recording) + filesep;
    if ~isfolder(saving_fig_folder)
        mkdir(saving_fig_folder);
    end


    %% ------ LOAD process_data.m's SAVED OUTPUT FOR THIS RECORDING ------

    required_files = ["base_wrench.csv", "wrench_wand.csv", "mocap_frames.csv", "wand_pose.csv"];
    for f = required_files
        if ~isfile(fullfile(processed_folder, f))
            error(['Missing %s in:\n%s\n' ...
                'Run process_data.m for this recording first (set its ' ...
                'folder to contact_motion/%s).'], f, char(processed_folder), recording);
        end
    end

    base_wrench_csv = readmatrix(fullfile(processed_folder, "base_wrench.csv"));
    sampling_time      = base_wrench_csv(:, 1);
    interp_base_wrench = base_wrench_csv(:, 2:end);
    N_samples = numel(sampling_time);

    wrench_wand_csv    = readmatrix(fullfile(processed_folder, "wrench_wand.csv"));
    interp_wrench_wand = wrench_wand_csv(:, 2:end);

    % Get the robot disks' poses (5 disks only -- the wand is not one of
    % them, see wand_pose.csv below)
    mocap_csv = readmatrix(fullfile(processed_folder, "mocap_frames.csv"));
    interp_rel_kinematics_disks_corr = reshape(mocap_csv(:, 2:end), [N_samples, 6, N_disks_robot]);

    % Load the wand pose, saved by process_data.m as its own file
    wand_pose_csv = readmatrix(fullfile(processed_folder, "wand_pose.csv"));
    wand_pose = wand_pose_csv(:, 2:end);


    %% ------ TRANSPORT THE RESENSE WRENCH TO THE ROBOT BASE FRAME (Ad_g) ------
    wrench_at_base = compute_wrench_at_base(wand_pose, interp_wrench_wand);


    %% ------ CONTACT MASK (see CONTACT GATING note at the top of this file) ------

    % Position of the Resense sensor frame's origin (wand mocap pose
    % shifted by the fixed offset g_fix), needed both for the contact
    % mask and for the geometric sanity-check plots further down.
    pos_sensor = wand_sensor_position(wand_pose);

    if contact_ref == "tip"
        % push_retract: contact happens at the robot's tip (disk 5).
        ref_pos = squeeze(interp_rel_kinematics_disks_corr(:, 4:6, tip_disk_index))';  % 3 x N_samples
    else % "base"
        % touching_base / touching_base_ang: contact happens near the
        % fixed base, i.e. the origin of the base-relative mocap frame
        % that all of interp_rel_kinematics_disks_corr is expressed in.
        ref_pos = zeros(3, N_samples);
    end
    dist_contact = vecnorm(pos_sensor - ref_pos, 2, 1);   % 1 x N_samples, 3D distance
    contact_mask = (dist_contact < contact_threshold)';   % N_samples x 1


    %% ------ EXTRA LOW-PASS FILTER, FOR THIS COMPARISON ONLY ------

    for it = 1:6
        interp_base_wrench(:, it) = butter_filtfilt(sampling_time, interp_base_wrench(:, it), compare_cutoffHz, compare_butterOrder);
        wrench_at_base(it, :) = butter_filtfilt(sampling_time, wrench_at_base(it, :)', compare_cutoffHz, compare_butterOrder)';
    end


    %% ------ NUMERICAL SUMMARY ------
    %   Forces-only bias/max-abs-err/correlation, computed both over the
    %   whole recording (diagnostic) and restricted to CONTACT_MASK (the
    %   numbers to quote). All 6 components' RMSE is also kept, whole
    %   recording only, as a diagnostic.

    rmse_wrench = rmse(wrench_at_base', interp_base_wrench);

    %   Signed residual, Ad_g-transported Resense minus ATI: positive
    %   means Resense reads higher than ATI on average.
    error_wrench = wrench_at_base' - interp_base_wrench;   % N_samples x 6

    [bias_all, max_abs_err_all, corr_all] = force_stats(wrench_at_base', interp_base_wrench, error_wrench, true(N_samples, 1), force_idx);
    [bias_c,   max_abs_err_c,   corr_c]   = force_stats(wrench_at_base', interp_base_wrench, error_wrench, contact_mask,       force_idx);
    rmse_c = rmse(wrench_at_base(force_idx, contact_mask)', interp_base_wrench(contact_mask, force_idx));

    N_contact = nnz(contact_mask);
    contact_frac = N_contact / N_samples * 100;

    fprintf("=== %s (contact ref: %s, threshold %.2f m, %d/%d samples in contact, %.1f%%) ===\n", ...
        recording, contact_ref, contact_threshold, N_contact, N_samples, contact_frac);
    fprintf("ATI vs Ad_g-transported Resense, whole-recording RMSE (all 6, diagnostic):\n");
    for it = 1:6
        fprintf("  %s = %.4f %s\n", wrench_labels(it), rmse_wrench(it), wrench_units(it));
    end
    fprintf("Forces only, CONTACT SAMPLES ONLY (Comment 5.4 -- quote these):\n");
    for k = 1:numel(force_idx)
        it = force_idx(k);
        fprintf("  %s: RMSE = %.4f %s, bias = %+.4f %s, max_abs_err = %.4f %s, r = %.4f\n", ...
            wrench_labels(it), rmse_c(k), wrench_units(it), bias_c(k), wrench_units(it), max_abs_err_c(k), wrench_units(it), corr_c(k));
    end
    fprintf("\n");

    %   Per-recording file, alongside the data it was computed from.
    fid = fopen(fullfile(processed_folder, "wrench_RMSEs.txt"), 'w');
    fprintf(fid, "ATI vs Ad_g-transported Resense (%d Hz comparison filter)\n", compare_cutoffHz);
    fprintf(fid, "Contact ref: %s, threshold %.2f m, %d/%d samples in contact (%.1f%%)\n\n", ...
        contact_ref, contact_threshold, N_contact, N_samples, contact_frac);
    fprintf(fid, "Whole-recording RMSE, all 6 components (diagnostic only):\n");
    for it = 1:6
        fprintf(fid, "RMSE_%s = %.4f %s\n", wrench_labels(it), rmse_wrench(it), wrench_units(it));
    end
    fprintf(fid, "\nForces only, WHOLE RECORDING (diagnostic -- diluted by no-contact samples):\n");
    for k = 1:numel(force_idx)
        it = force_idx(k);
        fprintf(fid, "%s: RMSE = %.4f %s, bias = %+.4f %s, max_abs_err = %.4f %s, r = %.4f\n", ...
            wrench_labels(it), rmse_wrench(it), wrench_units(it), bias_all(k), wrench_units(it), max_abs_err_all(k), wrench_units(it), corr_all(k));
    end
    fprintf(fid, "\nForces only, CONTACT SAMPLES ONLY (Comment 5.4 -- quote these):\n");
    for k = 1:numel(force_idx)
        it = force_idx(k);
        fprintf(fid, "%s: RMSE = %.4f %s, bias = %+.4f %s, max_abs_err = %.4f %s, r = %.4f\n", ...
            wrench_labels(it), rmse_c(k), wrench_units(it), bias_c(k), wrench_units(it), max_abs_err_c(k), wrench_units(it), corr_c(k));
    end
    fclose(fid);

    %   Combined cross-recording summary, for the manuscript table.
    fprintf(summary_fid, "%s (contact ref: %s, threshold %.2f m, %d/%d samples in contact, %.1f%%):\n", ...
        recording, contact_ref, contact_threshold, N_contact, N_samples, contact_frac);
    for k = 1:numel(force_idx)
        it = force_idx(k);
        fprintf(summary_fid, "  %s: RMSE = %.4f %s, bias = %+.4f %s, max_abs_err = %.4f %s, r = %.4f\n", ...
            wrench_labels(it), rmse_c(k), wrench_units(it), bias_c(k), wrench_units(it), max_abs_err_c(k), wrench_units(it), corr_c(k));
    end
    fprintf(summary_fid, "\n");


    %% ------ PLOTS ------

    % ATI vs Resense-transported-to-base (Ad_g), all 6 wrench components.
    fig = figure("Name", "Forces (Ad_g)");
    force_labels = {'F_x [N]', 'F_y [N]', 'F_z [N]'};
    for it = 1:3
        subplot(3, 1, it)
        plot(sampling_time, interp_base_wrench(:, it), 'b')
        hold on
        plot(sampling_time, wrench_at_base(it, :), 'r')
        ylabel(force_labels{it})
        grid on
        if it == 3, xlabel("Time [s]"); end
    end
    legend('ATI', 'Ad_g Resense')
    savefig(saving_fig_folder + fig.Name)
    saveas(fig, saving_fig_folder + fig.Name, 'png')

    fig = figure("Name", "Torques (Ad_g)");
    torque_labels = {'T_x [Nm]', 'T_y [Nm]', 'T_z [Nm]'};
    for it = 1:3
        subplot(3, 1, it)
        plot(sampling_time, interp_base_wrench(:, 3 + it), 'b')
        hold on
        plot(sampling_time, wrench_at_base(3 + it, :), 'r')
        ylabel(torque_labels{it})
        grid on
        if it == 3, xlabel("Time [s]"); end
    end
    legend('ATI', 'Ad_g Resense')
    savefig(saving_fig_folder + fig.Name)
    saveas(fig, saving_fig_folder + fig.Name, 'png')


    % Tip disk vs wand sensor position -- sanity check on the g_fix offset.
    fig = figure("Name", "Position disk and wand");
    pos_labels = {'p_x [m]', 'p_y [m]', 'p_z [m]'};
    for it = 1:3
        subplot(3, 1, it)
        plot(sampling_time, interp_rel_kinematics_disks_corr(:, 3 + it, tip_disk_index), 'b', 'LineWidth', 1)
        hold on
        plot(sampling_time, pos_sensor(it, :), 'r', 'LineWidth', 1)
        ylabel(pos_labels{it})
        grid on
        if it == 3, xlabel("Time [s]"); end
    end
    legend('robot tip', 'wand')
    savefig(saving_fig_folder + fig.Name)
    saveas(fig, saving_fig_folder + fig.Name, 'png')


    % Distance to the contact reference -- what CONTACT_MASK is built from.
    fig = figure("Name", "Distance sensor from contact reference");
    plot(sampling_time, dist_contact, 'b', 'LineWidth', 1)
    hold on
    yline(contact_threshold, 'r--', 'LineWidth', 1)
    ylabel('Distance [m]')
    legend('distance to ' + contact_ref, 'contact threshold')
    grid on
    xlabel("Time [s]")
    savefig(saving_fig_folder + fig.Name)
    saveas(fig, saving_fig_folder + fig.Name, 'png')

end

fclose(summary_fid);


%% ====== LOCAL FUNCTIONS ======

function [bias_f, max_abs_err_f, corr_f] = force_stats(wrench_at_base_t, interp_base_wrench, error_wrench, mask, force_idx)
    %   Mean signed bias, max absolute error, and Pearson correlation,
    %   for the given force components (force_idx into the 6-wide
    %   wrench), restricted to the samples where mask is true.
    %
    %   wrench_at_base_t, interp_base_wrench, error_wrench : N_samples x 6
    %   mask                                                : N_samples x 1 logical

    n = numel(force_idx);
    bias_f = zeros(1, n);
    max_abs_err_f = zeros(1, n);
    corr_f = zeros(1, n);
    for k = 1:n
        it = force_idx(k);
        bias_f(k) = mean(error_wrench(mask, it));
        max_abs_err_f(k) = max(abs(error_wrench(mask, it)));
        cc = corrcoef(wrench_at_base_t(mask, it), interp_base_wrench(mask, it));
        corr_f(k) = cc(1, 2);
    end
end

function wrench_at_base = compute_wrench_at_base(disk_kinematics_wand, wrench_wand)
    %   Transports the Resense HEX12 wand wrench from its own sensor
    %   frame to the robot base frame, via the wand's mocap pose and the
    %   wand's fixed sensor-to-mocap-frame offset (g_fix).
    %
    %   disk_kinematics_wand : N_samples x 6 [roll pitch yaw px py pz],
    %                          the wand's own mocap pose over time
    %   wrench_wand           : N_samples x 6 [Fx Fy Fz Tx Ty Tz], the
    %                          wand's own measured wrench over time
    %   wrench_at_base         : 6 x N_samples

    N_samples = size(disk_kinematics_wand, 1);
    g_fix = wand_sensor_offset();

    wrench_at_base = zeros(6, N_samples);
    for it_t = 1:N_samples
        wand_XYZ_xyz = disk_kinematics_wand(it_t, :);

        R = eul2rotm(wand_XYZ_xyz(1:3), 'XYZ');
        r = wand_XYZ_xyz(4:6)';

        g = [
          R     r
          0 0 0 1
        ];

        g_s = g*g_fix;
        R_s = g_s(1:3, 1:3);
        r_s = g_s(1:3, 4);
        wrench_wand_it_t = wrench_wand(it_t, :)';

        Ad_g_=[R_s zeros(3,3)
                hat_(r_s)*R_s R_s];

        %   Equivalent wrench at the base, by the action-reaction principle.
        wrench_at_base(:, it_t) = -Ad_g_*wrench_wand_it_t;
    end
end

function pos_sensor = wand_sensor_position(disk_kinematics_wand)
    %   Position (3 x N_samples) of the Resense sensor frame's origin,
    %   i.e. the wand's mocap pose shifted by the same fixed offset
    %   (g_fix) compute_wrench_at_base uses for the wrench transport.

    N_samples = size(disk_kinematics_wand, 1);
    g_fix = wand_sensor_offset();

    pos_sensor = zeros(3, N_samples);
    for it_t = 1:N_samples
        wand_XYZ_xyz = disk_kinematics_wand(it_t, :);

        R = eul2rotm(wand_XYZ_xyz(1:3), 'XYZ');
        r = wand_XYZ_xyz(4:6)';

        g = [
          R     r
          0 0 0 1
        ];

        g_s = g*g_fix;
        pos_sensor(:, it_t) = g_s(1:3, 4);
    end
end

function g_fix = wand_sensor_offset()
    %   Fixed rigid-body offset between the wand's mocap frame and its
    %   Resense sensor frame.

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

function y = butter_filtfilt(t, x, fc, n)
    % Zero-phase Butterworth low-pass filtering, robust to irregular
    % sampling. Same implementation as process_data.m's own local
    % function of the same name (kept local here too, rather than
    % shared, per this script's own local-functions convention).
    %
    % It estimates Fs from the mean inter-sample interval, resample the
    % signal onto a uniform grid at that rate before filtering

    Fs = (numel(t) - 1) / (t(end) - t(1));            % mean-based rate
    t_uniform = linspace(t(1), t(end), numel(t))';    % regular grid, same span & count
    x_uniform = interp1(t, x, t_uniform, 'linear');
    [b, a] = butter(n, fc/(Fs/2), "low");
    y_uniform = filtfilt(b, a, x_uniform);
    y = interp1(t_uniform, y_uniform, t, 'linear');   % back onto original timestamps
end
