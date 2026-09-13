%% compare_ati_resense_wrench.m
%
% Compares the ATI base force/torque sensor against the Resense contact
% wand, for the two contact_motion recordings that both have this
% cross-sensor check and are actually shown in the manuscript
% (push_retract, touching_base -- see RECORDINGS below; a third
% recording, touching_base_ang, also has both sensors but is not
% referenced anywhere in the manuscript, so it is intentionally left out
% of this comparison rather than reported on numbers nobody asked about).
% The Resense wand's measured wrench is transported from its own
% sensor frame to the robot base frame (via its mocap pose and a fixed
% sensor-to-mocap offset, Ad_g) and plotted against the ATI sensor's own
% base-frame reading, plus a couple of geometric sanity checks on the
% wand's tracked position.
%
% This script does not reprocess any raw sensor data itself: for each
% recording it reads base_wrench.csv, wrench_wand.csv and
% mocap_frames.csv straight from that recording's processed/ folder,
% i.e. exactly what process_data.m already produced for it (same
% filtering, resampling and mocap correction as the rest of the
% pipeline). It only performs the Ad_g transport locally, which
% process_data.m does not do -- that comparison is diagnostic, not
% something the released dataset needs. On top of process_data.m's own
% filtering, both wrenches are further low-pass filtered here
% (compare_cutoffHz, currently 2 Hz) for this comparison only;
% process_data.m's own saved CSVs are unaffected.
%
% CONTACT GATING: for most of any of these recordings the wand is not
% actually touching anything (it is retracted, or the recording has not
% reached the contact phase yet), during which both sensors read close
% to zero and agree trivially -- lumping those samples in with the real
% contact samples dilutes the RMSE with an easy, uninformative match.
% Each recording's contact window is instead identified geometrically,
% from the mocap-tracked distance between the wand sensor and whatever
% it is meant to be touching: the tip disk for push_retract, or the base
% origin for touching_base -- see CONTACT_REF below. The two recordings
% touch different things, so their distance thresholds
% (CONTACT_DIST_THRESHOLD) are set differently and are not both "disk
% radius": push_retract presses directly on the tip disk (outer diameter
% 80 mm, see Table~tab:robot_parameters in the manuscript, i.e. a 40 mm
% radius), so 50 mm (radius + 10 mm margin) is used there, consistent
% with the ~38 mm mean sensor-to-disk-centre distance already reported
% in the manuscript for this recording. touching_base instead presses on
% the mocap wand's own tracking stick near the base, not on the base
% disk itself, which sits much further from the base-frame origin than
% a disk radius would suggest (empirically, minimum ~65 mm rather than
% ~40 mm) -- so 100 mm is kept there, as before. Both the
% whole-recording and the contact-only statistics are computed and
% saved, but the contact-only ones are what should be quoted in the
% manuscript.
%
% Numerical summary (reviewer Comment 5.4 asked for RMSE, mean bias, max
% absolute error, and correlation, quantifying the qualitative
% "close/good agreement" language in the technical validation): computed
% for all 6 wrench components, but ONLY the 3 force components are
% reported in the manuscript. Torque is deliberately excluded from the
% quantitative report: the moment-arm term (r x F) in the Ad_g transport
% amplifies any sub-cm uncertainty in the wand's fixed sensor offset
% (g_fix) by the applied force, so torque residuals grow with contact
% force. This is not a simple, correctable sign error either -- checked
% by testing a moment-arm sign flip, and separately by decomposing the
% transported torque into its own-rotation and moment-arm-cross pieces:
% which torque axis disagrees in sign is inconsistent across recordings
% (including touching_base_ang, a recording with both sensors that is
% not part of this script's RECORDINGS -- see the note above), so no
% single correction fixes it everywhere. That points to a real
% calibration issue in the wand's fixed sensor offset (g_fix) rather
% than a code bug, and is out of scope for this revision.
% Torque RMSE is still computed and saved per recording, for anyone who
% wants to look, but it is not part of the reported validation numbers.
%
% Per-recording outputs, in that recording's own processed/ folder
% (alongside the data they were computed from -- this script does not
% touch or recompute any of process_data.m's own saved CSVs):
%   wrench_RMSEs.txt   - all 6 components' whole-recording RMSE, plus the
%                        forces-only bias/max-abs-error/correlation
%                        block, both whole-recording and contact-only
% Combined output, in this script's own folder (spans multiple
% recordings, so it does not belong inside any one recording's
% processed/ folder):
%   contact_force_validation_summary.txt - forces-only, contact-only
%                        RMSE/bias/max abs error/correlation for every
%                        recording in RECORDINGS, side by side -- this
%                        is what the manuscript table (Comment 5.4) is
%                        built from.
% Plots are saved per recording into this script's own figures/<recording>/
% folder; the distance plot marks the contact threshold and the
% resulting contact mask.
%
% process_data.m must have been run for every recording in RECORDINGS
% first, so that each one's processed/ folder contains base_wrench.csv,
% wrench_wand.csv and mocap_frames.csv.
%
% Run this script directly; it locates the dataset relative to its own
% file location, so MATLAB's current folder does not matter.

close all;
clear;
clc;

%% ====== PATHS / SETTINGS ======

% Located from this script's own file location rather than a path
% relative to MATLAB's current folder: this script lives inside
% code/postprocessing/tests/, and data/ is code/'s sibling folder, so
% climb up to code/ and step across into data/.
this_script_folder = fileparts(mfilename('fullpath'));
code_folder = fileparts(fileparts(this_script_folder));
data_root = fullfile(fileparts(code_folder), "data");

%   The two contact_motion recordings referenced in the manuscript's
%   contact-force validation section (fig:wrench_push_retract,
%   fig:wrench_touch_base). touching_base_ang also has both an ATI base
%   sensor and a Resense contact wand, but it is not shown in the
%   manuscript, so it is intentionally left out here -- see the header
%   comment.
recordings = ["push_retract", "touching_base"];

%   What each recording's wand is actually touching, and how close (in
%   metres, 3D) it has to be for that to count as contact -- see the
%   CONTACT GATING note above. push_retract pushes/retracts directly
%   against the tip disk (40 mm radius + 10 mm margin = 0.05 m);
%   touching_base presses on the mocap wand's own tracking stick near
%   the base rather than on the base disk itself, so its threshold is
%   kept at the larger, empirically-checked 0.10 m used previously.
contact_refs       = ["tip",  "base"];
contact_thresholds = [0.05,   0.10 ];

% Number of tracked OptiTrack disks for these recordings: the 5 robot
% disks plus the Resense wand (disk index 6, 1-based) -- see
% outils/data_optitrack.m.
N_disks = 6;
wand_disk_index = 6;
tip_disk_index = 5;

%   process_data.m's own filtering (cutoffHz there, currently 15 Hz) is
%   tuned for the released dataset. The raw ATI-vs-Resense comparison is
%   noisy enough at that cutoff that a stronger filter is applied here,
%   on top of it, purely for this diagnostic comparison -- it does not
%   change anything process_data.m saves. Applied identically to every
%   recording in RECORDINGS so the numbers stay comparable across them.
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

    required_files = ["base_wrench.csv", "wrench_wand.csv", "mocap_frames.csv"];
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

    mocap_csv = readmatrix(fullfile(processed_folder, "mocap_frames.csv"));
    interp_rel_kinematics_disks_corr = reshape(mocap_csv(:, 2:end), [N_samples, 6, N_disks]);

    % The wand has no residual-offset correction of its own (see
    % align_mocap_and_fbgs.m); this is its raw mocap pose, carried
    % through unchanged into mocap_frames.csv.
    wand_pose = interp_rel_kinematics_disks_corr(:, :, wand_disk_index);


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
    %   All 6 components' RMSE, whole recording (diagnostic only -- see
    %   header comment). Forces-only bias/max-abs-error/correlation are
    %   computed twice: once over the whole recording (diagnostic), and
    %   once restricted to CONTACT_MASK (the numbers to actually quote).

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

    %   Per-recording file, in the recording's own processed/ folder,
    %   alongside the data it was computed from. All 6 components'
    %   whole-recording RMSE are kept here as a diagnostic (including
    %   torque), even though only the contact-only forces are reported
    %   in the manuscript -- see the header comment at the top of this
    %   file.
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

    %   Combined cross-recording summary (forces only, contact-only),
    %   for the manuscript table -- this spans multiple recordings, so
    %   it lives next to this script rather than inside any one
    %   recording's processed/ folder.
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


    % Robot tip (disk 5) vs the wand sensor's transported position, as a
    % geometric sanity check on the Ad_g offset (g_fix, inside
    % compute_wrench_at_base below).
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


    % 3D distance between the wand sensor and its contact reference
    % (robot tip or base origin, see CONTACT_REF), with the contact
    % threshold marked -- this is exactly what CONTACT_MASK is built
    % from, so it doubles as a sanity check on the gating.
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
