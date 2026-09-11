%% check_fbg_delay_5_30.m
%
% This script supports the response to Reviewer 5, Comment 5.30.
%
% THE QUESTION THIS SCRIPT ANSWERS
% ---------------------------------
% process_data.m corrects every FBG timestamp by a fixed, hardcoded
% amount (lag_FBGS = 13.5 ms) before it ever computes a synchronization
% check. The reviewer pointed out that this mixes delay CORRECTION with
% delay ESTIMATION: whatever check_temporal_sync.m reports afterwards is
% measuring the RESIDUAL lag left over after the correction, not an
% independent measurement of the true delay.
%
% This script estimates that delay the other way around: on the raw,
% UNCORRECTED FBG timestamps. It reuses align_mocap_and_fbgs.m (outils/),
% the same spatial alignment code process_data.m itself now calls, but
% never applies the fbgs_time = fbgs_time - lag_FBGS/1000 correction that
% process_data.m applies afterwards.
%
% THE ANALYSIS ITSELF MATCHES process_data.m EXACTLY
% -----------------------------------------------------
% Earlier versions of this script invented their own way of picking "the"
% delay out of check_temporal_sync.m's three-axis result (largest |r|,
% triangle-closure checks, and so on). That was never how the rest of
% this codebase actually does it. process_data.m itself, right after its
% own call to check_temporal_sync, does this:
%
%     valid = find(abs(sync_results.r_OF) > 0.95);
%     lag_OF = sync_results.lag_OF;
%     mean_lag_OF = mean(lag_OF(valid));
%
% i.e. keep only the axes whose Mocap->FBG correlation is strong
% (|r| > 0.95) and average their lag. This script reproduces exactly that
% -- same threshold, same averaging -- on the uncorrected data, so the
% result is directly comparable to what process_data.m would report if it
% ran this same check before applying lag_FBGS instead of after.
%
% WHICH RECORDINGS THIS USES
% ----------------------------
% circle_fast, Lissajous_fast, plane_x_fast and plane_y_fast: the fast
% dynamic_motion trajectories, run over each recording's FULL duration
% (no windowing), exactly as process_data.m itself runs check_temporal_sync.
%
% WHAT THE SCRIPT PRODUCES
% -------------------------
% One row per trajectory, saved to summary_fbg_delay_5_30.csv: the three
% per-axis Delta_OF lags and r values check_temporal_sync.m computes, which
% axes pass the |r| > 0.95 threshold, and mean_lag_OF -- the number to cite
% in the response letter / manuscript. check_temporal_sync.m's own
% sync_results.txt and figures are written per trajectory under this
% script's own figures/ folder.
%
% While we are already computing sync_results for these four
% trajectories, the same call also returns the Motor->Mocap and
% Motor->FBGS lags (lag_MM, lag_MF). process_data.m does not currently
% correct for either of these -- they are reported here (same
% |r| > 0.95 + mean approach) purely to characterize whether
% motor-referenced synchronization needs attention too. No correction is
% added anywhere for them.
%
% Only Mocap<->FBGS, Mocap<->Motor and Motor<->FBGS are compared. Motor
% vs Mark10 (tendon tension) and Motor vs ATI are deliberately NOT
% included: those are force/torque measurements, not a second estimate
% of the rod's pose, so a lag between them and the motor angle is not
% the kind of pose-synchronization error this comment is about.
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

% Found the same way as the other review-response scripts in this
% folder: from this script's own file location, not from MATLAB's
% "current folder" setting (see audit_sampling_intervals_5_19.m for why
% that distinction matters).
this_script_folder = fileparts(mfilename('fullpath'));
reviews_folder      = fileparts(this_script_folder);
repository_root     = fileparts(reviews_folder);

data_root = fullfile(repository_root, "revised", "figshare_revised", "data");

% This must point at the SAME postprocessing/ tree that produced the
% released dataset in revised/figshare_revised/data (the one data_root,
% above, points at) -- not the older working copy under
% data_collection/dataCollectionPack/, which is a separate, out-of-date
% snapshot of this code.
postprocessing_folder = fullfile(repository_root, "revised", ...
    "figshare_revised", "code", "postprocessing");
outils_folder = fullfile(postprocessing_folder, "outils");
addpath(outils_folder);

% Must match process_data.m exactly, since we are deliberately
% reproducing its spatial-alignment step.
align_window_s = 10;
FBGS_tip_index = 480;

% The |r_OF| threshold process_data.m itself uses to decide which axes'
% Mocap->FBG lag are trustworthy enough to average.
r_OF_threshold = 0.95;

% The four fast dynamic_motion trajectories. bending_axis follows the
% exact same rule stated in process_data.m ("'y' for plane_y experiments,
% 'x' for all rest"). use_resense is false for all four: none of these
% are contact recordings, so there is no Resense wand to track.
trajectory_names  = {"circle_fast",  "Lissajous_fast", "plane_x_fast", "plane_y_fast"};
bending_axes      = {"x",            "x",              "x",            "y"};


%% ====================================================================
%%  ACCUMULATORS
%% ====================================================================

% summary_table: one row per trajectory -- the per-axis Delta_OF lag and
% r, which axes passed the |r| > 0.95 threshold, and the resulting
% mean_lag_OF, plus the same characterization (mean lag over axes with
% |r| > threshold, and how many axes qualified) for Motor->Mocap and
% Motor->FBGS. This is the table saved to summary_fbg_delay_5_30.csv.
summary_table = table();


%% ====================================================================
%%  PROCESS EACH TRAJECTORY
%% ====================================================================

for traj_index = 1:numel(trajectory_names)

    trajectory_name = trajectory_names{traj_index};
    bending_axis = bending_axes{traj_index};

    recording_folder = fullfile(data_root, "dynamic_motion", trajectory_name);

    fprintf("\n==== %s (bending_axis = ""%s"") ====\n", trajectory_name, bending_axis);

    %% ---- Load motor, cable and ATI data exactly as process_data.m does ----
    motor = readtable(fullfile(recording_folder, "dataMotor.csv"));
    time_actuators = motor.timestamp;
    measured_angles = [motor.rel_angle1_rad, motor.rel_angle2_rad, ...
        motor.rel_angle3_rad, motor.rel_angle4_rad];

    mk_1_x    = readtable(fullfile(recording_folder, "dataMark10_+x.csv"));
    mk_2_y    = readtable(fullfile(recording_folder, "dataMark10_+y.csv"));
    mk_1_negx = readtable(fullfile(recording_folder, "dataMark10_-x.csv"));
    mk_2_negy = readtable(fullfile(recording_folder, "dataMark10_-y.csv"));

    time_cables = cell(1,4);
    cable_tensions = cell(1,4);
    time_cables{1} = mk_1_x.timestamp;       cable_tensions{1} = mk_1_x.tension_N_/2;
    time_cables{2} = mk_2_y.timestamp;       cable_tensions{2} = mk_2_y.tension_N_/2;
    time_cables{3} = mk_1_negx.timestamp;    cable_tensions{3} = mk_1_negx.tension_N_/2;
    time_cables{4} = mk_2_negy.timestamp;    cable_tensions{4} = mk_2_negy.tension_N_/2;

    ati = readtable(fullfile(recording_folder, "dataATIFT.csv"));
    tA = ati.timestamp;
    ATI_F = [ati.Fx_N_, ati.Fy_N_, ati.Fz_N_];
    ATI_T = [ati.Tx_Nm_, ati.Ty_Nm_, ati.Tz_Nm_];
    ATI_FT = [ATI_F ATI_T];

    %% ---- Load and spatially align OptiTrack + FBG, WITHOUT the FBG ----
    %% ---- temporal (pipeline-delay) correction -------------------------
    use_resense = false;   % dynamic_motion recordings never have the Resense wand
    [~, mocap_timestamps, ~, rel_kinematics_disks_corr, ...
        fbgs_time_uncorrected, fbgs_shapes, ~, ~] = ...
        align_mocap_and_fbgs(recording_folder, use_resense, align_window_s, bending_axis);

    %% ---- Run the same synchronization check process_data.m uses, but --
    %% ---- feed it the UNCORRECTED FBG timestamps, over the recording's --
    %% ---- FULL duration (no windowing) -- exactly like process_data.m --
    trajectory_output_folder = fullfile(this_script_folder, "figures", "fbg_delay_" + trajectory_name);
    if ~isfolder(trajectory_output_folder)
        mkdir(trajectory_output_folder);
    end

    sync_results = check_temporal_sync(time_actuators, measured_angles, ...
        mocap_timestamps, rel_kinematics_disks_corr, ...
        fbgs_time_uncorrected, fbgs_shapes, FBGS_tip_index, ...
        time_cables, cable_tensions, tA, ATI_FT, ...
        trajectory_output_folder);

    %% ---- Exactly process_data.m's own averaging logic -----------------
    valid = find(abs(sync_results.r_OF) > r_OF_threshold);
    lag_OF = sync_results.lag_OF;
    r_OF = sync_results.r_OF;
    mean_lag_OF = mean(lag_OF(valid));

    axis_labels = {'px', 'py', 'pz'};   % char vectors, not string scalars -- strjoin needs char
    if isempty(valid)
        valid_axis_labels = "(none)";
    else
        valid_axis_labels = string(strjoin(axis_labels(valid), ", "));
    end

    fprintf("  Delta_OF per axis: px = %+.1f ms (r=%+.2f), py = %+.1f ms (r=%+.2f), pz = %+.1f ms (r=%+.2f)\n", ...
        lag_OF(1), r_OF(1), lag_OF(2), r_OF(2), lag_OF(3), r_OF(3));
    fprintf("  Axes with |r| > %.2f: %s\n", r_OF_threshold, valid_axis_labels);
    fprintf("  mean_lag_OF = %+.2f ms\n", mean_lag_OF);

    %% ---- Motor synchronization: characterize only, no correction -----
    %% ---- Same |r| > threshold + mean approach as above, applied to ---
    %% ---- the two other lag types check_temporal_sync.m computes that -
    %% ---- are actually pose estimates (Mocap and FBGS are both shape/ -
    %% ---- pose measurements of the rod; Mark10 and ATI are force/ -----
    %% ---- torque, not pose, so they are excluded here). process_data.m
    %% ---- does not correct for either of these; this only reports ----
    %% ---- whether it would need to. -------------------------------
    valid_MM = abs(sync_results.r_MM) > r_OF_threshold;
    mean_lag_MM = mean(sync_results.lag_MM(valid_MM));
    n_valid_MM = nnz(valid_MM);

    valid_MF = abs(sync_results.r_MF) > r_OF_threshold;
    mean_lag_MF = mean(sync_results.lag_MF(valid_MF));
    n_valid_MF = nnz(valid_MF);

    fprintf("  Motor->Mocap  : mean_lag_MM = %+.2f ms (%d/%d axes with |r| > %.2f)\n", ...
        mean_lag_MM, n_valid_MM, numel(sync_results.r_MM), r_OF_threshold);
    fprintf("  Motor->FBGS   : mean_lag_MF = %+.2f ms (%d/%d axes with |r| > %.2f)\n", ...
        mean_lag_MF, n_valid_MF, numel(sync_results.r_MF), r_OF_threshold);

    summary_table = [summary_table; ...
        make_summary_row(trajectory_name, lag_OF, r_OF, valid_axis_labels, mean_lag_OF, ...
            mean_lag_MM, n_valid_MM, mean_lag_MF, n_valid_MF)];

end


%% ====================================================================
%%  SAVE THE TABLE
%% ====================================================================

output_folder = fullfile(this_script_folder, "figures");
if ~isfolder(output_folder)
    mkdir(output_folder);
end

summary_csv_path = fullfile(output_folder, "summary_fbg_delay_5_30.csv");
writetable(summary_table, summary_csv_path);
fprintf("\nSummary table (one row per trajectory) written to:\n  %s\n", summary_csv_path);


%% ====================================================================
%%  WRITE THE MEASURED DELAY FOR process_data.m TO LOAD
%% ====================================================================
%
% process_data.m no longer hardcodes the FBG pipeline-delay correction.
% It loads it from the plain-text file written here, so that delay
% ESTIMATION (this script) and delay CORRECTION (process_data.m) stay in
% two separate files, as requested by the reviewer -- process_data.m
% never computes its own correction value, it only ever reads one that
% was measured here.
%
% Sign convention: check_temporal_sync.m's lag_OF(d) is the lag of FBG
% relative to Mocap, computed by peak_lag(mocap, fbg, ...). Per how
% peak_lag uses xcorr (see check_temporal_sync.m), that comes out
% NEGATIVE when FBG is late relative to Mocap -- which matches what this
% script measures above (all four trajectories give a negative
% Mean_Lag_OF_ms). process_data.m instead defines lag_FBGS as the
% POSITIVE amount it SUBTRACTS from fbgs_time
% (fbgs_time = fbgs_time - lag_FBGS/1000), so the sign is flipped here to
% go from "check_temporal_sync's lag_OF convention" to
% "process_data.m's lag_FBGS convention".

mean_lag_OF_all = mean(summary_table.Mean_Lag_OF_ms);
measured_lag_FBGS_ms = -mean_lag_OF_all;

lag_FBGS_file = fullfile(postprocessing_folder, "measured_fbg_delay_ms.txt");
fid = fopen(lag_FBGS_file, 'w');
fprintf(fid, '%.6f', measured_lag_FBGS_ms);
fclose(fid);

fprintf("\nMean lag_OF across the %d trajectories above: %+.2f ms\n", ...
    height(summary_table), mean_lag_OF_all);
fprintf("-> lag_FBGS (the value process_data.m will subtract) written to:\n  %s\n  value = %.4f ms\n", ...
    lag_FBGS_file, measured_lag_FBGS_ms);


%% ====================================================================
%%  FINAL SUMMARY
%% ====================================================================

fprintf("\n====================================================================\n");
fprintf("SUMMARY -- compare against Temporal synchronization in the manuscript\n");
fprintf("====================================================================\n");
for row_index = 1:height(summary_table)
    row = summary_table(row_index, :);
    fprintf("%s: mean_lag_OF = %+.2f ms (axes used: %s)\n", ...
        row.Trajectory, row.Mean_Lag_OF_ms, row.ValidAxes);
    fprintf("   Motor->Mocap = %+.2f ms (%d axes), Motor->FBGS = %+.2f ms (%d axes)\n", ...
        row.Mean_Lag_MM_ms, row.N_Valid_MM, row.Mean_Lag_MF_ms, row.N_Valid_MF);
end


%% ====================================================================
%%  LOCAL FUNCTIONS
%% ====================================================================

function one_row = make_summary_row(trajectory_name, lag_OF, r_OF, valid_axis_labels, mean_lag_OF, ...
        mean_lag_MM, n_valid_MM, mean_lag_MF, n_valid_MF)
    % Builds one row of the SUMMARY table: the per-axis Delta_OF lag and
    % r that check_temporal_sync.m computed (px, py, pz), which of those
    % axes passed process_data.m's own |r| > 0.95 threshold, and the
    % resulting mean_lag_OF -- the number to cite in the response letter
    % and manuscript. Also includes the same characterization (mean lag
    % over axes with |r| > threshold, and how many axes qualified out of
    % how many exist) for Motor->Mocap and Motor->FBGS -- reported for
    % completeness only; process_data.m does not correct for either of
    % them. Motor vs Mark10 and Motor vs ATI are not compared here: they
    % are force/torque signals, not a second pose estimate of the rod.
    one_row = table( ...
        string(trajectory_name), ...
        lag_OF(1), r_OF(1), ...
        lag_OF(2), r_OF(2), ...
        lag_OF(3), r_OF(3), ...
        string(valid_axis_labels), ...
        mean_lag_OF, ...
        mean_lag_MM, n_valid_MM, ...
        mean_lag_MF, n_valid_MF, ...
        'VariableNames', { ...
            'Trajectory', ...
            'Delta_OF_px_ms', 'Delta_OF_px_r', ...
            'Delta_OF_py_ms', 'Delta_OF_py_r', ...
            'Delta_OF_pz_ms', 'Delta_OF_pz_r', ...
            'ValidAxes', 'Mean_Lag_OF_ms', ...
            'Mean_Lag_MM_ms', 'N_Valid_MM', ...
            'Mean_Lag_MF_ms', 'N_Valid_MF'});
end
