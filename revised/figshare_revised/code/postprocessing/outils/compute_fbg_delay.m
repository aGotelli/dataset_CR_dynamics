function compute_fbg_delay(data_root, align_window_s, FBGS_tip_index)
%COMPUTE_FBG_DELAY Measure the FBG pipeline delay relative to Mocap, and
%   save it for process_data.m to load.
%
%   For each of the four fast dynamic_motion trajectories (circle_fast,
%   Lissajous_fast, plane_x_fast, plane_y_fast), loads and spatially
%   aligns that recording's Mocap and FBG data (without applying any FBG
%   temporal correction), then cross-correlates Mocap, FBGS, motor angle
%   and tendon tension against each other over the recording's full
%   duration via check_temporal_sync. The Mocap<->FBGS lag on the axes
%   whose correlation exceeds r_OF_threshold is averaged into
%   mean_lag_OF, and the resulting value (sign-flipped into
%   process_data.m's lag_FBGS convention) is written to
%   measured_fbg_delay_ms.txt next to this function. Motor<->Mocap,
%   Motor<->FBGS and Motor<->Tendon are also characterized the same way
%   and included in the saved summary table; Motor<->ATI is not computed.
%
%   One row per trajectory is written to summary_fbg_delay.csv, and each
%   trajectory's own check_temporal_sync figures/sync_results.txt are
%   written under this function's figures/ folder.
%
%   data_root         - path to the dataset's data/ folder (set in
%                        process_data.m, passed in here so it isn't
%                        duplicated in multiple places)
%   align_window_s    - must match process_data.m's own value: this
%                        deliberately reproduces its spatial-alignment
%                        step
%   FBGS_tip_index     - must match process_data.m's own value: the FBG
%                        sample index used as "the tip" throughout


%% ====================================================================
%%  SETTINGS
%% ====================================================================

%   Self-locating from this function's own file location (outils/), used
%   to know where to write measured_fbg_delay_ms.txt and this function's
%   own figures/ output -- data location, window and tip index all come
%   from process_data.m via the arguments above so they can't drift out
%   of sync with it.
this_folder = fileparts(mfilename('fullpath'));

%   The |r_OF| threshold process_data.m itself uses to decide which axes'
%   Mocap->FBG lag are trustworthy enough to average.
r_OF_threshold = 0.95;

%   The four fast dynamic_motion trajectories. bending_axis (used only
%   for the log line below) follows the exact same rule
%   align_mocap_and_fbgs.m applies internally from the folder name
%   ("'y' for plane_y experiments, 'x' for all rest"). use_resense is
%   false for all four: none of these are contact recordings, so there
%   is no Resense wand to track.
trajectory_names  = {"circle_fast",  "Lissajous_fast", "plane_x_fast", "plane_y_fast"};
bending_axes      = {"x",            "x",              "x",            "y"};


%% ====================================================================
%%  ACCUMULATORS
%% ====================================================================

%   summary_table: one row per trajectory -- the per-axis Delta_OF lag and
%   r, which axes passed the |r| > 0.95 threshold, and the resulting
%   mean_lag_OF, plus the same characterization for Motor->Mocap,
%   Motor->FBGS and Motor->Tendon. This is the table saved to
%   summary_fbg_delay.csv.
summary_table = table();


%% ====================================================================
%%  PROCESS EACH TRAJECTORY
%% ====================================================================

for traj_index = 1:numel(trajectory_names)

    trajectory_name = trajectory_names{traj_index};
    bending_axis = bending_axes{traj_index};

    recording_folder = fullfile(data_root, "dynamic_motion", trajectory_name);

    fprintf("\n==== %s (bending_axis = ""%s"") ====\n", trajectory_name, bending_axis);

    %% ---- Load motor and tendon-tension data ---------------------------
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

    %% ---- Load and spatially align OptiTrack + FBG, WITHOUT the FBG ----
    %% ---- temporal (pipeline-delay) correction -------------------------
    use_resense = false;   % dynamic_motion recordings never have the Resense wand
    [~, mocap_timestamps, ~, rel_kinematics_disks_corr, ...
        fbgs_time_uncorrected, fbgs_shapes, ~, ~] = ...
        align_mocap_and_fbgs(recording_folder, use_resense, align_window_s);

    %% ---- Run the same synchronization check process_data.m uses, but --
    %% ---- feed it the UNCORRECTED FBG timestamps, over the recording's --
    %% ---- FULL duration (no windowing) -- exactly like process_data.m --
    %% ---- ATI is not loaded here, so it's passed as [] and skipped. ----
    trajectory_output_folder = fullfile(this_folder, "figures", "fbg_delay_" + trajectory_name);
    if ~isfolder(trajectory_output_folder)
        mkdir(trajectory_output_folder);
    end

    sync_results = check_temporal_sync(time_actuators, measured_angles, ...
        mocap_timestamps, rel_kinematics_disks_corr, ...
        fbgs_time_uncorrected, fbgs_shapes, FBGS_tip_index, ...
        time_cables, cable_tensions, trajectory_output_folder);

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

    %% ---- Motor, Mocap, FBGS and Tendon synchronization: characterize --
    %% ---- only, no correction -- same |r| > threshold + mean approach --
    %% ---- as above, applied to the other lag types check_temporal_sync -
    %% ---- computes (Motor<->ATI is skipped: check_temporal_sync was ---
    %% ---- called with [] for the ATI inputs, so lag_MA/r_MA come back --
    %% ---- as NaN and are not reported here). ---------------------------
    valid_MM = abs(sync_results.r_MM) > r_OF_threshold;
    mean_lag_MM = mean(sync_results.lag_MM(valid_MM));
    n_valid_MM = nnz(valid_MM);

    valid_MF = abs(sync_results.r_MF) > r_OF_threshold;
    mean_lag_MF = mean(sync_results.lag_MF(valid_MF));
    n_valid_MF = nnz(valid_MF);

    valid_MC = abs(sync_results.r_MC) > r_OF_threshold;
    mean_lag_MC = mean(sync_results.lag_MC(valid_MC));
    n_valid_MC = nnz(valid_MC);

    fprintf("  Motor->Mocap  : mean_lag_MM = %+.2f ms (%d/%d axes with |r| > %.2f)\n", ...
        mean_lag_MM, n_valid_MM, numel(sync_results.r_MM), r_OF_threshold);
    fprintf("  Motor->FBGS   : mean_lag_MF = %+.2f ms (%d/%d axes with |r| > %.2f)\n", ...
        mean_lag_MF, n_valid_MF, numel(sync_results.r_MF), r_OF_threshold);
    fprintf("  Motor->Tendon : mean_lag_MC = %+.2f ms (%d/%d tendons with |r| > %.2f)\n", ...
        mean_lag_MC, n_valid_MC, numel(sync_results.r_MC), r_OF_threshold);

    summary_table = [summary_table; ...
        make_summary_row(trajectory_name, lag_OF, r_OF, valid_axis_labels, mean_lag_OF, ...
            mean_lag_MM, n_valid_MM, mean_lag_MF, n_valid_MF, mean_lag_MC, n_valid_MC)];

end


%% ====================================================================
%%  SAVE THE TABLE
%% ====================================================================

output_folder = fullfile(this_folder, "figures");
if ~isfolder(output_folder)
    mkdir(output_folder);
end

summary_csv_path = fullfile(output_folder, "summary_fbg_delay.csv");
writetable(summary_table, summary_csv_path);
fprintf("\nSummary table (one row per trajectory) written to:\n  %s\n", summary_csv_path);


%% ====================================================================
%%  WRITE THE MEASURED DELAY FOR process_data.m TO LOAD
%% ====================================================================
%
%   Sign convention: check_temporal_sync.m's lag_OF(d) is the lag of FBG
%   relative to Mocap, computed by peak_lag(mocap, fbg, ...). Per how
%   peak_lag uses xcorr (see check_temporal_sync.m), that comes out
%   NEGATIVE when FBG is late relative to Mocap. process_data.m instead
%   defines lag_FBGS as the POSITIVE amount it SUBTRACTS from fbgs_time
%   (fbgs_time = fbgs_time - lag_FBGS/1000), so the sign is flipped here
%   to go from "check_temporal_sync's lag_OF convention" to
%   "process_data.m's lag_FBGS convention".

mean_lag_OF_all = mean(summary_table.Mean_Lag_OF_ms);
measured_lag_FBGS_ms = -mean_lag_OF_all;

lag_FBGS_file = fullfile(this_folder, "measured_fbg_delay_ms.txt");
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
fprintf("SUMMARY -- Temporal synchronization\n");
fprintf("====================================================================\n");
for row_index = 1:height(summary_table)
    row = summary_table(row_index, :);
    fprintf("%s: mean_lag_OF = %+.2f ms (axes used: %s)\n", ...
        row.Trajectory, row.Mean_Lag_OF_ms, row.ValidAxes);
    fprintf("   Motor->Mocap = %+.2f ms (%d axes), Motor->FBGS = %+.2f ms (%d axes), Motor->Tendon = %+.2f ms (%d tendons)\n", ...
        row.Mean_Lag_MM_ms, row.N_Valid_MM, row.Mean_Lag_MF_ms, row.N_Valid_MF, row.Mean_Lag_MC_ms, row.N_Valid_MC);
end

end


%% ====================================================================
%%  HELPER FUNCTIONS
%% ====================================================================

function one_row = make_summary_row(trajectory_name, lag_OF, r_OF, valid_axis_labels, mean_lag_OF, ...
        mean_lag_MM, n_valid_MM, mean_lag_MF, n_valid_MF, mean_lag_MC, n_valid_MC)
    %   Builds one row of the summary table: the per-axis Delta_OF lag
    %   and r (px, py, pz), which of those axes passed the |r| > 0.95
    %   threshold, and the resulting mean_lag_OF. Also includes the same
    %   characterization (mean lag over channels with |r| > threshold,
    %   and how many channels qualified out of how many exist) for
    %   Motor->Mocap, Motor->FBGS and Motor->Tendon.
    one_row = table( ...
        string(trajectory_name), ...
        lag_OF(1), r_OF(1), ...
        lag_OF(2), r_OF(2), ...
        lag_OF(3), r_OF(3), ...
        string(valid_axis_labels), ...
        mean_lag_OF, ...
        mean_lag_MM, n_valid_MM, ...
        mean_lag_MF, n_valid_MF, ...
        mean_lag_MC, n_valid_MC, ...
        'VariableNames', { ...
            'Trajectory', ...
            'Delta_OF_px_ms', 'Delta_OF_px_r', ...
            'Delta_OF_py_ms', 'Delta_OF_py_r', ...
            'Delta_OF_pz_ms', 'Delta_OF_pz_r', ...
            'ValidAxes', 'Mean_Lag_OF_ms', ...
            'Mean_Lag_MM_ms', 'N_Valid_MM', ...
            'Mean_Lag_MF_ms', 'N_Valid_MF', ...
            'Mean_Lag_MC_ms', 'N_Valid_MC'});
end
