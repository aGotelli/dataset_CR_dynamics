function compute_sensors_delay(data_root, align_window_s, FBGS_tip_index)
%COMPUTE_FBG_DELAY Measure the FBG pipeline delay relative to Mocap, and
%   save it for process_data.m to load.
%
%   For each of the four fast dynamic_motion trajectories (circle_fast,
%   Lissajous_fast, plane_x_fast, plane_y_fast), loads and spatially
%   aligns that recording's Mocap and FBG data, then cross-correlates 
%   Mocap, FBGS and motor angle against each other over the recording's 
%   full duration via check_temporal_sync. 
%   The Mocap<->FBGS lag on the axes whose correlation exceeds 
%   r_OF_threshold is averaged into mean_lag_OF, and the resulting value 
%   (sign-flipped into process_data.m's lag_FBGS convention). Motor<->Mocap 
%   and Motor<->FBGS are also reported (characterization only, no 
%   correction applied for either -- see below). Averaged vlues against 
%   the four trajectories are written to 
%   data/postprocess_calibration/measured_sensors_delay_ms.txt.

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

%   The |r_OF| threshold process_data.m itself uses to decide which axes'
%   Mocap->FBG lag are trustworthy enough to average.
r_OF_threshold = 0.95;

%   The four fast dynamic_motion trajectories. bending_axis (used only
%   for the log line below) follows the exact same rule
%   align_mocap_and_fbgs.m applies internally from the folder name
%   ("'y' for plane_y experiments, 'x' for all rest"). None of these are
%   contact recordings, so there is no Resense wand to load separately
%   here either.
trajectory_names  = {"circle_fast",  "Lissajous_fast", "plane_x_fast", "plane_y_fast"};
bending_axes      = {"x",            "x",              "x",            "y"};

%   Restricts the Motor->Mocap / Motor->FBGS diagnostic below to the
%   (motor, mocap-axis) pair each trajectory actually excites, matching
%   Section 2.6's stated rule: "only the axis predominantly excited by
%   each trajectory is used, to avoid kinematic coupling artifacts on
%   passive axes." circle_fast/Lissajous_fast drive both tendon pairs and
%   genuinely excite multiple axes, so every (motor, axis) combination
%   stays a candidate for those two ([] below). plane_x_fast/plane_y_fast
%   drive a single tendon pair; without this restriction, the blanket
%   |r| > threshold selection used for the other two trajectories also
%   picks up a spurious, coincidental correlation between the driven
%   motor and a passive axis it doesn't actually excite (e.g. tens of ms,
%   nothing like the sub-3ms lags the genuinely-excited pairs show), which
%   is not a synchronization delay. Motor 1 = M+x / axis 1 = px, Motor 2 =
%   M+y / axis 2 = py (see check_temporal_sync.m's mot_lbl/pos_lbl).
dominant_MM_pair  = {[],             [],               [1 1],          [2 2]};


%% ====================================================================
%%  ACCUMULATORS
%% ====================================================================

%   summary_table: one row per trajectory -- the per-axis Delta_OF lag and
%   r, which axes passed the |r| > 0.95 threshold, and the resulting
%   mean_lag_OF, plus the same characterization for Motor->Mocap and
%   Motor->FBGS. Held in memory only, to average across trajectories
%   below -- not saved to disk.
summary_table = table();


%% ====================================================================
%%  PROCESS EACH TRAJECTORY
%% ====================================================================

for traj_index = 1:numel(trajectory_names)

    trajectory_name = trajectory_names{traj_index};
    bending_axis = bending_axes{traj_index};
    mm_pair = dominant_MM_pair{traj_index};

    recording_folder = fullfile(data_root, "dynamic_motion", trajectory_name);

    fprintf("\n==== %s (bending_axis = ""%s"") ====\n", trajectory_name, bending_axis);

    %% ---- Load motor data -----------------------------------------------
    motor = readtable(fullfile(recording_folder, "dataMotor.csv"));
    time_actuators = motor.timestamp;
    measured_angles = [motor.rel_angle1_rad, motor.rel_angle2_rad, ...
        motor.rel_angle3_rad, motor.rel_angle4_rad];

    %% ---- Load and spatially align OptiTrack + FBG, WITHOUT the FBG ----
    %% ---- temporal (pipeline-delay) correction -------------------------
    %   align_mocap_and_fbgs only handles the robot's 5 disks now (the
    %   Resense wand, when present, is loaded separately by process_data.m
    %   -- see align_mocap_and_fbgs.m), so it no longer takes a
    %   use_resense flag. has_fbgs_data must be true here: this function's
    %   whole purpose is cross-correlating FBG against mocap, and all four
    %   trajectories it uses do have a dataFBGS.csv, but it is still
    %   detected the same way process_data.m does rather than hardcoded.
    has_fbgs_data = isfile(fullfile(recording_folder, "dataFBGS.csv"));
    [mocap_timestamps, ~, rel_kinematics_disks_corr, ...
        fbgs_time_uncorrected, fbgs_shapes, ~, ~] = ...
        align_mocap_and_fbgs(recording_folder, has_fbgs_data, align_window_s, data_root);

    %% ---- Run the same synchronization check process_data.m uses, but --
    %% ---- feed it the UNCORRECTED FBG timestamps, over the recording's --
    %% ---- FULL duration (no windowing) -- exactly like process_data.m --
    %% ---- sync_results.txt is not saved (saving_folder = '') -- only --
    %% ---- the final measured delay, below, is written to disk. --------
    sync_results = check_temporal_sync(time_actuators, measured_angles, ...
        mocap_timestamps, rel_kinematics_disks_corr, ...
        fbgs_time_uncorrected, fbgs_shapes, FBGS_tip_index, '');

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

    %% ---- Motor, Mocap and FBGS synchronization: characterize only, ----
    %% ---- no correction -- same |r| > threshold + mean approach as -----
    %% ---- above, applied to the other lag types check_temporal_sync ----
    %% ---- computes, restricted to each trajectory's dominant (motor, --
    %% ---- axis) pair for the single-DOF planar trajectories -- see -----
    %% ---- dominant_MM_pair above. --------------------------------------
    if isempty(mm_pair)
        mm_candidate_mask = true(size(sync_results.r_MM));
    else
        mm_candidate_mask = false(size(sync_results.r_MM));
        mm_candidate_mask(mm_pair(1), mm_pair(2)) = true;
    end

    valid_MM = (abs(sync_results.r_MM) > r_OF_threshold) & mm_candidate_mask;
    mean_lag_MM = mean(sync_results.lag_MM(valid_MM));
    n_valid_MM = nnz(valid_MM);

    valid_MF = (abs(sync_results.r_MF) > r_OF_threshold) & mm_candidate_mask;
    mean_lag_MF = mean(sync_results.lag_MF(valid_MF));
    n_valid_MF = nnz(valid_MF);

    fprintf("  Motor->Mocap  : mean_lag_MM = %+.2f ms (%d/%d candidate axes with |r| > %.2f)\n", ...
        mean_lag_MM, n_valid_MM, nnz(mm_candidate_mask), r_OF_threshold);
    fprintf("  Motor->FBGS   : mean_lag_MF = %+.2f ms (%d/%d candidate axes with |r| > %.2f)\n", ...
        mean_lag_MF, n_valid_MF, nnz(mm_candidate_mask), r_OF_threshold);

    summary_table = [summary_table; ...
        make_summary_row(trajectory_name, lag_OF, r_OF, valid_axis_labels, mean_lag_OF, ...
            mean_lag_MM, n_valid_MM, mean_lag_MF, n_valid_MF)];

end


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

calibration_folder = fullfile(data_root, "postprocess_calibration");
if ~isfolder(calibration_folder)
    mkdir(calibration_folder);
end

lag_FBGS_file = fullfile(calibration_folder, "measured_sensors_delay_ms.txt");
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
    fprintf("   Motor->Mocap = %+.2f ms (%d axes), Motor->FBGS = %+.2f ms (%d axes)\n", ...
        row.Mean_Lag_MM_ms, row.N_Valid_MM, row.Mean_Lag_MF_ms, row.N_Valid_MF);
end

end


%% ====================================================================
%%  HELPER FUNCTIONS
%% ====================================================================

function one_row = make_summary_row(trajectory_name, lag_OF, r_OF, valid_axis_labels, mean_lag_OF, ...
        mean_lag_MM, n_valid_MM, mean_lag_MF, n_valid_MF)
    %   Builds one row of the in-memory summary table: the per-axis
    %   Delta_OF lag and r (px, py, pz), which of those axes passed the
    %   |r| > 0.95 threshold, and the resulting mean_lag_OF. Also
    %   includes the same characterization (mean lag over the candidate
    %   axes with |r| > threshold, and how many qualified out of how many
    %   were candidates) for Motor->Mocap and Motor->FBGS.
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
