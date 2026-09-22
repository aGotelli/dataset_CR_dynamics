close all;
clear;
clc;

%   load required paths
addpath("outils\")
addpath("tests\")

%% ====== PATHS / SETTINGS ======
data_root = fullfile("../../", "data/");
folder = fullfile(data_root, "contact_motion/","touching_base/");


%%  Postprocessing properties

%  Define filter (Butterworth) parameters
cutoffHz    = 15;   %   cutoff frequency
butterOrder = 4;    %   order


%  Define subsampling frequency
samplingHz = 100;


%   Plots switches
plot_mocap_fbgs_corrections = true;
plot_filtered               = false;
plot_interpolation          = false;
plot_validation             = true; %   RMSE comparison plots (RMSE numbers/RMSEs.txt always computed)
plot_disk_num = 5;  %   Which disk to plot (5 = robot tip)



%  Saving data and figures config
saving_folder = fullfile( folder,  "processed/");
saving_fig_folder = fullfile( saving_folder,  "figures/");

mkdir(saving_folder);
mkdir(saving_fig_folder);




%%  Robot setup properties
%   These properties are specific the robot used for the dataset collection
%   and the recording setup (calibration sweeps and delays)

%   Nominal z-position (m) of each of the 5 robot disks along the fiber in
%   the undeformed/straight configuration
disk_z_positions_m = [0 0.12 0.24 0.36 0.48];

%   FBG sample index
FBGS_disk_indices = max(round(disk_z_positions_m*1000), 1);
FBGS_tip_index = FBGS_disk_indices(5);

%   Window of calibration sweeps used for FBG and mocap alignement
align_window_s = 10;


%   Flag to load resense: automatically detected from whether this
%   recording's folder contains a dataResenseFT.csv
use_resense = isfile(fullfile(folder, "dataResenseFT.csv"));


%   Several of the dataset's CSV column headers (e.g. "Fx (N)") aren't
%   valid MATLAB identifiers, so every readtable call below and in
%   outils/ sanitizes them and raises this warning. Silenced once here
%   for the whole session.
warning('off', 'MATLAB:table:ModifiedAndSavedVarnames');


%   Both of these are one-off, dataset-wide calibration constants, saved
%   under data/postprocess_calibration/ so every script that loads them
%   (align_mocap_and_fbgs.m, process_data.m) agrees on the same values.
mocap_correction_file = fullfile(data_root, "postprocess_calibration", "mocap_correction.csv");
if ~isfile(mocap_correction_file)
    compute_mocap_correction(data_root, disk_z_positions_m);
end

lag_FBGS_file = fullfile(data_root, "postprocess_calibration", "measured_sensors_delay_ms.txt");
if ~isfile(lag_FBGS_file)
    compute_sensors_delay(data_root, align_window_s, FBGS_tip_index);
end



%% ====== LOAD DATA ======
ati = readtable(fullfile(folder, "dataATIFT.csv"));

%   Flag to load actuator data (motor angles + Mark10 tendon tensions). 
%   Some recordings (e.g. contact_motion/push_retract) were captured 
%   without the actuator rig running.
has_actuator_data = isfile(fullfile(folder, "dataMotor.csv"));

%   Flag to load FBG data: some recordings (e.g. the FT-sensor-only
%   contact_motion/touching_base and touching_base_ang) have no
%   dataFBGS.csv at all. Passed into align_mocap_and_fbgs so it can skip
%   the FBG load/alignment entirely for those recordings.
has_fbgs_data = isfile(fullfile(folder, "dataFBGS.csv"));

if use_resense
    resense = readtable(fullfile(folder, "dataResenseFT.csv"));

    time_resense = resense.timestamp_s_;

    contact_wrench = [resense.Fx resense.Fy resense.Fz resense.Tx/1000 resense.Ty/1000 resense.Tz/1000];
end


%   Number of tracked robot disks (fixed -- the Resense wand, when
%   present, is not one of them; it is loaded separately below).
N_disks_robot = 5;

%   Load and spatially align the OptiTrack and FBG data for this recording.
[mocap_timestamps, rel_kinematics_disks, rel_kinematics_disks_corr, ...
    fbgs_time, fbgs_shapes, fbgs_curvatures, fbgs_angles] = ...
    align_mocap_and_fbgs(folder, has_fbgs_data, align_window_s, data_root);



%   Load the Resense contact wand's pose separately
if use_resense
    [~, ~, ~, ~, rel_kinematics_disks_all] = data_optitrack(fullfile(folder, "dataOptiTrack.csv"), true);
    rel_kinematics_wand = rel_kinematics_disks_all(:, :, N_disks_robot + 1);
end

%   Load the FBG pipeline-delay correction, measured separately (see the
%   generation step above -- lag_FBGS_file is guaranteed to exist by now).
%   Skipped when this recording has no FBG data (fbgs_time is then empty).
if has_fbgs_data
    lag_FBGS = str2double(fileread(lag_FBGS_file));
    fbgs_time = fbgs_time - lag_FBGS/1000;
end


if plot_mocap_fbgs_corrections && has_fbgs_data
    plot_correction_figures(mocap_timestamps, rel_kinematics_disks, rel_kinematics_disks_corr, ...
        fbgs_time, fbgs_shapes, FBGS_tip_index);
end



%   Load (or synthesize) actuator data: motor encoder timestamps/angles,
%   and MK10 tendon tensions. See has_actuator_data above.
if has_actuator_data
    motor = readtable(fullfile(folder, "dataMotor.csv"));

    mk_1_negx = readtable(fullfile(folder, "dataMark10_-x.csv"));
    mk_1_x    = readtable(fullfile(folder, "dataMark10_+x.csv"));
    mk_2_negy = readtable(fullfile(folder, "dataMark10_-y.csv"));
    mk_2_y    = readtable(fullfile(folder, "dataMark10_+y.csv"));

    time_actuators = motor.timestamp;
    target_angles = [motor.target1_rad, motor.target2_rad, motor.target3_rad, motor.target4_rad];
    measured_angles   = [motor.rel_angle1_rad, motor.rel_angle2_rad, motor.rel_angle3_rad, motor.rel_angle4_rad];

    time_tendons = cell(1,4);
    tendon_tensions  = cell(1,4);
    time_tendons{1} = mk_1_x.timestamp;       tendon_tensions{1} = mk_1_x.tension_N_/2;
    time_tendons{2} = mk_2_y.timestamp;       tendon_tensions{2} = mk_2_y.tension_N_/2;
    time_tendons{3} = mk_1_negx.timestamp;    tendon_tensions{3} = mk_1_negx.tension_N_/2;
    time_tendons{4} = mk_2_negy.timestamp;    tendon_tensions{4} = mk_2_negy.tension_N_/2;
else
    %   No actuator rig for this recording: create dummy values to keep
    %   postprocessing pipeline intact
    time_actuators = mocap_timestamps;
    target_angles = zeros(numel(time_actuators), 4);
    measured_angles = zeros(numel(time_actuators), 4);

    time_tendons = cell(1,4);
    tendon_tensions = cell(1,4);
    for it = 1:4
        time_tendons{it} = time_actuators;
        tendon_tensions{it} = zeros(numel(time_actuators), 1);
    end
end

%   Extract timestamp and force/torque measurement from mini40 (ATI)
%   readATIFT.py assigns the timestamp when the blocking 5-sample DAQ
%   read returns, i.e. at the end of that averaging window rather than
%   its center (Comment 5.33). At the acquisition script's fixed 1 kHz
%   sample clock, a 5-sample block spans 5 ms, so shifting the timestamp
%   back by half that window re-centers it on the block it was averaged
%   over.
ati_block_samples = 5;
ati_sample_rate_hz = 1000;
tA = ati.timestamp - 0.5 * ati_block_samples / ati_sample_rate_hz;

ATI_F = [ati.Fx_N_, ati.Fy_N_, ati.Fz_N_];
ATI_T = [ati.Tx_Nm_, ati.Ty_Nm_, ati.Tz_Nm_];
ATI_FT = [ATI_F ATI_T];

%% ====== FILTER (BUTTER + FILTFILT) ======
%
%   butter_filtfilt returns each signal on its own internal uniform time
%   grid (same span & sample count as its raw input -- see the function
%   below), not back on the raw irregular timestamps. The "_f" time
%   vectors captured below are that grid; they feed directly into the
%   INTERPOLATION section further down instead of being discarded.

%   Measured angles and tendon tension
measured_angles_f   = zeros(size(measured_angles));
tendon_tensions_f = cell(1,4);
time_tendons_f = cell(1,4);
for it = 1:4
    [time_actuators_f, measured_angles_f(:,it)]   = butter_filtfilt(time_actuators, measured_angles(:,it),   cutoffHz, butterOrder);

    [time_tendons_f{it}, tendon_tensions_f{it}] = butter_filtfilt(time_tendons{it}, tendon_tensions{it}, cutoffHz, butterOrder);
end


%   Force and Torque measurements
ATI_F_f = zeros(size(ATI_F));
ATI_T_f = zeros(size(ATI_T));
for k = 1:3
    [tA_f, ATI_F_f(:,k)] = butter_filtfilt(tA, ATI_F(:,k), cutoffHz, butterOrder);
    [tA_f, ATI_T_f(:,k)] = butter_filtfilt(tA, ATI_T(:,k), cutoffHz, butterOrder);
end
%   Compuse the wrench (force first convention)
ATI_FT_f = [ATI_F_f ATI_T_f];

%   N_fbgs_points is 0 when this recording has no FBG data (see
%   has_fbgs_data -- align_mocap_and_fbgs then returns fbgs_shapes sized
%   3 x 0 x 0), so it is always safe to compute unconditionally here.
N_fbgs_points = size(fbgs_shapes, 2);

%   Filter FBG shapes, angle and curvature. Skipped entirely when this
%   recording has no FBG data (fbgs_time would be empty, and
%   butter_filtfilt cannot estimate a sampling rate from it).
if has_fbgs_data
    fbgs_shapes_f = zeros(size(fbgs_shapes));   % 3 x 502 x N_time_fbgs
    for coord = 1:3
        for s = 1:N_fbgs_points
            [fbgs_time_f, fbgs_shapes_f(coord, s, :)] = butter_filtfilt(fbgs_time, squeeze(fbgs_shapes(coord, s, :)), cutoffHz, butterOrder);
        end
    end

    fbgs_angles_t = zeros(size(fbgs_angles));
    fbgs_curvatures_f = zeros(size(fbgs_curvatures));
    for it = 1:26
        [fbgs_time_f, fbgs_angles_t(:,it)] = butter_filtfilt(fbgs_time, fbgs_angles(:,it), cutoffHz, butterOrder);
        [fbgs_time_f, fbgs_curvatures_f(:,it)] = butter_filtfilt(fbgs_time, fbgs_curvatures(:,it), cutoffHz, butterOrder);
    end
end

%   Filter the disks kinematics (relative to robot base)
rel_kinematics_disks_f = zeros(size(rel_kinematics_disks));
rel_kinematics_disks_corr_f = zeros(size(rel_kinematics_disks));
for it=1:N_disks_robot

    for k=1:6
        [mocap_timestamps_f, rel_kinematics_disks_f(:, k, it)] = butter_filtfilt(mocap_timestamps, rel_kinematics_disks(:, k, it), cutoffHz, butterOrder);
        [mocap_timestamps_f, rel_kinematics_disks_corr_f(:, k, it)] = butter_filtfilt(mocap_timestamps, rel_kinematics_disks_corr(:, k, it), cutoffHz, butterOrder);

    end
end

%   Filter the wand pose (no per-disk correction defined for it -- see
%   align_mocap_and_fbgs)
if use_resense
    rel_kinematics_wand_f = zeros(size(rel_kinematics_wand));
    for k=1:6
        [mocap_timestamps_f, rel_kinematics_wand_f(:, k)] = butter_filtfilt(mocap_timestamps, rel_kinematics_wand(:, k), cutoffHz, butterOrder);
    end
end

%   (if used) filter Resense HEX12 F/T measurments
if use_resense
    contact_wrench_f = zeros(size(contact_wrench));
    for k = 1:6
        [time_resense_f, contact_wrench_f(:,k)] = butter_filtfilt(time_resense, contact_wrench(:,k), cutoffHz, butterOrder);
    end
end



%   plot the extracted data (this figure overlays FBGS, so it needs FBG
%   data; skipped for recordings without it -- see has_fbgs_data)
if plot_filtered
    plot_filtered_figures(time_tendons, tendon_tensions, time_tendons_f, tendon_tensions_f, ...
        time_actuators, measured_angles, time_actuators_f, measured_angles_f, target_angles, ...
        tA, tA_f, ATI_T_f, ATI_F_f, ...
        mocap_timestamps, rel_kinematics_disks, mocap_timestamps_f, rel_kinematics_disks_f, plot_disk_num);
end



%% ====== INTERPOLATION ======


%   Find the max initial time (last sensor to start streaming). Built up
%   incrementally so fbgs_time(1)/time_resense(1) are only touched when
%   that sensor is actually present -- fbgs_time is empty for recordings
%   with no FBG data (see has_fbgs_data), and indexing an empty array
%   errors.
init_time = max([time_actuators(1), ...
    time_tendons{1}(1), time_tendons{2}(1), time_tendons{3}(1), time_tendons{4}(1), ...
    tA(1), mocap_timestamps(1)]);

if has_fbgs_data
    init_time = max(init_time, fbgs_time(1));
end

if use_resense
    init_time = max(init_time, time_resense(1));
end

%   Find the min final time (first sensor to stop streaming); same
%   incremental-build reasoning as init_time above.
end_time = min([time_actuators(end), ...
    time_tendons{1}(end), time_tendons{2}(end), time_tendons{3}(end), time_tendons{4}(end), ...
    tA(end), mocap_timestamps(end)]);

if has_fbgs_data
    end_time = min(end_time, fbgs_time(end));
end

if use_resense
    end_time = min(end_time, time_resense(end));
end

%   Compute the relative timestamp with respect to the initial timestamp
relative_time_motors = time_actuators - init_time;

relative_time_tendons{1} = time_tendons{1} - init_time;       
relative_time_tendons{2} = time_tendons{2} - init_time;     
relative_time_tendons{3} = time_tendons{3} - init_time;   
relative_time_tendons{4} = time_tendons{4} - init_time;  

relative_time_ATI = tA - init_time;

relative_time_mocap = mocap_timestamps - init_time;

relative_time_fbgs = fbgs_time - init_time;

if use_resense
    relative_time_resense = time_resense - init_time;
end

%   Same, but for each FILTERED signal's own uniform time grid (returned
%   by butter_filtfilt above) rather than the raw irregular timestamps.
%   Used below wherever a FILTERED signal is interpolated onto the final
%   common grid; the raw, unfiltered base wrench keeps using
%   relative_time_ATI above instead.
relative_time_motors_f = time_actuators_f - init_time;

relative_time_tendons_f{1} = time_tendons_f{1} - init_time;
relative_time_tendons_f{2} = time_tendons_f{2} - init_time;
relative_time_tendons_f{3} = time_tendons_f{3} - init_time;
relative_time_tendons_f{4} = time_tendons_f{4} - init_time;

relative_time_ATI_f = tA_f - init_time;

relative_time_mocap_f = mocap_timestamps_f - init_time;

if has_fbgs_data
    relative_time_fbgs_f = fbgs_time_f - init_time;
end

if use_resense
    relative_time_resense_f = time_resense_f - init_time;
end

%   Compute the number of samples
N_samples = floor(samplingHz*(end_time - init_time));
sampling_dt = 1/samplingHz;
sampling_time = (0:sampling_dt:sampling_dt*(N_samples-1))';

%   Interpolate data at the given points

%   Angles and tensions
interp_angles = zeros(N_samples, 4);
interp_tensions = zeros(N_samples, 4);
for it=1:4

    interp_angles(:, it) = interp1(relative_time_motors_f, measured_angles_f(:,it), sampling_time)';

    interp_tensions(:, it) = interp1(relative_time_tendons_f{it}, tendon_tensions_f{it}, sampling_time)';
end

%   Wrench at the base
interp_base_wrench = zeros(N_samples, 6);
interp_base_wrench_raw = zeros(N_samples, 6);
for it=1:6

    interp_base_wrench(:, it) = interp1(relative_time_ATI_f, ATI_FT_f(:, it), sampling_time)';
    interp_base_wrench_raw(:, it) = interp1(relative_time_ATI, ATI_FT(:, it), sampling_time)';
end

%   Kinematics of disks
interp_rel_kinematics_disks_corr = zeros(N_samples, 6, N_disks_robot);
for it=1:N_disks_robot

    for k=1:6
        interp_rel_kinematics_disks_corr(:, k, it) = interp1(relative_time_mocap_f, rel_kinematics_disks_corr_f(:, k, it), sampling_time);

    end
end

%   Kinematics of the wand
if use_resense
    interp_rel_kinematics_wand = zeros(N_samples, 6);
    for k=1:6
        interp_rel_kinematics_wand(:, k) = interp1(relative_time_mocap_f, rel_kinematics_wand_f(:, k), sampling_time);
    end
end

%   FBG shapes, curvature and angle. Skipped entirely when this
%   recording has no FBG data -- see has_fbgs_data.
if has_fbgs_data
    interp_fbgs_shapes = zeros(3, N_fbgs_points, N_samples);
    for coord = 1:3
        for s = 1:N_fbgs_points
            interp_fbgs_shapes(coord, s, :) = interp1(relative_time_fbgs_f, squeeze(fbgs_shapes_f(coord, s, :)), sampling_time);
        end
    end

    interp_fbgs_angles = zeros(N_samples, 26);
    interp_fbgs_curvatures = zeros(N_samples, 26);
    for it = 1:26
        interp_fbgs_angles(:,it) = interp1(relative_time_fbgs_f, fbgs_angles_t(:, it), sampling_time)';
        interp_fbgs_curvatures(:,it) = interp1(relative_time_fbgs_f, fbgs_curvatures_f(:, it), sampling_time)';
    end
end


%   Contact wrench (Resense HEX12)
if use_resense

    interp_contact_wrench = zeros(N_samples, 6);
    for it=1:6

        interp_contact_wrench(:, it) = interp1(relative_time_resense_f, contact_wrench_f(:, it), sampling_time)';
    end

end






%  Plot interpolated data

if plot_interpolation
    plot_interpolation_figures(relative_time_motors_f, measured_angles_f, sampling_time, interp_angles, ...
        relative_time_tendons_f, tendon_tensions_f, interp_tensions, ...
        relative_time_ATI_f, ATI_FT_f, interp_base_wrench, ...
        relative_time_mocap_f, rel_kinematics_disks_corr_f, interp_rel_kinematics_disks_corr, plot_disk_num);
end



%%  For Contact case compute the pose of the FT sensor
if use_resense
  
    g_fix = wand_sensor_offset();

    contact_pose = 0*interp_rel_kinematics_wand;
    for it_t = 1:N_samples
        wand_XYZ_xyz = interp_rel_kinematics_wand(it_t, :);

        R = eul2rotm(wand_XYZ_xyz(1:3), 'XYZ');
        r = wand_XYZ_xyz(4:6)';

        g = [
          R     r
          0 0 0 1
        ];

        g_s = g*g_fix;
        R_s = g_s(1:3, 1:3);
        r_s = g_s(1:3, 4);

        XYZ_s = rotm2eul(R_s, 'XYZ');

        contact_pose(it_t, :) = [
            XYZ_s   r_s'
        ];

    end

end


%%  Save the interpolated data

interp_time_angles      = [sampling_time interp_angles];
interp_time_tensions    = [sampling_time interp_tensions];
interp_time_base_wrench = [sampling_time interp_base_wrench];
interp_time_mocap_frames_corr = reshape(interp_rel_kinematics_disks_corr, [N_samples, 6*N_disks_robot]);
interp_time_mocap_frames_corr = [sampling_time interp_time_mocap_frames_corr];

if use_resense
    interp_contact_wrench = [sampling_time interp_contact_wrench];
    interp_time_hex12_pose = [sampling_time contact_pose];
end



writematrix(interp_time_angles, fullfile(saving_folder , "angles.csv"));
writematrix(interp_time_tensions, fullfile(saving_folder ,"tendon_tensions.csv"));
writematrix(interp_time_base_wrench, fullfile(saving_folder , "base_wrench.csv"));
writematrix(interp_time_mocap_frames_corr, fullfile(saving_folder , "mocap_frames.csv"));

%   Save the postprocessing parameters used to generate this dataset, so
%   they can be loaded back with readtable(".../processing_parameters.csv")
param_names = {"cutoff_frequency_Hz"; "butterworth_order"; "resampling_frequency_Hz"; ...
    "align_window_s"; "N_disks_robot"; "use_resense"; "has_actuator_data"; "has_fbgs_data"};
param_values = {cutoffHz; butterOrder; samplingHz; ...
    align_window_s; N_disks_robot; use_resense; has_actuator_data; has_fbgs_data};
processing_parameters = table(param_names, param_values, 'VariableNames', {'parameter', 'value'});
writetable(processing_parameters, fullfile(saving_folder, "processing_parameters.csv"));

%   FBGS: save as N_samples x (1 + 3*N_fbgs_points), one time column
%   followed by one [x y z] triplet per reconstruction point:
%   columns: [time, x_0,y_0,z_0, x_1,y_1,z_1, ..., x_501,y_501,z_501]
if has_fbgs_data
    interp_fbgs_flat = reshape(permute(interp_fbgs_shapes, [3 1 2]), N_samples, []);
    interp_time_fbgs = [sampling_time interp_fbgs_flat];
    writematrix(interp_time_fbgs, fullfile(saving_folder, "fbgs_shapes.csv"));

    interp_time_fbgs_strain = [sampling_time interp_fbgs_curvatures interp_fbgs_angles];
    writematrix(interp_time_fbgs_strain, fullfile(saving_folder, "fbgs_strains.csv"));
end

if use_resense
    writematrix(interp_contact_wrench, fullfile(saving_folder , "contact_wrench.csv"));
    writematrix(interp_time_hex12_pose, fullfile(saving_folder , "contact_pose.csv"));
end

%%  Compute metrics for dataset techinical validation
technical_validation(saving_folder, saving_fig_folder, N_disks_robot, N_fbgs_points, has_fbgs_data, plot_validation);

fprintf("   SAVED DATA");


%%  HELPER FUNCTIONS



function [t_uniform, y_uniform] = butter_filtfilt(t, x, fc, n)
    % Zero-phase Butterworth low-pass filtering, robust to irregular
    % sampling.
    %
    % It estimates Fs from the mean inter-sample interval, resamples the
    % signal onto a uniform grid at that rate, and filters on that grid.
    %
    % Returns the signal ON THE UNIFORM GRID (t_uniform), not
    % re-interpolated back onto the original irregular timestamps: that
    % round trip would discard the uniform grid built here only to force
    % the caller to redo equivalent interpolation work when resampling
    % onto the final common time base. t_uniform spans the same
    % start/end and sample count as the input t, so callers use it
    % directly wherever they previously used t.

    Fs = (numel(t) - 1) / (t(end) - t(1));            % mean-based rate
    t_uniform = linspace(t(1), t(end), numel(t))';    % regular grid, same span & count
    x_uniform = interp1(t, x, t_uniform, 'linear');
    [b, a] = butter(n, fc/(Fs/2), "low");
    y_uniform = filtfilt(b, a, x_uniform);
end


function plot_correction_figures(mocap_timestamps, rel_kinematics_disks, rel_kinematics_disks_corr, ...
        fbgs_time, fbgs_shapes, FBGS_tip_index)
    %   PLOT_CORRECTION_FIGURES  Mocap raw-vs-corrected disk poses, and a
    %   raw/corrected mocap vs FBGS comparison at the tip. Only the first
    %   3 seconds of the recording are shown.

    mocap_time_rel = mocap_timestamps - mocap_timestamps(1);
    idx_init = mocap_time_rel <= 3.0;
    mocap_time_rel_init = mocap_time_rel(idx_init);
    rel_kinematics_disks_init = rel_kinematics_disks(idx_init, :, :);
    rel_kinematics_disks_corr_init = rel_kinematics_disks_corr(idx_init, :, :);


    figure('Name', 'Disks Position')
    subplot(3, 1, 1)
    plot(mocap_time_rel_init, rel_kinematics_disks_init(:, 4, 1), 'r')
    plot(mocap_time_rel_init, rel_kinematics_disks_corr_init(:, 4, 1), '--r', 'LineWidth', 2)
    hold on;
    plot(mocap_time_rel_init, rel_kinematics_disks_init(:, 4, 2), 'g')
    plot(mocap_time_rel_init, rel_kinematics_disks_corr_init(:, 4, 2), '--g', 'LineWidth', 2)
    plot(mocap_time_rel_init, rel_kinematics_disks_init(:, 4, 3), 'b')
    plot(mocap_time_rel_init, rel_kinematics_disks_corr_init(:, 4, 3), '--b', 'LineWidth', 2)
    plot(mocap_time_rel_init, rel_kinematics_disks_init(:, 4, 4), 'w')
    plot(mocap_time_rel_init, rel_kinematics_disks_corr_init(:, 4, 4), '--w', 'LineWidth', 2)
    plot(mocap_time_rel_init, rel_kinematics_disks_init(:, 4, 5), 'c')
    plot(mocap_time_rel_init, rel_kinematics_disks_corr_init(:, 4, 5), '--c', 'LineWidth', 2)
    ylabel("p_x [m]")
    grid on

    subplot(3, 1, 2)
    plot(mocap_time_rel_init, rel_kinematics_disks_init(:, 5, 1), 'r')
    plot(mocap_time_rel_init, rel_kinematics_disks_corr_init(:, 5, 1), '--r', 'LineWidth', 2)
    hold on;
    plot(mocap_time_rel_init, rel_kinematics_disks_init(:, 5, 2), 'g')
    plot(mocap_time_rel_init, rel_kinematics_disks_corr_init(:, 5, 2), '--g', 'LineWidth', 2)
    plot(mocap_time_rel_init, rel_kinematics_disks_init(:, 5, 3), 'b')
    plot(mocap_time_rel_init, rel_kinematics_disks_corr_init(:, 5, 3), '--b', 'LineWidth', 2)
    plot(mocap_time_rel_init, rel_kinematics_disks_init(:, 5, 4), 'w')
    plot(mocap_time_rel_init, rel_kinematics_disks_corr_init(:, 5, 4), '--w', 'LineWidth', 2)
    plot(mocap_time_rel_init, rel_kinematics_disks_init(:, 5, 5), 'c')
    plot(mocap_time_rel_init, rel_kinematics_disks_corr_init(:, 5, 5), '--c', 'LineWidth', 2)
    ylabel("p_y [m]")
    grid on


    subplot(3, 1, 3)
    plot(mocap_time_rel_init, rel_kinematics_disks_init(:, 6, 1), 'r')
    plot(mocap_time_rel_init, rel_kinematics_disks_corr_init(:, 6, 1), '--r', 'LineWidth', 2)
    hold on;
    plot(mocap_time_rel_init, rel_kinematics_disks_init(:, 6, 2), 'g')
    plot(mocap_time_rel_init, rel_kinematics_disks_corr_init(:, 6, 2), '--g', 'LineWidth', 2)
    plot(mocap_time_rel_init, rel_kinematics_disks_init(:, 6, 3), 'b')
    plot(mocap_time_rel_init, rel_kinematics_disks_corr_init(:, 6, 3), '--b', 'LineWidth', 2)
    plot(mocap_time_rel_init, rel_kinematics_disks_init(:, 6, 4), 'w')
    plot(mocap_time_rel_init, rel_kinematics_disks_corr_init(:, 6, 4), '--w', 'LineWidth', 2)
    plot(mocap_time_rel_init, rel_kinematics_disks_init(:, 6, 5), 'c')
    plot(mocap_time_rel_init, rel_kinematics_disks_corr_init(:, 6, 5), '--c', 'LineWidth', 2)
    ylabel("p_z [m]")
    xlabel("Time [s]")
    grid on
    legend('disk_0','disk_1','disk_2','disk_3','disk_4')


    figure('Name', 'Disks Orientation (EUL XYZ)')
    subplot(3, 1, 1)
    plot(mocap_time_rel_init, rel_kinematics_disks_init(:, 1, 1), 'r')
    plot(mocap_time_rel_init, rel_kinematics_disks_corr_init(:, 1, 1), '--r', 'LineWidth', 2)
    hold on;
    plot(mocap_time_rel_init, rel_kinematics_disks_init(:, 1, 2), 'g')
    plot(mocap_time_rel_init, rel_kinematics_disks_corr_init(:, 1, 2), '--g', 'LineWidth', 2)
    plot(mocap_time_rel_init, rel_kinematics_disks_init(:, 1, 3), 'b')
    plot(mocap_time_rel_init, rel_kinematics_disks_corr_init(:, 1, 3), '--b', 'LineWidth', 2)
    plot(mocap_time_rel_init, rel_kinematics_disks_init(:, 1, 4), 'w')
    plot(mocap_time_rel_init, rel_kinematics_disks_corr_init(:, 1, 4), '--w', 'LineWidth', 2)
    plot(mocap_time_rel_init, rel_kinematics_disks_init(:, 1, 5), 'c')
    plot(mocap_time_rel_init, rel_kinematics_disks_corr_init(:, 1, 5), '--c', 'LineWidth', 2)
    ylabel("Roll [rad]")
    grid on

    subplot(3, 1, 2)
    plot(mocap_time_rel_init, rel_kinematics_disks_init(:, 2, 1), 'r')
    plot(mocap_time_rel_init, rel_kinematics_disks_corr_init(:, 2, 1), '--r', 'LineWidth', 2)
    hold on;
    plot(mocap_time_rel_init, rel_kinematics_disks_init(:, 2, 2), 'g')
    plot(mocap_time_rel_init, rel_kinematics_disks_corr_init(:, 2, 2), '--g', 'LineWidth', 2)
    plot(mocap_time_rel_init, rel_kinematics_disks_init(:, 2, 3), 'b')
    plot(mocap_time_rel_init, rel_kinematics_disks_corr_init(:, 2, 3), '--b', 'LineWidth', 2)
    plot(mocap_time_rel_init, rel_kinematics_disks_init(:, 2, 4), 'w')
    plot(mocap_time_rel_init, rel_kinematics_disks_corr_init(:, 2, 4), '--w', 'LineWidth', 2)
    plot(mocap_time_rel_init, rel_kinematics_disks_init(:, 2, 5), 'c')
    plot(mocap_time_rel_init, rel_kinematics_disks_corr_init(:, 2, 5), '--c', 'LineWidth', 2)
    ylabel("Pitch [rad]")
    grid on


    subplot(3, 1, 3)
    plot(mocap_time_rel_init, rel_kinematics_disks_init(:, 3, 1), 'r')
    plot(mocap_time_rel_init, rel_kinematics_disks_corr_init(:, 3, 1), '--r', 'LineWidth', 2)
    hold on;
    plot(mocap_time_rel_init, rel_kinematics_disks_init(:, 3, 2), 'g')
    plot(mocap_time_rel_init, rel_kinematics_disks_corr_init(:, 3, 2), '--g', 'LineWidth', 2)
    plot(mocap_time_rel_init, rel_kinematics_disks_init(:, 3, 3), 'b')
    plot(mocap_time_rel_init, rel_kinematics_disks_corr_init(:, 3, 3), '--b', 'LineWidth', 2)
    plot(mocap_time_rel_init, rel_kinematics_disks_init(:, 3, 4), 'w')
    plot(mocap_time_rel_init, rel_kinematics_disks_corr_init(:, 3, 4), '--w', 'LineWidth', 2)
    plot(mocap_time_rel_init, rel_kinematics_disks_init(:, 3, 5), 'c')
    plot(mocap_time_rel_init, rel_kinematics_disks_corr_init(:, 3, 5), '--c', 'LineWidth', 2)
    ylabel("Yaw [rad]")
    xlabel("Time [s]")
    grid on
    legend('disk_0','disk_1','disk_2','disk_3','disk_4')




    





    %   Extract plotting slices from the rotated shapes
    XYZ_xyz_disk = rel_kinematics_disks(:, :, 5);
    XYZ_xyz_disk_corr = rel_kinematics_disks_corr(:, :, 5);



    figure("Name", "Tip Orientation");
    vars = {'Roll', 'Pitch', 'Yaw'};
    for it = 1:3
        index_plot = it;
        subplot(3,1,index_plot)

        plot(mocap_timestamps, XYZ_xyz_disk(:, it), "b", "LineWidth", 2.0)
        hold on
        plot(mocap_timestamps, XYZ_xyz_disk_corr(:, it), "r", "LineWidth", 2.0)


        grid on
        ylabel([vars{it} ' [rad]'])


        if it == 3
            xlabel("Time [s]")
        end

        if it == 1
            title("Raw")
        end

    end
    legend('OptiTrack (raw)', 'OptiTrack (corrected)')


    xyz_FBGS     = squeeze(fbgs_shapes(:, FBGS_tip_index, :));


    figure("Name", "Tip Position");
    vars = {'p_x', 'p_y', 'p_z'};
    for it = 1:3
        index_plot = it;
        subplot(3,1,index_plot)

        plot(mocap_timestamps, XYZ_xyz_disk(:, it + 3), "b", "LineWidth", 2.0)
        hold on
        plot(fbgs_time, xyz_FBGS(it, :), "r", "LineWidth", 2.0)
        plot(mocap_timestamps, XYZ_xyz_disk_corr(:, it + 3), "g", "LineWidth", 2.0)


        grid on
        ylabel([vars{it} ' [m]'])


        if it == 3
            xlabel("Time [s]")
        end

        if it == 1
            title("Raw")
        end

    end
    legend('OptiTrack (raw)', 'FBGS', 'OptiTrack (corrected)')




    
end


function plot_filtered_figures(time_tendons, tendon_tensions, time_tendons_f, tendon_tensions_f, ...
        time_actuators, measured_angles, time_actuators_f, measured_angles_f, target_angles, ...
        tA, tA_f, ATI_T_f, ATI_F_f, ...
        mocap_timestamps, rel_kinematics_disks, mocap_timestamps_f, rel_kinematics_disks_f, plot_disk_num)
    %   PLOT_FILTERED_FIGURES  Sanity-check plots for the Butterworth
    %   filtering step: raw signal on its own raw timestamps vs filtered
    %   signal on its own uniform filter-grid timestamps, per sensor.

    figure("Name","Tendon Tensions");
    for it = 1:4
        subplot(4,1,it)

        plot(time_tendons{it}, tendon_tensions{it}, "b", "LineWidth", 2.0);
        hold on
        plot(time_tendons_f{it}, tendon_tensions_f{it}, "r", "LineWidth", 2.0);
        ylabel("Tension [N]")

        title("Tendon " + it + ": raw vs filtered tension")
        if it == 4
            xlabel("Time (raw timestamp)")
        end

    end
    legend('Raw', 'Filtered')


    figure("Name","Motor Angles");
    for it = 1:4
        subplot(4,1,it)

        plot(time_actuators, measured_angles(:,it),   "b", "LineWidth", 2.0)
        hold on
        plot(time_actuators_f, measured_angles_f(:,it),   "r", "LineWidth", 2.0)
        plot(time_actuators, target_angles(:,it), "--g","LineWidth", 2.0);
        ylabel("Angle [rad]")
        grid on

        title("Motor " + it + ": measured vs filtered vs target angle")
        if it == 4
            xlabel("Time (raw timestamp)")
        end

    end
    legend('Raw', 'Filtered', 'Target')


    figure("Name","ATI FT (filtered)");
    subplot(2,1,1)
    plot(tA_f, ATI_T_f(:,1), "r"); hold on
    plot(tA_f, ATI_T_f(:,2), "g");
    plot(tA_f, ATI_T_f(:,3), "b");
    grid on; ylabel("Torque [Nm]"); legend("Tx","Ty","Tz")
    title("ATI Torques (filtered)")

    subplot(2,1,2)
    plot(tA_f, ATI_F_f(:,1), "r"); hold on
    plot(tA_f, ATI_F_f(:,2), "g");
    plot(tA_f, ATI_F_f(:,3), "b");
    grid on; ylabel("Force [N]"); xlabel("Time (raw timestamp)")
    legend("Fx","Fy","Fz")
    title("ATI Forces (filtered)")


    figure("Name","Mocap disk kinematics" + int2str(plot_disk_num));
    XYZ_xyz = rel_kinematics_disks(:, :, plot_disk_num);
    XYZ_xyz_f = rel_kinematics_disks_f(:, :, plot_disk_num);

    for it = 1:3
        index_plot = it*2 -1;
        subplot(3,2,index_plot)

        plot(mocap_timestamps, XYZ_xyz(:, it), "b", "LineWidth", 2.0)
        hold on
        plot(mocap_timestamps_f, XYZ_xyz_f(:, it), "r", "LineWidth", 2.0)
        ylabel("Euler Angle [rad]")
        grid on

        if it == 3
            xlabel("Time [s]")
        end

    end

    for it = 1:3
        index_plot = it*2;
        subplot(3,2,index_plot)

        plot(mocap_timestamps, XYZ_xyz(:, 3 + it), "b", "LineWidth", 2.0)
        hold on
        plot(mocap_timestamps_f, XYZ_xyz_f(:, 3 + it), "r", "LineWidth", 2.0)

        ylabel("Position [m]")
        grid on

        if it == 3
            xlabel("Time [s]")
        end

    end

end


function plot_interpolation_figures(relative_time_motors, measured_angles_f, sampling_time, interp_angles, ...
        relative_time_tendons, tendon_tensions_f, interp_tensions, ...
        relative_time_ATI, ATI_FT_f, interp_base_wrench, ...
        relative_time_mocap, rel_kinematics_disks_f, interp_rel_kinematics_disks, plot_disk_num)
    %   PLOT_INTERPOLATION_FIGURES  Sanity-check plots for the common-grid
    %   resampling step: filtered signal (line) vs resampled signal
    %   (points), per sensor.

    figure("Name","Actuators Angles");
    for it = 1:4
        subplot(4,1,it)

        plot(relative_time_motors, measured_angles_f(:,it),   "b", "LineWidth", 2.0); hold on
        plot(sampling_time, interp_angles(:,it), "or","MarkerSize", 3);
        ylabel("Angle [rad]")
        grid on

        title("Actuator " + it)
        if it == 4
            xlabel("Time [s]")
        end

    end


    figure("Name","Interpolated Tendon Tensions");
    for it = 1:4
        subplot(4,1,it)

        plot(relative_time_tendons{it}, tendon_tensions_f{it}, "b", "LineWidth", 2.0); hold on
        plot(sampling_time, interp_tensions(:,it), "or","MarkerSize", 3);
        ylabel("Tension [N]")
        grid on

        title("Tendon " + it)
        if it == 4
            xlabel("Time [s]")
        end

    end


    figure("Name","ATI FT");
    for it = 1:3
        index_plot = it*2 -1;
        subplot(3,2,index_plot)

        plot(relative_time_ATI, ATI_FT_f(:, it), "b", "LineWidth", 2.0); hold on
        plot(sampling_time, interp_base_wrench(:,it), "or","MarkerSize", 3);
        ylabel("Force [N]")
        grid on

        if it == 3
            xlabel("Time [s]")
        end

    end

    for it = 1:3
        index_plot = it*2;
        subplot(3,2,index_plot)

        plot(relative_time_ATI, ATI_FT_f(:, 3 + it), "b", "LineWidth", 2.0); hold on
        plot(sampling_time, interp_base_wrench(:,3 + it), "or","MarkerSize", 3);
        ylabel("Torque [Nm]")
        grid on

        if it == 3
            xlabel("Time [s]")
        end

    end


    figure("Name","Mocap disk " + int2str(plot_disk_num));
    XYZ_xyz_f = rel_kinematics_disks_f(:, :, plot_disk_num);
    interp_XYZ_xyz = interp_rel_kinematics_disks(:, :, plot_disk_num);

    for it = 1:3
        index_plot = it*2 -1;
        subplot(3,2,index_plot)

        plot(relative_time_mocap, XYZ_xyz_f(:, it), "b", "LineWidth", 2.0); hold on
        plot(sampling_time, interp_XYZ_xyz(:, it), "or","MarkerSize", 3);
        ylabel("Euler Angle [rad]")
        grid on

        if it == 3
            xlabel("Time [s]")
        end

    end

    for it = 1:3
        index_plot = it*2;
        subplot(3,2,index_plot)

        plot(relative_time_mocap, XYZ_xyz_f(:, 3 + it), "b", "LineWidth", 2.0); hold on
        plot(sampling_time, interp_XYZ_xyz(:, 3 + it), "or","MarkerSize", 3);

        ylabel("Position [m]")
        grid on

        if it == 3
            xlabel("Time [s]")
        end

    end

end



