close all;
clear;
clc;

%   load required paths
addpath("outils\")

%% ====== PATHS / SETTINGS ======
folder = fullfile("../../", "data/","dynamic_motion/","Lissajous_fast/");


%%  Postprocessing properties

%  Define filter (Butterworth) parameters
cutoffHz    = 15;   %   cutoff frequency
butterOrder = 4;    %   order


%  Define subsampling frequency
samplingHz = 100;


%   Plots switches
plot_mocap_fbgs_corrections = false;
plot_filtered               = false;
plot_interpolation          = false;
plot_validation              = false; %   RMSE comparison plots (RMSE numbers/RMSEs.txt always computed)
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



%% ====== LOAD DATA ======
motor = readtable(fullfile(folder, "dataMotor.csv"));

mk_1_negx = readtable(fullfile(folder, "dataMark10_-x.csv"));
mk_1_x    = readtable(fullfile(folder, "dataMark10_+x.csv"));
mk_2_negy = readtable(fullfile(folder, "dataMark10_-y.csv"));
mk_2_y    = readtable(fullfile(folder, "dataMark10_+y.csv"));

ati = readtable(fullfile(folder, "dataATIFT.csv"));

if use_resense
    resense = readtable(fullfile(folder, "dataResenseFT.csv"));

    time_resense = resense.timestamp_s_;

    wrench_wand = [resense.Fx resense.Fy resense.Fz resense.Tx/1000 resense.Ty/1000 resense.Tz/1000];
end


%   Bending plane: set to 'x' or 'y' — the axis along which the rod bends
if(contains(folder, "_y_"))
    bending_axis = 'y';        % 'y' for plane_y experiments
else
    bending_axis = 'x';        % 'x' for all rest
end

%   Load and spatially align the OptiTrack and FBG data for this recording.
[N_disks, mocap_timestamps, rel_kinematics_disks, rel_kinematics_disks_corr, ...
    fbgs_time, fbgs_shapes, fbgs_curvatures, fbgs_angles] = ...
    align_mocap_and_fbgs(folder, use_resense, align_window_s, bending_axis);

%   Load the FBG pipeline-delay correction, measured separately.
%
%   If that file does not exist, no correction is applied (lag_FBGS = 0).
lag_FBGS_file = fullfile(fileparts(mfilename('fullpath')), "measured_fbg_delay_ms.txt");
if isfile(lag_FBGS_file)
    lag_FBGS = str2double(fileread(lag_FBGS_file));
else
    lag_FBGS = 0;
end
fbgs_time = fbgs_time - lag_FBGS/1000;



%   Extract timestamps, target and measured angles from motors encoders
time_actuators = motor.timestamp;                     

target_angles = [motor.target1_rad, motor.target2_rad, motor.target3_rad, motor.target4_rad];
measured_angles   = [motor.rel_angle1_rad, motor.rel_angle2_rad, motor.rel_angle3_rad, motor.rel_angle4_rad];

%   Extract timestamp and cable tensions from the MK10 force gauges
time_cables = cell(1,4);
cable_tensions  = cell(1,4);


time_cables{1} = mk_1_x.timestamp;       cable_tensions{1} = mk_1_x.tension_N_/2;
time_cables{2} = mk_2_y.timestamp;       cable_tensions{2} = mk_2_y.tension_N_/2;
time_cables{3} = mk_1_negx.timestamp;    cable_tensions{3} = mk_1_negx.tension_N_/2;
time_cables{4} = mk_2_negy.timestamp;    cable_tensions{4} = mk_2_negy.tension_N_/2;

%   Extract timestamp and force/torque measurement from mini40 (ATI)
tA = ati.timestamp;

ATI_F = [ati.Fx_N_, ati.Fy_N_, ati.Fz_N_];
ATI_T = [ati.Tx_Nm_, ati.Ty_Nm_, ati.Tz_Nm_];
ATI_FT = [ATI_F ATI_T];


%% ====== FILTER (BUTTER + FILTFILT) ======

%   Measured angles and cables tension
measured_angles_f   = zeros(size(measured_angles));
cable_tensions_f = cell(1,4);
for it = 1:4
    measured_angles_f(:,it)   = butter_filtfilt(time_actuators, measured_angles(:,it),   cutoffHz, butterOrder);

    cable_tensions_f{it} = butter_filtfilt(time_cables{it}, cable_tensions{it}, cutoffHz, butterOrder);
end


%   Force and Torque measurements
ATI_F_f = zeros(size(ATI_F));
ATI_T_f = zeros(size(ATI_T));
for k = 1:3
    ATI_F_f(:,k) = butter_filtfilt(tA, ATI_F(:,k), cutoffHz, butterOrder);
    ATI_T_f(:,k) = butter_filtfilt(tA, ATI_T(:,k), cutoffHz, butterOrder);
end
%   Compuse the wrench (force first convention)
ATI_FT_f = [ATI_F_f ATI_T_f];

%   Filter FBG shapes
fbgs_shapes_f = zeros(size(fbgs_shapes));   % 3 x 502 x N_time_fbgs
N_fbgs_points = size(fbgs_shapes, 2);
for coord = 1:3
    for s = 1:N_fbgs_points
        fbgs_shapes_f(coord, s, :) = butter_filtfilt(fbgs_time, squeeze(fbgs_shapes(coord, s, :)), cutoffHz, butterOrder);
    end
end

%   Filter FBG angle and curvature
fbgs_angles_t = zeros(size(fbgs_angles));
fbgs_curvatures_f = zeros(size(fbgs_curvatures));
for it = 1:26
    fbgs_angles_t(:,it) = butter_filtfilt(fbgs_time, fbgs_angles(:,it), cutoffHz, butterOrder);
    fbgs_curvatures_f(:,it) = butter_filtfilt(fbgs_time, fbgs_curvatures(:,it), cutoffHz, butterOrder);
end

%   Filter the disks kinematics (relative to robot base)
rel_kinematics_disks_f = zeros(size(rel_kinematics_disks));
rel_kinematics_disks_corr_f = zeros(size(rel_kinematics_disks));
for it=1:N_disks

    for k=1:6  
        rel_kinematics_disks_f(:, k, it) = butter_filtfilt(mocap_timestamps, rel_kinematics_disks(:, k, it), cutoffHz, butterOrder);
        rel_kinematics_disks_corr_f(:, k, it) = butter_filtfilt(mocap_timestamps, rel_kinematics_disks_corr(:, k, it), cutoffHz, butterOrder);
        
    end
end

%   (if used) filter Resense HEX12 F/T measurments
if use_resense
    wrench_wand_f = zeros(size(wrench_wand));
    for k = 1:6
        wrench_wand_f(:,k) = butter_filtfilt(time_resense, wrench_wand(:,k), cutoffHz, butterOrder);
    end
end



%   plot the extracted data
if plot_mocap_fbgs_corrections
    plot_correction_figures(mocap_timestamps, rel_kinematics_disks, rel_kinematics_disks_corr, ...
        fbgs_time, fbgs_shapes, FBGS_tip_index);
end

if plot_filtered
    plot_filtered_figures(time_cables, cable_tensions, cable_tensions_f, ...
        time_actuators, measured_angles, measured_angles_f, target_angles, ...
        tA, ATI_T_f, ATI_F_f, ...
        mocap_timestamps, rel_kinematics_disks, rel_kinematics_disks_f, plot_disk_num);
end



%% ====== INTERPOLATION ======


%   Find the max initial time (last sensor to start streaming)
init_time = max([time_actuators(1), ...
    time_cables{1}(1), time_cables{2}(1), time_cables{3}(1), time_cables{4}(1), ...
    tA(1), mocap_timestamps(1), fbgs_time(1)]);

if use_resense
    init_time = max([time_actuators(1), ...
        time_cables{1}(1), time_cables{2}(1), time_cables{3}(1), time_cables{4}(1), ...
        tA(1), time_resense(1), ...
        mocap_timestamps(1), fbgs_time(1)]);
end

%   Find the min final time (first sensor to stop streaming)
end_time = min([time_actuators(end), ...
    time_cables{1}(end), time_cables{2}(end), time_cables{3}(end), time_cables{4}(end), ...
    tA(end), mocap_timestamps(end), fbgs_time(end)]);

if use_resense
    end_time = min([time_actuators(end), ...
        time_cables{1}(end), time_cables{2}(end), time_cables{3}(end), time_cables{4}(end), ...
        tA(end), time_resense(end), ...
        mocap_timestamps(end), fbgs_time(end)]);
end

%   Compute the relative timestamp with respect to the initial timestamp
relative_time_motors = time_actuators - init_time;

relative_time_cables{1} = time_cables{1} - init_time;       
relative_time_cables{2} = time_cables{2} - init_time;     
relative_time_cables{3} = time_cables{3} - init_time;   
relative_time_cables{4} = time_cables{4} - init_time;  

relative_time_ATI = tA - init_time;

relative_time_mocap = mocap_timestamps - init_time;

relative_time_fbgs = fbgs_time - init_time;

if use_resense
    relative_time_resense = time_resense - init_time;
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

    interp_angles(:, it) = interp1(relative_time_motors, measured_angles_f(:,it), sampling_time)';

    interp_tensions(:, it) = interp1(relative_time_cables{it}, cable_tensions_f{it}, sampling_time)';
end

%   Wrench at the base
interp_base_wrench = zeros(N_samples, 6);
interp_base_wrench_raw = zeros(N_samples, 6);
for it=1:6

    interp_base_wrench(:, it) = interp1(relative_time_ATI, ATI_FT_f(:, it), sampling_time)';
    interp_base_wrench_raw(:, it) = interp1(relative_time_ATI, ATI_FT(:, it), sampling_time)';
end

%   Kinematics of disks
interp_rel_kinematics_disks = zeros(N_samples, 6, N_disks);
interp_rel_kinematics_disks_corr = zeros(N_samples, 6, N_disks);
for it=1:N_disks

    for k=1:6  
        interp_rel_kinematics_disks(:, k, it) = interp1(relative_time_mocap, rel_kinematics_disks_f(:, k, it), sampling_time);
        interp_rel_kinematics_disks_corr(:, k, it) = interp1(relative_time_mocap, rel_kinematics_disks_corr_f(:, k, it), sampling_time);

    end
end

%   FBG shapes
interp_fbgs_shapes = zeros(3, N_fbgs_points, N_samples);
for coord = 1:3
    for s = 1:N_fbgs_points
        interp_fbgs_shapes(coord, s, :) = interp1(relative_time_fbgs, squeeze(fbgs_shapes_f(coord, s, :)), sampling_time);
    end
end

%   FBG curvature and angle
interp_fbgs_angles = zeros(N_samples, 26);
interp_fbgs_curvatures = zeros(N_samples, 26);
for it = 1:26
    interp_fbgs_angles(:,it) = interp1(relative_time_fbgs, fbgs_angles_t(:, it), sampling_time)';
    interp_fbgs_curvatures(:,it) = interp1(relative_time_fbgs, fbgs_curvatures_f(:, it), sampling_time)';
end


%   Default for plot_interpolation_figures (which takes this as an argument)
wrench_at_base = [];

%   Contact wrench
if use_resense

    interp_wrench_wand = zeros(N_samples, 6);
    for it=1:6
    
        interp_wrench_wand(:, it) = interp1(relative_time_resense, wrench_wand_f(:, it), sampling_time)';
    end

    %   Use the interpolated data to compute the equivalent wrench at base
    %   of the robot (used for cross-validation). Wand pose here is the
    %   UNCORRECTED in-memory mocap kinematics (disk 6) -- used only for
    %   the plot_interpolation_figures diagnostic plot below.
    %   validate_processed_recording recomputes this again from the
    %   CORRECTED, released mocap_frames.csv when it builds the
    %   FT-sensor RMSE, via the same compute_wrench_at_base helper, so
    %   the transform itself is written once and cannot drift between
    %   the two call sites.
    wrench_at_base = compute_wrench_at_base(interp_rel_kinematics_disks(:, :, 6), interp_wrench_wand);

end






%  Plot interpolated data

if plot_interpolation
    plot_interpolation_figures(relative_time_motors, measured_angles_f, sampling_time, interp_angles, ...
        relative_time_cables, cable_tensions_f, interp_tensions, ...
        relative_time_ATI, ATI_FT_f, interp_base_wrench, ...
        relative_time_mocap, rel_kinematics_disks_f, interp_rel_kinematics_disks, plot_disk_num, ...
        use_resense, interp_base_wrench_raw, wrench_at_base);
end







%%  Save the interpolated data

interp_time_angles      = [sampling_time interp_angles];
interp_time_tensions    = [sampling_time interp_tensions];
interp_time_base_wrench = [sampling_time interp_base_wrench];
interp_time_mocap_frames_corr = reshape(interp_rel_kinematics_disks_corr, [N_samples, 6*N_disks]);
interp_time_mocap_frames_corr = [sampling_time interp_time_mocap_frames_corr];

if use_resense
    interp_wrench_wand = [sampling_time interp_wrench_wand];
end



writematrix(interp_time_angles, fullfile(saving_folder , "angles.csv"));
writematrix(interp_time_tensions, fullfile(saving_folder ,"cable_tensions.csv"));
writematrix(interp_time_base_wrench, fullfile(saving_folder , "base_wrench.csv"));
writematrix(interp_time_mocap_frames_corr, fullfile(saving_folder , "mocap_frames.csv"));

%   FBGS: save as N_samples x (1 + 3*N_fbgs_points)
%   columns: [time, x_0..x_501, y_0..y_501, z_0..z_501]
interp_fbgs_flat = reshape(permute(interp_fbgs_shapes, [3 1 2]), N_samples, []);
interp_time_fbgs = [sampling_time interp_fbgs_flat];
writematrix(interp_time_fbgs, fullfile(saving_folder, "fbgs_shapes.csv"));

if use_resense
    writematrix(interp_wrench_wand, fullfile(saving_folder , "wrench_wand.csv"));
end


interp_time_fbgs_strain      = [sampling_time interp_fbgs_curvatures interp_fbgs_angles];
writematrix(interp_time_fbgs_strain, fullfile(saving_folder, "fbgs_strains.csv"));

%%  Validate the saved data (re-reads the CSVs just written above, so
%   this checks what was actually released, not this run's in-memory
%   state -- see validate_processed_recording)
validate_processed_recording(saving_folder, saving_fig_folder, N_disks, N_fbgs_points, use_resense, plot_validation);
fprintf("   SAVED DATA");


%%  HELPER FUNCTIONS



function y = butter_filtfilt(t, x, fc, n)
    % Zero-phase Butterworth low-pass filtering, robust to irregular
    % sampling.
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


function [A] = hat_(x)

    A=zeros(3,3);
    
    A(1,2)=-x(3);
    A(1,3)=x(2);
    A(2,3)=-x(1);
    
    A(2,1)=x(3);
    A(3,1)=-x(2);
    A(3,2)=x(1);
end

function plot_correction_figures(mocap_timestamps, rel_kinematics_disks, rel_kinematics_disks_corr, ...
        fbgs_time, fbgs_shapes, FBGS_tip_index)
    %   PLOT_CORRECTION_FIGURES  Mocap raw-vs-corrected disk poses, and a
    %   raw/corrected mocap vs FBGS comparison at the tip. Only the first
    %   3 seconds of the recording are shown (the window used to compute
    %   the per-disk correction itself).

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


function plot_filtered_figures(time_cables, cable_tensions, cable_tensions_f, ...
        time_actuators, measured_angles, measured_angles_f, target_angles, ...
        tA, ATI_T_f, ATI_F_f, ...
        mocap_timestamps, rel_kinematics_disks, rel_kinematics_disks_f, plot_disk_num)
    %   PLOT_FILTERED_FIGURES  Sanity-check plots for the Butterworth
    %   filtering step: raw vs filtered signal, per sensor, on each
    %   sensor's own (unaligned) raw timestamps.

    figure("Name","Tendon Tensions");
    for it = 1:4
        subplot(4,1,it)

        plot(time_cables{it}, cable_tensions{it}, "b", "LineWidth", 2.0);
        hold on
        plot(time_cables{it}, cable_tensions_f{it}, "r", "LineWidth", 2.0);
        ylabel("Tension [N]")

        title("Cable " + it + ": raw vs filtered tension")
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
        plot(time_actuators, measured_angles_f(:,it),   "r", "LineWidth", 2.0)
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
    plot(tA, ATI_T_f(:,1), "r"); hold on
    plot(tA, ATI_T_f(:,2), "g");
    plot(tA, ATI_T_f(:,3), "b");
    grid on; ylabel("Torque [Nm]"); legend("Tx","Ty","Tz")
    title("ATI Torques (filtered)")

    subplot(2,1,2)
    plot(tA, ATI_F_f(:,1), "r"); hold on
    plot(tA, ATI_F_f(:,2), "g");
    plot(tA, ATI_F_f(:,3), "b");
    grid on; ylabel("Force [N]"); xlabel("Time (raw timestamp)")
    legend("Fx","Fy","Fz")
    title("ATI Forces (filtered)")


    figure("Name","Mocap disk kinematics" + int2str(plot_disk_num));
    xyz_XYZ = rel_kinematics_disks(:, :, plot_disk_num);
    XYZ_xyz_f = rel_kinematics_disks_f(:, :, plot_disk_num);

    %   NOTE: columns 1:3 are Euler angles [rad], columns 4:6 are position
    %   [m] (same convention as elsewhere in this file). The two loops
    %   below were previously swapped -- fixed so the ylabel matches what
    %   is actually plotted.
    for it = 1:3
        index_plot = it*2 -1;
        subplot(3,2,index_plot)

        plot(mocap_timestamps, xyz_XYZ(:, it), "b", "LineWidth", 2.0)
        hold on
        plot(mocap_timestamps, XYZ_xyz_f(:, it), "r", "LineWidth", 2.0)
        ylabel("Euler Angle [rad]")
        grid on

        if it == 3
            xlabel("Time [s]")
        end

    end

    for it = 1:3
        index_plot = it*2;
        subplot(3,2,index_plot)

        plot(mocap_timestamps, xyz_XYZ(:, 3 + it), "b", "LineWidth", 2.0)
        hold on
        plot(mocap_timestamps, XYZ_xyz_f(:, 3 + it), "r", "LineWidth", 2.0)

        ylabel("Position [m]")
        grid on

        if it == 3
            xlabel("Time [s]")
        end

    end

end


function plot_interpolation_figures(relative_time_motors, measured_angles_f, sampling_time, interp_angles, ...
        relative_time_cables, cable_tensions_f, interp_tensions, ...
        relative_time_ATI, ATI_FT_f, interp_base_wrench, ...
        relative_time_mocap, rel_kinematics_disks_f, interp_rel_kinematics_disks, plot_disk_num, ...
        use_resense, interp_base_wrench_raw, wrench_at_base)
    %   PLOT_INTERPOLATION_FIGURES  Sanity-check plots for the common-grid
    %   resampling step: filtered signal (line) vs resampled signal
    %   (points), per sensor. If use_resense, also plots the
    %   Resense-wand-vs-ATI base wrench cross-check.

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


    figure("Name","Cables Tensions");
    for it = 1:4
        subplot(4,1,it)

        plot(relative_time_cables{it}, cable_tensions_f{it}, "b", "LineWidth", 2.0); hold on
        plot(sampling_time, interp_tensions(:,it), "or","MarkerSize", 3);
        ylabel("Tension [N]")
        grid on

        title("Cable " + it)
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


    %%  Plot wrench contact
    if use_resense
        figure("Name", "Forces")
        subplot(3, 1, 1)
        plot(sampling_time, interp_base_wrench_raw(:, 1), 'b')
        hold on
        plot(sampling_time, wrench_at_base(1, :), 'r')
        ylabel("Fx [N]")
        grid on

        subplot(3, 1, 2)
        plot(sampling_time, interp_base_wrench_raw(:, 2), 'b')
        hold on
        plot(sampling_time, wrench_at_base(2, :), 'r')
        ylabel("Fy [N]")
        grid on

        subplot(3, 1, 3)
        plot(sampling_time, interp_base_wrench_raw(:, 3), 'b')
        hold on
        plot(sampling_time, wrench_at_base(3, :), 'r')
        ylabel("Fz [N]")
        xlabel("Time [s]")
        grid on

        legend('ATI', 'Ad_g Resense')


        figure("Name", "Torques")
        subplot(3, 1, 1)
        plot(sampling_time, interp_base_wrench_raw(:, 4), 'b')
        hold on
        plot(sampling_time, wrench_at_base(4, :), 'r')
        ylabel("Tx [Nm]")
        grid on

        subplot(3, 1, 2)
        plot(sampling_time, interp_base_wrench_raw(:, 5), 'b')
        hold on
        plot(sampling_time, wrench_at_base(5, :), 'r')
        ylabel("Ty [Nm]")
        grid on

        subplot(3, 1, 3)
        plot(sampling_time, interp_base_wrench_raw(:, 6), 'b')
        hold on
        plot(sampling_time, wrench_at_base(6, :), 'r')
        ylabel("Tz [Nm]")
        xlabel("Time [s]")
        grid on

        legend('ATI', 'Ad_g Resense')

    end

end



function wrench_at_base = compute_wrench_at_base(disk_kinematics_wand, wrench_wand)
    %   COMPUTE_WRENCH_AT_BASE  Transports the Resense HEX12 wand wrench
    %   from its own sensor frame to the robot base frame, via the wand's
    %   mocap pose and the wand's fixed sensor-to-mocap-frame offset
    %   (g_fix). Shared by the live diagnostic plot in process_data.m
    %   (fed the UNCORRECTED wand pose, matching this run's in-memory
    %   state) and validate_processed_recording (fed the CORRECTED wand
    %   pose loaded back from mocap_frames.csv, the one actually
    %   released) -- kept in one place so the transform itself can never
    %   drift between the two.
    %
    %   disk_kinematics_wand : N_samples x 6 [roll pitch yaw px py pz],
    %                          the wand's own mocap pose over time
    %   wrench_wand           : N_samples x 6 [Fx Fy Fz Tx Ty Tz], the
    %                          wand's own measured wrench over time
    %   wrench_at_base         : 6 x N_samples

    N_samples = size(disk_kinematics_wand, 1);

    %   Compute the relative fixed transformation from the mocap frame to
    %   the Resense HEX12 sensor frame
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

        %   Compute equivalent wrench with action-reaction principle
        wrench_at_base(:, it_t) = -Ad_g_*wrench_wand_it_t;
    end
end


function validate_processed_recording(saving_folder, saving_fig_folder, N_disks, N_fbgs_points, use_resense, plot_validation)
    %   VALIDATE_PROCESSED_RECORDING  Recomputes every consistency check
    %   in the manuscript's Technical Validation section (mocap vs FBGS
    %   shape RMSE per disk, mocap vs motor cable-length RMSE, and -- for
    %   contact recordings -- ATI base wrench vs Resense wand wrench
    %   RMSE), reading ONLY the CSVs process_data.m just wrote to
    %   saving_folder. This is deliberate: it validates the released
    %   data itself, not this run's in-memory intermediate state, so a
    %   change to what gets saved can never silently drift from what
    %   gets validated. It also means this function can be called
    %   standalone later, on any already-processed recording, without
    %   rerunning the raw-data pipeline (e.g. to batch-build one RMSE
    %   table across every recording in data/).
    %
    %   Figures are only generated when plot_validation is true; the
    %   RMSE numbers themselves (and RMSEs.txt) are always computed and
    %   saved.

    %   ---- Load back what was released ----
    angles_csv = readmatrix(fullfile(saving_folder, "angles.csv"));
    sampling_time = angles_csv(:, 1);
    interp_angles = angles_csv(:, 2:end);
    N_samples = size(angles_csv, 1);

    mocap_csv = readmatrix(fullfile(saving_folder, "mocap_frames.csv"));
    interp_rel_kinematics_disks_corr = reshape(mocap_csv(:, 2:end), [N_samples, 6, N_disks]);

    fbgs_csv = readmatrix(fullfile(saving_folder, "fbgs_shapes.csv"));
    interp_fbgs_shapes = permute(reshape(fbgs_csv(:, 2:end), [N_samples, 3, N_fbgs_points]), [2 3 1]);

    base_wrench_csv = readmatrix(fullfile(saving_folder, "base_wrench.csv"));
    interp_base_wrench = base_wrench_csv(:, 2:end);

    %   FBG sample index nearest each disk -- same convention as
    %   process_data.m's FBGS_disk_indices (disk z-position in mm along
    %   the ~1mm-spaced fiber).
    disk_z_positions_m = [0 0.12 0.24 0.36 0.48];
    FBGS_disk_indices = max(round(disk_z_positions_m*1000), 1);

    %   ---- Mocap vs FBGS: RMSE at every disk ----
    N_disks_robot = 5;
    RMSE_disks = zeros(N_disks_robot, 3);
    RMSE_disks_perc_motion = zeros(N_disks_robot, 3);
    for d = 1:N_disks_robot
        xyz_disk_d = interp_rel_kinematics_disks_corr(:, 4:6, d);
        xyz_FBGS_d = squeeze(interp_fbgs_shapes(:, FBGS_disk_indices(d), :))';

        RMSE_disks(d, :) = rmse(xyz_FBGS_d, xyz_disk_d);

        range_disk_d = max(xyz_disk_d) - min(xyz_disk_d);
        RMSE_disks_perc_motion(d, :) = (RMSE_disks(d, :)./range_disk_d)*100;
    end

    %   ---- Mocap vs motor-encoder cable length ----
    N_interp = 10;
    [delta_cable_measured, delta_cable_computed] = compare_cable_lenght(interp_rel_kinematics_disks_corr, interp_angles, sampling_time, N_interp);

    RMSE_cables = rmse(delta_cable_computed, delta_cable_measured);
    range_cables = max(delta_cable_measured) - min(delta_cable_measured);
    RMSE_cables_perc_motion = (RMSE_cables./range_cables)*100;
    idx_0 = find(range_cables <= 1e-2);
    RMSE_cables_perc_motion(idx_0) = 0*RMSE_cables_perc_motion(idx_0);

    %   ---- ATI base wrench vs Resense wand wrench transported to the
    %   base (FT-sensor consistency comment). Contact recordings only.
    %   Uses the wand's CORRECTED pose (disk 6 of mocap_frames.csv) --
    %   see compute_wrench_at_base. ----
    if use_resense
        wrench_wand_csv = readmatrix(fullfile(saving_folder, "wrench_wand.csv"));
        interp_wrench_wand = wrench_wand_csv(:, 2:end);

        wand_pose_corr = interp_rel_kinematics_disks_corr(:, :, 6);
        wrench_at_base = compute_wrench_at_base(wand_pose_corr, interp_wrench_wand);

        RMSE_wrench = rmse(wrench_at_base', interp_base_wrench);
        range_wrench = max(interp_base_wrench) - min(interp_base_wrench);
        RMSE_wrench_perc_motion = (RMSE_wrench./range_wrench)*100;
    end

    %   ---- Save RMSEs ----
    fid = fopen(fullfile(saving_folder, "RMSEs.txt"), 'w');
    for d = 1:N_disks_robot
        fprintf(fid, 'RMSE_disk_%d = [%s]\n', d, strjoin(string(RMSE_disks(d, :)), ', '));
        fprintf(fid, 'RMSE_disk_%d_perc_motion = [%s]\n', d, strjoin(string(RMSE_disks_perc_motion(d, :)), ', '));
    end
    fprintf(fid, 'RMSE_cables = [%s]\n', strjoin(string(RMSE_cables), ', '));
    fprintf(fid, 'RMSE_cables_perc_motion = [%s]\n', strjoin(string(RMSE_cables_perc_motion), ', '));
    if use_resense
        fprintf(fid, 'RMSE_wrench = [%s]\n', strjoin(string(RMSE_wrench), ', '));
        fprintf(fid, 'RMSE_wrench_perc_motion = [%s]\n', strjoin(string(RMSE_wrench_perc_motion), ', '));
    end
    fclose(fid);

    %   ---- Plots ----
    if plot_validation
        XYZ_xyz_disk = interp_rel_kinematics_disks_corr(:, :, 5);
        xyz_FBGS     = squeeze(interp_fbgs_shapes(:, FBGS_disk_indices(5), :));

        interp_xy_tip = interp_rel_kinematics_disks_corr(:, 4:5, 5);
        fig = figure("Name", "Tip Trajectory xy plane");
        plot(interp_xy_tip(:, 1), interp_xy_tip(:, 2), 'LineWidth', 1)
        hold on
        plot(xyz_FBGS(1, :), xyz_FBGS(2, :), "r", "LineWidth", 1.0)
        grid on
        xlim([-.35 .35])
        ylim([-.35 .35])
        xlabel("p_x [m]")
        ylabel("p_y [m]")
        legend('OptiTrack (corrected)', 'FBGS')
        savefig(saving_fig_folder + fig.Name)
        saveas(fig, saving_fig_folder + fig.Name, 'png')


        fig = figure("Name", "Motors Angles");
        for it=1:4
            subplot(4, 1, it)
            plot(sampling_time, interp_angles(:, it), 'b', 'LineWidth', 2)
            grid on
            ylabel("Angle [rad]")
        end
        xlabel('Time [s]')
        savefig(saving_fig_folder + fig.Name)
        saveas(fig, saving_fig_folder + fig.Name, 'png')


        fig = figure("Name", "Tip Position Interpolated");
        vars = {'p_x', 'p_y', 'p_z'};
        for it = 1:3
            subplot(3,1,it)

            plot(sampling_time, XYZ_xyz_disk(:, it + 3), "b", "LineWidth", 2.0)
            hold on
            plot(sampling_time, xyz_FBGS(it, :), "r", "LineWidth", 2.0)

            grid on
            ylabel([vars{it} ' [m]'])

            if it == 3
                xlabel("Time [s]")
            end
        end
        legend('OptiTrack (corrected)', 'FBGS')
        savefig(saving_fig_folder + fig.Name)
        saveas(fig, saving_fig_folder + fig.Name, 'png')


        cable_labels = {'+x', '+y', '-x', '-y'};
        pairs = {[1 3], [2 4]};          % x-pair, y-pair
        pair_names = {"x", "y"};
        for p = 1:2
            fig = figure("Name", "Cable Length Change – " + pair_names{p} + " pair");
            idx = pairs{p};
            for k = 1:2
                ax = subplot(2,1,k);
                set(ax, 'Color', 'w');
                c = idx(k);
                plot(sampling_time, delta_cable_computed(:,c)*1e3,  'b',  'LineWidth', 2);  hold on
                plot(sampling_time, delta_cable_measured(:,c)*1e3,  'r--','LineWidth', 2);
                grid on; ylabel('\Delta \ell_c [mm]')
                title(['Cable ' cable_labels{c}])
                if k == 1
                    legend('MoCap (computed)', 'Motor (measured)')
                end
                if k == 2, xlabel('Time [s]'); end
            end
            savefig(saving_fig_folder + fig.Name)
            saveas(fig, saving_fig_folder + fig.Name, 'png')
        end

        if use_resense
            force_labels = {'Fx','Fy','Fz'};
            torque_labels = {'Tx','Ty','Tz'};

            figure("Name", "Forces (validation)")
            for k = 1:3
                subplot(3,1,k)
                plot(sampling_time, interp_base_wrench(:, k), 'b'); hold on
                plot(sampling_time, wrench_at_base(k, :), 'r')
                ylabel(force_labels{k} + " [N]")
                grid on
                if k == 3, xlabel("Time [s]"); end
            end
            legend('ATI (base)', 'Resense (transported)')

            figure("Name", "Torques (validation)")
            for k = 1:3
                subplot(3,1,k)
                plot(sampling_time, interp_base_wrench(:, 3+k), 'b'); hold on
                plot(sampling_time, wrench_at_base(3+k, :), 'r')
                ylabel(torque_labels{k} + " [Nm]")
                grid on
                if k == 3, xlabel("Time [s]"); end
            end
            legend('ATI (base)', 'Resense (transported)')
        end
    end
end
