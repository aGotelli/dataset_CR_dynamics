close all;
clear;
clc;

addpath("outils\")


%%  Load reference data

%% ====== PATHS / SETTINGS ======
folder = fullfile("..", "dataCollectionPack/data","dynamic_motion/","circle_slow/");

cutoffHz    = 20;   % Butterworth cutoff
butterOrder = 4;


samplingHz = 100;


%   Bending plane: set to 'x' or 'y' — the axis along which the rod bends
bending_axis = 'x';        % 'y' for plane_y experiments, 'x' for all rest

%   Plots
plot_mocap_fbgs_corrections = false;
plot_filtered = false;
plot_interpolation = false;
plot_disk_num = 5;

FBGS_tip_index = 481;

align_window_s = 10;

use_resense = false;


%  SAVING DATA AND PLOTS
saving_folder = fullfile( folder,  "processed/");
saving_fig_folder = fullfile( saving_folder,  "figures/");
mkdir(saving_folder);
mkdir(saving_fig_folder);


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

filename = fullfile(folder, "dataOptiTrack.csv");
[N_disks, mocap_timestamps, poses_disks, rel_poses_disks, rel_kinematics_disks] = data_optitrack(filename, use_resense);


filename = fullfile(folder, "dataFBGS.csv");
[fbgs_time, fbgs_shapes] = data_fbgs(filename);


%   Apply rotation of -90 deg along y axis to ALL shapes
R_y = axang2rotm([0 1 0 -pi/2]);
N_time_fbgs = size(fbgs_shapes, 3);
for t = 1:N_time_fbgs
    fbgs_shapes(:, :, t) = R_y * fbgs_shapes(:, :, t);
end


mocap_time_rel  = mocap_timestamps - mocap_timestamps(1);
idx_align      = mocap_time_rel <= align_window_s;

XYZ_xyz_disk = rel_kinematics_disks(:, :, 5);
tip_xy_mocap  = XYZ_xyz_disk(idx_align, 4:5);            
tip_xy_mocap_centered = tip_xy_mocap - mean(tip_xy_mocap, 1);
[U_m, S_m, V_m] = svd(tip_xy_mocap_centered, 'econ');



R_m = eye(3);
R_m(1:2, 1:2) = V_m;
axang_m = rotm2axang(R_m);
theta_z_mocap = axang_m(4);

% theta_z_mocap = atan2(R_m(1, 2), R_m(1, 1));

if strcmpi(bending_axis, 'y')
    theta_z_mocap = pi/2 - theta_z_mocap;       % map onto y-axis
else
    theta_z_mocap = 0 - theta_z_mocap;          % map onto x-axis
end

%   The fiber now evolves in z, but bending leaks into both x and y.
%   Use SVD on the tip x-y trajectory (first 10 s only, planar portion)
%   to find the bending direction, then rotate about z.
fbgs_time_rel  = fbgs_time - fbgs_time(1);
idx_align      = fbgs_time_rel <= align_window_s;

tip_xy_all     = squeeze(fbgs_shapes(1:2, end, :));   % 2 x N_time
tip_xy         = tip_xy_all(:, idx_align);            % 2 x N_align
tip_xy_centered = tip_xy - mean(tip_xy, 2);
[U_f, S_f, V_f] = svd(tip_xy_centered, 'econ');

R_f = eye(3);
R_f(1:2, 1:2) = U_f;
axang_f = rotm2axang(R_f);
theta_z_fbgs = axang_f(4);

if strcmpi(bending_axis, 'y')
    theta_z_fbgs = pi/2 - theta_z_fbgs;       % map onto y-axis
else
    theta_z_fbgs = 0 - theta_z_fbgs;          % map onto x-axis
end

if strcmpi(bending_axis, 'y')
    theta_z = theta_z_fbgs + theta_z_mocap;
else
    % theta_z = theta_z_fbgs - theta_z_mocap;
    theta_z = theta_z_fbgs - theta_z_mocap;
end


R_z = axang2rotm([0 0 1 theta_z]);
for t = 1:N_time_fbgs
    fbgs_shapes(:, :, t) = R_z * fbgs_shapes(:, :, t);
end




%%  Correct pose mocap
idx_init      = mocap_time_rel <= 3.0;

rel_kinematics_disks_init = rel_kinematics_disks(idx_init, :, :);
mocap_time_rel_init = mocap_time_rel(idx_init);

%   Remove residual offset
pos_disks = [
    0    0    0    0    0
    0    0    0    0    0
    0    0.12 0.24 0.36 0.48
];

g_correction = zeros(4, 4, N_disks);
rel_kinematics_disks_corr = zeros(size(rel_kinematics_disks));
for it=1:N_disks

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



%% ====== EXTRACT MOTOR SIGNALS ======
time_actuators = motor.timestamp;                     

target_angles = [motor.target1_rad, motor.target2_rad, motor.target3_rad, motor.target4_rad];
measured_angles   = [motor.rel_angle1_rad, motor.rel_angle2_rad, motor.rel_angle3_rad, motor.rel_angle4_rad];

%% ====== EXTRACT FORCES (RAW TIME) ======
time_cables = cell(1,4);
cable_tensions  = cell(1,4);


time_cables{1} = mk_1_x.timestamp;       cable_tensions{1} = mk_1_x.tension_N_/2;
time_cables{2} = mk_2_y.timestamp;       cable_tensions{2} = mk_2_y.tension_N_/2;
time_cables{3} = mk_1_negx.timestamp;    cable_tensions{3} = mk_1_negx.tension_N_/2;
time_cables{4} = mk_2_negy.timestamp;    cable_tensions{4} = mk_2_negy.tension_N_/2;

%% ====== EXTRACT ATI FT (RAW TIME) ======
tA = ati.timestamp;

ATI_F = [ati.Fx_N_, ati.Fy_N_, ati.Fz_N_];
ATI_T = [ati.Tx_Nm_, ati.Ty_Nm_, ati.Tz_Nm_];


%% ====== FILTER (BUTTER + FILTFILT) ======
measured_angles_f   = zeros(size(measured_angles));
cable_tensions_f = cell(1,4);
for it = 1:4
    measured_angles_f(:,it)   = butter_filtfilt(time_actuators, measured_angles(:,it),   cutoffHz, butterOrder);

    cable_tensions_f{it} = butter_filtfilt(time_cables{it}, cable_tensions{it}, cutoffHz, butterOrder);
end

ATI_F_f = zeros(size(ATI_F));
ATI_T_f = zeros(size(ATI_T));
for k = 1:3
    ATI_F_f(:,k) = butter_filtfilt(tA, ATI_F(:,k), cutoffHz, butterOrder);
    ATI_T_f(:,k) = butter_filtfilt(tA, ATI_T(:,k), cutoffHz, butterOrder);
end

ATI_FT_f = [ATI_F_f ATI_T_f];
ATI_FT = [ATI_F ATI_T];

fbgs_shapes_f = zeros(size(fbgs_shapes));   % 3 x 502 x N_time_fbgs
N_fbgs_points = size(fbgs_shapes, 2);
for coord = 1:3
    for s = 1:N_fbgs_points
        fbgs_shapes_f(coord, s, :) = butter_filtfilt(fbgs_time, squeeze(fbgs_shapes(coord, s, :)), cutoffHz, butterOrder);
    end
end


rel_kinematics_disks_f = zeros(size(rel_kinematics_disks));
rel_kinematics_disks_corr_f = zeros(size(rel_kinematics_disks));
for it=1:N_disks

    for k=1:6  
        rel_kinematics_disks_f(:, k, it) = butter_filtfilt(mocap_timestamps, rel_kinematics_disks(:, k, it), cutoffHz, butterOrder);
        rel_kinematics_disks_corr_f(:, k, it) = butter_filtfilt(mocap_timestamps, rel_kinematics_disks_corr(:, k, it), cutoffHz, butterOrder);
        
    end
end

if use_resense
    wrench_wand_f = zeros(size(wrench_wand));
    for k = 1:6
        wrench_wand_f(:,k) = butter_filtfilt(time_resense, wrench_wand(:,k), cutoffHz, butterOrder);
    end
end


%% ====== PLOT: 4 SUBPLOTS (MOTOR TARGET/MEAS + FORCE) ======


if plot_mocap_fbgs_corrections

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
    ylabel("Roll [m]")
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
    ylabel("Pitch [m]")
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
    ylabel("Yaw [m]")
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
    
    legend('OptiTrack', 'FBGS')


end

if plot_filtered
    % figure("Name","Motors + corresponding cable force (filtered)");
    figure("Name","Tendon Tensions");

    for it = 1:4
        subplot(4,1,it)
    
        plot(time_cables{it}, cable_tensions{it}, "b", "LineWidth", 2.0);
        hold on
        plot(time_cables{it}, cable_tensions_f{it}, "r", "LineWidth", 2.0);
        ylabel("Tension [N]")
    
        title("Motor " + it + " (meas/target) + mapped force")
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
  
    
        title("Motor " + it + " (meas/target) + mapped force")
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
    
    
    for it = 1:3
        index_plot = it*2 -1;
        subplot(3,2,index_plot)
    
        plot(mocap_timestamps, xyz_XYZ(:, 3 + it), "b", "LineWidth", 2.0)
        hold on
        plot(mocap_timestamps, XYZ_xyz_f(:, 3 + it), "r", "LineWidth", 2.0)
        ylabel("Euler Angle [RAD]")
        grid on
    
       
    
        if it == 3
            xlabel("Time [s]")
        end
    
    end
    
    for it = 1:3
        index_plot = it*2;
        subplot(3,2,index_plot)
     
        plot(mocap_timestamps, xyz_XYZ(:, it), "b", "LineWidth", 2.0)
        hold on
        plot(mocap_timestamps, XYZ_xyz_f(:, it), "r", "LineWidth", 2.0)

        ylabel("Position [m]")
        grid on
    
       
    
        if it == 3
            xlabel("Time [s]")
        end
    
    end

end



%%  Interpolate at the same frequency

%   First get the motor start time
time_start_motors = time_actuators(1);
relative_time_motors = time_actuators - time_start_motors;
time_end_motors = relative_time_motors(end);


relative_time_cables{1} = time_cables{1} - time_start_motors;       
relative_time_cables{2} = time_cables{2} - time_start_motors;     
relative_time_cables{3} = time_cables{3} - time_start_motors;   
relative_time_cables{4} = time_cables{4} - time_start_motors;  

relative_time_ATI = tA - time_start_motors;


relative_time_mocap = mocap_timestamps - time_start_motors;

relative_time_fbgs = fbgs_time - time_start_motors;

%   Now define interpolation points for the given frequency
N_samples = floor(samplingHz*time_end_motors);
sampling_dt = 1/samplingHz;
sampling_time = (0:sampling_dt:sampling_dt*(N_samples-1))';

%   Interpolate data at the given points
interp_angles = zeros(N_samples, 4);
interp_tensions = zeros(N_samples, 4);
for it=1:4

    interp_angles(:, it) = interp1(relative_time_motors, measured_angles_f(:,it), sampling_time)';

    interp_tensions(:, it) = interp1(relative_time_cables{it}, cable_tensions_f{it}, sampling_time)';
end

interp_base_wrench = zeros(N_samples, 6);
interp_base_wrench_raw = zeros(N_samples, 6);
for it=1:6

    interp_base_wrench(:, it) = interp1(relative_time_ATI, ATI_FT_f(:, it), sampling_time)';
    interp_base_wrench_raw(:, it) = interp1(relative_time_ATI, ATI_FT(:, it), sampling_time)';
end

interp_rel_kinematics_disks = zeros(N_samples, 6, N_disks);
interp_rel_kinematics_disks_corr = zeros(N_samples, 6, N_disks);
for it=1:N_disks

    for k=1:6  
        interp_rel_kinematics_disks(:, k, it) = interp1(relative_time_mocap, rel_kinematics_disks_f(:, k, it), sampling_time);
        interp_rel_kinematics_disks_corr(:, k, it) = interp1(relative_time_mocap, rel_kinematics_disks_corr_f(:, k, it), sampling_time);

    end
end

interp_fbgs_shapes = zeros(3, N_fbgs_points, N_samples);
for coord = 1:3
    for s = 1:N_fbgs_points
        interp_fbgs_shapes(coord, s, :) = interp1(relative_time_fbgs, squeeze(fbgs_shapes_f(coord, s, :)), sampling_time);
    end
end

if use_resense

    relative_time_resense = time_resense - time_start_motors;

    interp_wrench_wand = zeros(N_samples, 6);
    for it=1:6
    
        interp_wrench_wand(:, it) = interp1(relative_time_resense, wrench_wand_f(:, it), sampling_time)';
    end
end






if use_resense

    %%  Now compute the wrench
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
    
        g= [
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

end




%%  Plot interpolated data

if plot_interpolation
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
    
       
    
        title("Actuator " + it)
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
        ylabel("Euler Angle [RAD]")
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
        grid on
        
        subplot(3, 1, 2)
        plot(sampling_time, interp_base_wrench_raw(:, 2), 'b')
        hold on
        plot(sampling_time, wrench_at_base(2, :), 'r')
        grid on
        
        subplot(3, 1, 3)
        plot(sampling_time, interp_base_wrench_raw(:, 3), 'b')
        hold on
        plot(sampling_time, wrench_at_base(3, :), 'r')
        grid on
        
        legend('ATI', 'Ad_g Resense')
        
        
        figure("Name", "Torques")
        subplot(3, 1, 1)
        plot(sampling_time, interp_base_wrench_raw(:, 4), 'b')
        hold on
        plot(sampling_time, wrench_at_base(4, :), 'r')
        grid on
        
        subplot(3, 1, 2)
        plot(sampling_time, interp_base_wrench_raw(:, 5), 'b')
        hold on
        plot(sampling_time, wrench_at_base(5, :), 'r')
        grid on
        
        subplot(3, 1, 3)
        plot(sampling_time, interp_base_wrench_raw(:, 6), 'b')
        hold on
        plot(sampling_time, wrench_at_base(6, :), 'r')
        grid on
        
        legend('ATI', 'Ad_g Resense')
    
    end


end





%%  Plot robot tip

%   FBGS and Mocap
%   Extract plotting slices from the rotated shapes
XYZ_xyz_disk = interp_rel_kinematics_disks(:, :, 5);
xyz_FBGS     = squeeze(interp_fbgs_shapes(:, FBGS_tip_index, :));


interp_xy_tip = interp_rel_kinematics_disks(:, 4:5, 5);
fig = figure("Name", "Tip Trajectory xy plane");
plot(interp_xy_tip(:, 1), interp_xy_tip(:, 2), 'LineWidth', 1)
hold on
plot(xyz_FBGS(1, :), xyz_FBGS(2, :), "r", "LineWidth", 1.0)
grid on
xlim([-.35 .35])
ylim([-.35 .35])
xlabel("p_x [m]")
ylabel("p_y [m]")
savefig(saving_fig_folder + fig.Name)
saveas(fig, saving_fig_folder + fig.Name, 'png')




%% Temporal correlation

sync_results = check_temporal_sync(time_actuators, measured_angles, ...
    mocap_timestamps, rel_kinematics_disks_corr, ...
    fbgs_time, fbgs_shapes, FBGS_tip_index, saving_folder);







%%  On the processed data, perform comparisons



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
    index_plot = it;
    subplot(3,1,index_plot)

    plot(sampling_time, XYZ_xyz_disk(:, it + 3), "b", "LineWidth", 2.0)
    set(gca,"FontSize",20)
    hold on
    plot(sampling_time, xyz_FBGS(it, :), "r", "LineWidth", 2.0)
    set(gca,"FontSize",20)

    grid on
    ylabel([vars{it} ' [m]'], "FontSize", 20)


    if it == 3
        xlabel("Time [s]", "FontSize", 20)
    end

    % if it == 1
    %     title("Raw")
    % end

end

% legend('OptiTrack', 'FBGS')
savefig(saving_fig_folder + fig.Name)
saveas(fig, saving_fig_folder + fig.Name, 'png')

RMSE_tip = rmse(xyz_FBGS', XYZ_xyz_disk(:, 4:6))

%   Compute range of motion
range_tip = max(XYZ_xyz_disk(:, 4:6)) - min(XYZ_xyz_disk(:, 4:6));

RMSE_tip_perc_motion = (RMSE_tip./range_tip)*100

%   Mocap and cables
N_interp = 10;
[delta_cable_measured, delta_cable_computed] = compare_cable_lenght(interp_rel_kinematics_disks, interp_angles, sampling_time, N_interp);

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
        % if k == 1
        %     legend('MoCap (computed)', 'Motor (measured)', 'Mocap (10)''Location', 'best')
        % end
        if k == 2, xlabel('Time [s]'); end
    end
    savefig(saving_fig_folder + fig.Name)
    saveas(fig, saving_fig_folder + fig.Name, 'png')
end

RMSE_cables = rmse(delta_cable_computed, delta_cable_measured)
%   Compute range of motion
range_cables = max(delta_cable_measured) - min(delta_cable_measured);
RMSE_cables_perc_motion = (RMSE_cables./range_cables)*100;
idx_0 = find(range_cables <= 1e-2);
RMSE_cables_perc_motion(idx_0) = 0*RMSE_cables_perc_motion(idx_0)



% Save RMSEs
fid = fopen(fullfile(saving_folder , "RMSEs.txt"), 'w');
fprintf(fid, 'RMSE_tip = [%s]\n', strjoin(string(RMSE_tip), ', '));
fprintf(fid, 'RMSE_tip_perc_motion = [%s]\n', strjoin(string(RMSE_tip_perc_motion), ', '));
fprintf(fid, 'RMSE_cables = [%s]\n', strjoin(string(RMSE_cables), ', '));
fprintf(fid, 'RMSE_cables_perc_motion = [%s]\n', strjoin(string(RMSE_cables_perc_motion), ', '));

% 3. Close the file
fclose(fid);




%%  Save the interpolated data
interp_time_angles      = [sampling_time interp_angles];
interp_time_tensions    = [sampling_time interp_tensions];
interp_time_base_wrench = [sampling_time interp_base_wrench];
% interp_time_base_wrench_raw = [sampling_time interp_base_wrench_raw];
% interp_time_mocap_frames = reshape(interp_rel_kinematics_disks, [N_samples, 6*N_disks]);
interp_time_mocap_frames_corr = reshape(interp_rel_kinematics_disks_corr, [N_samples, 6*N_disks]);
% interp_time_mocap_frames = [sampling_time interp_time_mocap_frames];
interp_time_mocap_frames_corr = [sampling_time interp_time_mocap_frames_corr];

if use_resense
    interp_wrench_wand = [sampling_time interp_wrench_wand];
end




writematrix(interp_time_angles, fullfile(saving_folder , "angles.csv"));
writematrix(interp_time_tensions, fullfile(saving_folder ,"cable_tensions.csv"));
writematrix(interp_time_base_wrench, fullfile(saving_folder , "base_wrench.csv"));
% writematrix(interp_time_base_wrench_raw, fullfile(saving_folder , "base_wrench_raw.csv"));
% writematrix(interp_time_mocap_frames, fullfile(saving_folder , "mocap_frames.csv"));
writematrix(interp_time_mocap_frames_corr, fullfile(saving_folder , "mocap_frames.csv"));

%   FBGS: save as N_samples x (1 + 3*N_fbgs_points)
%   columns: [time, x_0..x_501, y_0..y_501, z_0..z_501]
interp_fbgs_flat = reshape(permute(interp_fbgs_shapes, [3 1 2]), N_samples, []);
interp_time_fbgs = [sampling_time interp_fbgs_flat];
writematrix(interp_time_fbgs, fullfile(saving_folder, "fbgs_shapes.csv"));

if use_resense
    writematrix(interp_wrench_wand, fullfile(saving_folder , "wrench_wand.csv"));
    % writematrix(interp_wrench_wand, fullfile(saving_folder , "wrench_wand.csv"));
end


%   Save the workspace as reference
save(fullfile(saving_folder, 'matlab_workspace'));


%% ====== HELPER FUNCTION ======
function y = butter_filtfilt(t, x, fc, n)
    Fs = 1/median(diff(t));                 % estimate sampling rate from timestamps
    [b,a] = butter(n, fc/(Fs/2), "low");    % Butterworth
    y = filtfilt(b,a, x);                   % zero-phase filtering
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