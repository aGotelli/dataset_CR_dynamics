close all;
clear;
clc;

%% ====== PATHS / SETTINGS ======
folder = fullfile("dataCollectionPack","20260225/","circle_slow/");

cutoffHz    = 20;   % Butterworth cutoff
butterOrder = 4;


samplingHz = 100;


%   Plots
plot_filtered = true;
plot_interpolation = true;
plot_disk_num = 5;


%% ====== LOAD DATA ======
motor = readtable(fullfile(folder, "dataMotor.csv"));

mk_1_negx = readtable(fullfile(folder, "dataMark10_-x.csv"));
mk_1_x    = readtable(fullfile(folder, "dataMark10_+x.csv"));
mk_2_negy = readtable(fullfile(folder, "dataMark10_-y.csv"));
mk_2_y    = readtable(fullfile(folder, "dataMark10_+y.csv"));

ati = readtable(fullfile(folder, "dataATIFT.csv"));

N_frames_static_begin = 10; %how many frames to use to compute relative pose for Vicon
filename = fullfile(folder, "dataOptiTrack.csv");
% [N_disks, timestamp_vicon, rel_kinematics_disks] = data_optitrack(filename, N_frames_static_begin);

filename = fullfile(folder, "dataFBGS.csv");
[fbgs_time, fbgs_shapes] = data_fbgs(filename);


figure("Name", "Tip Position");
% xyz_XYZ = rel_kinematics_disks(:, :, 5);
xyz_FBGS = squeeze( fbgs_shapes(:, end, :) );

vars = {'p_x', 'p_y', 'p_z'};
for it = 1:3
    index_plot = it*2 -1;
    subplot(3,2,index_plot)

    % plot(timestamp_vicon - timestamp_vicon(1), xyz_XYZ(:, it), "b", "LineWidth", 2.0)
    hold on
    plot(fbgs_time - fbgs_time(1), xyz_FBGS(it, :), "b", "LineWidth", 2.0)
    grid on
    ylabel([vars{it} ' [m]'])


    if it == 3
        xlabel("Time [s]")
    end

    if it == 1
        title("Raw")
    end

end


%%  Align FBGS frame with robot frame via SVD on tip trajectory
%   For plane_x motion the tip moves in the robot x-z plane -> zero motion in y.
%   SVD on the (centered) tip positions gives principal directions:
%     U(:,1)  most variance  -> robot x-axis (lateral bending direction)
%     U(:,3)  least variance -> robot y-axis (plane normal)
%     z = cross(x,y)         -> robot z-axis (right-handed)

align_window_s = 10;
fbgs_time_rel = fbgs_time - fbgs_time(1);
idx_align = fbgs_time_rel <= align_window_s;

tip_pos_all = squeeze(fbgs_shapes(:, end, :));   % 3 x N_time
tip_pos = tip_pos_all(:, idx_align);             % 3 x N_time_align

if size(tip_pos, 2) < 3
    warning('Less than 3 FBGS samples in first %.2f s; using all samples for alignment.', align_window_s);
    tip_pos = tip_pos_all;
end

tip_mean   = mean(tip_pos, 2);
tip_centered = tip_pos - tip_mean;

[U, ~, ~] = svd(tip_centered, 'econ');

x_hat = U(:, 1);                       % primary motion direction
y_hat = U(:, 3);                       % plane normal (least variance)
z_hat = cross(x_hat, y_hat);           % right-handed z
z_hat = z_hat / norm(z_hat);

%   Build initial rotation from SVD basis
R_align = [x_hat, y_hat, z_hat]';

%   1) Enforce initial tip in negative z direction
[~, idx_start] = min(abs(fbgs_time_rel - 0));
tip_start_aligned = R_align * tip_pos_all(:, idx_start);
if tip_start_aligned(3) > 0
    R_pi_about_x = [1 0 0; 0 -1 0; 0 0 -1];
    R_align = R_pi_about_x * R_align;
end

%   2) Enforce known initial motion direction (+x) with rotation about z
check_time = 1;
check_time_s = min(check_time, align_window_s);
[~, idx_check] = min(abs(fbgs_time_rel - check_time_s));


tip_start_aligned = R_align * tip_pos_all(:, idx_start);
tip_check_aligned = R_align * tip_pos_all(:, idx_check);
delta_x_motion = tip_check_aligned(1) - tip_start_aligned(1);

if delta_x_motion < 0
    R_pi_about_z = [-1 0 0; 0 -1 0; 0 0 1];
    R_align = R_pi_about_z * R_align;
end

%   Build rotation: R_align * p_fbgs = p_robot
%   (already built above, and corrected by rotation if needed)

%   Apply to every cross-section at every time step
N_time_fbgs = size(fbgs_shapes, 3);
for t = 1:N_time_fbgs
    fbgs_shapes(:, :, t) = R_align * fbgs_shapes(:, :, t);
end

% figure("Name", "Tip Position");
% xyz_XYZ = rel_kinematics_disks(:, :, 5);
xyz_FBGS = squeeze( fbgs_shapes(:, end, :) );

vars = {'p_x', 'p_y', 'p_z'};
for it = 1:3
    index_plot = it*2 ;
    subplot(3,2,index_plot)

    % plot(timestamp_vicon - timestamp_vicon(1), xyz_XYZ(:, it), "b", "LineWidth", 2.0)
    hold on
    plot(fbgs_time - fbgs_time(1), xyz_FBGS(it, :), "r", "LineWidth", 2.0)
    grid on
    ylabel([vars{it} ' [m]'])


    if it == 3
        xlabel("Time [s]")
    end

    if it == 1
        title("Realigned")
    end

end


%% ====== EXTRACT MOTOR SIGNALS ======
time_actuators = motor.timestamp;                     

target_angles = [motor.target1_rad, motor.target2_rad, motor.target3_rad, motor.target4_rad];
measured_angles   = [motor.rel_angle1_rad, motor.rel_angle2_rad, motor.rel_angle3_rad, motor.rel_angle4_rad];

%% ====== EXTRACT FORCES (RAW TIME) ======
% Your mapping:
% motor1 -> Mark10 1_x
% motor2 -> Mark10 2_y
% motor3 -> Mark10 1_-x
% motor4 -> Mark10 2_-y
time_cables = cell(1,4);%   Need to use cell as data has different lenght
cable_tensions  = cell(1,4);


time_cables{1} = mk_1_x.timestamp;       cable_tensions{1} = mk_1_x.tension_N_;
time_cables{2} = mk_2_y.timestamp;       cable_tensions{2} = mk_2_y.tension_N_;
time_cables{3} = mk_1_negx.timestamp;    cable_tensions{3} = mk_1_negx.tension_N_;
time_cables{4} = mk_2_negy.timestamp;    cable_tensions{4} = mk_2_negy.tension_N_;

%% ====== EXTRACT ATI FT (RAW TIME) ======
tA = ati.timestamp;
% tA_dt = datetime(tA, "ConvertFrom","posixtime");

ATI_F = [ati.Fx_N_, ati.Fy_N_, ati.Fz_N_];
ATI_T = [ati.Tx_Nm_, ati.Ty_Nm_, ati.Tz_Nm_];

%% ====== TIMESTAMP COMPARISON ======
%   Check that all sensors start recording BEFORE the motors begin moving.
%   Raw absolute timestamps are plotted so we can see which sensors lead/lag.

figure("Name", "Timesteps Comparisons");
plot(time_actuators,      'w',  'LineWidth', 1.5); hold on
plot(time_cables{1},      'r',  'LineWidth', 1.5);
plot(time_cables{2},      'g',  'LineWidth', 1.5);
plot(time_cables{3},      'b',  'LineWidth', 1.5);
plot(time_cables{4},      'c',  'LineWidth', 1.5);
plot(tA,                  'y',  'LineWidth', 1.5);
plot(fbgs_time,           'm',  'LineWidth', 1.5);
% plot(timestamp_vicon,   'k',  'LineWidth', 1.5);   % uncomment when OptiTrack is enabled
grid on
xlabel("Sample index")
ylabel("Absolute timestamp [s]")
title("Raw timestamps – verify all sensors start before motors")
legend("Motors", "Mark10 +x", "Mark10 +y", "Mark10 -x", "Mark10 -y", "ATI FT", "FBGS" ...
       , 'Location', 'northwest')
set(gca, 'Color', [0.15 0.15 0.15])   % dark background so white line is visible

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

fbgs_shapes_f = zeros(size(fbgs_shapes));   % 3 x 502 x N_time_fbgs
N_fbgs_points = size(fbgs_shapes, 2);
for coord = 1:3
    for s = 1:N_fbgs_points
        fbgs_shapes_f(coord, s, :) = butter_filtfilt(fbgs_time, squeeze(fbgs_shapes(coord, s, :)), cutoffHz, butterOrder);
    end
end


rel_kinematics_disks_f = zeros(size(rel_kinematics_disks));
for it=1:N_disks

    for k=1:6  
        rel_kinematics_disks_f(:, k, it) = butter_filtfilt(timestamp_vicon, rel_kinematics_disks(:, k, it), cutoffHz, butterOrder);
        
    end
end


%% ====== PLOT: 4 SUBPLOTS (MOTOR TARGET/MEAS + FORCE) ======

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
    
    
    
    
    % xyz_XYZ = rel_kinematics_disks_f(:, :, plot_disk_num);
    % 
    % x_data = xyz_XYZ(:, 1);
    % y_data = xyz_XYZ(:, 2);
    % z_data = xyz_XYZ(:, 3);
    % 
    % Rot_X = xyz_XYZ(:, 4);
    % Rot_Y = xyz_XYZ(:, 5);
    % Rot_Z = xyz_XYZ(:, 6);
    % 
    % figure("Name", "Kinematics Relative")
    % subplot(3, 2, 1)
    % plot(timestamp_vicon, Rot_X)
    % grid on
    % xlabel("Time [s]")
    % ylabel("Roll [RAD]")
    % 
    % subplot(3, 2, 2)
    % plot(timestamp_vicon, x_data)
    % grid on
    % xlabel("Time [s]")
    % ylabel("X [m]")
    % 
    % subplot(3, 2, 3)
    % plot(timestamp_vicon, Rot_Y)
    % grid on
    % xlabel("Time [s]")
    % ylabel("Pitch [RAD]")
    % 
    % subplot(3, 2, 4)
    % plot(timestamp_vicon, y_data)
    % grid on
    % xlabel("Time [s]")
    % ylabel("Y [m]")
    % 
    % subplot(3, 2, 5)
    % plot(timestamp_vicon, Rot_Z)
    % grid on
    % xlabel("Time [s]")
    % ylabel("Yaw [RAD]")
    % 
    % subplot(3, 2, 6)
    % plot(timestamp_vicon, z_data)
    % grid on
    % xlabel("Time [s]")
    % ylabel("Z [m]")


    figure("Name","Vicon disk kinematics" + int2str(plot_disk_num));
    xyz_XYZ = rel_kinematics_disks(:, :, plot_disk_num);
    xyz_XYZ_f = rel_kinematics_disks_f(:, :, plot_disk_num);
    
    
    for it = 1:3
        index_plot = it*2 -1;
        subplot(3,2,index_plot)
    
        plot(timestamp_vicon, xyz_XYZ(:, 3 + it), "b", "LineWidth", 2.0)
        hold on
        plot(timestamp_vicon, xyz_XYZ_f(:, 3 + it), "r", "LineWidth", 2.0)
        ylabel("Euler Angle [RAD]")
        grid on
    
       
    
        if it == 3
            xlabel("Time [s]")
        end
    
    end
    
    for it = 1:3
        index_plot = it*2;
        subplot(3,2,index_plot)
     
        plot(timestamp_vicon, xyz_XYZ(:, it), "b", "LineWidth", 2.0)
        hold on
        plot(timestamp_vicon, xyz_XYZ_f(:, it), "r", "LineWidth", 2.0)

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


relative_time_vicon = timestamp_vicon - time_start_motors;

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
for it=1:6

    interp_base_wrench(:, it) = interp1(relative_time_ATI, ATI_FT_f(:, it), sampling_time)';
end

interp_rel_kinematics_disks = zeros(N_samples, 6, N_disks);
for it=1:N_disks

    for k=1:6  
        interp_rel_kinematics_disks(:, k, it) = interp1(relative_time_vicon, rel_kinematics_disks_f(:, k, it), sampling_time);
    end
end

interp_fbgs_shapes = zeros(3, N_fbgs_points, N_samples);
for coord = 1:3
    for s = 1:N_fbgs_points
        interp_fbgs_shapes(coord, s, :) = interp1(relative_time_fbgs, squeeze(fbgs_shapes_f(coord, s, :)), sampling_time);
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





    figure("Name","Vicon disk " + int2str(plot_disk_num));
    xyz_XYZ_f = rel_kinematics_disks_f(:, :, plot_disk_num);
    interp_xyz_XYZ = interp_rel_kinematics_disks(:, :, plot_disk_num);
    
    
    for it = 1:3
        index_plot = it*2 -1;
        subplot(3,2,index_plot)
    
        plot(relative_time_vicon, xyz_XYZ_f(:, 3 + it), "b", "LineWidth", 2.0); hold on
        plot(sampling_time, interp_xyz_XYZ(:, 3 + it), "or","MarkerSize", 3);
        ylabel("Euler Angle [RAD]")
        grid on
    
       
    
        if it == 3
            xlabel("Time [s]")
        end
    
    end
    
    for it = 1:3
        index_plot = it*2;
        subplot(3,2,index_plot)
     
        plot(relative_time_vicon, xyz_XYZ_f(:, it), "b", "LineWidth", 2.0); hold on
        plot(sampling_time, interp_xyz_XYZ(:, it), "or","MarkerSize", 3);

        ylabel("Position [m]")
        grid on
    
       
    
        if it == 3
            xlabel("Time [s]")
        end
    
    end




end
%%  Save the interpolated data
interp_time_angles      = [sampling_time interp_angles];
interp_time_tensions    = [sampling_time interp_tensions];
interp_time_base_wrench = [sampling_time interp_base_wrench];
interp_time_vicon_frames = zeros(N_samples, 7, N_disks);
for it=1:N_disks
    interp_time_vicon_frames(:, :, it) = [sampling_time interp_rel_kinematics_disks(:, :, it)];
end


saving_folder = fullfile( folder,  "processed");
mkdir(saving_folder);

writematrix(interp_time_angles, fullfile(saving_folder , "angles.csv"));
writematrix(interp_time_tensions, fullfile(saving_folder ,"cable_tensions.csv"));
writematrix(interp_time_base_wrench, fullfile(saving_folder , "base_wrench.csv"));
writematrix(interp_time_vicon_frames, fullfile(saving_folder , "vicon_frames.csv"));

%   FBGS: save as N_samples x (1 + 3*N_fbgs_points)
%   columns: [time, x_0..x_501, y_0..y_501, z_0..z_501]
interp_fbgs_flat = reshape(permute(interp_fbgs_shapes, [3 1 2]), N_samples, []);
interp_time_fbgs = [sampling_time interp_fbgs_flat];
writematrix(interp_time_fbgs, fullfile(saving_folder, "fbgs_shapes.csv"));


%% ====== HELPER FUNCTION ======
function y = butter_filtfilt(t, x, fc, n)
    Fs = 1/median(diff(t));                 % estimate sampling rate from timestamps
    [b,a] = butter(n, fc/(Fs/2), "low");    % Butterworth
    y = filtfilt(b,a, x);                   % zero-phase filtering
end
