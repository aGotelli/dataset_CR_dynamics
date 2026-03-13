close all;
clear;
clc;

addpath("outils\")

%% ====== PATHS / SETTINGS ======
folder = fullfile("..", "dataCollectionPack","20260304/","plane_y_fast/");

cutoffHz    = 30;   % Butterworth cutoff
butterOrder = 4;


samplingHz = 100;


%   Bending plane: set to 'x' or 'y' — the axis along which the rod bends
bending_axis = 'y';        % 'y' for plane_y experiments, 'x' for plane_x

%   Plots
plot_filtered = false;
plot_interpolation = true;
plot_disk_num = 5;


%% ====== LOAD DATA ======
motor = readtable(fullfile(folder, "dataMotor.csv"));

mk_1_negx = readtable(fullfile(folder, "dataMark10_-x.csv"));
mk_1_x    = readtable(fullfile(folder, "dataMark10_+x.csv"));
mk_2_negy = readtable(fullfile(folder, "dataMark10_-y.csv"));
mk_2_y    = readtable(fullfile(folder, "dataMark10_+y.csv"));

ati = readtable(fullfile(folder, "dataATIFT.csv"));

filename = fullfile(folder, "dataOptiTrack.csv");
[N_disks, mocap_timestamps, poses_disks, rel_poses_disks, rel_kinematics_disks] = data_optitrack(filename);

%%  TODO: Check convention frames in mocap data processing


% 
% %  Extract the data from the static measures
% filename = "dataCollectionPack\20260310\static_test\dataOptiTrack.csv";
% [N_disks_static, mocap_timestamps_static, poses_disks_static, rel_poses_disks_static, rel_kinematics_disks_static] = data_optitrack(filename);
% 
% %% ====== CORRECT LOCAL-FRAME BIAS (STATIC CALIBRATION) ======
% %   Straight rod => ideal pose is pure z-translation.
% %   Bias lives in local frame: T_meas = T_true * T_bias
% %   => T_corrected = T_dyn * inv(T_bias)
% for it = 2:N_disks
%     T_static = mean(rel_poses_disks_static(:,:,it,:), 4);
%     T_ideal  = eye(4);  T_ideal(3,4) = T_static(3,4);
%     inv_T_bias = T_static \ T_ideal;          % = inv(T_bias)
%     rel_poses_disks(:,:,it,:) = pagemtimes(rel_poses_disks(:,:,it,:), inv_T_bias);
%     %   Recompute kinematics for this disk
%     R_corr = squeeze(rel_poses_disks(1:3,1:3,it,:));
%     r_corr = squeeze(rel_poses_disks(1:3,4,it,:));
%     rel_kinematics_disks(:,:,it) = [rotm2eul(R_corr,'XYZ')  r_corr'];
% end


filename = fullfile(folder, "dataFBGS.csv");
[fbgs_time, fbgs_shapes] = data_fbgs(filename);


%   Apply rotation of -90 deg along y axis to ALL shapes
R_y = axang2rotm([0 1 0 -pi/2]);
N_time_fbgs = size(fbgs_shapes, 3);
for t = 1:N_time_fbgs
    fbgs_shapes(:, :, t) = R_y * fbgs_shapes(:, :, t);
end

%   The fiber now evolves in z, but bending leaks into both x and y.
%   Use SVD on the tip x-y trajectory (first 10 s only, planar portion)
%   to find the bending direction, then rotate about z.
align_window_s = 10;
fbgs_time_rel  = fbgs_time - fbgs_time(1);
idx_align      = fbgs_time_rel <= align_window_s;

tip_xy_all     = squeeze(fbgs_shapes(1:2, end, :));   % 2 x N_time
tip_xy         = tip_xy_all(:, idx_align);            % 2 x N_align
tip_xy_centered = tip_xy - mean(tip_xy, 2);
[U_xy, ~, ~] = svd(tip_xy_centered, 'econ');

%   U_xy(:,1) = principal direction of tip motion in the x-y plane.
%   Rotate it onto the desired bending axis.
alpha = atan2(U_xy(2,1), U_xy(1,1));
if strcmpi(bending_axis, 'y')
    theta_z = pi/2 - alpha;       % map onto y-axis
else
    theta_z = 0 - alpha;          % map onto x-axis
end


R_z = [ cos(theta_z), -sin(theta_z), 0;
        sin(theta_z),  cos(theta_z), 0;
        0,              0,           1];

for t = 1:N_time_fbgs
    fbgs_shapes(:, :, t) = R_z * fbgs_shapes(:, :, t);
end


if strcmpi(bending_axis, 'y')
    fbgs_shapes(1, :, :) = - fbgs_shapes(1, :, :);
else
    fbgs_shapes(2, :, :) = - fbgs_shapes(2, :, :);
end


%   Extract plotting slices from the rotated shapes
XYZ_xyz_disk = rel_kinematics_disks(:, :, 5);
index = 480;
xyz_FBGS     = squeeze(fbgs_shapes(:, index, :));
xyz_FBGS_end = squeeze(fbgs_shapes(:, end, :));

figure("Name", "Tip Position");
vars = {'p_x', 'p_y', 'p_z'};
for it = 1:3
    index_plot = it;
    subplot(3,1,index_plot)

    plot(mocap_timestamps, XYZ_xyz_disk(:, it + 3), "g", "LineWidth", 2.0)
    hold on
    plot(fbgs_time, xyz_FBGS(it, :), "r", "LineWidth", 2.0)
    % plot(fbgs_time - fbgs_time(1), xyz_FBGS_end(it, :), "r", "LineWidth", 2.0)

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
% plot(mocap_timestamps,   'k',  'LineWidth', 1.5);   % uncomment when OptiTrack is enabled
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
ATI_FT = [ATI_F ATI_T];

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
        rel_kinematics_disks_f(:, k, it) = butter_filtfilt(mocap_timestamps, rel_kinematics_disks(:, k, it), cutoffHz, butterOrder);
        
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
for it=1:N_disks

    for k=1:6  
        interp_rel_kinematics_disks(:, k, it) = interp1(relative_time_mocap, rel_kinematics_disks_f(:, k, it), sampling_time);
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




end

%%  SAVING DATA AND PLOTS
saving_folder = fullfile( folder,  "processed/");
saving_fig_folder = fullfile( saving_folder,  "figures/");
mkdir(saving_folder);
mkdir(saving_fig_folder);


%%  Plot robot tip
interp_xy_tip = interp_rel_kinematics_disks(:, 4:5, 5);
fig = figure("Name", "Tip Trajectory xy plane");
plot(interp_xy_tip(:, 1), interp_xy_tip(:, 2), 'LineWidth', 1)
grid on
grid on
xlim([-.35 .35])
ylim([-.35 .35])
xlabel("p_x [m]")
ylabel("p_y [m]")
savefig(saving_fig_folder + fig.Name)
saveas(fig, saving_fig_folder + fig.Name, 'png')




%%  On the processed data, perform comparisons

%   FBGS and Mocap
%   Extract plotting slices from the rotated shapes
XYZ_xyz_disk = interp_rel_kinematics_disks(:, :, 5);
index = 480;
xyz_FBGS     = squeeze(interp_fbgs_shapes(:, index, :));

fig = figure("Name", "Tip Position Interpolated");
vars = {'p_x', 'p_y', 'p_z'};
for it = 1:3
    index_plot = it;
    subplot(3,1,index_plot)

    plot(sampling_time, XYZ_xyz_disk(:, it + 3), "g", "LineWidth", 2.0)
    hold on
    plot(sampling_time, xyz_FBGS(it, :), "r", "LineWidth", 2.0)

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
savefig(saving_fig_folder + fig.Name)
saveas(fig, saving_fig_folder + fig.Name, 'png')

RMSE_tip = rmse(xyz_FBGS', XYZ_xyz_disk(:, 4:6))

%   Compute range of motion
range_tip = max(XYZ_xyz_disk(:, 4:6)) - min(XYZ_xyz_disk(:, 4:6));

RMSE_tip_perc_motion = (RMSE_tip./range_tip)*100


%   Mocap and cables
[delta_cable_measured, delta_cable_computed] = compare_cable_lenght(interp_rel_kinematics_disks, interp_angles, sampling_time);

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
        plot(sampling_time, delta_cable_computed(:,c)*1e3,  'b',  'LineWidth', 1.5);  hold on
        plot(sampling_time, delta_cable_measured(:,c)*1e3,  'r--','LineWidth', 1.5);
        grid on; ylabel('\DeltaL [mm]')
        title(['Cable ' cable_labels{c}])
        if k == 1
            legend('MoCap (computed)', 'Motor (measured)', 'Location', 'best')
        end
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
interp_time_base_wrench_raw = [sampling_time interp_base_wrench_raw];
interp_time_mocap_frames = zeros(N_samples, 7, N_disks);
for it=1:N_disks
    interp_time_mocap_frames(:, :, it) = [sampling_time interp_rel_kinematics_disks(:, :, it)];
end




writematrix(interp_time_angles, fullfile(saving_folder , "angles.csv"));
writematrix(interp_time_tensions, fullfile(saving_folder ,"cable_tensions.csv"));
writematrix(interp_time_base_wrench, fullfile(saving_folder , "base_wrench.csv"));
writematrix(interp_time_base_wrench_raw, fullfile(saving_folder , "base_wrench_raw.csv"));
writematrix(interp_time_mocap_frames, fullfile(saving_folder , "mocap_frames.csv"));

%   FBGS: save as N_samples x (1 + 3*N_fbgs_points)
%   columns: [time, x_0..x_501, y_0..y_501, z_0..z_501]
interp_fbgs_flat = reshape(permute(interp_fbgs_shapes, [3 1 2]), N_samples, []);
interp_time_fbgs = [sampling_time interp_fbgs_flat];
writematrix(interp_time_fbgs, fullfile(saving_folder, "fbgs_shapes.csv"));

%   Save the workspace as reference
save(fullfile(saving_folder, 'matlab_workspace'));


%% ====== HELPER FUNCTION ======
function y = butter_filtfilt(t, x, fc, n)
    Fs = 1/median(diff(t));                 % estimate sampling rate from timestamps
    [b,a] = butter(n, fc/(Fs/2), "low");    % Butterworth
    y = filtfilt(b,a, x);                   % zero-phase filtering
end
