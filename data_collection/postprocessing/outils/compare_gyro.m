close all;
clear;
clc;

%% ====== PATHS / SETTINGS ======
folder = fullfile("dataCollectionPack","20260304/gyro/","gyro_test/");




%   Bending plane: set to 'x' or 'y' — the axis along which the rod bends
bending_axis = 'x';        % 'y' for plane_y experiments, 'x' for plane_x



%% ====== LOAD DATA ======


filename = fullfile(folder, "dataOptiTrack.csv");
[N_disks, mocap_timestamps, poses_disks, rel_poses_disks, rel_kinematics_disks] = data_optitrack(filename);


filename = fullfile(folder, "dataFBGS.csv");
[fbgs_time, fbgs_shapes] = data_fbgs(filename);



XYZ_xyz = rel_kinematics_disks(:, :, 5);
index = 476;
xyz_FBGS = squeeze( fbgs_shapes(:, index, :) );
xyz_FBGS_end = squeeze( fbgs_shapes(:, end, :) );



%   Apply rotation of 90 deg along y axis to ALL shapes
R_y = axang2rotm([0 1 0 pi/2]);
N_time_fbgs = size(fbgs_shapes, 3);
for t = 1:N_time_fbgs
    fbgs_shapes(:, :, t) = R_y * fbgs_shapes(:, :, t);
end

%   The fiber now evolves in -z, but bending leaks into both x and y.
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


%   Re-extract plotting slices from the rotated shapes
xyz_FBGS     = squeeze(fbgs_shapes(:, index, :));
xyz_FBGS_end = squeeze(fbgs_shapes(:, end, :));

figure("Name", "Tip Position");
vars = {'p_x', 'p_y', 'p_z'};
for it = 1:3
    index_plot = it*2 -1;
    subplot(3,2,index_plot)

    plot(mocap_timestamps, XYZ_xyz(:, it + 3), "g", "LineWidth", 2.0)
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



rel_kinematics_disks_f = zeros(size(rel_kinematics_disks));
for it=1:N_disks

    for k=1:6  
        rel_kinematics_disks_f(:, k, it) = butter_filtfilt(mocap_timestamps, rel_kinematics_disks(:, k, it), cutoffHz, butterOrder);
        
    end
end





%% ====== HELPER FUNCTION ======
function y = butter_filtfilt(t, x, fc, n)
    Fs = 1/median(diff(t));                 % estimate sampling rate from timestamps
    [b,a] = butter(n, fc/(Fs/2), "low");    % Butterworth
    y = filtfilt(b,a, x);                   % zero-phase filtering
end
