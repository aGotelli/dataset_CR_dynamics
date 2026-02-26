close all;
clear;
clc;


%% ====== PATHS / SETTINGS ======
folder = fullfile("dataCollectionPack\","20260225\", "plane_x_slow/");

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
[N_disks, timestamp_vicon, rel_kinematics_disks] = data_optitrack(filename, N_frames_static_begin);

filename = fullfile(folder, "dataFBGS.csv");
[fbgs_time, fbgs_shapes] = data_fbgs(filename);



%%  Postprocess FBGS data

%   FBGS grows in the X axis so it needs to be rotated down




figure("Name", "Tip Position");
xyz_XYZ = rel_kinematics_disks(:, :, 5);
xyz_FBGS = squeeze( fbgs_shapes(:, end, :) );

for it = 1:3
    index_plot = it*2 -1;
    subplot(3,2,index_plot)

    plot(timestamp_vicon - timestamp_vicon(1), xyz_XYZ(:, it), "b", "LineWidth", 2.0)
    hold on
    plot(fbgs_time - fbgs_time(1), xyz_FBGS(it, :), "r", "LineWidth", 2.0)
    ylabel("Position [m]")
    grid on



    if it == 3
        xlabel("Time [s]")
    end

end
legend('OptiTrack', 'FBGS')

return


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

% close all
%% ====== HELPER FUNCTION ======
function y = butter_filtfilt(t, x, fc, n)
    Fs = 1/median(diff(t));                 % estimate sampling rate from timestamps
    [b,a] = butter(n, fc/(Fs/2), "low");    % Butterworth
    y = filtfilt(b,a, x);                   % zero-phase filtering
end
