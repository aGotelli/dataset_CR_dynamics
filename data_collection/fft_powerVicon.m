close all; clear; clc;

%% ====== PATHS / SETTINGS ======
folder = fullfile("dataCollectionPack","20260127","circle_angle_150_speed_3");

plot_disk_nums = 5; % Which Vicon disks to inspect (indices)

% % %% ====== LOAD DATA (unchanged from process_data.m) ======
% % motor = readtable(fullfile(folder, "datasequence_circle_radius_150p0.csv"));
% % 
% % mk_1_negx = readtable(fullfile(folder, "dataMark10_1_-x.csv"));
% % mk_1_x    = readtable(fullfile(folder, "dataMark10_1_x.csv"));
% % mk_2_negy = readtable(fullfile(folder, "dataMark10_2_-y.csv"));
% % mk_2_y    = readtable(fullfile(folder, "dataMark10_2_y.csv"));
% % 
% % ati = readtable(fullfile(folder, "dataATIFT.csv"));

N_frames_static_begin = 10; % how many frames to use to compute relative pose for Vicon
filename_vicon = fullfile(folder, "dataVicon.csv");
[N_disks, timestamp_vicon, rel_kinematics_disks] = data_vicon(filename_vicon, N_frames_static_begin);

% % %% ====== PREPARE SIGNALS ======
% % time_actuators = motor.timestamp;
% % measured_angles   = [motor.rel_angle1_rad, motor.rel_angle2_rad, motor.rel_angle3_rad, motor.rel_angle4_rad];
% % 
% % time_cables = cell(1,4);
% % cable_tensions  = cell(1,4);
% % time_cables{1} = mk_1_x.timestamp;       cable_tensions{1} = mk_1_x.tension_N_;
% % time_cables{2} = mk_2_y.timestamp;       cable_tensions{2} = mk_2_y.tension_N_;
% % time_cables{3} = mk_1_negx.timestamp;    cable_tensions{3} = mk_1_negx.tension_N_;
% % time_cables{4} = mk_2_negy.timestamp;    cable_tensions{4} = mk_2_negy.tension_N_;
% % 
% % tA = ati.timestamp;
% % ATI_F = [ati.Fx_N_, ati.Fy_N_, ati.Fz_N_];
% % ATI_T = [ati.Tx_Nm_, ati.Ty_Nm_, ati.Tz_Nm_];
% % 
% % %% ====== Sampling frequencies (median dt) ======
% % fs_motor = 1/median(diff(time_actuators));
% % fs_cables = zeros(1,4);
% % for i=1:4
% %     fs_cables(i) = 1/median(diff(time_cables{i}));
% % end
% % fs_ati = 1/median(diff(tA));
% % fs_vicon = 1/median(diff(timestamp_vicon));

%% ====== FFT su Vicon (posizioni e rotazioni) ======
% Scegli il disco
figure

for disk_idx = 1:N_disks

    x = rel_kinematics_disks(:, (1:3), disk_idx); % Nx6: X Y Z rotX rotY rotZ
    N  = size(x,1);       
    fs = 1/median(diff(timestamp_vicon));     % frequenza di campionamento Vicon
    
    y = fft(x);
    f = (0:N-1)*(fs/N);     % frequency range
    power = abs(y).^2/N;    % power of the DFT
    
    subplot(3,2,disk_idx)
    plot(f,power)
    xlabel('Frequency')
    ylabel('Power')
    title (['fft - disk-' num2str(disk_idx)],FontSize=12)
end
