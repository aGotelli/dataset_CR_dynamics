clc
clear all
close all

% ajoute le dossier outils
addpath('Dyn_Essai_release_Beam_Andrea/')
addpath('Dyn_Essai_release_Beam_Andrea/outils/')
addpath('Dyn_Essai_release_Beam_Andrea/IDM/')
addpath('Dyn_Essai_release_Beam_Andrea/methode spectral/')

Config.option=odeset('RelTol',10^(-8),'AbsTol',10^(-8));


path = "..\data_collection\dataCollectionPack\planar motion\plane_x_angle_90_speed_2\processed\";


%   If you want to plot in real time
Config.plot = false;


%%  Define position frames Vicon
Config.X_meas = [0, 0.1189, 0.2388, 0.3577, 0.48];
% Config.observation_points = Config.X_meas/Config.L; %   Normalized domain


%%  Defining deformable DoFs
% K1, K2, K3, ...
Config.V_a = [ 0, 1, 1, 0, 0, 0];

%   Defining size of the basis
Const.dim_base_k = [0, 3, 3, 0, 0, 0];

%   Resulting dimension q
Const.dim_base   = Config.V_a*Const.dim_base_k';


%%  Load gemetric parameters
parameters_dataset_robot




%% Pose of the base
r_0 = [0;0;0];
Q_0 = [0.7071068 0 0.7071068 0]';


%%  Static init

Const.q         = zeros(Const.dim_base,1);
Const.q_dot     = zeros(Const.dim_base,1);
Const.q_dot_dot = zeros(Const.dim_base,1);

Const.eta_0   = zeros(6,1);
Const.eta_dot_0 = zeros(6,1);

Const.r_0 = r_0;
Const.q_0 = Q_0; 
Const.F1 = zeros(6,1);



%%  Take data measurements

cable_tensions = load(path + "cable_tensions.csv");
time_angles_motor = load(path + "angles.csv");




Config.data.dt = 0.01;
Config.data.time = cable_tensions(:, 1);


%   Compute difference tension
tau_1 = cable_tensions(:, 2) - cable_tensions(:, 4);
tau_2 = cable_tensions(:, 3) - cable_tensions(:, 5);
Config.data.tau = [
    tau_1'
    tau_2'
];

% 
% plot(tau_1);
% plot(tau_2);


%% Time integration






[q,q_dot,q_dot_dot, position_disks_simu, simulated_wrench_at_base, cables_displacements] = Time_integration_Newton_beam_actuated_spectral(Const, Config);

save(path + "simulation_results")


%%  Plotting results
close all
load(path + "simulation_results")

vicon_frames_stacked = load(path + "vicon_frames.csv");
N_times_vicon = size(vicon_frames_stacked, 1);
vicon_frames = reshape(vicon_frames_stacked, [N_times_vicon, 7, 5]);
disk_num = 5;
time_kinematics_tip = vicon_frames(:, :, disk_num);
time_simu = time_kinematics_tip(:, 1);


time_base_wrench = load(path + "base_wrench.csv");



tip_frame = squeeze( position_disks_simu(5, :, :) )';


figure("Name", "Tip Position")
subplot(3, 1, 1)
plot(time_kinematics_tip(:, 1), time_kinematics_tip(:, 2), 'b', 'LineWidth', 1)
hold on
plot(Config.data.time, tip_frame(:, 1), 'r', 'LineWidth', 1)
grid on

subplot(3, 1, 2)
plot(time_kinematics_tip(:, 1), time_kinematics_tip(:, 3), 'b', 'LineWidth', 1)
hold on
plot(Config.data.time, tip_frame(:, 2), 'r', 'LineWidth', 1)
grid on

subplot(3, 1, 3)
plot(time_kinematics_tip(:, 1), time_kinematics_tip(:, 4), 'b', 'LineWidth', 1)
hold on
plot(Config.data.time, tip_frame(:, 3), 'r', 'LineWidth', 1)
grid on
legend('Measured', 'Simulated')
% 
% figure("Name", "Wrench at base (Torques)")
% subplot(3, 1, 1)
% plot(time_base_wrench(:, 1), time_base_wrench(:, 2), 'b', 'LineWidth', 1)
% hold on
% plot(time_simu, simulated_wrench_at_base(:, 1), 'r', 'LineWidth', 1)
% 
% subplot(3, 1, 2)
% plot(time_base_wrench(:, 1), time_base_wrench(:, 3), 'b', 'LineWidth', 1)
% hold on
% plot(time_simu, simulated_wrench_at_base(:, 2), 'r', 'LineWidth', 1)
% 
% subplot(3, 1, 3)
% plot(time_base_wrench(:, 1), time_base_wrench(:, 4), 'b', 'LineWidth', 1)
% hold on
% plot(time_simu, simulated_wrench_at_base(:, 3), 'r', 'LineWidth', 1)
% 
% 
% 
% figure("Name", "Wrench at base (Forces)")
% subplot(3, 1, 1)
% plot(time_base_wrench(:, 1), time_base_wrench(:, 5), 'b', 'LineWidth', 1)
% hold on
% plot(time_simu, simulated_wrench_at_base(:, 4), 'r', 'LineWidth', 1)
% 
% subplot(3, 1, 2)
% plot(time_base_wrench(:, 1), time_base_wrench(:, 6), 'b', 'LineWidth', 1)
% hold on
% plot(time_simu, simulated_wrench_at_base(:, 5), 'r', 'LineWidth', 1)
% 
% subplot(3, 1, 3)
% plot(time_base_wrench(:, 1), time_base_wrench(:, 7), 'b', 'LineWidth', 1)
% hold on
% plot(time_simu, simulated_wrench_at_base(:, 6), 'r', 'LineWidth', 1)
% 

figure("Name", "Torque")
plot(time_base_wrench(:, 1), time_base_wrench(:, 6), 'b', 'LineWidth', 1)
hold on
plot(time_simu, -simulated_wrench_at_base(:, 2), 'r', 'LineWidth', 1)
legend('Measured', 'Simulated')


%   Plot the cable displacements
cable_spool_radius = 0.02;
time_cables = time_angles_motor(:, 1);
motor_angles = time_angles_motor(:, 2:end);
measured_cables_pulled = motor_angles*cable_spool_radius;
figure("Name", "Cable Displacement")
subplot(2, 1, 1)
plot(time_cables, measured_cables_pulled(:, 1), 'b', 'LineWidth', 1)
hold on 
plot(time_simu, cables_displacements(:, 1), 'r', 'LineWidth', 1)
grid on


subplot(2, 1, 2)
plot(time_cables, measured_cables_pulled(:, 2), 'b', 'LineWidth', 1)
hold on
plot(time_simu, cables_displacements(:, 2), 'r', 'LineWidth', 1)
grid on

legend('Measured', 'Simulated')
% 
% %%  Compare data with vicon
% 
% rod_positions = zeros(length(Config.X_meas), 3, Config.Nt);
% for it_t=1:Config.Nt
% 
%     q_it = q(:, it_t);
%     dq_it = q_dot(:, it_t);
%     ddq_it = q_dot_dot(:, it_t);
% 
%     Const.q = q_it;
%     Const.dq = dq_it;
%     Const.ddq = ddq_it;
% 
%     CI = [Const.q_0;Const.r_0;Const.eta_0];
% 
%     [X X3] = ode45(@(t,y) Forward_eta(t,y,time,Const,Config), Config.X_meas,CI,Config.option); % Solve ODE
% 
% 
%     rod_positions(:, :, it_t) = X3(:,5:7);
% 
% end
% 
% %%  Plots
% 
% close all
% 
% 
% %plot vicon trajectory
% disk_num = 5;
% xyz_XYZ = rel_kinematics_disks{disk_num};
% begin_release_plot = begin_release + 25;
% tip_position_vicon = xyz_XYZ(begin_release_plot:end, 1:3)';
% time_vicon_rel = time_vicon(begin_release_plot:end) - time_vicon(begin_release_plot);
% 
% 
% 
% 
% %   Plot the tip position
% tip_position = squeeze( rod_positions(end, :, :) );
% 
% last = find(time_vicon_rel > 10, 1, 'first');
% 
% tip_position_vicon = tip_position_vicon(:, 1:last);
% time_vicon_plot = time_vicon_rel(1:last);
% 
% figure("Name","Tip Position")
% subplot(3, 1, 1)
% plot(time_vicon_plot, tip_position_vicon(1, :), '-b')
% hold on;
% plot(Config.time, tip_position(1, :), '-r')
% grid on
% xlabel("Time [s]")
% ylabel("p_x [m]")
% 
% subplot(3, 1, 2)
% plot(time_vicon_plot, tip_position_vicon(2, :), '-b')
% hold on;
% plot(Config.time, tip_position(2, :), '-r')
% grid on
% xlabel("Time [s]")
% ylabel("p_y [m]")
% 
% subplot(3, 1, 3)
% plot(time_vicon_plot, tip_position_vicon(3, :), '-b')
% hold on;
% plot(Config.time, tip_position(3, :), '-r')
% grid on
% xlabel("Time [s]")
% ylabel("p_z [m]")
% 
% 
% function error_norm = findConfig(q, Const, Config)
% 
%     positions_simu = rod_shape(0, q, Const, Config)';
% 
%     figure(1)
%     plot3(positions_simu(1, :), positions_simu(2, :), positions_simu(3, :), '-r')
%     hold on
%     plot3(Const.position_data(1, :), Const.position_data(2, :), Const.position_data(3, :), '-b')
%     grid on;
%     xlim([-.5, .5]);
%     ylim([-.5, .5]);
%     zlim([-.5, .5]);
%     drawnow;
%     error = positions_simu - Const.position_data;
%     error_norm = norm(error(:));
% 
% end
