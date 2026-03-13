clc
clear all
close all


path = fullfile("..", "dataCollectionPack", "20260304", 'plane_y_fast/');
load_path = fullfile(path, 'processed/');
savepath = fullfile(path, "gvs/");
saving_fig_folder = fullfile(savepath, "figures/");

mkdir(savepath)
mkdir(saving_fig_folder)

% ajoute le dossier outils
addpath(genpath("GVS_outils"))
addpath('GVS_outils/outils/')
addpath('GVS_outils/IDM/')
addpath('GVS_outils/methode spectral/')

Config.option=odeset('RelTol',10^(-8),'AbsTol',10^(-8));


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

cable_tensions = load(fullfile(load_path, "cable_tensions.csv"));
time_angles_motor = load(fullfile(load_path, "angles.csv"));




Config.data.dt = 0.01;
Config.data.time = cable_tensions(:, 1);


%   Compute difference tension
tau_1 = cable_tensions(:, 2) - cable_tensions(:, 4);
tau_2 = cable_tensions(:, 3) - cable_tensions(:, 5);
Config.data.tau = [
    tau_1'
    tau_2'
];



%% Time integration

[q,q_dot,q_dot_dot, position_disks_simu, simulated_wrench_at_base, cables_displacements] = Time_integration_Newton_beam_actuated_spectral(Const, Config);

save(fullfile(savepath,"simulation_results"))


%%  Plotting results
close all
load(fullfile(savepath, "simulation_results"))

mocap_frames_stacked = load(fullfile(load_path, "mocap_frames.csv"));
N_times_vicon = size(mocap_frames_stacked, 1);
vicon_frames = reshape(mocap_frames_stacked, [N_times_vicon, 7, 5]);
disk_num = 5;
time_kinematics_tip = vicon_frames(:, :, disk_num);
time_simu = time_kinematics_tip(:, 1);


time_base_wrench = load(fullfile(load_path, "base_wrench.csv"));
fbgs_shapes_stacked = load(fullfile(load_path, "fbgs_shapes.csv"));

time_base_wrench_raw = load(fullfile(load_path, "base_wrench_raw.csv"));


tip_frame = squeeze( position_disks_simu(5, :, :) )';
time_fbgs = fbgs_shapes_stacked(:, 1);
fbgs_xyz_stacked = fbgs_shapes_stacked(:, 2:end);
tip_index_fbgs = 476;
tip_start_col = 3*(tip_index_fbgs - 1) + 1;
tip_fbgs = fbgs_xyz_stacked(:, tip_start_col:tip_start_col+2);


fig = figure("Name", "Tip Position");
subplot(3, 1, 1)
plot(time_kinematics_tip(:, 1), time_kinematics_tip(:, 5), 'g', 'LineWidth', 1)
hold on
plot(time_fbgs, tip_fbgs(:, 1), 'b', 'LineWidth', 1)
plot(Config.data.time, tip_frame(:, 1), 'r', 'LineWidth', 1)
grid on

subplot(3, 1, 2)
plot(time_kinematics_tip(:, 1), time_kinematics_tip(:, 6), 'g', 'LineWidth', 1)
hold on
plot(time_fbgs, tip_fbgs(:, 2), 'b', 'LineWidth', 1)
plot(Config.data.time, tip_frame(:, 2), 'r', 'LineWidth', 1)
grid on

subplot(3, 1, 3)
plot(time_kinematics_tip(:, 1), time_kinematics_tip(:, 7), 'g', 'LineWidth', 1)
hold on
plot(time_fbgs, tip_fbgs(:, 3), 'b', 'LineWidth', 1)
plot(Config.data.time, -tip_frame(:, 3), 'r', 'LineWidth', 1)
grid on
legend('OptiTrack', 'FBGS', 'Simulated')

savefig(saving_fig_folder + fig.Name)
saveas(fig, saving_fig_folder + fig.Name, 'png')


fig = figure("Name", "Torque");
subplot(2, 1, 1)
plot(time_simu, -simulated_wrench_at_base(:, 3), 'r', 'LineWidth', 1)
hold on
% plot(time_base_wrench(:, 1), time_base_wrench(:, 5), 'b', 'LineWidth', 2)
plot(time_base_wrench(:, 1), time_base_wrench_raw(:, 5), 'g', 'LineWidth', 2)



subplot(2, 1, 2)
plot(time_simu, -simulated_wrench_at_base(:, 2), 'r', 'LineWidth', 1)
hold on
% plot(time_base_wrench(:, 1), time_base_wrench(:, 6), 'b', 'LineWidth', 2)
plot(time_base_wrench(:, 1), time_base_wrench_raw(:, 6), 'g', 'LineWidth', 2)
legend('Measured', 'Simulated')

savefig(saving_fig_folder + fig.Name)
saveas(fig, saving_fig_folder + fig.Name, 'png')


%   Plot the cable displacements
cable_spool_radius = 0.02;
time_cables = time_angles_motor(:, 1);
motor_angles = time_angles_motor(:, 2:end);
measured_cables_pulled = motor_angles*cable_spool_radius;
fig = figure("Name", "Cable Displacement");
subplot(2, 1, 1)
plot(time_cables, measured_cables_pulled(:, 1), 'g', 'LineWidth', 1)
hold on 
plot(time_simu, cables_displacements(:, 1), 'r', 'LineWidth', 1)
grid on


subplot(2, 1, 2)
plot(time_cables, measured_cables_pulled(:, 2), 'g', 'LineWidth', 1)
hold on
plot(time_simu, -cables_displacements(:, 2), 'r', 'LineWidth', 1)
grid on

legend('Measured', 'Simulated')

savefig(saving_fig_folder + fig.Name)
saveas(fig, saving_fig_folder + fig.Name, 'png')


%%  Compute RMSE

% RMSE_tip = sqrt( (1/N_times_vicon)*sum( (time_kinematics_tip(:, 5:7) - tip_fbgs(:, 1:3)).^2 ) )

