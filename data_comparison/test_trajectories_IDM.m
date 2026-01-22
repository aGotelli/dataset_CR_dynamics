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




Config.data.dt = 0.01;
Config.data.time = cable_tensions(:, 1);


%   Compute difference tension
tau_1 = cable_tensions(:, 2) - cable_tensions(:, 4);
tau_2 = cable_tensions(:, 3) - cable_tensions(:, 5);
Config.data.tau = [
    tau_1'
    tau_2'
];



%% Time loop

Beta = 1/4; % interpolation 1/4; % valeur moyenne 1/6
Gamma = 1/2;

h = Config.data.dt;
a = Gamma/(Beta*h);
b = 1/(Beta*h^2);

N_times = length( Config.data.time );

for it_t=1:N_times

    time = Config.data.time(it_t);
        
    Const.q         = rand(Const.dim_base,1);
    Const.q_dot     = rand(Const.dim_base,1);
    Const.q_dot_dot = rand(Const.dim_base,1);
        
    [~, L1, L2] = internalActuation(zeros(2, 1), Const, Config);
    [F0, Q_a, J] = TIDM_spectral(Const.q_0,Const.r_0,Const.q_0,Const.r_0,Const.eta_0,Const.eta_dot_0,a,b,time,Const,Config);
    Q_ad = Q_a + Const.Dee*Const.q_dot + Const.Kee*Const.q;

    L = [
        L1 L2
    ];

    Delta_tau = L\Q_ad;
    
end
    
% 
% %%  Plotting results
% load(path + "simulation_results")
% 
% vicon_frames_stacked = load(path + "vicon_frames.csv");
% N_times_vicon = size(vicon_frames_stacked, 1);
% vicon_frames = reshape(vicon_frames_stacked, [N_times_vicon, 7, 5]);
% disk_num = 5;
% time_kinematics_tip = vicon_frames(:, :, disk_num);
% time_simu = time_kinematics_tip(:, 1);
% 
% 
% time_base_wrench = load(path + "base_wrench.csv");
% 
% 
% 
% tip_frame = squeeze( position_disks_simu(5, :, :) )';
% 
% 
% figure("Name", "Tip Position")
% subplot(3, 1, 1)
% plot(time_kinematics_tip(:, 1), time_kinematics_tip(:, 2), 'b', 'LineWidth', 1)
% hold on
% plot(Config.data.time, tip_frame(:, 1), 'r', 'LineWidth', 1)
% grid on
% 
% subplot(3, 1, 2)
% plot(time_kinematics_tip(:, 1), time_kinematics_tip(:, 3), 'b', 'LineWidth', 1)
% hold on
% plot(Config.data.time, tip_frame(:, 2), 'r', 'LineWidth', 1)
% grid on
% 
% subplot(3, 1, 3)
% plot(time_kinematics_tip(:, 1), time_kinematics_tip(:, 4), 'b', 'LineWidth', 1)
% hold on
% plot(Config.data.time, tip_frame(:, 3), 'r', 'LineWidth', 1)
% grid on
% legend('Measured', 'Simulated')
% 
% figure("Name", "Wrench at base (Torques)")
% subplot(3, 1, 1)
% plot(time_base_wrench(:, 1), time_base_wrench(:, 2), 'b', 'LineWidth', 1)
% hold on
% plot(time_simu, wrench_at_base(:, 1), 'r', 'LineWidth', 1)
% 
% subplot(3, 1, 2)
% plot(time_base_wrench(:, 1), time_base_wrench(:, 3), 'b', 'LineWidth', 1)
% hold on
% plot(time_simu, wrench_at_base(:, 2), 'r', 'LineWidth', 1)
% 
% subplot(3, 1, 3)
% plot(time_base_wrench(:, 1), time_base_wrench(:, 4), 'b', 'LineWidth', 1)
% hold on
% plot(time_simu, wrench_at_base(:, 3), 'r', 'LineWidth', 1)
% 
% 
% 
% figure("Name", "Wrench at base (Forces)")
% subplot(3, 1, 1)
% plot(time_base_wrench(:, 1), time_base_wrench(:, 5), 'b', 'LineWidth', 1)
% hold on
% plot(time_simu, wrench_at_base(:, 4), 'r', 'LineWidth', 1)
% 
% subplot(3, 1, 2)
% plot(time_base_wrench(:, 1), time_base_wrench(:, 6), 'b', 'LineWidth', 1)
% hold on
% plot(time_simu, wrench_at_base(:, 5), 'r', 'LineWidth', 1)
% 
% subplot(3, 1, 3)
% plot(time_base_wrench(:, 1), time_base_wrench(:, 7), 'b', 'LineWidth', 1)
% hold on
% plot(time_simu, wrench_at_base(:, 6), 'r', 'LineWidth', 1)
% 
% 
% figure("Name", "Torque")
% plot(time_base_wrench(:, 1), time_base_wrench(:, 6), 'b', 'LineWidth', 1)
% hold on
% plot(time_simu, -wrench_at_base(:, 2), 'r', 'LineWidth', 1)