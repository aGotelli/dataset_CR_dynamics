% 
close all
clear all
clc


addpath("ODEs/")
addpath("utilities/")
addpath("rod_properties/")
addpath("implicit_integration/")
addpath("explicit_integration/")



%%  Load processed data


path = fullfile("../../../../","data","dynamic_motion/",'star_fast/');
load_path = fullfile(path,'processed/');
savepath = fullfile(path,"gvs/");
saving_fig_folder = fullfile(savepath,"figures/");

mkdir(savepath)
mkdir(saving_fig_folder)


tendon_tensions = load(fullfile(load_path,"tendon_tensions.csv"));
time_base_wrench = load(fullfile(load_path,"base_wrench.csv"));

%   Load this from file
dt = 0.01;
time = tendon_tensions(:,1);

%   Differential cable tension (the actuation is antagonistic)
tau_1 = tendon_tensions(:,2) - tendon_tensions(:,4);
tau_2 = tendon_tensions(:,3) - tendon_tensions(:,5);
tau = [
    tau_1'
    tau_2'
];




%%  Simulation setup


DoFs = [0, 3, 3, 0, 0, 0];

%   Configurations for the simulation
[Const, Config] = simulationConfigurations(DoFs, time(end), dt);

%   Position of the rod base
Const.r_X0 = [0;0;0];

%   Quaternion of rod base [w x y z]
Const.Q_X0 = [0.7071068 0 0.7071068 0]';


%   Cosserat rod generalized coordinates
q      = zeros(Const.dim_base, 1);
dot_q  = zeros(Const.dim_base, 1);
ddot_q = zeros(Const.dim_base, 1);


Config.plot_simu = true;

[t_stack_implicit, states_stack] = ...
    CosseratRodImplicitSimulation(q, dot_q, ddot_q, tau, Const, Config);





%%  Plotting results

wrench_base_simu = states_stack.Lambda_X0';


fig = figure("Name", "Torque");
subplot(2, 1, 1)
plot(time_base_wrench(:, 1), time_base_wrench(:, 5), 'b', 'LineWidth', 2)
hold on
plot(t_stack_implicit, wrench_base_simu(:, 3), 'r', 'LineWidth', 1)
% plot(time_base_wrench(:, 1), time_base_wrench(:, 5), 'b', 'LineWidth', 2)
set(gca,"FontSize",20)
grid on
ylabel("T_x [Nm]", "FontSize", 20)


subplot(2, 1, 2)
plot(time_base_wrench(:, 1), time_base_wrench(:, 6), 'b', 'LineWidth', 2)
hold on
plot(t_stack_implicit, wrench_base_simu(:, 2), 'r', 'LineWidth', 1)
% plot(time_base_wrench(:, 1), time_base_wrench(:, 6), 'b', 'LineWidth', 2)
set(gca,"FontSize",20)
grid on
% legend('Measured', 'Simulated')
ylabel("T_y [Nm]", "FontSize", 20)
xlabel("Time [s]", "FontSize", 20)

savefig(saving_fig_folder + fig.Name)
saveas(fig, saving_fig_folder + fig.Name, 'png')




%%  Compute RMSE

torque_ati = time_base_wrench(1:end-1, 5:6);
torque_simu = [wrench_base_simu(:, 3) wrench_base_simu(:, 2)];
RMSE_Torques = rmse(torque_simu, torque_ati)

%   Compute range of motion
range_torques = max(torque_ati) - min(torque_ati);

RMSE_torques_perc_range = (RMSE_Torques./range_torques)*100

% Save RMSEs
fid = fopen(fullfile(load_path , "RMSEs_torques.txt"), 'w');
fprintf(fid, 'RMSE_tip = [%s]\n', strjoin(string(RMSE_Torques), ', '));
fprintf(fid, 'RMSE_torques_perc_range = [%s]\n', strjoin(string(RMSE_torques_perc_range), ', '));

% 3. Close the file
fclose(fid);

