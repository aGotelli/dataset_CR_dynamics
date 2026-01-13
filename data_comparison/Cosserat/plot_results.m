%%  INITIALIZATION
close all
clear
clc

load("data/Config.mat")
load("data/Const.mat")

load("data/ground_truth.mat")
% load("data/state_estimation_results_no_gains.mat")
load("data/state_estimation_results.mat")

% state_estimation_results_world = load("data/state_estimation_results_world.mat");
% state_estimation_results_world = state_estimation_results_world.state_estimation_results;

time_ground_truth    = ground_truth.t_stack;
N_steps_ground_truth = length(time_ground_truth);

% time_no_gains    = state_estimation_results_no_gains.t_stack;
% N_steps_no_gains = length(time_no_gains);

time    = state_estimation_results.t_stack;
N_steps = length(time);


N_points = length(Config.backward_integration_domain);


%%  Plot the values we use in the state estimation injection

%   Rod tip
eta_X1_star   = state_estimation_results.eta_X1_star;
eta_X1        = state_estimation_results.eta_X1;
Lambda_X1_bar = state_estimation_results.Lambda_X1_bar;

eta_X1_star_ground_truth   = squeeze(ground_truth.eta_stack( end, :, :))';


% eta_world_X1_star   = state_estimation_results_world.eta_X1_star;
% eta_world_X1        = state_estimation_results_world.eta_X1;
% Lambda_world_X1_bar = state_estimation_results_world.Lambda_X1_bar;



% %   Rod base
% Lambda_X0_star = state_estimation_results.Lambda_X0_star;
% Lambda_X0      = state_estimation_results.Lambda_X0;
% eta_X0_bar     = state_estimation_results.eta_X0_bar;


%   Find the first non zero value
it_t_init = find(time, 1);

figure("Name","Rod Tip")
subplot(3, 1, 1)
plot(time(it_t_init:end),   eta_X1_star(it_t_init:end, 2),   'b', 'DisplayName', 'eta X1 star', 'LineWidth', 2)
hold on
plot(time_ground_truth,   eta_X1_star_ground_truth(:, 2),   '--k', 'DisplayName', 'eta X1 star', 'LineWidth', 2)
plot(time(it_t_init:end),        eta_X1(it_t_init:end, 2), 'g', 'DisplayName', 'eta X1', 'LineWidth', 2)
yyaxis right
plot(time(it_t_init:end), Lambda_X1_bar(it_t_init:end, 2),   'r', 'DisplayName', 'Lambda X1 bar', 'LineWidth', 2)
grid on
xlabel("Time [s]")
ylabel("Torque [Nm]")
title("C_y")
legend()

subplot(3, 1, 2)
plot(time(it_t_init:end),   eta_X1_star(it_t_init:end, 4),   'b', 'DisplayName', 'eta X1 star', 'LineWidth', 2)
hold on
plot(time_ground_truth,   eta_X1_star_ground_truth(:, 4),   '--k', 'DisplayName', 'eta X1 star', 'LineWidth', 2)
plot(time(it_t_init:end),        eta_X1(it_t_init:end, 4), 'g', 'DisplayName', 'eta X1', 'LineWidth', 2)
yyaxis right
plot(time(it_t_init:end), Lambda_X1_bar(it_t_init:end, 4),   'r', 'DisplayName', 'Lambda X1 bar', 'LineWidth', 2)
grid on
xlabel("Time [s]")
ylabel("Force [N]")
title("F_x")
legend()


subplot(3, 1, 3)
plot(time(it_t_init:end),   eta_X1_star(it_t_init:end, 6),   'b', 'DisplayName', 'eta X1 star', 'LineWidth', 2)
hold on
plot(time_ground_truth,   eta_X1_star_ground_truth(:, 6),   '--k', 'DisplayName', 'eta X1 star', 'LineWidth', 2)
plot(time(it_t_init:end),        eta_X1(it_t_init:end, 6), 'g', 'DisplayName', 'eta X1', 'LineWidth', 2)
yyaxis right
plot(time(it_t_init:end), Lambda_X1_bar(it_t_init:end, 6),   'r', 'DisplayName', 'Lambda X1 bar', 'LineWidth', 2)
grid on
xlabel("Time [s]")
ylabel("Force [N]")
title("F_z")
legend()




% 
% 
% 
% 
% figure("Name","Rod Base")
% subplot(3, 1, 1)
% plot(time(it_t_init:end), Lambda_X0_star(it_t_init:end, 2),   'b', 'DisplayName', 'Lambda X0 star', 'LineWidth', 2)
% hold on
% plot(time(it_t_init:end),      Lambda_X0(it_t_init:end, 2), 'g', 'DisplayName', 'Lambda X0', 'LineWidth', 2)
% plot(time(it_t_init:end),     eta_X0_bar(it_t_init:end, 2),   'r', 'DisplayName', 'eta X0 bar', 'LineWidth', 2)
% grid on
% xlabel("Time [s]")
% ylabel("Torque [Nm]")
% title("C_y")
% legend()
% 
% subplot(3, 1, 2)
% plot(time(it_t_init:end), Lambda_X0_star(it_t_init:end, 4),   'b', 'DisplayName', 'Lambda X0 star', 'LineWidth', 2)
% hold on
% plot(time(it_t_init:end),      Lambda_X0(it_t_init:end, 4), 'g', 'DisplayName', 'Lambda X0', 'LineWidth', 2)
% plot(time(it_t_init:end),     eta_X0_bar(it_t_init:end, 4),   'r', 'DisplayName', 'eta X0 bar', 'LineWidth', 2)
% grid on
% xlabel("Time [s]")
% ylabel("Force [N]")
% title("F_x")
% legend()
% 
% 
% subplot(3, 1, 3)
% plot(time(it_t_init:end), Lambda_X0_star(it_t_init:end, 6),   'b', 'DisplayName', 'Lambda X0 star', 'LineWidth', 2)
% hold on
% plot(time(it_t_init:end),      Lambda_X0(it_t_init:end, 6), 'g', 'DisplayName', 'Lambda X0', 'LineWidth', 2)
% plot(time(it_t_init:end),     eta_X0_bar(it_t_init:end, 6),   'r', 'DisplayName', 'eta X0 bar', 'LineWidth', 2)
% grid on
% xlabel("Time [s]")
% ylabel("Force [N]")
% title("F_z")
% legend()





%%  Comparison local and global
figure("Name","Rod Tip local/global")
subplot(3, 1, 1)
% plot(time(it_t_init:end), Lambda_X1_bar(it_t_init:end, 2),         'b', 'DisplayName', 'Lambda X1 bar', 'LineWidth', 2)
% hold on
% plot(time(it_t_init:end), Lambda_world_X1_bar(it_t_init:end, 2),   'r', 'DisplayName', 'Lambda X1 bar world', 'LineWidth', 2)
grid on
xlabel("Time [s]")
ylabel("Torque [Nm]")
title("C_y")
legend()

subplot(3, 1, 2)
plot(time(it_t_init:end), Lambda_X1_bar(it_t_init:end, 4),         'b', 'DisplayName', 'Lambda X1 bar', 'LineWidth', 2)
% hold on
% plot(time(it_t_init:end), Lambda_world_X1_bar(it_t_init:end, 4),   'r', 'DisplayName', 'Lambda X1 bar world', 'LineWidth', 2)
grid on
xlabel("Time [s]")
ylabel("Force [N]")
title("F_x")
legend()


subplot(3, 1, 3)
plot(time(it_t_init:end), Lambda_X1_bar(it_t_init:end, 6),         'b', 'DisplayName', 'Lambda X1 bar', 'LineWidth', 2)
% hold on
% plot(time(it_t_init:end), Lambda_world_X1_bar(it_t_init:end, 6),   'r', 'DisplayName', 'Lambda X1 bar world', 'LineWidth', 2)
grid on
xlabel("Time [s]")
ylabel("Force [N]")
title("F_z")
legend()