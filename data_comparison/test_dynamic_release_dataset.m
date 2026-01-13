clc
clear all
close all

% ajoute le dossier outils
addpath('Dyn_Essai_release_Beam_Andrea/')
addpath('Dyn_Essai_release_Beam_Andrea/outils/')
addpath('Dyn_Essai_release_Beam_Andrea/IDM/')
addpath('Dyn_Essai_release_Beam_Andrea/methode spectral/')

Config.option=odeset('RelTol',10^(-8),'AbsTol',10^(-8));

% Variable pour visualisation du transitoire
% 0 pas de visualisation
% 1 visualisation
Config.Trace_transitoire = 0;
    
% Initialisation des variables pour le tracé
r_dot_trace_save = [];
T_trace_save = [];
x_trace_save = [];
y_trace_save = [];
z_trace_save = [];
           
% Initialisation boucle de temps
Config.dt    = 0.005;
Config.t_fin = 10.0;
Config.time  = 0:Config.dt:Config.t_fin;
Config.Nt    = length(Config.time);

% longueur de la poutre
Config.L = 0.48;
% Pas de discrétisation du corps
Config.dX = Config.L/101;
% Construction du corps et de ses paramètres
Config.X  = 0:Config.dX:Config.L;
% Nombre de noeuds discrétisant le corps
Config.Nx = length(Config.X);

% Définitio des variables actionnées
% K1, K2, K3, ...
% si valeur est à 1 -> la variable est actionnée
% si valeur est à 0 -> la variable n'est pas actionnée
Config.V_a = [0,1,1,0,0,0];
% Définition de la taille de la base pour chaque variable
Const.dim_base_k = 3*Config.V_a;
% Calcul de la taille de q 
Const.dim_base   = Config.V_a*Const.dim_base_k';

% Valeur de la gravité
Const.Gamma_g = 9.81;


% Parametre constants de la simulation
% Valeurs physique de la poutre 
parameters_dataset_robot;

% Initialisation du temps à 0
time = 0;

% Position et orientation initiales de la tete
r_0 = [0;0;0];
% q_0 = [1 0 0 0]';
Q_0 = [0.7071068 0 0.7071068 0]';
eta_0 = [0 0 0 0 0 0]'; % vitesse de la tête

% ------------ Calcul des gain elastique et ammortissement --------------- %

% ------ Initialisation ----- %
Kee_0 = zeros(Const.dim_base*Const.dim_base,1);
Dee_0 = zeros(Const.dim_base*Const.dim_base,1);

% ------- Calcul ------ %
CI = [Kee_0; Dee_0];
[X Y1] = ode45(@(t,y) elastic_gain(t,y,time,Const,Config),Config.X,CI,Config.option); % Solve ODE

% ------- Affectation des résultats -----------%
Kee_vec = Y1(end,1:Const.dim_base*Const.dim_base);
Dee_vec = Y1(end,1+Const.dim_base*Const.dim_base:2*Const.dim_base*Const.dim_base);

Kee = reshape(Kee_vec, [Const.dim_base Const.dim_base]);
Dee = reshape(Dee_vec, [Const.dim_base Const.dim_base]);

Const.Kee = Kee;
Const.Dee = Dee;
% Const.Dee = 0*1e-3*Kee;

%%  Process data

data_vicon

%   Cosserat rod generalized coordinates
Const.q = zeros(Const.dim_base, 1);



%   Add values from data
Const.position_data = position_data_static;


%   Recorded disks are 0.12 m from each other, and there are 5 disks in
%   total so 4 interval so the robot lenght is .48 m

%   Lenght of robot
Const.L = .48;
% Config.X_meas = [0, 0.12, 0.24, 0.36, 0.48];
% Config.X_meas = [0, 0.1189, 0.2388, 0.3577, 0.4772];
Config.X_meas = [0, 0.1189, 0.2388, 0.3577, 0.48];
Config.observation_points = Config.X_meas/Const.L; %   Normalized domain

Const.T = 0;

%%  Static init
% Initialisation de la base q

Const.q = zeros(Const.dim_base,1);
Const.q_dot     = zeros(Const.dim_base,1);
Const.q_dot_dot = zeros(Const.dim_base,1);

Const.eta_dot   = zeros(6,1);
Const.eta_dot_0   = zeros(6,1);

Const.r_0 = r_0;
Const.q_0 = Q_0; 
Const.eta_0 = eta_0;
Const.F1 = zeros(6,1);



%   Stima di q risolvendo il residuo vettoriale su tutte le coordinate
opts = optimoptions(@fsolve, ...
    'Display','iter', ...
    'FunctionTolerance',1e-4, ...
    'StepTolerance',1e-5, ...
    'MaxIterations',50, ...
    'MaxFunctionEvaluations',100);

[q_opt, fval, exitflag, output] = fsolve(@(q) findConfig(q, Const, Config), Const.q, opts);
% if exitflag <= 0
%     warning('fsolve did not converge. Exit message: %s', output.message);
% end
Const.q = q_opt;




%% Integration temporelle
close all


Config.plot = false;

new_simu = true

if new_simu
    tic
    [T,q,q_dot,q_dot_dot, position_disks_simu] = Time_integration_Newton_beam_released_spectral(Config.time,Const,Config);
    tsimul = toc
    save('dynamic_release_dataset.mat');
else
    load("dynamic_release_dataset.mat");
end

%%  Compare data with vicon

rod_positions = zeros(length(Config.X_meas), 3, Config.Nt);
for it_t=1:Config.Nt

    q_it = q(:, it_t);
    dq_it = q_dot(:, it_t);
    ddq_it = q_dot_dot(:, it_t);

    Const.q = q_it;
    Const.dq = dq_it;
    Const.ddq = ddq_it;

    CI = [Const.q_0;Const.r_0;Const.eta_0];

    [X X3] = ode45(@(t,y) Forward_eta(t,y,time,Const,Config), Config.X_meas,CI,Config.option); % Solve ODE


    rod_positions(:, :, it_t) = X3(:,5:7);

end

%%  Plots

close all


%plot vicon trajectory
disk_num = 5;
xyz_XYZ = rel_kinematics_disks{disk_num};
begin_release_plot = begin_release + 25;
tip_position_vicon = xyz_XYZ(begin_release_plot:end, 1:3)';
time_vicon_rel = time_vicon(begin_release_plot:end) - time_vicon(begin_release_plot);




%   Plot the tip position
tip_position = squeeze( rod_positions(end, :, :) );

last = find(time_vicon_rel > 10, 1, 'first');

tip_position_vicon = tip_position_vicon(:, 1:last);
time_vicon_plot = time_vicon_rel(1:last);

figure("Name","Tip Position")
subplot(3, 1, 1)
plot(time_vicon_plot, tip_position_vicon(1, :), '-b')
hold on;
plot(Config.time, tip_position(1, :), '-r')
grid on
xlabel("Time [s]")
ylabel("p_x [m]")

subplot(3, 1, 2)
plot(time_vicon_plot, tip_position_vicon(2, :), '-b')
hold on;
plot(Config.time, tip_position(2, :), '-r')
grid on
xlabel("Time [s]")
ylabel("p_y [m]")

subplot(3, 1, 3)
plot(time_vicon_plot, tip_position_vicon(3, :), '-b')
hold on;
plot(Config.time, tip_position(3, :), '-r')
grid on
xlabel("Time [s]")
ylabel("p_z [m]")


function error_norm = findConfig(q, Const, Config)

    positions_simu = rod_shape(0, q, Const, Config)';

    figure(1)
    plot3(positions_simu(1, :), positions_simu(2, :), positions_simu(3, :), '-r')
    hold on
    plot3(Const.position_data(1, :), Const.position_data(2, :), Const.position_data(3, :), '-b')
    grid on;
    xlim([-.5, .5]);
    ylim([-.5, .5]);
    zlim([-.5, .5]);
    drawnow;
    error = positions_simu - Const.position_data;
    error_norm = norm(error(:));
    
end
