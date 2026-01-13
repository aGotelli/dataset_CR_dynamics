%   This file will be used to estimate the values of the robot used for the
%   dataset project. The estimation will be done as an optmization problem
close all;
clear;
clc


addpath("Cosserat/")
addpath("Cosserat/ODEs/")
addpath("Cosserat/ODEs/tendon/")
addpath("Cosserat/utilities/")
addpath("Cosserat/rod_properties/")
addpath("Cosserat/implicit_integration/")
addpath("Data")
%%  Process data 

%   folder to load
data = "oscillation40s_from_tendon_release-x_3";

vicon = readtable(fullfile("Data", data, "dataVicon_.csv"));

disk_names = {'disk_0', 'disk_1', 'disk_2', 'disk_3', 'disk_4'};
N_disks = length(disk_names);
% % % figure
% % % xlabel('X (m)'); ylabel('Y (m)'); zlabel('Z (m)');
% % % hold on; grid on
% % % plot3(vicon.disk_0X_m_,vicon.disk_0Y_m_, vicon.disk_0Z_m_)
% % % plot3(vicon.disk_1X_m_,vicon.disk_1Y_m_, vicon.disk_1Z_m_)
% % % plot3(vicon.disk_2X_m_,vicon.disk_2Y_m_, vicon.disk_2Z_m_)
% % % plot3(vicon.disk_3X_m_,vicon.disk_3Y_m_, vicon.disk_3Z_m_)
% % % plot3(vicon.disk_4X_m_,vicon.disk_4Y_m_, vicon.disk_4Z_m_)

disk_pos_mean = zeros(3, 4);
disk_pos_stdv = zeros(3, 4);

disk_Rot_mean = zeros(3, 4);
disk_Rot_stdv = zeros(3, 4);

%   Individuato il rilascio intorno al campione ~330: prendo media prima
begin = 300;    %   campioni statici

%   Extract X, Y, Z position and euler angles for each disk - before motion start
for it = 1:N_disks
    
    index = N_disks + 1 - it; % invert order (frame 1 is at the end of the robot)

    disk = disk_names{it};
    
    % Extract coordinates
    x_data = vicon.([disk 'X_m_']);
    y_data = vicon.([disk 'Y_m_']);
    z_data = vicon.([disk 'Z_m_']);

    x_mean = mean(x_data(1:begin));
    y_mean = mean(y_data(1:begin));
    z_mean = mean(z_data(1:begin));
    disk_pos_mean(:, index) = [x_mean y_mean z_mean]';

    x_stdv = std(x_data(1:begin));
    y_stdv = std(y_data(1:begin));
    z_stdv = std(z_data(1:begin));
    disk_pos_stdv(:, index) = [x_stdv y_stdv z_stdv]';


    Rot_X = vicon.([disk 'RotX_rad_']);
    Rot_Y = vicon.([disk 'RotY_rad_']);
    Rot_Z = vicon.([disk 'RotZ_rad_']);


    Rot_X_mean = mean(Rot_X(1:begin));
    Rot_Y_mean = mean(Rot_Y(1:begin));
    Rot_Z_mean = mean(Rot_Z(1:begin));
    disk_Rot_mean(:, index) = [Rot_X_mean Rot_Y_mean Rot_Z_mean]';

    Rot_X_stdv = std(Rot_X(1:begin));
    Rot_Y_stdv = std(Rot_Y(1:begin));
    Rot_Z_stdv = std(Rot_Z(1:begin));
    disk_Rot_stdv(:, index) = [Rot_X_stdv Rot_Y_stdv Rot_Z_stdv]';
end

%%  Express the poses wrt the one at the robot base

R = eul2rotm(disk_Rot_mean', "XYZ");
r = reshape(disk_pos_mean, [3, 1, N_disks]);
g_abs = [
    R   r
    zeros(1, 3, N_disks) ones(1, 1, N_disks)
];

g_inv = inv_g(g_abs(:, :, 1));

g = pagemtimes(g_inv, g_abs);
position_data = squeeze(g(1:3, 4, :));


%%  Find correct values

[Const, Config] = simulationConfigurations();

%   Lenght of the rod
Const.L = 0.48;


%  MATERIAL PARAMETERS
%   Specific weight
Const.rho = 7800;

%   Stifness
Const.EI = 0.092;






%  STIFFNESS AND INERTIA OF CROSS SECTION
Const.rhoAg = Const.rho*Const.Area*Const.g;


Const.H_cal = diag([Const.GIxx, Const.EI, Const.EI, Const.EA, Const.GA, Const.GA]);

[Kee, Dee] = computeGeneralisedStiffnessDampingMatrices(Const, Config);

Const.Kee = Kee;
Const.Dee = Dee;


%   Cosserat rod generalized coordinates
Const.q = zeros(Const.dim_base, 1);



%   Add values from data
Const.position_data = position_data;


%   Recorded disks are 0.12 m from each other, and there are 5 disks in
%   total so 4 interval so the robot lenght is .48 m

%   Lenght of robot
Const.L = .48;
X_meas = [0, 0.12, 0.24, 0.36, 0.48];
Config.observation_points = X_meas/Const.L; %   Normalized domain

Const.T = 0;
% Const.tau_sel=1;
Const.Q_X0 = [0.7071068 0 0.7071068 0]';


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

%%  Simulation release



simulation_results = dynamicSimulationCosserat(q_opt, Const, Config);




%% Plot 3D frames
figure("Name",'Frame Positions and Orientations' )
hold on;
axis equal;
grid on;
xlabel('X (m)');
ylabel('Y (m)');
zlabel('Z (m)');

% Define axis length for visualization
axis_length = 0.05;

g_rel = zeros(4, 4, N_disks);

for it=1:N_disks-1
    g_pre = g(:, :, it);
    g_next = g(:, :, it + 1);
    g_rel(:, :, it) = inv_g(g_pre)*g_next;
end

% Plot each frame
for i = 1:N_disks

    % Extract position
    pos = g(1:3, 4, i);
    
    % Extract rotation matrix
    R_frame = g(1:3, 1:3, i);
    
    % Plot frame axes (x=red, y=green, z=blue)
    quiver3(pos(1), pos(2), pos(3), R_frame(1,1)*axis_length, R_frame(2,1)*axis_length, R_frame(3,1)*axis_length, 'r', 'LineWidth', 2);
    quiver3(pos(1), pos(2), pos(3), R_frame(1,2)*axis_length, R_frame(2,2)*axis_length, R_frame(3,2)*axis_length, 'g', 'LineWidth', 2);
    quiver3(pos(1), pos(2), pos(3), R_frame(1,3)*axis_length, R_frame(2,3)*axis_length, R_frame(3,3)*axis_length, 'b', 'LineWidth', 2);

    plot3(positions_simu(1, :), positions_simu(2, :), positions_simu(3, :), '*r', 'MarkerSize', 12)
    
    % Add frame number label
    text(pos(1), pos(2), pos(3), sprintf(' %d', i), 'FontSize', 12, 'FontWeight', 'bold');
end

view(3);


t_end = 3;
dt    = 10e-3;
Beta  = 1/4;
Gamma = 1/2;
a     = Gamma/(Beta*dt);
b     = 1/(Beta*dt^2);
Config.a     = a;
Config.b     = b;
% Const.T=0;

dyn_sim = dynamicSimulationCosserat(q_opt,Const,Config);

% 3) Animazione semplice
t = dyn_sim.t_stack(:);
r_tip = squeeze(dyn_sim.r_stack(end,:,:)).';
figure;
h_line = plot3(nan,nan,nan,'b-','LineWidth',1.5); hold on;
h_tip  = plot3(nan,nan,nan,'ro','MarkerSize',6,'LineWidth',2);
grid on; axis equal; xlabel('X'); ylabel('Y'); zlabel('Z'); title('Simulazione nel tempo');
for k = 1:size(r_tip,1)
    r_shape = squeeze(dyn_sim.r_stack(:,:,k));
    set(h_line,'XData',r_shape(:,1),'YData',r_shape(:,2),'ZData',r_shape(:,3));
    set(h_tip,'XData',r_tip(k,1),'YData',r_tip(k,2),'ZData',r_tip(k,3));
    drawnow;
end












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

