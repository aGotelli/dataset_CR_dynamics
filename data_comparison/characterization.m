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

addpath("Data\")
%%  Process data 

%   folder to load
data = "statics_xz_plane_4";

vicon = readtable("Data\" + data + "\dataVicon_.csv");
tendon1 = readtable("Data\" + data + "\dataMark10_1_-x_.csv");
tendon2 = readtable("Data\" + data + "\dataMark10_2_-y_.csv");



disk_names = {'disk_0', 'disk_1', 'disk_2', 'disk_3', 'disk_4'};
N_disks = length(disk_names);

disk_pos_mean = zeros(3, 4);
disk_pos_stdv = zeros(3, 4);

disk_Rot_mean = zeros(3, 4);
disk_Rot_stdv = zeros(3, 4);

begin = 200;    %   first 200 samples not good for tendon elasticity

%   Extract X, Y, Z position and euler angles for each disk
for it = 1:N_disks
    
    index = N_disks + 1 - it; % invert order (frame 1 is at the end of the robot)

    disk = disk_names{it};
    
    % Extract coordinates
    x_data = vicon.([disk 'X_m_']);
    y_data = vicon.([disk 'Y_m_']);
    z_data = vicon.([disk 'Z_m_']);

    x_mean = mean(x_data(begin:end));
    y_mean = mean(y_data(begin:end));
    z_mean = mean(z_data(begin:end));
    disk_pos_mean(:, index) = [x_mean y_mean z_mean]';

    x_stdv = std(x_data(begin:end));
    y_stdv = std(y_data(begin:end));
    z_stdv = std(z_data(begin:end));
    disk_pos_stdv(:, index) = [x_stdv y_stdv z_stdv]';


    Rot_X = vicon.([disk 'RotX_rad_']);
    Rot_Y = vicon.([disk 'RotY_rad_']);
    Rot_Z = vicon.([disk 'RotZ_rad_']);


    Rot_X_mean = mean(Rot_X(begin:end));
    Rot_Y_mean = mean(Rot_Y(begin:end));
    Rot_Z_mean = mean(Rot_Z(begin:end));
    disk_Rot_mean(:, index) = [Rot_X_mean Rot_Y_mean Rot_Z_mean]';

    Rot_X_stdv = std(Rot_X(begin:end));
    Rot_Y_stdv = std(Rot_Y(begin:end));
    Rot_Z_stdv = std(Rot_Z(begin:end));
    disk_Rot_stdv(:, index) = [Rot_X_stdv Rot_Y_stdv Rot_Z_stdv]';

    
    % % Create figure for each disk
    % figure('Name', disk);
    % 
    % % X subplot
    % subplot(3,1,1);
    % plot(Rot_X);
    % ylabel('X (m)');
    % title([disk ' - roll']);
    % grid on;
    % 
    % % Y subplot
    % subplot(3,1,2);
    % plot(Rot_Y);
    % ylabel('Y (m)');
    % title([disk ' - pitch']);
    % grid on;
    % 
    % % Z subplot
    % subplot(3,1,3);
    % plot(Rot_Z);
    % ylabel('Z (m)');
    % xlabel('Sample');
    % title([disk ' - yaw']);
    % grid on;
end


tau_1_mean = mean(tendon1.tension_N_(begin:end));
tau_1_stdv = std(tendon1.tension_N_(begin:end));

tau_2_mean = mean(tendon2.tension_N_(begin:end));
tau_2_stdv = std(tendon2.tension_N_(begin:end));

% % Plot tendon data
% figure('Name', 'Tendon Forces');
% 
% % Tendon 1 subplot
% subplot(2,1,1);
% plot(tendon1.tension_N_);
% ylabel('Force (N)');
% title('Tendon 1 (-x direction)');
% grid on;
% 
% % Tendon 2 subplot
% subplot(2,1,2);
% plot(tendon2.tension_N_);
% ylabel('Force (N)');
% xlabel('Sample');
% title('Tendon 2 (-y direction)');
% grid on;



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


clc

%   Configurations for the simulation
[Const, Config] = simulationConfigurations();

%   Position of the rod base
Const.r_X0 = [0;0;0];

%   Quaternion of rod base [w x y z]
Const.Q_X0 = [0.7071068 0 0.7071068 0]';
% Const.Q_X0 = [1, 0, 0, 0]';


%   Cosserat rod generalized coordinates
q      = zeros(Const.dim_base, 1);
Const.q = q;

%   Remove dynamics from Jacobian (reuse existing and tested code)
Config.a = 0;
Config.b = 0;

%   Positioning cable
d = 37.5e-3;
Const.D = [0 0 d]';
Const.D_prime = zeros(3, 1);



%   Add values from data
Const.T = tau_1_mean;
Const.position_data = position_data;



%   Recorded disks are 0.12 m from each other, and there are 5 disks in
%   total so 4 interval so the robot lenght is .48 m

%   Lenght of robot
Const.L = .48;
X_meas = [0, 0.12, 0.24, 0.36, 0.48];
Config.observation_points = X_meas/Const.L; %   Normalized domain


%   Initial guess
EI = Const.EIyy;

EI = fsolve(@(EI) findStiffness(EI, q, Const, Config), EI)
Const.EI = EI;


positions_simu = find_rod_state_NR(q, Const, Config);





% Plot 3D frames
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










