%%  Process data 
addpath("Data")


%   folder to load
data = "oscillation40s_from_tendon_release-x_3";
% data = "statics_0";

vicon = readtable(fullfile("Data", data, "dataVicon_.csv"));

disk_names = {'disk_0', 'disk_1', 'disk_2', 'disk_3', 'disk_4'};
N_disks = length(disk_names);




disk_pos_mean = zeros(3, 4);
disk_pos_stdv = zeros(3, 4);

disk_Rot_mean = zeros(3, 4);
disk_Rot_stdv = zeros(3, 4);

%   Individuato il rilascio intorno al campione ~330: prendo media prima
begin_release = 300;    %   campioni statici

%   Extract X, Y, Z position and euler angles for each disk - before motion start
for it = 1:N_disks
    

    disk = disk_names{it};
    
    % Extract coordinates
    x_data = vicon.([disk 'X_m_']);
    y_data = vicon.([disk 'Y_m_']);
    z_data = vicon.([disk 'Z_m_']);

    x_mean = mean(x_data(1:begin_release));
    y_mean = mean(y_data(1:begin_release));
    z_mean = mean(z_data(1:begin_release));
    disk_pos_mean(:, it) = [x_mean y_mean z_mean]';

    x_stdv = std(x_data(1:begin_release));
    y_stdv = std(y_data(1:begin_release));
    z_stdv = std(z_data(1:begin_release));
    disk_pos_stdv(:, it) = [x_stdv y_stdv z_stdv]';


    Rot_X = vicon.([disk 'RotX_rad_']);
    Rot_Y = vicon.([disk 'RotY_rad_']);
    Rot_Z = vicon.([disk 'RotZ_rad_']);


    Rot_X_mean = mean(Rot_X(1:begin_release));
    Rot_Y_mean = mean(Rot_Y(1:begin_release));
    Rot_Z_mean = mean(Rot_Z(1:begin_release));
    disk_Rot_mean(:, it) = [Rot_X_mean Rot_Y_mean Rot_Z_mean]';

    Rot_X_stdv = std(Rot_X(1:begin_release));
    Rot_Y_stdv = std(Rot_Y(1:begin_release));
    Rot_Z_stdv = std(Rot_Z(1:begin_release));
    disk_Rot_stdv(:, it) = [Rot_X_stdv Rot_Y_stdv Rot_Z_stdv]';

    % name = "Raw " + disk;
    % figure("Name", name)
    % subplot(3, 2, 1)
    % plot(Rot_X)
    % grid on
    % ylabel("Roll [RAD]")
    % 
    % subplot(3, 2, 2)
    % plot(x_data)
    % grid on
    % ylabel("X [m]")
    % 
    % subplot(3, 2, 3)
    % plot(Rot_Y)
    % grid on
    % ylabel("Pitch [RAD]")
    % 
    % subplot(3, 2, 4)
    % plot(y_data)
    % grid on
    % ylabel("Y [m]")
    % 
    % subplot(3, 2, 5)
    % plot(Rot_Z)
    % grid on
    % ylabel("Yaw [RAD]")
    % 
    % subplot(3, 2, 6)
    % plot(z_data)
    % grid on
    % ylabel("Z [m]")
end

%%  Express the poses wrt the one at the robot base

R = eul2rotm(disk_Rot_mean', "XYZ");
r = reshape(disk_pos_mean, [3, 1, N_disks]);
g_abs = [
    R   r
    zeros(1, 3, N_disks) ones(1, 1, N_disks)
];

disk_1_pose = g_abs(:, :, 1);
disk_1_pose_inv = inv_g( disk_1_pose );

%   Rotation to reference frame
Rs = axang2rotm([1 0 0 pi]);
g_s = eye(4);
g_s(1:3, 1:3) = Rs;

disk_1_pose_inv = g_s*disk_1_pose_inv;

disk_poses_static = pagemtimes(disk_1_pose_inv, g_abs);
position_data_static = squeeze(disk_poses_static(1:3, 4, :));


%%  Extract dynamic release
time_vicon = vicon.timestamp - vicon.timestamp(1);
N_time = length(time_vicon);

kinematics_disks = {};
rel_kinematics_disks = {};
poses_disks = {};
for it = 1:N_disks
    

    disk = disk_names{it};
    
    % Extract coordinates
    x_data = vicon.([disk 'X_m_']);
    y_data = vicon.([disk 'Y_m_']);
    z_data = vicon.([disk 'Z_m_']);

    
    Rot_X = vicon.([disk 'RotX_rad_']);
    Rot_Y = vicon.([disk 'RotY_rad_']);
    Rot_Z = vicon.([disk 'RotZ_rad_']);

    kinematics_disks{it} = [x_data y_data z_data Rot_X Rot_Y Rot_Z];


    %   Compose the corresponding rotation and position vector
    r = zeros(3, 1, N_time);
    r(1, 1, :) = x_data;
    r(2, 1, :) = y_data;
    r(3, 1, :) = z_data;

    XYZ_Eul = [
        Rot_X'
        Rot_Y'
        Rot_Z'
    ];
    
    R = eul2rotm(XYZ_Eul', "XYZ");

    bottom = repmat([0 0 0 1], 1, 1, N_time);
    g_disk_abs = [
         R  r
        bottom
    ];

    %   Compute relative pose
    g_disk = pagemtimes(disk_1_pose_inv, g_disk_abs);
    poses_disks{it} = g_disk;
    
    %   Deconstruct poses
    x = squeeze(g_disk(1, 4, :));
    y = squeeze(g_disk(2, 4, :));
    z = squeeze(g_disk(3, 4, :));

    EUL = rotm2eul(g_disk(1:3, 1:3, :), 'XYZ');

    rel_kinematics_disks{it} = [x y z EUL];

end

%   Dummy plots
% 
% 
% 
% disk_num = 5;
% 
% 
% 
% xyz_XYZ = kinematics_disks{disk_num};
% 
% x_data = xyz_XYZ(:, 1);
% y_data = xyz_XYZ(:, 2);
% z_data = xyz_XYZ(:, 3);
% 
% Rot_X = xyz_XYZ(:, 4);
% Rot_Y = xyz_XYZ(:, 5);
% Rot_Z = xyz_XYZ(:, 6);
% 
% figure("Name", "Kinematics")
% subplot(3, 2, 1)
% plot(time_vicon, Rot_X)
% grid on
% xlabel("Time [s]")
% ylabel("Roll [RAD]")
% 
% subplot(3, 2, 2)
% plot(time_vicon, x_data)
% grid on
% xlabel("Time [s]")
% ylabel("X [m]")
% 
% subplot(3, 2, 3)
% plot(time_vicon, Rot_Y)
% grid on
% xlabel("Time [s]")
% ylabel("Pitch [RAD]")
% 
% subplot(3, 2, 4)
% plot(time_vicon, y_data)
% grid on
% xlabel("Time [s]")
% ylabel("Y [m]")
% 
% subplot(3, 2, 5)
% plot(time_vicon, Rot_Z)
% grid on
% xlabel("Time [s]")
% ylabel("Yaw [RAD]")
% 
% subplot(3, 2, 6)
% plot(time_vicon, z_data)
% grid on
% xlabel("Time [s]")
% ylabel("Z [m]")
% 
% 
% 
% xyz_XYZ = rel_kinematics_disks{disk_num};
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
% plot(time_vicon, Rot_X)
% grid on
% xlabel("Time [s]")
% ylabel("Roll [RAD]")
% 
% subplot(3, 2, 2)
% plot(time_vicon, x_data)
% grid on
% xlabel("Time [s]")
% ylabel("X [m]")
% 
% subplot(3, 2, 3)
% plot(time_vicon, Rot_Y)
% grid on
% xlabel("Time [s]")
% ylabel("Pitch [RAD]")
% 
% subplot(3, 2, 4)
% plot(time_vicon, y_data)
% grid on
% xlabel("Time [s]")
% ylabel("Y [m]")
% 
% subplot(3, 2, 5)
% plot(time_vicon, Rot_Z)
% grid on
% xlabel("Time [s]")
% ylabel("Yaw [RAD]")
% 
% subplot(3, 2, 6)
% plot(time_vicon, z_data)
% grid on
% xlabel("Time [s]")
% ylabel("Z [m]")


%%  Video Motion
plots = false;


if plots
    disk_num = 5;

    xyz_XYZ = rel_kinematics_disks{disk_num};

    x_data = xyz_XYZ(:, 1);
    y_data = xyz_XYZ(:, 2);
    z_data = xyz_XYZ(:, 3);
    
    Rot_X = xyz_XYZ(:, 4);
    Rot_Y = xyz_XYZ(:, 5);
    Rot_Z = xyz_XYZ(:, 6);
    
    figure("Name", "Kinematics (relative)")
    subplot(3, 2, 1)
    plot(time_vicon, Rot_X)
    grid on
    xlabel("Time [s]")
    ylabel("Roll [RAD]")
    
    subplot(3, 2, 2)
    plot(time_vicon, x_data)
    grid on
    xlabel("Time [s]")
    ylabel("X [m]")
    
    subplot(3, 2, 3)
    plot(time_vicon, Rot_Y)
    grid on
    xlabel("Time [s]")
    ylabel("Pitch [RAD]")
    
    subplot(3, 2, 4)
    plot(time_vicon, y_data)
    grid on
    xlabel("Time [s]")
    ylabel("Y [m]")
    
    subplot(3, 2, 5)
    plot(time_vicon, Rot_Z)
    grid on
    xlabel("Time [s]")
    ylabel("Yaw [RAD]")
    
    subplot(3, 2, 6)
    plot(time_vicon, z_data)
    grid on
    xlabel("Time [s]")
    ylabel("Z [m]")
    
        colors = [
        1 0 0;  % Red
        0 1 0;  % Green
        0 0 1;  % Blue
        1 0 1;  % Magenta
        0 1 1   % Cyan
        ];
    
    
    X = zeros(1, disk_num);
    Y = zeros(1, disk_num);
    Z = zeros(1, disk_num);
    for it_t=1:N_time
        

        
        
        for disk_num=1:N_disks
            
            xyz_XYZ = rel_kinematics_disks{disk_num};
            
            X(disk_num) = xyz_XYZ(it_t, 1);
            Y(disk_num) = xyz_XYZ(it_t, 2);
            Z(disk_num) = xyz_XYZ(it_t, 3);
        

        end
        figure(2)
        scatter3(X, Y, Z, 100, colors, 'filled'); % '100' sets the marker size
        grid on;
        xlim([-1, 1])
        ylim([-1, 1])
        zlim([-1, 1])
        xlabel("X [m]")
        ylabel("Y [m]")
        zlabel("Z [m]")
        drawnow

    end

end