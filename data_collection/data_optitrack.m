function [N_disks, timestamp, rel_kinematics_disks, poses_disks, kinematics_disks, disk_poses_static] = data_optitrack(filename, N_frames_static_begin)
    
% folder = fullfile("dataCollectionPack","20260225/", "circle_slow/");
% N_frames_static_begin = 10; %how many frames to use to compute relative pose for Vicon
% filename = fullfile(folder, "dataOptiTrack.csv");
    
    mocap = readtable(filename);
    
    disk_names = {'disk_0', 'disk_1', 'disk_2', 'disk_3', 'disk_4'};
    N_disks = length(disk_names);


    



    %%  TMP modify this later

    optitrack = mocap.Variables;
    optitrack = optitrack(1:end-1, :);

    timestamp = optitrack(:, 1);
    N_time = length(timestamp);


    %%  Sort the data disk-wise
    N_disks = 5;

    disk_info = 9;

    disks_data = zeros(N_time, disk_info, N_disks);

    first = 2;
    for it=1:N_disks


        last = first + disk_info - 1;

        disk_data = optitrack(:, first:last);

        disk_id = disk_data(1,1);
        disks_data(:, :, disk_id + 1) = disk_data;

        first = last + 1;
        
    end
    
    
    
    %%   Extract the first static data 
    
    disk_pos_mean = zeros(3, N_disks);
    disk_pos_stdv = zeros(3, N_disks);
    
    disk_Rot_mean = zeros(3, N_disks);
    disk_Rot_stdv = zeros(3, N_disks);
    
    %   Extract X, Y, Z position and euler angles for each disk - before motion start
    for it = 1:N_disks
        
        disk_data = disks_data(:, :, it);
        
        % Extract coordinates
        x_data = disk_data(:, 3);
        y_data = disk_data(:, 4);
        z_data = disk_data(:, 5);
    
        x_mean = mean(x_data(1:N_frames_static_begin));
        y_mean = mean(y_data(1:N_frames_static_begin));
        z_mean = mean(z_data(1:N_frames_static_begin));
        disk_pos_mean(:, it) = [x_mean y_mean z_mean]';
    
        x_stdv = std(x_data(1:N_frames_static_begin));
        y_stdv = std(y_data(1:N_frames_static_begin));
        z_stdv = std(z_data(1:N_frames_static_begin));
        disk_pos_stdv(:, it) = [x_stdv y_stdv z_stdv]';
    
    
        qx = disk_data(:, 6);
        qy = disk_data(:, 7);
        qz = disk_data(:, 8);
        qw = disk_data(:, 9);

    
        qx_mean = mean(qx(1:N_frames_static_begin));
        qy_mean = mean(qy(1:N_frames_static_begin));
        qz_mean = mean(qz(1:N_frames_static_begin));
        qw_mean = mean(qw(1:N_frames_static_begin));

        disk_Rot_mean(:, it) = quat2eul([qw_mean qx_mean qy_mean qz_mean], "XYZ")';



        qx_std = std(qx(1:N_frames_static_begin));
        qy_std = std(qy(1:N_frames_static_begin));
        qz_std = std(qz(1:N_frames_static_begin));
        qw_std = std(qw(1:N_frames_static_begin));


        disk_Rot_stdv(:, it) = quat2eul([qx_std qy_std qz_std qw_std], "XYZ")';
    
       
    end
    
    % Express the poses wrt the one at the robot base
    
    R = eul2rotm(disk_Rot_mean', "XYZ");
    r = reshape(disk_pos_mean, [3, 1, N_disks]);
    g_abs = [
        R   r
        zeros(1, 3, N_disks) ones(1, 1, N_disks)
    ];
    
    disk_1_pose = g_abs(:, :, 1);
    
    %   Rotation to reference frame
    Rs = axang2rotm([1 0 0 pi]);
    g_s = eye(4);
    g_s(1:3, 1:3) = Rs;
    
    disk_1_pose_inv = g_s*inv_g( disk_1_pose );
    
    disk_poses_static = pagemtimes(disk_1_pose_inv, g_abs);
    
    
    
    
    %%  Process Vicon Data 
    kinematics_disks     = zeros(N_time, 6, N_disks);
    rel_kinematics_disks = zeros(N_time, 6, N_disks);

    poses_disks = zeros(4, 4, N_disks, N_time);
    for it = 1:N_disks


        disk_data = disks_data(:, :, it);
        
        % Extract coordinates
        x_data = disk_data(:, 3);
        y_data = disk_data(:, 4);
        z_data = disk_data(:, 5);
       
    
        qx = disk_data(:, 6);
        qy = disk_data(:, 7);
        qz = disk_data(:, 8);
        qw = disk_data(:, 9);

        Rot_XYZ = quat2eul([qw qx qy qz], "XYZ");

        Rot_X = Rot_XYZ(:, 1);
        Rot_Y = Rot_XYZ(:, 2);
        Rot_Z = Rot_XYZ(:, 3);

        kinematics_disks(:, :, it) = [x_data y_data z_data Rot_X Rot_Y Rot_Z];
        
    
        % disk_data = disk_names{it};
        % 
        % % Extract coordinates
        % x_data = mocap.([disk_data 'X_m_']);
        % y_data = mocap.([disk_data 'Y_m_']);
        % z_data = mocap.([disk_data 'Z_m_']);
        % 
        % 
        % Rot_X = mocap.([disk_data 'RotX_rad_']);
        % Rot_Y = mocap.([disk_data 'RotY_rad_']);
        % Rot_Z = mocap.([disk_data 'RotZ_rad_']);
    
        % kinematics_disks(:, :, it) = [x_data y_data z_data Rot_X Rot_Y Rot_Z];
    
    
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
        poses_disks(:, :, it, :) = g_disk;
        
        %   Deconstruct poses
        x = squeeze(g_disk(1, 4, :));
        y = squeeze(g_disk(2, 4, :));
        z = squeeze(g_disk(3, 4, :));
    
        EUL = rotm2eul(g_disk(1:3, 1:3, :), 'XYZ');
    
        rel_kinematics_disks(:, :, it) = [x y z EUL];
    
    end

end