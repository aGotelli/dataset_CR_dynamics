function [N_disks, timestamp_vicon, rel_kinematics_disks, poses_disks, kinematics_disks, disk_poses_static] = data_vicon(filename, N_frames_static_begin)
    

    
    
    vicon = readtable(filename);
    
    disk_names = {'disk_0', 'disk_1', 'disk_2', 'disk_3', 'disk_4'};
    N_disks = length(disk_names);
    
    
    timestamp_vicon = vicon.timestamp;
    N_time = length(timestamp_vicon);
    
    
    
    %%   Extract the first static data 
    
    disk_pos_mean = zeros(3, N_disks);
    disk_pos_stdv = zeros(3, N_disks);
    
    disk_Rot_mean = zeros(3, N_disks);
    disk_Rot_stdv = zeros(3, N_disks);
    
    %   Extract X, Y, Z position and euler angles for each disk - before motion start
    for it = 1:N_disks
        
        disk = disk_names{it};
        
        % Extract coordinates
        x_data = vicon.([disk 'X_m_']);
        y_data = vicon.([disk 'Y_m_']);
        z_data = vicon.([disk 'Z_m_']);
    
        x_mean = mean(x_data(1:N_frames_static_begin));
        y_mean = mean(y_data(1:N_frames_static_begin));
        z_mean = mean(z_data(1:N_frames_static_begin));
        disk_pos_mean(:, it) = [x_mean y_mean z_mean]';
    
        x_stdv = std(x_data(1:N_frames_static_begin));
        y_stdv = std(y_data(1:N_frames_static_begin));
        z_stdv = std(z_data(1:N_frames_static_begin));
        disk_pos_stdv(:, it) = [x_stdv y_stdv z_stdv]';
    
    
        Rot_X = vicon.([disk 'RotX_rad_']);
        Rot_Y = vicon.([disk 'RotY_rad_']);
        Rot_Z = vicon.([disk 'RotZ_rad_']);
    
    
        Rot_X_mean = mean(Rot_X(1:N_frames_static_begin));
        Rot_Y_mean = mean(Rot_Y(1:N_frames_static_begin));
        Rot_Z_mean = mean(Rot_Z(1:N_frames_static_begin));
        disk_Rot_mean(:, it) = [Rot_X_mean Rot_Y_mean Rot_Z_mean]';
    
        Rot_X_stdv = std(Rot_X(1:N_frames_static_begin));
        Rot_Y_stdv = std(Rot_Y(1:N_frames_static_begin));
        Rot_Z_stdv = std(Rot_Z(1:N_frames_static_begin));
        disk_Rot_stdv(:, it) = [Rot_X_stdv Rot_Y_stdv Rot_Z_stdv]';
    
       
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
        
    
        disk = disk_names{it};
        
        % Extract coordinates
        x_data = vicon.([disk 'X_m_']);
        y_data = vicon.([disk 'Y_m_']);
        z_data = vicon.([disk 'Z_m_']);
    
        
        Rot_X = vicon.([disk 'RotX_rad_']);
        Rot_Y = vicon.([disk 'RotY_rad_']);
        Rot_Z = vicon.([disk 'RotZ_rad_']);
    
        kinematics_disks(:, :, it) = [x_data y_data z_data Rot_X Rot_Y Rot_Z];
    
    
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