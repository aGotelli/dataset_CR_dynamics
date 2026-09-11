function [N_disks, timestamps, poses_disks, rel_poses_disks, rel_kinematics_disks, is_valid_disk] = data_optitrack(filename, use_resense)
    
    %   Read the data table
    mocap = readtable(filename);
    
    %   Parse the disks name (saved with a mocap-independent python script)
    if use_resense
        disk_names = {'disk_0', 'disk_1', 'disk_2', 'disk_3', 'disk_4', 'disk_5'};
    else
        disk_names = {'disk_0', 'disk_1', 'disk_2', 'disk_3', 'disk_4'};
    end
    N_disks = length(disk_names);

    
    %   Extract timestamp
    timestamps = mocap.timestamp_s;
    N_time = length(timestamps);

    
    %   Read Motive's per-disk, per-sample validity flag.
    is_valid_disk = zeros(N_time, N_disks);
    for it = 1:N_disks

        label = disk_names{it} + "_is_valid";

        is_valid_disk(:, it) = mocap.( label );

    end



    %%   Extract data for every disk
    poses_disks = zeros(4, 4, N_disks, N_time);
    bottom = repmat([0 0 0 1], 1, 1, N_time);

    for it = 1:N_disks
        
        disk = disk_names{it};

        
        % Extract coordinates
        x_data = mocap.([disk '_x']);
        y_data = mocap.([disk '_y']);
        z_data = mocap.([disk '_z']);
       
        qx = mocap.([disk '_qx']);
        qy = mocap.([disk '_qy']);
        qz = mocap.([disk '_qz']);
        qw = mocap.([disk '_qw']);

               
   
    
        %  Compose the corresponding rotation and position vector
        r = zeros(3, 1, N_time);
        r(1, 1, :) = x_data;
        r(2, 1, :) = y_data;
        r(3, 1, :) = z_data;

        R = quat2rotm([qw qx qy qz]);

        
        g_disk_abs = [
             R  r
            bottom
        ];

        poses_disks(:, :, it, :) = g_disk_abs;
        
    end

    
    %   Create frame at the base with correct orientation
    R_ref = [
     0 1 0
     0 0 -1
     -1 0 0
    ];

    %   For position take first second of recording (120 samples)
    r_ref = mean(squeeze(poses_disks(1:3, 4, 1, 1:120)), 2);
    r_ref_std = std(squeeze(poses_disks(1:3, 4, 1, 1:120)), 0, 2);
   

    rel_transf = [
        R_ref'  -(R_ref'*r_ref)
        0 0 0 1
    ];
    rel_transf_pages = repmat(rel_transf, [1 1 1 N_time]);

    
    rel_poses_disks = zeros(size(poses_disks));
    rel_kinematics_disks = zeros(N_time, 6, N_disks);
    
    for it = 1:N_disks
        %   Extract disk pose
        pose_disk = poses_disks(:, :, it, :);

        %   Compute relative pose wrt base of the robot
        rel_pose_disk = pagemtimes(rel_transf_pages, pose_disk);

        %   Stack it
        rel_poses_disks(:, :, it, :) = rel_pose_disk;
        
        %   Compute corresponding xyz Roll Pitch Yaw (Eul. XYZ)
        r_disk = squeeze( rel_pose_disk(1:3, 4, :, :) );
        R_disk = squeeze( rel_pose_disk(1:3, 1:3, :, :) );
        XYZ_disk = rotm2eul(R_disk, 'XYZ');

        rel_kinematics_disks(:, :, it) = [
          XYZ_disk   r_disk'
        ];

    end


    
    %   remove last snapshot (typically contains NaN)
    timestamps = timestamps(1:end-1);
    poses_disks = poses_disks(:, :, :, 1:end-1);
    rel_poses_disks = rel_poses_disks(:, :, :, 1:end-1);
    rel_kinematics_disks = rel_kinematics_disks(1:end-1, :, :);

end