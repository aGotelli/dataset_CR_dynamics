function [N_disks, timestamps, poses_disks, rel_poses_disks, rel_kinematics_disks] = data_optitrack(filename)
    

    mocap = readtable(filename);
    
    disk_names = {'disk_0', 'disk_1', 'disk_2', 'disk_3', 'disk_4'};
    N_disks = length(disk_names);


    timestamps = mocap.timestamp_s;
    N_time = length(timestamps);


    make_video = false;
    plots = true;




    %%   First check data for disks is correct
    
    indices = zeros(N_time, N_disks);
    for it=1:N_disks

        label = disk_names{it} + "_is_valid";

        indices(:, it) = mocap.( label );

    end



    %%   Extract data for every disk
    poses_disks = zeros(4, 4, N_disks, N_time);
    bottom = repmat([0 0 0 1], 1, 1, N_time);


    if(plots)
        figure("Name", "Position Disks Raw")
    end
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
        
        if(plots)
            subplot(5, 1, it)
            time = timestamps - timestamps(1);
            plot(time, x_data, 'r', 'LineWidth', 2)
            hold on
            plot(time, y_data, 'g', 'LineWidth', 2)
            plot(time, z_data, 'b', 'LineWidth', 2)
            title(disk)
        end
    end

    
    


   


    %%  Compute relative kinematics
    %   Frame 0 is at base


    %   From this we obtain the set of poses for the mocap system 
    %   Knowing that the frame at the base does not move, we can compute
    %   the relative poses at each time-step

    pose_disk_0 = poses_disks(:, :, 1, :);

    R_0 = pose_disk_0(1:3, 1:3, 1, :);
    r_0 = pose_disk_0(1:3, 4, 1, :);

    R_0T = pagetranspose(R_0);
    r_0i = -pagemtimes(R_0T, r_0);
    bottom = repmat([0 0 0 1], 1, 1, 1, N_time);
    inv_pose_disk_0 = [
      R_0T  r_0i
      bottom
    ];



    %   OptiTrack/Motion software defines Z pointing down;
    R_fix = axang2rotm([1 0 0 pi]);
    g_fix = eye(4);
    % g_fix(1:3, 1:3) = R_fix;

    rel_poses_disks = zeros(size(poses_disks));

    rel_kinematics_disks = zeros(N_time, 6, N_disks);
    
    if(plots)
        figure("Name", "Position Disks (Relative)")
    end
    for it = 1:N_disks
        pose_disk = poses_disks(:, :, it, :);

        rel_pose_disk = pagemtimes( g_fix, pagemtimes(inv_pose_disk_0, pose_disk) );
        % rel_pose_disk(1:3, 4, :, :) = -rel_pose_disk(1:3, 4, :, :);
        rel_poses_disks(:, :, it, :) = rel_pose_disk;

        r_disk = squeeze( rel_pose_disk(1:3, 4, :, :) );
        R_disk = squeeze( rel_pose_disk(1:3, 1:3, :, :) );
        XYZ_disk = rotm2eul(R_disk, 'XYZ');

        rel_kinematics_disks(:, :, it) = [
          XYZ_disk   r_disk'
        ];

        disk = disk_names{it};
        
        if(plots)
            subplot(5, 1, it)
            time = timestamps - timestamps(1);
            plot(time, r_disk(1, :), 'r', 'LineWidth', 2)
            hold on
            plot(time, r_disk(2, :), 'g', 'LineWidth', 2)
            plot(time, r_disk(3, :), 'b', 'LineWidth', 2)
            title(disk)
        end
    end


    
    %remove last snapshot
    timestamps = timestamps(1:end-1);
    poses_disks = poses_disks(:, :, :, 1:end-1);
    rel_poses_disks = rel_poses_disks(:, :, :, 1:end-1);
    rel_kinematics_disks = rel_kinematics_disks(1:end-1, :, :);


    
    plot_poses = rel_poses_disks;
    if make_video

        axis_len = 0.05; % length of frame axes in meters


        fig = figure("Name", "Video Shape");
        % set(fig, 'Color', 'k');

        for it_t = 1:N_time
    
            time = timestamps(it_t) - timestamps(1);
    
            % ax = gca;
            % set(ax, 'Color', 'k', 'XColor', 'w', 'YColor', 'w', 'ZColor', 'w');
            hold on
    
            for it = 1:N_disks
                g = plot_poses(:, :, it, it_t);   % 4x4 relative pose
                p = g(1:3, 4);                          % origin
                R = g(1:3, 1:3);                        % rotation matrix
    
                x_ax = R(:, 1);
                y_ax = R(:, 2);
                z_ax = R(:, 3);
    
                % Plot frame axes: X=red, Y=green, Z=blue
                quiver3(p(1), p(2), p(3), x_ax(1)*axis_len, x_ax(2)*axis_len, x_ax(3)*axis_len, 'r', 'LineWidth', 2, 'AutoScale', 'off');
                quiver3(p(1), p(2), p(3), y_ax(1)*axis_len, y_ax(2)*axis_len, y_ax(3)*axis_len, 'g', 'LineWidth', 2, 'AutoScale', 'off');
                quiver3(p(1), p(2), p(3), z_ax(1)*axis_len, z_ax(2)*axis_len, z_ax(3)*axis_len, 'b', 'LineWidth', 2, 'AutoScale', 'off');
    
                % Label with disk number (0-indexed)
                % text(p(1), p(2), p(3), sprintf('  %d', it-1), 'Color', 'w', 'FontSize', 10);
            end
    
            grid on
            xlim([-0.5, 0.5])
            ylim([-0.5, 0.5])
            zlim([-0.5, 0.5])
            xlabel('X', 'Color', 'w')
            ylabel('Y', 'Color', 'w')
            zlabel('Z', 'Color', 'w')
            title("Time " + num2str(time, '%.2f') + " s", 'Color', 'w')
            view(3)   % standard 3D perspective: azimuth -37.5, elevation 30
            drawnow
            hold off
    
        end

    end



end