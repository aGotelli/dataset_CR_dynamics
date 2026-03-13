function [delta_cable_measured, delta_cable_computed] = compare_cable_lenght(disk_kinematics, angles_data, time)
    %% ====== SETTINGS ======
    
    d = 0.0375;       % cable offset from backbone center [m] (37.5 mm)
    N_interp = 40;    % interpolation points along backbone (from 5 disks)
    r_spool = 0.02;   % spool radius [m] — SET to your actual spool radius
    
    N_disks = 5;
    N_ref   = 10;     % initial frames averaged as "straight" reference
    
    
    
    N_time = length(time);
    
    %   Parse disk kinematics
    %   Euler angles follow MATLAB's 'XYZ' intrinsic convention.
    disk_euler = zeros(N_time, 3, N_disks);
    disk_pos   = zeros(N_time, 3, N_disks);
    for it = 1:N_disks
        col0 = (it-1)*6;
        disk_euler(:, :, it) = disk_kinematics(:, col0 + (1:3));
        disk_pos(:, :, it)   = disk_kinematics(:, col0 + (4:6));
    end
    
    
    %% ====== UNWRAP EULER ANGLES ======
    %   Euler angles near ±π can wrap, producing large jumps that corrupt the
    %   rotation→quaternion conversion.  Unwrap each component over time.
    %   The yaw (dim 3, rotation about Z) should be zero for a planar robot but
    %   accumulates large drift after unwrapping — force it to zero.
    for it = 1:N_disks
        for dim = 1:3
            disk_euler(:, dim, it) = unwrap(disk_euler(:, dim, it));
        end
    end
    
    
    %% ====== CABLE GEOMETRY ======
    %   4 antagonistic cables at distance d from backbone, in local-frame axes.
    %   Motor 1 → +x    Motor 2 → +y    Motor 3 → -x    Motor 4 → -y
    cable_offsets = [
         d  0  0      % cable 1  (+x)
         0  d  0      % cable 2  (+y)
        -d  0  0      % cable 3  (-x)
         0 -d  0      % cable 4  (-y)
    ];
    
    %% ====== INTERPOLATE DISK FRAMES & COMPUTE CABLE LENGTHS ======
    %   Parameterise the 5 disks by normalised arc-length  s ∈ [0, 1]
    s_disks  = linspace(0, 1, N_disks);
    s_interp = linspace(0, 1, N_interp);
    
    cable_lengths = zeros(N_time, 4);
    
    % fprintf('Computing cable lengths (%d time steps) ...\n', N_time);
    for t = 1:N_time
    
        % --- Disk rotations, positions, quaternions at this time step ---
        eul_all = squeeze(disk_euler(t, :, :))';     % N_disks × 3
        R_all   = eul2rotm(eul_all, 'XYZ');          % 3×3×N_disks
        p_all   = squeeze(disk_pos(t, :, :));         % 3×N_disks
        q_all   = rotm2quat(R_all);                   % N_disks × 4  [w x y z]
    
        %   Ensure quaternion hemisphere consistency (avoid sign-flip artefacts)
        for k = 2:N_disks
            if dot(q_all(k,:), q_all(k-1,:)) < 0
                q_all(k,:) = -q_all(k,:);
            end
        end
    
        % --- Interpolate positions  (pchip) ---
        p_interp = zeros(3, N_interp);
        for dim = 1:3
            p_interp(dim,:) = interp1(s_disks, p_all(dim,:), s_interp, 'pchip');
        end
    
        % --- Interpolate quaternions  (pchip + renormalise) ---
        q_interp = zeros(N_interp, 4);
        for dim = 1:4
            q_interp(:,dim) = interp1(s_disks, q_all(:,dim), s_interp, 'pchip')';
        end
        q_interp = q_interp ./ vecnorm(q_interp, 2, 2);
        R_interp = quat2rotm(q_interp);               % 3×3×N_interp
    
        % --- Cable attachment points & lengths ---
        for c = 1:4
            offset = cable_offsets(c,:)';
            %   R_interp * offset  →  3×1×N_interp  →  squeeze → 3×N_interp
            offset_world = squeeze(pagemtimes(R_interp, offset));
            cable_pts    = p_interp + offset_world;
    
            cable_lengths(t,c) = sum(vecnorm(diff(cable_pts, 1, 2), 2, 1));
        end
    
        % if mod(t, 500) == 0, fprintf('  %d / %d\n', t, N_time); end
    end
    % fprintf('Done.\n');
    
    %% ====== STRAIGHT-CONFIGURATION REFERENCE ======
    cable_lengths_ref = mean(cable_lengths(1:N_ref, :), 1);
    
    %% ====== DELTA CABLE LENGTHS (from MoCap geometry) ======
    delta_cable_computed = cable_lengths - cable_lengths_ref;
    
    %   Sign convention: negate all, then restore +x/-x (cables 1 & 3).
    delta_cable_computed(:, [2 4]) = -delta_cable_computed(:, [2 4]);
    
    %% ====== DELTA CABLE LENGTHS (from motor angles) ======
    %   Sign convention: check whether positive motor angle means pulling
    %   (cable shortening) or releasing.  Adjust the sign of r_spool if needed.
    motor_angles_ref     = mean(angles_data(1:N_ref, :), 1);
    delta_cable_measured = (angles_data - motor_angles_ref) * r_spool;
    

end