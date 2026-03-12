close all
clear
clc

%% ====== SETTINGS ======
folder = fullfile("dataCollectionPack", "20260304", "plane_x_slow", "processed");

d = 0.0375;       % cable offset from backbone center [m] (37.5 mm)
N_interp = 40;    % interpolation points along backbone (from 5 disks)
r_spool = 0.02;   % spool radius [m] — SET to your actual spool radius

N_disks = 5;
N_ref   = 10;     % initial frames averaged as "straight" reference

%% ====== LOAD DATA ======
vicon_data  = readmatrix(fullfile(folder, "vicon_frames.csv"));
angles_data = readmatrix(fullfile(folder, "angles.csv"));

N_time = size(vicon_data, 1);
time   = vicon_data(:, 1);

%   Parse disk kinematics
%   vicon_frames.csv stores a 3-D array (N_time × 7 × N_disks) written
%   page-by-page so that each disk occupies 7 consecutive columns:
%       [time, eul1, eul2, eul3, px, py, pz]
%   Euler angles follow MATLAB's 'XYZ' intrinsic convention.
disk_euler = zeros(N_time, 3, N_disks);
disk_pos   = zeros(N_time, 3, N_disks);
for it = 1:N_disks
    col0 = (it-1)*7;
    disk_euler(:, :, it) = vicon_data(:, col0 + (2:4));
    disk_pos(:, :, it)   = vicon_data(:, col0 + (5:7));
end

%   Parse motor angles  [motor1(+x), motor2(+y), motor3(-x), motor4(-y)]
motor_angles = angles_data(:, 2:5);

%% ====== UNWRAP EULER ANGLES ======
%   Euler angles near ±π can wrap, producing large jumps that corrupt the
%   rotation→quaternion conversion.  Unwrap each component over time.
%   The yaw (dim 3, rotation about Z) should be zero for a planar robot but
%   accumulates large drift after unwrapping — force it to zero.
for it = 1:N_disks
    for dim = 1:3
        disk_euler(:, dim, it) = unwrap(disk_euler(:, dim, it));
    end
    disk_euler(:, 3, it) = 0;   % yaw forced to zero
end

% %% ====== PLOT EULER ANGLES (raw vs unwrapped) ======
% eul_names = {'Roll (X)', 'Pitch (Y)', 'Yaw (Z)'};
% for it = 1:N_disks
%     raw_eul = zeros(N_time, 3);
%     for dim = 1:3
%         col0 = (it-1)*7;
%         raw_eul(:, dim) = vicon_data(:, col0 + 1 + dim);
%     end
% 
%     figure("Name", sprintf("Euler Angles – disk %d", it-1));
%     for dim = 1:3
%         subplot(3,1,dim)
%         plot(time, rad2deg(raw_eul(:,dim)),           'b', 'LineWidth', 1.2); hold on
%         plot(time, rad2deg(disk_euler(:, dim, it)),   'r', 'LineWidth', 1.2);
%         grid on
%         ylabel([eul_names{dim} ' [deg]'])
%         if dim == 1
%             legend('Raw', 'Unwrapped', 'Location', 'best')
%             title(sprintf('Disk %d', it-1))
%         end
%         if dim == 3, xlabel('Time [s]'); end
%     end
% end

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

fprintf('Computing cable lengths (%d time steps) ...\n', N_time);
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

    if mod(t, 500) == 0, fprintf('  %d / %d\n', t, N_time); end
end
fprintf('Done.\n');

%% ====== STRAIGHT-CONFIGURATION REFERENCE ======
cable_lengths_ref = mean(cable_lengths(1:N_ref, :), 1);

%% ====== DELTA CABLE LENGTHS (from MoCap geometry) ======
delta_cable_computed = cable_lengths - cable_lengths_ref;

%   Sign convention: negate all, then restore +x/-x (cables 1 & 3).
delta_cable_computed(:, [2 4]) = -delta_cable_computed(:, [2 4]);

%% ====== DELTA CABLE LENGTHS (from motor angles) ======
%   Sign convention: check whether positive motor angle means pulling
%   (cable shortening) or releasing.  Adjust the sign of r_spool if needed.
motor_angles_ref     = mean(motor_angles(1:N_ref, :), 1);
delta_cable_measured = (motor_angles - motor_angles_ref) * r_spool;

%% ====== PLOTS ======
cable_labels = {'+x', '+y', '-x', '-y'};
pairs = {[1 3], [2 4]};          % x-pair, y-pair
pair_names = {"x", "y"};

for p = 1:2
    fig = figure("Name", "Cable Length Change – " + pair_names{p} + " pair");
    idx = pairs{p};
    for k = 1:2
        ax = subplot(2,1,k);
        set(ax, 'Color', 'w');
        c = idx(k);
        plot(time, delta_cable_computed(:,c)*1e3,  'b',  'LineWidth', 1.5);  hold on
        plot(time, delta_cable_measured(:,c)*1e3,  'r--','LineWidth', 1.5);
        grid on; ylabel('\DeltaL [mm]')
        title(['Cable ' cable_labels{c}])
        if k == 1
            legend('MoCap (computed)', 'Motor (measured)', 'Location', 'best')
        end
        if k == 2, xlabel('Time [s]'); end
    end
    savefig(fig, fullfile(folder, "cable_" + pair_names{p} + ".fig"));
    exportgraphics(fig, fullfile(folder, "cable_" + pair_names{p} + ".png"), 'Resolution', 300, 'BackgroundColor', 'white');
end