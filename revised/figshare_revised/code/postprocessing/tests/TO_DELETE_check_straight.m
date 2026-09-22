%% compare_straight_config_offset.m
% Sanity check for Comment 5.22: compares the per-disk static offset
% implied by references/straight_config/ (averaged over the whole
% recording) against a fresh offset computed from the first 3 seconds
% of individual recordings (the pre-motion hold, before any command
% is sent). Both are expressed as disk_k pose relative to disk_0, so
% no nominal/expected geometry is needed.

clear; clc;

%% --- EDIT THESE PATHS to match your layout ---
straight_config_file = fullfile('..','..','data','references','straight_config','dataOptiTrack.csv');
recording_files = {
    fullfile('..','..','data','dynamic_motion','circle_fast','dataOptiTrack.csv')
    fullfile('..','..','data','dynamic_motion','plane_x_slow','dataOptiTrack.csv')
    fullfile('..','..','data','quasi_static','static_bend_x_180','dataOptiTrack.csv')
};
n_disks     = 5;      % disks 0..4
window_s    = 3.0;    % pre-motion window, seconds
fs_nominal  = 120;    % OptiTrack rate, Hz

%% --- straight_config: average over the FULL recording ---
T_straight = load_disk_poses(straight_config_file, n_disks);
rel_straight = relative_to_disk0(T_straight, n_disks);

fprintf('--- straight_config (full recording, disk_k rel. disk_0) ---\n');
print_relative_table(rel_straight, n_disks);

%% --- each recording: average over the first window_s seconds only ---
for i = 1:numel(recording_files)
    T_rec = load_disk_poses(recording_files{i}, n_disks);
    n_window = round(window_s * fs_nominal);
    T_rec_window = restrict_frames(T_rec, n_disks, 1:n_window);
    rel_rec = relative_to_disk0(T_rec_window, n_disks);

    fprintf('\n--- %s (first %.1fs, disk_k rel. disk_0) ---\n', recording_files{i}, window_s);
    print_relative_table(rel_rec, n_disks);

    fprintf('--- difference vs straight_config (recording - straight_config) ---\n');
    print_difference_table(rel_rec, rel_straight, n_disks);
end

%% ================= Helpers =================

function T = load_disk_poses(csv_path, n_disks)
    % ADAPT the column-name pattern below if it doesn't match your
    % actual dataOptiTrack.csv header exactly.
    tbl = readtable(csv_path);
    T = cell(n_disks,1);
    for k = 0:n_disks-1
        valid = tbl.(sprintf('disk_%d_is_valid', k)) ~= 0;
        pose = [tbl.(sprintf('disk_%d_x',  k))(valid), ...
                tbl.(sprintf('disk_%d_y',  k))(valid), ...
                tbl.(sprintf('disk_%d_z',  k))(valid), ...
                tbl.(sprintf('disk_%d_qx', k))(valid), ...
                tbl.(sprintf('disk_%d_qy', k))(valid), ...
                tbl.(sprintf('disk_%d_qz', k))(valid), ...
                tbl.(sprintf('disk_%d_qw', k))(valid)];
        T{k+1} = pose;
    end
end

function T_out = restrict_frames(T, n_disks, idx)
    T_out = cell(n_disks,1);
    for k = 1:n_disks
        keep = idx(idx <= size(T{k},1));
        T_out{k} = T{k}(keep,:);
    end
end

function rel = relative_to_disk0(T, n_disks)
    % Mean pose per disk, then disk k expressed relative to disk 0.
    % Component-wise quaternion averaging is fine here since all
    % windows are near the straight configuration (small angular spread).
    mean_pos  = cell(n_disks,1);
    mean_quat = cell(n_disks,1);
    for k = 1:n_disks
        mean_pos{k}  = mean(T{k}(:,1:3), 1);
        q = mean(T{k}(:,4:7), 1);
        mean_quat{k} = q / norm(q);
    end

    R0 = quat2rotm_xyzw(mean_quat{1});
    p0 = mean_pos{1};

    rel = cell(n_disks,1);
    for k = 1:n_disks
        Rk = quat2rotm_xyzw(mean_quat{k});
        pk = mean_pos{k};
        R_rel = R0' * Rk;
        p_rel = (R0' * (pk - p0)')';
        eul = rotm2eul(R_rel, 'XYZ');
        rel{k} = [p_rel, eul];   % [dx dy dz roll pitch yaw]
    end
end

function R = quat2rotm_xyzw(q)
    % q = [qx qy qz qw] (scalar-last, per Table 3) -> MATLAB wants scalar-first
    R = quat2rotm([q(4) q(1) q(2) q(3)]);
end

function print_relative_table(rel, n_disks)
    fprintf('%6s %10s %10s %10s %10s %10s %10s\n', 'disk','dx[mm]','dy[mm]','dz[mm]','roll[deg]','pitch[deg]','yaw[deg]');
    for k = 1:n_disks
        r = rel{k};
        fprintf('%6d %10.3f %10.3f %10.3f %10.3f %10.3f %10.3f\n', k-1, ...
            1e3*r(1), 1e3*r(2), 1e3*r(3), rad2deg(r(4)), rad2deg(r(5)), rad2deg(r(6)));
    end
end

function print_difference_table(rel_a, rel_b, n_disks)
    fprintf('%6s %10s %10s %10s %10s %10s %10s\n', 'disk','ddx[mm]','ddy[mm]','ddz[mm]','droll[deg]','dpitch[deg]','dyaw[deg]');
    for k = 1:n_disks
        d = rel_a{k} - rel_b{k};
        fprintf('%6d %10.3f %10.3f %10.3f %10.3f %10.3f %10.3f\n', k-1, ...
            1e3*d(1), 1e3*d(2), 1e3*d(3), rad2deg(d(4)), rad2deg(d(5)), rad2deg(d(6)));
    end
end