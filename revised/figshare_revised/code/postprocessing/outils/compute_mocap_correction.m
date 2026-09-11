function compute_mocap_correction(data_root, disk_z_positions)
%COMPUTE_MOCAP_CORRECTION Compute and save the per-disk mocap residual-
%   offset correction, from the dedicated straight/reference recording.
%
%   The correction is a static property of the physical setup (marker
%   mounting on the robot), not something that changes recording to
%   recording. It's computed once here and saved so align_mocap_and_fbgs.m
%   can just load it instead of recomputing it from each recording's own
%   first few seconds.
%
%   data_root         - path to the dataset's data/ folder (set in
%                        process_data.m, passed in here so it isn't
%                        duplicated in multiple places)
%   disk_z_positions - nominal z-position (m) of each of the 5 robot
%                       disks along the fiber in the undeformed/straight
%                       configuration (set in process_data.m, passed in
%                       here so it isn't duplicated in two places)

%% ====== SETTINGS ======
folder = fullfile(data_root, "references", "straight_config/");

use_resense = false;

N_disks_robot = 5;

%% ====== LOAD DATA ======
filename = fullfile(folder, "dataOptiTrack.csv");
[~, ~, ~, rel_poses_disks, rel_kinematics_disks] = data_optitrack(filename, use_resense);

%% ====== COMPUTE PER-DISK CORRECTION ======
%   Averaged over the WHOLE recording, not just an initial window: unlike
%   the per-recording fallback align_mocap_and_fbgs.m used to have (which
%   can only trust its first few seconds before the robot starts moving),
%   straight_config is held straight for its entire duration, so using
%   every sample reduces noise in the estimate instead of discarding data.
pos_disks = zeros(3, length(disk_z_positions));
pos_disks(3, :) = disk_z_positions;

correction_kinematics = zeros(N_disks_robot, 6);   %   one row per disk: [roll pitch yaw px py pz]
for it = 1:N_disks_robot

    g_disk_ref = eye(4);
    g_disk_ref(1:3, 4) = pos_disks(:, it);

    EUL_disk_t = rel_kinematics_disks(:, 1:3, it)';
    r_disk_t = rel_kinematics_disks(:, 4:6, it)';

    EUL_disk = mean(EUL_disk_t, 2);
    r_disk = mean(r_disk_t, 2);

    R_disk = eul2rotm(EUL_disk', 'XYZ');

    g_meas_m1 = [
        R_disk' -R_disk'*r_disk
        0   0   0   1
    ];

    g_correction = g_meas_m1*g_disk_ref;

    R_correction = g_correction(1:3, 1:3);
    r_correction = g_correction(1:3, 4);
    EUL_correction = rotm2eul(R_correction, 'XYZ');

    correction_kinematics(it, :) = [EUL_correction r_correction'];

end

%% ====== SAVE ======
%   One row per disk: [roll pitch yaw px py pz]. Saved next to this
%   function so align_mocap_and_fbgs.m can find it relative to its own
%   folder, the same way process_data.m locates measured_fbg_delay_ms.txt.
writematrix(correction_kinematics, fullfile(fileparts(mfilename('fullpath')), "mocap_correction.csv"));

fprintf("   SAVED MOCAP CORRECTION\n");

end
