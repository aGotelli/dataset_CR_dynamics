function [N_disks, mocap_timestamps, rel_kinematics_disks, rel_kinematics_disks_corr, ...
    fbgs_time, fbgs_shapes, fbgs_curvatures, fbgs_angles] = ...
    align_mocap_and_fbgs(folder, use_resense, align_window_s, bending_axis)
%ALIGN_MOCAP_AND_FBGS Load one recording's OptiTrack and FBG data and put
%   them in a common, spatially-aligned frame.
%
%   This function was extracted out of process_data.m (Reviewer 5, Comment
%   5.30) so that the exact same spatial-alignment logic can be reused by
%   a second script, check_fbg_delay_5_30.m, which needs these same
%   aligned signals but WITHOUT the FBG temporal (pipeline-delay)
%   correction applied. Keeping this logic in one place means the two
%   scripts are guaranteed to be working from identical spatial alignment
%   -- the only difference between them is whether the caller applies a
%   time shift to fbgs_time afterwards or not.
%
%   Deliberately NOT done in here: the FBG timestamp correction
%   (fbgs_time = fbgs_time - lag_FBGS/1000 in process_data.m). That is a
%   TEMPORAL correction, not a spatial one, and separating it from this
%   function is the whole point of this refactor -- see
%   check_fbg_delay_5_30.m, which calls this function and does NOT apply
%   that correction, so it can measure the FBG delay on uncorrected data.
%
%   Inputs:
%     folder          - path to one recording's folder (containing
%                        dataOptiTrack.csv and dataFBGS.csv)
%     use_resense     - passed straight through to data_optitrack (true
%                        for recordings that also track the Resense wand)
%     align_window_s  - length, in seconds, of the initial portion of the
%                        recording used to determine the bending-plane
%                        rotation (both mocap and FBG use their own first
%                        align_window_s seconds)
%     bending_axis    - 'x' or 'y', the axis the robot is expected to
%                        bend along in this recording (see process_data.m
%                        for which recordings use which)
%
%   Outputs:
%     N_disks                    - number of tracked OptiTrack disks
%     mocap_timestamps            - OptiTrack timestamps (unchanged)
%     rel_kinematics_disks        - mocap disk poses, RAW (no per-disk
%                                    residual-offset correction)
%     rel_kinematics_disks_corr   - mocap disk poses, WITH the per-disk
%                                    residual-offset correction applied
%     fbgs_time                   - FBG timestamps, UNCORRECTED (see
%                                    above -- no pipeline-delay shift)
%     fbgs_shapes                 - FBG reconstructed shapes, rotated
%                                    into the robot body frame and
%                                    aligned to the mocap bending plane
%     fbgs_curvatures, fbgs_angles - passed straight through from
%                                    data_fbgs, unchanged


filename = fullfile(folder, "dataOptiTrack.csv");
[N_disks, mocap_timestamps, poses_disks, rel_poses_disks, rel_kinematics_disks] = data_optitrack(filename, use_resense);


filename = fullfile(folder, "dataFBGS.csv");
[fbgs_time, fbgs_shapes, fbgs_curvatures, fbgs_angles] = data_fbgs(filename);


%   Apply rotation of -90 deg along y axis to ALL shapes
R_y = axang2rotm([0 1 0 -pi/2]);
N_time_fbgs = size(fbgs_shapes, 3);
for t = 1:N_time_fbgs
    fbgs_shapes(:, :, t) = R_y * fbgs_shapes(:, :, t);
end


mocap_time_rel  = mocap_timestamps - mocap_timestamps(1);
idx_align      = mocap_time_rel <= align_window_s;

XYZ_xyz_disk = rel_kinematics_disks(:, :, 5);
tip_xy_mocap  = XYZ_xyz_disk(idx_align, 4:5);            
tip_xy_mocap_centered = tip_xy_mocap - mean(tip_xy_mocap, 1);
[U_m, S_m, V_m] = svd(tip_xy_mocap_centered, 'econ');



R_m = eye(3);
R_m(1:2, 1:2) = V_m;
axang_m = rotm2axang(R_m);
theta_z_mocap = axang_m(4);

% theta_z_mocap = atan2(R_m(1, 2), R_m(1, 1));

if strcmpi(bending_axis, 'y')
    theta_z_mocap = pi/2 - theta_z_mocap;       % map onto y-axis
else
    theta_z_mocap = 0 - theta_z_mocap;          % map onto x-axis
end

%   The fiber now evolves in z, but bending leaks into both x and y.
%   Use SVD on the tip x-y trajectory (first 10 s only, planar portion)
%   to find the bending direction, then rotate about z.
fbgs_time_rel  = fbgs_time - fbgs_time(1);
idx_align      = fbgs_time_rel <= align_window_s;

tip_xy_all     = squeeze(fbgs_shapes(1:2, end, :));   % 2 x N_time
tip_xy         = tip_xy_all(:, idx_align);            % 2 x N_align
tip_xy_centered = tip_xy - mean(tip_xy, 2);
[U_f, S_f, V_f] = svd(tip_xy_centered, 'econ');

R_f = eye(3);
R_f(1:2, 1:2) = U_f;
axang_f = rotm2axang(R_f);
theta_z_fbgs = axang_f(4);

if strcmpi(bending_axis, 'y')
    theta_z_fbgs = pi/2 - theta_z_fbgs;       % map onto y-axis
else
    theta_z_fbgs = 0 - theta_z_fbgs;          % map onto x-axis
end

if strcmpi(bending_axis, 'y')
    theta_z = theta_z_fbgs + theta_z_mocap;
else
    theta_z = theta_z_fbgs - theta_z_mocap;
end


R_z = axang2rotm([0 0 1 theta_z]);
for t = 1:N_time_fbgs
    fbgs_shapes(:, :, t) = R_z * fbgs_shapes(:, :, t);
end




%%  Correct pose mocap (only frame of the robot)
idx_init      = mocap_time_rel <= 3.0;

rel_kinematics_disks_init = rel_kinematics_disks(idx_init, :, :);
mocap_time_rel_init = mocap_time_rel(idx_init);

%   Remove residual offset
pos_disks = [
    0    0    0    0    0
    0    0    0    0    0
    0    0.12 0.24 0.36 0.48
];

N_disks_robot = 5;

g_correction = zeros(4, 4, N_disks_robot);
rel_kinematics_disks_corr = zeros(size(rel_kinematics_disks));
for it=1:N_disks_robot

    g_disk_ref = eye(4);
    g_disk_ref(1:3, 4) = pos_disks(:, it);


    
    EUL_disk_t = rel_kinematics_disks_init(:, 1:3, it)';
    r_disk_t = rel_kinematics_disks_init(:, 4:6, it)';

    EUL_disk = mean(EUL_disk_t, 2);
    r_disk = mean(r_disk_t, 2);

    R_disk = eul2rotm(EUL_disk', 'XYZ');

    g_meas_m1 = [
        R_disk' -R_disk'*r_disk
        0   0   0   1
    ];

    g_correction(:,:, it) = g_meas_m1*g_disk_ref;

    

    rel_poses_disk = rel_poses_disks(:, :, it, :);

    rel_poses_disk_corr = pagemtimes(rel_poses_disk, g_correction(:,:, it));


    r_disk_corr = squeeze( rel_poses_disk_corr(1:3,   4, :, :) );
    R_disk_corr = squeeze( rel_poses_disk_corr(1:3, 1:3, :, :) );
    XYZ_disk_corr = rotm2eul(R_disk_corr, 'XYZ');

    rel_kinematics_disks_corr(:, :, it) = [
      XYZ_disk_corr   r_disk_corr'
    ];

end

end
