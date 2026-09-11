function [N_disks, mocap_timestamps, rel_kinematics_disks, rel_kinematics_disks_corr, ...
    fbgs_time, fbgs_shapes, fbgs_curvatures, fbgs_angles] = ...
    align_mocap_and_fbgs(folder, use_resense, align_window_s)
%ALIGN_MOCAP_AND_FBGS Load one recording's OptiTrack and FBG data and put
%   them in a common, spatially-aligned frame.
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



%   Bending plane: set to 'x' or 'y' — the axis along which the rod bends
if(contains(folder, "_y_"))
    bending_axis = 'y';        % 'y' for plane_y experiments
else
    bending_axis = 'x';        % 'x' for all rest
end


%%  Mocap section

%   Extract data
filename = fullfile(folder, "dataOptiTrack.csv");
[N_disks, mocap_timestamps, ~, rel_poses_disks, rel_kinematics_disks] = data_optitrack(filename, use_resense);

%   Temporal variable used to defined align window
mocap_time_rel  = mocap_timestamps - mocap_timestamps(1);
idx_align      = mocap_time_rel <= align_window_s;

%   Extract kinematics tip disk which present the most ample motion
XYZ_xyz_tip_disk = rel_kinematics_disks(:, :, 5);
tip_xy_mocap  = XYZ_xyz_tip_disk(idx_align, 4:5);   

%   Center to compute the plane of motion
tip_xy_mocap_centered = tip_xy_mocap - mean(tip_xy_mocap, 1);
[~, ~, V_m] = svd(tip_xy_mocap_centered, 'econ');

%   Angle of the principal (max-variance) direction w.r.t. the x-axis.
%   Read directly off V_m instead of padding it into a 3x3 rotation
%   matrix and going through rotm2axang: SVD does not fix the sign of
%   its singular vectors, so det(V_m) can come out -1 depending on the
%   data, and rotm2axang is undefined for a matrix that isn't a proper
%   rotation.
theta_z_mocap = atan2(V_m(2, 1), V_m(1, 1));

if bending_axis == 'y'
    theta_z_mocap = pi/2 - theta_z_mocap;       % map onto y-axis
else
    theta_z_mocap = 0 - theta_z_mocap;          % map onto x-axis
end

%%  FBG section

%   Extract data
filename = fullfile(folder, "dataFBGS.csv");
[fbgs_time, fbgs_shapes, fbgs_curvatures, fbgs_angles] = data_fbgs(filename);

%   Apply rotation of -90 deg along y axis to ALL shapes (align with mocap
%   convention)
R_y = axang2rotm([0 1 0 -pi/2]);
N_time_fbgs = size(fbgs_shapes, 3);
for t = 1:N_time_fbgs
    fbgs_shapes(:, :, t) = R_y * fbgs_shapes(:, :, t);
end

%   The fiber now evolves in z, but bending leaks into both x and y.
%   Use SVD on the tip x-y trajectory (first 10 s only, planar portion)
%   to find the bending direction, then rotate about z.
fbgs_time_rel  = fbgs_time - fbgs_time(1);
idx_align      = fbgs_time_rel <= align_window_s;

tip_xy_all     = squeeze(fbgs_shapes(1:2, end, :));   % 2 x N_time
tip_xy         = tip_xy_all(:, idx_align);            % 2 x N_align
tip_xy_centered = (tip_xy - mean(tip_xy, 2))'; %   Transpose to N_align x 2
[~, ~, V_f] = svd(tip_xy_centered, 'econ');

theta_z_fbgs = atan2(V_f(2, 1), V_f(1, 1));

if bending_axis == 'y'
    theta_z_fbgs = pi/2 - theta_z_fbgs;       % map onto y-axis
else
    theta_z_fbgs = 0 - theta_z_fbgs;          % map onto x-axis
end


%%  Realign section

if bending_axis == 'y'
    theta_z = theta_z_fbgs + theta_z_mocap;
else
    theta_z = theta_z_fbgs - theta_z_mocap;
end

R_z = axang2rotm([0 0 1 theta_z]);
for t = 1:N_time_fbgs
    fbgs_shapes(:, :, t) = R_z * fbgs_shapes(:, :, t);
end




%%  Correct pose mocap (only frame of the robot)

%   The per-disk residual-offset correction is a static property of the
%   physical setup (marker mounting), not something that should vary
%   recording to recording. It's computed once, from the dedicated
%   straight/reference recording, by outils/compute_mocap_correction.m,
%   which saves it next to this file. That file must exist -- this does
%   NOT fall back to recomputing an approximate correction per recording.
correction_file = fullfile(fileparts(mfilename('fullpath')), "mocap_correction.csv");

if ~isfile(correction_file)
    error("align_mocap_and_fbgs:missingCorrection", ...
        "Mocap correction file not found: %s\nRun outils/compute_mocap_correction.m first.", correction_file);
end

correction_kinematics = readmatrix(correction_file);   % N_disks_robot x 6, [roll pitch yaw px py pz]

N_disks_robot = 5;
rel_kinematics_disks_corr = zeros(size(rel_kinematics_disks));

for it = 1:N_disks_robot

    R_correction = eul2rotm(correction_kinematics(it, 1:3), 'XYZ');
    r_correction = correction_kinematics(it, 4:6)';

    g_correction = [
        R_correction  r_correction
        0   0   0     1
    ];

    rel_poses_disk = rel_poses_disks(:, :, it, :);
    rel_poses_disk_corr = pagemtimes(rel_poses_disk, g_correction);

    r_disk_corr = squeeze( rel_poses_disk_corr(1:3,   4, :, :) );
    R_disk_corr = squeeze( rel_poses_disk_corr(1:3, 1:3, :, :) );
    XYZ_disk_corr = rotm2eul(R_disk_corr, 'XYZ');

    rel_kinematics_disks_corr(:, :, it) = [
      XYZ_disk_corr   r_disk_corr'
    ];

end

end
