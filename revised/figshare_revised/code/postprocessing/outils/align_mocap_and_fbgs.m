function [mocap_timestamps, rel_kinematics_disks, rel_kinematics_disks_corr, ...
    fbgs_time, fbgs_shapes, fbgs_curvatures, fbgs_angles] = ...
    align_mocap_and_fbgs(folder, has_fbgs_data, align_window_s, data_root)
%ALIGN_MOCAP_AND_FBGS Load one recording's OptiTrack and (if present) FBG
%   data and put them in a common, spatially-aligned frame.
%
%   Only the robot's 5 tracked disks are handled here. The Resense contact
%   wand (a 6th OptiTrack rigid body, present only when the Resense sensor
%   is active) is loaded separately, by process_data.m calling
%   data_optitrack a second time -- it has no per-disk correction defined
%   for it and does not participate in the FBG alignment below, so
%   threading it through this function would only add a special case.
%
%   Inputs:
%     folder          - path to one recording's folder (containing
%                        dataOptiTrack.csv, and dataFBGS.csv when
%                        has_fbgs_data)
%     has_fbgs_data   - whether this recording has a dataFBGS.csv at all
%                        (some FT-sensor-only recordings, e.g.
%                        contact_motion/touching_base, do not). When
%                        false, the entire FBG load/alignment below is
%                        skipped -- it does not affect the mocap load or
%                        per-disk correction, which do not depend on FBG
%                        data being present.
%     align_window_s  - length, in seconds, of the initial portion of the
%                        recording used to determine the bending-plane
%                        rotation (both mocap and FBG use their own first
%                        align_window_s seconds)
%     data_root        - path to the dataset's data/ folder; used to
%                        locate data/postprocess_calibration/mocap_correction.csv
%
%   Outputs:
%     mocap_timestamps            - OptiTrack timestamps (unchanged)
%     rel_kinematics_disks        - mocap disk poses, RAW (no per-disk
%                                    residual-offset correction), for the
%                                    5 robot disks
%     rel_kinematics_disks_corr   - mocap disk poses, WITH the per-disk
%                                    residual-offset correction applied,
%                                    for the 5 robot disks
%     fbgs_time                   - FBG timestamps, UNCORRECTED (see
%                                    above -- no pipeline-delay shift).
%                                    Empty when has_fbgs_data is false.
%     fbgs_shapes                 - FBG reconstructed shapes, rotated
%                                    into the robot body frame and
%                                    aligned to the mocap bending plane.
%                                    Sized 3 x 0 x 0 when has_fbgs_data is
%                                    false, so that N_fbgs_points (=
%                                    size(fbgs_shapes, 2) downstream)
%                                    comes out 0 without special-casing.
%     fbgs_curvatures, fbgs_angles - passed straight through from
%                                    data_fbgs, unchanged. Empty when
%                                    has_fbgs_data is false.



%   Bending plane: set to 'x' or 'y' — the axis along which the rod bends
if(contains(folder, "_y_"))
    bending_axis = 'y';        % 'y' for plane_y experiments
else
    bending_axis = 'x';        % 'x' for all rest
end


%%  Mocap section

%   Extract data. Always requests the 5-robot-disk layout (use_resense =
%   false) -- the Resense wand, when present, is loaded separately by
%   process_data.m (see this function's header comment).
filename = fullfile(folder, "dataOptiTrack.csv");
[~, mocap_timestamps, ~, rel_poses_disks, rel_kinematics_disks] = data_optitrack(filename, false);



%   Correct pose mocap (only frame of the robot)

%   Loads the per-disk residual-offset correction computed and saved by
%   outils/compute_mocap_correction.m, and applies it to the 5 robot
%   disks.
correction_file = fullfile(data_root, "postprocess_calibration", "mocap_correction.csv");

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


%%  FBGS section

%   Everything below (mocap bending-plane angle, FBG load, and the
%   FBG-to-mocap realignment) exists purely to align the FBG shape to
%   mocap's frame, so all of it is skipped when this recording has no
%   FBG data
if has_fbgs_data

    %   Temporal variable used to defined align window
    mocap_time_rel  = mocap_timestamps - mocap_timestamps(1);
    idx_align      = mocap_time_rel <= align_window_s;

    %   Extract kinematics tip disk which present the most ample motion
    XYZ_xyz_tip_disk = rel_kinematics_disks_corr(:, :, 5);
    tip_xy_mocap  = XYZ_xyz_tip_disk(idx_align, 4:5);

    %   Center to compute the plane of motion
    tip_xy_mocap_centered = tip_xy_mocap - mean(tip_xy_mocap, 1);
    [~, ~, V_m] = svd(tip_xy_mocap_centered, 'econ');

    %   Flip axis to avoid pi ambiguity
    if bending_axis == 'y'
        ref_axis = [0; 1];
    else
        ref_axis = [1; 0];
    end
    if dot(V_m(:, 1), ref_axis) < 0
        V_m(:, 1) = -V_m(:, 1);
    end

    %   Angle of the principal (max-variance) direction w.r.t. the x-axis.
    %   No per-axis remapping: theta_z below only needs the difference
    %   with theta_z_fbgs (see Realign section), independent of axis.
    theta_z_mocap = atan2(V_m(2, 1), V_m(1, 1));

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

    %   Flip axis to avoid pi ambiguity
    if dot(V_f(:, 1), ref_axis) < 0
        V_f(:, 1) = -V_f(:, 1);
    end

    %   Angle of the principal (max-variance) direction w.r.t. the x-axis.
    theta_z_fbgs = atan2(V_f(2, 1), V_f(1, 1));


    %%  Realign section

    %   Rotate the FBG principal direction onto the mocap one; no
    %   bending_axis case needed (the previous pi/2-remap-then-add/
    %   subtract form for 'y' was buggy: it only gave zero correction
    %   when both angles equaled exactly pi/2, not whenever they agree).
    theta_z = theta_z_mocap - theta_z_fbgs;

    R_z = axang2rotm([0 0 1 theta_z]);
    for t = 1:N_time_fbgs
        fbgs_shapes(:, :, t) = R_z * fbgs_shapes(:, :, t);
    end

else
    %   No FBG data for this recording: empty placeholders. fbgs_shapes
    %   is 3 x 0 x 0 so that N_fbgs_points (= size(fbgs_shapes, 2)
    %   downstream, in process_data.m and technical_validation.m) comes
    %   out 0 without any special-casing there.
    fbgs_time = [];
    fbgs_shapes = zeros(3, 0, 0);
    fbgs_curvatures = [];
    fbgs_angles = [];
end




end
