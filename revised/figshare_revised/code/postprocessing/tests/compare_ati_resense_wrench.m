%% compare_ati_resense_wrench.m
%
% Compares the ATI base force/torque sensor against the Resense contact
% wand, for the contact_motion/push_retract recording. The Resense
% wand's measured wrench is transported from its own sensor frame to the
% robot base frame (via its mocap pose and a fixed sensor-to-mocap
% offset, Ad_g) and plotted against the ATI sensor's own base-frame
% reading, plus a couple of geometric sanity checks on the wand's
% tracked position.
%
% This script does not reprocess any raw sensor data itself: it reads
% base_wrench.csv, wrench_wand.csv and mocap_frames.csv straight from
% the recording's processed/ folder, i.e. exactly what process_data.m
% already produced for this recording (same filtering, resampling and
% mocap correction as the rest of the pipeline). It only performs the
% Ad_g transport locally, which process_data.m does not do -- that
% comparison is diagnostic, not something the released dataset needs.
% Nothing is written to disk; this script only plots and prints.
%
% process_data.m must have been run for this recording first (with
% folder set to contact_motion/push_retract) so that its processed/
% folder contains base_wrench.csv, wrench_wand.csv and mocap_frames.csv.
%
% Run this script directly; it locates the dataset relative to its own
% file location, so MATLAB's current folder does not matter.

close all;
clear;
clc;

%% ====== PATHS / SETTINGS ======

% Located from this script's own file location rather than a path
% relative to MATLAB's current folder: this script lives inside
% code/postprocessing/tests/, and data/ is code/'s sibling folder, so
% climb up to code/ and step across into data/.
this_script_folder = fileparts(mfilename('fullpath'));
code_folder = fileparts(fileparts(this_script_folder));
data_root = fullfile(fileparts(code_folder), "data");

folder = fullfile(data_root, "contact_motion", "push_retract");
processed_folder = fullfile(folder, "processed");

% Number of tracked OptiTrack disks for this recording: the 5 robot
% disks plus the Resense wand (disk index 6, 1-based) -- see
% outils/data_optitrack.m.
N_disks = 6;
wand_disk_index = 6;
tip_disk_index = 5;

saving_fig_folder = fullfile(this_script_folder, "figures") + filesep;
if ~isfolder(saving_fig_folder)
    mkdir(saving_fig_folder);
end


%% ====== LOAD process_data.m's SAVED OUTPUT FOR THIS RECORDING ======

required_files = ["base_wrench.csv", "wrench_wand.csv", "mocap_frames.csv"];
for f = required_files
    if ~isfile(fullfile(processed_folder, f))
        error(['Missing %s in:\n%s\n' ...
            'Run process_data.m for this recording first (set its ' ...
            'folder to contact_motion/push_retract).'], f, char(processed_folder));
    end
end

base_wrench_csv = readmatrix(fullfile(processed_folder, "base_wrench.csv"));
sampling_time      = base_wrench_csv(:, 1);
interp_base_wrench = base_wrench_csv(:, 2:end);
N_samples = numel(sampling_time);

wrench_wand_csv    = readmatrix(fullfile(processed_folder, "wrench_wand.csv"));
interp_wrench_wand = wrench_wand_csv(:, 2:end);

mocap_csv = readmatrix(fullfile(processed_folder, "mocap_frames.csv"));
interp_rel_kinematics_disks_corr = reshape(mocap_csv(:, 2:end), [N_samples, 6, N_disks]);

% The wand has no residual-offset correction of its own (see
% align_mocap_and_fbgs.m); this is its raw mocap pose, carried through
% unchanged into mocap_frames.csv.
wand_pose = interp_rel_kinematics_disks_corr(:, :, wand_disk_index);


%% ====== TRANSPORT THE RESENSE WRENCH TO THE ROBOT BASE FRAME (Ad_g) ======

wrench_at_base = compute_wrench_at_base(wand_pose, interp_wrench_wand);


%% ====== PRINT A QUICK NUMERICAL SUMMARY ======

wrench_labels = ["Fx", "Fy", "Fz", "Tx", "Ty", "Tz"];
wrench_units  = ["N", "N", "N", "Nm", "Nm", "Nm"];
rmse_wrench = rmse(wrench_at_base', interp_base_wrench);

fprintf("ATI vs Ad_g-transported Resense, RMSE over %d samples:\n", N_samples);
for it = 1:6
    fprintf("  %s = %.4f %s\n", wrench_labels(it), rmse_wrench(it), wrench_units(it));
end


%% ====== PLOTS ======

% ATI vs Resense-transported-to-base (Ad_g), all 6 wrench components.
fig = figure("Name", "Forces (Ad_g)");
force_labels = {'F_x [N]', 'F_y [N]', 'F_z [N]'};
for it = 1:3
    subplot(3, 1, it)
    plot(sampling_time, interp_base_wrench(:, it), 'b')
    hold on
    plot(sampling_time, wrench_at_base(it, :), 'r')
    ylabel(force_labels{it})
    grid on
    if it == 3, xlabel("Time [s]"); end
end
legend('ATI', 'Ad_g Resense')
savefig(saving_fig_folder + fig.Name)
saveas(fig, saving_fig_folder + fig.Name, 'png')

fig = figure("Name", "Torques (Ad_g)");
torque_labels = {'T_x [Nm]', 'T_y [Nm]', 'T_z [Nm]'};
for it = 1:3
    subplot(3, 1, it)
    plot(sampling_time, interp_base_wrench(:, 3 + it), 'b')
    hold on
    plot(sampling_time, wrench_at_base(3 + it, :), 'r')
    ylabel(torque_labels{it})
    grid on
    if it == 3, xlabel("Time [s]"); end
end
legend('ATI', 'Ad_g Resense')
savefig(saving_fig_folder + fig.Name)
saveas(fig, saving_fig_folder + fig.Name, 'png')


% Robot tip (disk 5) vs the wand sensor's transported position, as a
% geometric sanity check on the Ad_g offset (g_fix, inside
% compute_wrench_at_base below).
pos_sensor = wand_sensor_position(wand_pose);

fig = figure("Name", "Position disk and wand");
pos_labels = {'p_x [m]', 'p_y [m]', 'p_z [m]'};
for it = 1:3
    subplot(3, 1, it)
    plot(sampling_time, interp_rel_kinematics_disks_corr(:, 3 + it, tip_disk_index), 'b', 'LineWidth', 1)
    hold on
    plot(sampling_time, pos_sensor(it, :), 'r', 'LineWidth', 1)
    ylabel(pos_labels{it})
    grid on
    if it == 3, xlabel("Time [s]"); end
end
legend('robot tip', 'wand')
savefig(saving_fig_folder + fig.Name)
saveas(fig, saving_fig_folder + fig.Name, 'png')


% In-plane (xy) distance between the robot tip and the wand sensor: how
% far the contact wand strays from the tip it's pushing/retracting on.
disk_pos = squeeze(interp_rel_kinematics_disks_corr(:, 4:6, tip_disk_index));
diff_xy = disk_pos(:, 1:2) - pos_sensor(1:2, :)';
dist_xy = sqrt(diff_xy(:, 1).^2 + diff_xy(:, 2).^2);

fig = figure("Name", "Distance sensor from disk");
plot(sampling_time, dist_xy, 'b', 'LineWidth', 1)
ylabel('Distance [m]')
grid on
xlabel("Time [s]")
savefig(saving_fig_folder + fig.Name)
saveas(fig, saving_fig_folder + fig.Name, 'png')


%% ====== LOCAL FUNCTIONS ======

function wrench_at_base = compute_wrench_at_base(disk_kinematics_wand, wrench_wand)
    %   Transports the Resense HEX12 wand wrench from its own sensor
    %   frame to the robot base frame, via the wand's mocap pose and the
    %   wand's fixed sensor-to-mocap-frame offset (g_fix).
    %
    %   disk_kinematics_wand : N_samples x 6 [roll pitch yaw px py pz],
    %                          the wand's own mocap pose over time
    %   wrench_wand           : N_samples x 6 [Fx Fy Fz Tx Ty Tz], the
    %                          wand's own measured wrench over time
    %   wrench_at_base         : 6 x N_samples

    N_samples = size(disk_kinematics_wand, 1);
    g_fix = wand_sensor_offset();

    wrench_at_base = zeros(6, N_samples);
    for it_t = 1:N_samples
        wand_XYZ_xyz = disk_kinematics_wand(it_t, :);

        R = eul2rotm(wand_XYZ_xyz(1:3), 'XYZ');
        r = wand_XYZ_xyz(4:6)';

        g = [
          R     r
          0 0 0 1
        ];

        g_s = g*g_fix;
        R_s = g_s(1:3, 1:3);
        r_s = g_s(1:3, 4);
        wrench_wand_it_t = wrench_wand(it_t, :)';

        Ad_g_=[R_s zeros(3,3)
                hat_(r_s)*R_s R_s];

        %   Equivalent wrench at the base, by the action-reaction principle.
        wrench_at_base(:, it_t) = -Ad_g_*wrench_wand_it_t;
    end
end

function pos_sensor = wand_sensor_position(disk_kinematics_wand)
    %   Position (3 x N_samples) of the Resense sensor frame's origin,
    %   i.e. the wand's mocap pose shifted by the same fixed offset
    %   (g_fix) compute_wrench_at_base uses for the wrench transport.

    N_samples = size(disk_kinematics_wand, 1);
    g_fix = wand_sensor_offset();

    pos_sensor = zeros(3, N_samples);
    for it_t = 1:N_samples
        wand_XYZ_xyz = disk_kinematics_wand(it_t, :);

        R = eul2rotm(wand_XYZ_xyz(1:3), 'XYZ');
        r = wand_XYZ_xyz(4:6)';

        g = [
          R     r
          0 0 0 1
        ];

        g_s = g*g_fix;
        pos_sensor(:, it_t) = g_s(1:3, 4);
    end
end

function g_fix = wand_sensor_offset()
    %   Fixed rigid-body offset between the wand's mocap frame and its
    %   Resense sensor frame.

    R_fix_x = axang2rotm([1 0 0 pi/2]);
    R_fix_z = axang2rotm([0 0 1 pi/6]);
    R_fix = R_fix_x*R_fix_z;
    r_fix = [
        0
       -0.1137
        0
    ];
    g_fix = [
            R_fix r_fix
            0 0 0   1
        ];
end

function [A] = hat_(x)
    %   Skew-symmetric cross-product matrix of a 3-vector x, such that
    %   hat_(x)*v == cross(x, v).
    A=zeros(3,3);

    A(1,2)=-x(3);
    A(1,3)=x(2);
    A(2,3)=-x(1);

    A(2,1)=x(3);
    A(3,1)=-x(2);
    A(3,2)=x(1);
end
