close all;
clear;
clc;

addpath("outils\")

%% ====== PATHS / SETTINGS ======
folder = fullfile("..", "dataCollectionPack/data/references/straight_config/");

FBGS_tip_index = 478;



%% ====== LOAD DATA ======

mk_1_negx = readtable(fullfile(folder, "dataMark10_-x.csv"));
mk_1_x    = readtable(fullfile(folder, "dataMark10_+x.csv"));
mk_2_negy = readtable(fullfile(folder, "dataMark10_-y.csv"));
mk_2_y    = readtable(fullfile(folder, "dataMark10_+y.csv"));

filename = fullfile(folder, "dataOptiTrack.csv");
[N_disks, mocap_timestamps, poses_disks, rel_poses_disks, rel_kinematics_disks] = data_optitrack(filename);


filename = fullfile(folder, "dataFBGS.csv");
[fbgs_time, fbgs_shapes] = data_fbgs(filename);

N_time = length(mocap_timestamps);
xyz_disks = rel_kinematics_disks(:, 4:6, :);
XYZ_disks = rel_kinematics_disks(:, 1:3, :);
xyz_disks_mean = mean(xyz_disks)
xyz_disks_std = std(xyz_disks)
XYZ_disks_mean = mean(XYZ_disks)
XYZ_disks_std = std(XYZ_disks)

