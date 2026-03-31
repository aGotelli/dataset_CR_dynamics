close all;
clear;
clc;

%% ====== PATHS / SETTINGS ======
folder = fullfile("dataCollectionPack","20260310/","static_test/");



%% ====== LOAD DATA ======
% motor = readtable(fullfile(folder, "dataMotor.csv"));

mk_1_negx = readtable(fullfile(folder, "dataMark10_-x.csv"));
mk_1_x    = readtable(fullfile(folder, "dataMark10_+x.csv"));
mk_2_negy = readtable(fullfile(folder, "dataMark10_-y.csv"));
mk_2_y    = readtable(fullfile(folder, "dataMark10_+y.csv"));

ati = readtable(fullfile(folder, "dataATIFT.csv"));

filename = fullfile(folder, "dataOptiTrack.csv");
[N_disks, mocap_timestamps, poses_disks, rel_poses_disks, rel_kinematics_disks] = data_optitrack(filename);


filename = fullfile(folder, "dataFBGS.csv");
[fbgs_time, fbgs_shapes] = data_fbgs(filename);


tATI = ati.timestamp - ati.timestamp(1);
ATI_FT = [ati.Fx_N_, ati.Fy_N_, ati.Fz_N_ ati.Tx_Nm_, ati.Ty_Nm_, ati.Tz_Nm_];


ATI_FT_mean = mean(ATI_FT);
ATI_FT_std = std(ATI_FT);



time_cables{1} = mk_1_x.timestamp - mk_1_x.timestamp(1);          cable_tensions{1} = mk_1_x.tension_N_;
time_cables{2} = mk_2_y.timestamp - mk_2_y.timestamp(1);          cable_tensions{2} = mk_2_y.tension_N_;
time_cables{3} = mk_1_negx.timestamp - mk_1_negx.timestamp(1);    cable_tensions{3} = mk_1_negx.tension_N_;
time_cables{4} = mk_2_negy.timestamp - mk_2_negy.timestamp(1);    cable_tensions{4} = mk_2_negy.tension_N_;




figure("Name", "Cable Tensions")
for it=1:4
    subplot(4, 1, it)
    plot(time_cables{it}, cable_tensions{it})
    grid on
    ylabel("Tension [N]")

    if it==4
        xlabel("Time [s]")
    end
end


%   Compute the mean and std
for it=1:4

    cable_tensions_mean{it} = mean(cable_tensions{it});
    cable_tensions_std{it} = std(cable_tensions{it});
end


%%  For the mocap obtain the relative poses

rel_kinematics_disks_mean = mean(rel_kinematics_disks);
rel_kinematics_disks_std = std(rel_kinematics_disks);

%   Compose the poses
r = rel_kinematics_disks_mean;
R = eul2rotm(rel_kinematics_disks_std)