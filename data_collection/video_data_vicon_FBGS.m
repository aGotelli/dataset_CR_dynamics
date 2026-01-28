close all;
clear;
clc;

%% ====== PATHS / SETTINGS ======
folder = "dataCollectionPack\20260127\plane_y_angle_150_speed_1\";



N_frames_static_begin = 10; %how many frames to use to compute relative pose for Vicon
filename = folder + "dataVicon.csv";
[N_disks, timestamp_vicon, rel_kinematics_disks] = data_vicon(filename, N_frames_static_begin);

filename = folder + "dataFBGS.csv";
[fbgs_time, fbgs_shapes] = data_fbgs(filename);


%%  Loop
N_vicon = length(timestamp_vicon);
N_FBGS = length(fbgs_time);
N = min(N_vicon, N_FBGS);

figure("Name", "Video Shape")
for it_t=1:N

    time = timestamp_vicon(it_t) - timestamp_vicon(1);

    %   FBGS is growing on x we need to shift into evolving along the
    %   negative z axis
    R = axang2rotm([0 1 0 pi/2]);

    xyz_FBGS = fbgs_shapes(:, :, it_t);


    xyz_Vicon = squeeze( rel_kinematics_disks(it_t, 1:3, :) );

    plot3(xyz_FBGS(1, :), xyz_FBGS(2, :), xyz_FBGS(3, :), 'b');
    hold on
    plot3(xyz_Vicon(1, :), xyz_Vicon(2, :), xyz_Vicon(3, :), 'or');
    grid on
    xlim([-.5, .5])
    ylim([-.5, .5])
    zlim([-.5, .5])
    title("Time " + num2str(time))
    drawnow
    hold off

end