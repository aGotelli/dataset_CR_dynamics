close all
clear 
clc
%%


%   Load the following folders
folders = {'plane_x_slow', 'plane_y_slow', 'plane_xy_slow', 'star_slow', 'circle_slow', 'Lissajous_slow'};

save_path = "figures/";

mkdir(save_path)

for it=1:length(folders)
    


    folder = fullfile("dataCollectionPack","20260304/", folders{it}, "/processed/");

    mocap_frames = load(fullfile(folder , "vicon_frames.csv"));


    tip_position = mocap_frames(:, 33:35);

    name = "Tip " + folders{it};
    f = figure("Name", name);
    plot(tip_position(:, 1), tip_position(:, 2), 'LineWidth', 1);
    grid on
    xlim([-.35 .35])
    ylim([-.35 .35])
    xlabel("p_x [m]")
    ylabel("p_y [m]")

    savefig(save_path + f.Name)
    saveas(f, save_path + f.Name, 'png')

end