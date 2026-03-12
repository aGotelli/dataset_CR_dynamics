close all
clear 
clc
%%


%   Load the following folders
folders = {'plane_x', 'plane_y', 'plane_xy', 'star', 'circle', 'Lissajous'};

save_path = "figures/";

mkdir(save_path)

for it=1:length(folders)
    

    traj = {'_slow', '_fast'};
    col = {'b', 'r'};

    name = "Tip " + folders{it};
    f = figure("Name", name);
    for k=1:2

        folder = fullfile("dataCollectionPack","20260304/", string(folders{it}) + string(traj{k}), "/processed/");
    
        mocap_frames = load(fullfile(folder , "vicon_frames.csv"));
    
    
        tip_position = mocap_frames(:, 33:35);
    
        
        
        plot(tip_position(:, 1), tip_position(:, 2), 'LineWidth', 1, 'Color', col{k});
        hold on
        grid on
        xlim([-.35 .35])
        ylim([-.35 .35])
        xlabel("p_x [m]")
        ylabel("p_y [m]")
    
        savefig(save_path + f.Name)
        saveas(f, save_path + f.Name, 'png')

    end

end