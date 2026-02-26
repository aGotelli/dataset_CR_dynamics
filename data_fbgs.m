% function [time_fbgs, fbgs_shapes] = data_fbgs(filename)
    
    filename = "dataFBGS.csv";


    number_of_grating = 26;
    greating_index = 0:1:(number_of_grating-1);


    % fbgs_raw_data = readtable("dataCollectionPack\planar motion\plane_x_-y_angle_90_speed_2\dataFBGS_.csv");
    fbgs_raw_data = readtable(filename);


    time_fbgs = fbgs_raw_data.Timestamp;

    N_time_fbgs = length(time_fbgs);

    %%   Extract curvature and angles

    curvatures = zeros(N_time_fbgs, number_of_grating);
    angles = zeros(N_time_fbgs, number_of_grating);

    for it=1:number_of_grating
        

        column = ['Curvature_' char( int2str(greating_index(it)) )];
        curvatures(:, it) = fbgs_raw_data.( column );


        column = ['Angle_' char( int2str(greating_index(it)) )];
        angles(:, it) = fbgs_raw_data.( column );

    end



    %%  Extract recorded positions

    position_samples = 502; %   for x, y and z

    %   Shapes seem to be saved like:
    %   time x_0 y_0 z_0 x_1 y_1 z_1 ... ... ... x_n y_n z_n

    % shape_columns = 3*502;

    fbgs_shapes = zeros(3, position_samples, N_time_fbgs);

    index = 0;

    for sample=1:position_samples

        for coord=1:3 % for x y and z 

            

            column = ['Shape_' char(int2str( index ))];

            %   Update index
            index = index + 1;

            %   Extract column
            positions = fbgs_raw_data.( column );
        
            %   From mm to meters
            position_m = positions/1000;
        
            %   Convert iter in index data matrix
            fbgs_shapes(coord, sample, :) = reshape(position_m, [1, 1, N_time_fbgs]);

        end

        

    end

% end


%%  Sanity check, plot the motion


figure("Name", "Tip FBGS")
plot(time_fbgs, squeeze( fbgs_shapes(1, end, :) ), 'r')
hold on
plot(time_fbgs, squeeze( fbgs_shapes(2, end, :) ), 'g')
plot(time_fbgs, squeeze( fbgs_shapes(3, end, :) ), 'b')
grid on
xlabel("Time")
ylabel("Positions")

video_FBGS= true;
video_FBGS= false;
if video_FBGS

    figure("Name", 'FBGS Motion')

    for it_t=1:N_time_fbgs
        
        xyz = fbgs_shapes(:, :, it_t);

        plot3(xyz(1, :), xyz(2, :), xyz(3, :), 'b', 'LineWidth', 1)
        
        grid on
        xlim([-0.3, 0.3])
        ylim([-0.3, 0.3])
        zlim([-0.3, 0.3])
        title("Time " + num2str(time_fbgs(it_t)))
        drawnow
        

    end

end
