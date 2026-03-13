function [time_fbgs, fbgs_shapes] = data_fbgs(filename)

    number_of_grating = 26;
    greating_index = 0:1:(number_of_grating-1);


    fbgs_raw_data = readtable(filename);
    % fbgs_raw_data = readtable("dataCollectionPack\20260225\plane_x_slow\dataFBGS.csv");


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

    % NOTE:
    %   This loader returns FBGS shapes in the raw sensor frame.
    %   Any frame re-orientation/alignment should be applied by the caller
    %   (e.g., in process_data.m) to avoid redundant rotations.

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

end
