function [time_fbgs, fbgs_shapes, curvatures, angles] = data_fbgs(filename)

    %   Parameters of the FBG fiber installed on the robot
    number_of_grating = 26;
    greating_index = 0:1:(number_of_grating-1);

    %   Load the data
    fbgs_raw_data = readtable(filename);

    %   Extract timestamps and count number
    time_fbgs = fbgs_raw_data.Timestamp;
    N_time_fbgs = length(time_fbgs);

    %   Extract curvature and angles
    curvatures = zeros(N_time_fbgs, number_of_grating);
    angles = zeros(N_time_fbgs, number_of_grating);
    for it=1:number_of_grating
        

        column = ['Curvature_' char( int2str(greating_index(it)) )];
        curvature = fbgs_raw_data.( column );

        %   Curvature is in 1/mm -> convert to 1/m
        curvatures(:, it) = curvature*1000;
        
        %   Angle is already in radians
        column = ['Angle_' char( int2str(greating_index(it)) )];
        angles(:, it) = fbgs_raw_data.( column );

    end



    %  Extract recorded positions
    position_samples = 502; %   for x, y and z

    %   Shapes seem to be saved like:
    %   time x_0 y_0 z_0 x_1 y_1 z_1 ... ... ... x_n y_n z_n
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

end
