function [time_fbgs, fbgs_shapes] = data_fbgs(filename)

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

end


% %%  Sanity check, plot the motion

% figure("Name", "Tip FBGS")
% plot(time_fbgs, squeeze( shapes(1, end, :) ), 'r')
% hold on
% plot(time_fbgs, squeeze( shapes(2, end, :) ), 'g')
% plot(time_fbgs, squeeze( shapes(3, end, :) ), 'b')
% grid on
% xlabel("Time")
% ylabel("Positions")

% video_FBGS= false;
% if video_FBGS

%     figure("Name", 'FBGS Motion')

%     for it_t=1:N_time_fbgs
        
%         xyz = shapes(:, :, it_t);

%         plot3(xyz(1, :), xyz(2, :), xyz(3, :), 'b', 'LineWidth', 1)
        
%         grid on
%         xlim([-0.3, 0.3])
%         ylim([-0.3, 0.3])
%         zlim([-0.3, 0.3])
%         title("Time " + num2str(time_fbgs(it_t)))
%         drawnow
        

%     end

% end

% %%  Now use the curvature directly

% curvature_samples = size(angles, 2);

% L_FBGS = 0.5
% X_gratings = linspace(0, L_FBGS, curvature_samples);

% strain_rod = zeros(6, curvature_samples, N_time_fbgs);
% for it_t=1:N_time_fbgs
    
%     %   Extract curvature and angle for that timestep
%     angle_X = angles(it_t, :);
%     curvature_X = curvatures(it_t, :);

%     T = [
%         0*angle_X
%         cos(angle_X)
%         sin(angle_X)
%     ];

%     K = curvature_X.*T;

%     Gamma = repmat([1; 0; 0], [1 length(angle_X)]);

%     strain_rod(:, :, it_t) = [
%         K
%         Gamma
%     ];

% end

% %%  Find corresponding set of generalized coordinates


% addpath("..\data_comparison\Dyn_Essai_release_Beam_Andrea\")




% %  Defining required variables
% % K1, K2, K3, ...
% Config.V_a = [ 0, 1, 1, 0, 0, 0];

% %   Defining size of the basis
% Const.dim_base_k = [0, 3, 3, 0, 0, 0];

% %   Resulting dimension q
% Const.dim_base   = Config.V_a*Const.dim_base_k';

% %   Lenght of the FBGS
% Config.L = L_FBGS;

% K = squeeze( strain_rod(1:3, :, 100) );


% q = fsolve(@(q) findCoords(q, K, X_gratings, Const, Config), zeros(Const.dim_base, 1));

% K_q = zeros(3, curvature_samples);
% for it_x=1:length(X_gratings)

%         X = X_gratings(it_x);


%         Phi = Base_Phi(X, 0, Const, Config)';

        
%         K_q(2:3, it_x) = Phi*q;


% end


% figure("Name", "Curvature")
% subplot(2, 1, 1)
% plot(X_gratings, K(2, :), 'b')
% hold on 
% plot(X_gratings, K_q(2, :), 'r')
% xlabel("X [m]")
% ylabel("Curvature")

% subplot(2, 1, 2)
% plot(X_gratings, K(3, :), 'b')
% hold on 
% plot(X_gratings, K_q(3, :), 'r')
% xlabel("X [m]")
% ylabel("Curvature")
% legend('Measured', 'Reconstructed')



% function error = findCoords(q, K, X_gratings, Const, Config)

%     curvature_samples = length(X_gratings);

%     K_q = zeros(3, curvature_samples);
    
%     for it_x=1:length(X_gratings)

%         X = X_gratings(it_x);


%         Phi = Base_Phi(X, 0, Const, Config)';

        
%         K_q(2:3, it_x) = Phi*q;

    
%     end

%     error = K_q - K;


% end