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
%     %%  Curvature and strain
% 
%     index = 100;
% 
%     data = fbgs_raw_data.Variables;
%     curvatures = 1.5*(180*data(index, 2:27)/pi);
%     angles = (pi*data(index, 28:53)/180);
% 
%     xyz = fbgs_shapes(:, :, index);
% 
%     figure("Name", "Shape and Curvature")
%     subplot(3, 1, 1)
%     plot3(xyz(1, :), xyz(2, :), xyz(3, :), 'b', 'LineWidth', 2)
%     grid on
%     xlim([-0.3, 0.3])
%     ylim([-0.3, 0.3])
%     zlim([-0.3, 0.3])
% 
%     subplot(3, 1, 2)
%     plot(angles, 'r', 'LineWidth', 2)
%     grid on
% 
%     subplot(3, 1, 3)
%     plot(curvatures, 'r', 'LineWidth', 2)
%     grid on
% 
% 
%     T = [
%         0*angles
%         cos(angles)
%         sin(angles)
%     ];
% 
%     K = curvatures.*T;
% 
%     Gamma = repmat([1; 0; 0], [1 length(angles)]);
% 
%     strain_rod = [
%         K
%         Gamma
%     ];
% 
% 
%     figure("Name", "Strain and Shape")
% 
% 
%     subplot(2, 1, 2)
%     plot(K(1, :), 'r', 'LineWidth', 2)
%     hold on
%     plot(K(2, :), 'g', 'LineWidth', 2)
%     plot(K(3, :), 'b', 'LineWidth', 2)
%     grid on
% 
%     %  Polyfit to get the correct value
% 
%     %   Arc length parameter for the 26 gratings along the rod length
%     L_FBGS = 0.5;
%     X_gratings = linspace(0, L_FBGS, number_of_grating);
% 
%     %   Degree of the polynomial fit
%     poly_degree = 5;
% 
%     %   Fit each curvature component as a function of arc length X
%     p_K1 = polyfit(X_gratings, K(1, :), poly_degree);
%     p_K2 = polyfit(X_gratings, K(2, :), poly_degree);
%     p_K3 = polyfit(X_gratings, K(3, :), poly_degree);
% 
%     %   Evaluate fitted curvature on a fine grid for visualization
%     X_fine = linspace(0, L_FBGS, 200);
%     K1_fit = polyval(p_K1, X_fine);
%     K2_fit = polyval(p_K2, X_fine);
%     K3_fit = polyval(p_K3, X_fine);
% 
%     subplot(2, 1, 1)
%     plot(X_gratings, K(1, :), 'r.', 'MarkerSize', 10); hold on
%     plot(X_gratings, K(2, :), 'g.', 'MarkerSize', 10)
%     plot(X_gratings, K(3, :), 'b.', 'MarkerSize', 10)
%     plot(X_fine, K1_fit, 'r-', 'LineWidth', 2)
%     plot(X_fine, K2_fit, 'g-', 'LineWidth', 2)
%     plot(X_fine, K3_fit, 'b-', 'LineWidth', 2)
%     legend('K1 data','K2 data','K3 data','K1 fit','K2 fit','K3 fit')
%     xlabel('Arc length X [m]')
%     ylabel('Curvature [1/m]')
%     grid on
%     title('Curvature polyfit')
% 
%     %  Integrate rod kinematics via ode45
% 
%     %   Initial conditions:
%     %   - position p(0) = [0; 0; 0]
%     %   - orientation R(0) = I_3x3 stored as 9-vector (column-major)
%     R0 = eye(3);
%     p0 = [0; 0; 0];
%     y0 = [p0; R0(:)];   % 12-element state vector
% 
%     %   ODE: Cosserat rod kinematics
%     %   dp/dX = R * Gamma,  where Gamma = [1; 0; 0] (inextensible, no shear)
%     %   dR/dX = R * hat(K(X))
%     ode_fun = @(X, y) rod_kinematics(X, y, p_K1, p_K2, p_K3);
% 
%     [X_ode, Y_ode] = ode45(ode_fun, [0, L_FBGS], y0);
% 
%     %   Extract reconstructed shape
%     p_reconstructed = Y_ode(:, 1:3)';   % 3 x N_steps
% 
%     %   Plot reconstructed shape vs FBGS measured shape
%     figure("Name", "Reconstructed Shape vs FBGS")
%     xyz = fbgs_shapes(:, :, 1);
%     plot3(xyz(1, :), xyz(2, :), xyz(3, :), 'b', 'LineWidth', 2)
%     hold on
%     plot3(p_reconstructed(1, :), p_reconstructed(2, :), p_reconstructed(3, :), 'r--', 'LineWidth', 2)
%     legend('FBGS measured', 'ODE45 reconstructed')
%     xlabel('X [m]'); ylabel('Y [m]'); zlabel('Z [m]')
%     grid on
%     axis equal
%     title('Rod shape reconstruction from curvature polyfit')
% 
% 
% 
% 
% 
% 
% 
% 
% 
% %% Local function: Cosserat rod kinematic ODE
% function dydt = rod_kinematics(X, y, p_K1, p_K2, p_K3)
% 
%     %   Unpack state
%     R = reshape(y(4:12), [3, 3]);
% 
%     %   Evaluate curvature at current arc length
%     K = [polyval(p_K1, X);
%          polyval(p_K2, X);
%          polyval(p_K3, X)];
% 
%     %   Shear/extension strain: inextensible, unsheared rod
%     Gamma = [1; 0; 0];
% 
%     %   Hat map (skew-symmetric matrix) of K
%     K_hat = [  0,    -K(3),  K(2);
%               K(3),   0,    -K(1);
%              -K(2),  K(1),   0   ];
% 
%     %   Kinematics
%     dp = R * Gamma;
%     dR = R * K_hat;
% 
%     dydt = [dp; dR(:)];
% 
% end
% 
%     % subplot(3, 1, 1)
%     % plot3(xyz(1, :), xyz(2, :), xyz(3, :), 'b', 'LineWidth', 2)
%     % grid on
%     % xlim([-0.3, 0.3])
%     % ylim([-0.3, 0.3])
%     % zlim([-0.3, 0.3])
% 
% 
% % end
% 
% 
% % %%  Sanity check, plot the motion
% 
% % figure("Name", "Tip FBGS")
% % plot(time_fbgs, squeeze( shapes(1, end, :) ), 'r')
% % hold on
% % plot(time_fbgs, squeeze( shapes(2, end, :) ), 'g')
% % plot(time_fbgs, squeeze( shapes(3, end, :) ), 'b')
% % grid on
% % xlabel("Time")
% % ylabel("Positions")
% 
% 
% 
% % %%  Now use the curvature directly
% 
% % curvature_samples = size(angles, 2);
% 
% % L_FBGS = 0.5
% % X_gratings = linspace(0, L_FBGS, curvature_samples);
% 
% % strain_rod = zeros(6, curvature_samples, N_time_fbgs);
% % for it_t=1:N_time_fbgs
% 
% %     %   Extract curvature and angle for that timestep
% %     angle_X = angles(it_t, :);
% %     curvature_X = curvatures(it_t, :);
% 
% %     T = [
% %         0*angle_X
% %         cos(angle_X)
% %         sin(angle_X)
% %     ];
% 
% %     K = curvature_X.*T;
% 
% %     Gamma = repmat([1; 0; 0], [1 length(angle_X)]);
% 
% %     strain_rod(:, :, it_t) = [
% %         K
% %         Gamma
% %     ];
% 
% % end
% 
% % %%  Find corresponding set of generalized coordinates
% 
% 
% % addpath("..\data_comparison\Dyn_Essai_release_Beam_Andrea\")
% 
% 
% 
% 
% % %  Defining required variables
% % % K1, K2, K3, ...
% % Config.V_a = [ 0, 1, 1, 0, 0, 0];
% 
% % %   Defining size of the basis
% % Const.dim_base_k = [0, 3, 3, 0, 0, 0];
% 
% % %   Resulting dimension q
% % Const.dim_base   = Config.V_a*Const.dim_base_k';
% 
% % %   Lenght of the FBGS
% % Config.L = L_FBGS;
% 
% % K = squeeze( strain_rod(1:3, :, 100) );
% 
% 
% % q = fsolve(@(q) findCoords(q, K, X_gratings, Const, Config), zeros(Const.dim_base, 1));
% 
% % K_q = zeros(3, curvature_samples);
% % for it_x=1:length(X_gratings)
% 
% %         X = X_gratings(it_x);
% 
% 
% %         Phi = Base_Phi(X, 0, Const, Config)';
% 
% 
% %         K_q(2:3, it_x) = Phi*q;
% 
% 
% % end
% 
% 
% % figure("Name", "Curvature")
% % subplot(2, 1, 1)
% % plot(X_gratings, K(2, :), 'b')
% % hold on 
% % plot(X_gratings, K_q(2, :), 'r')
% % xlabel("X [m]")
% % ylabel("Curvature")
% 
% % subplot(2, 1, 2)
% % plot(X_gratings, K(3, :), 'b')
% % hold on 
% % plot(X_gratings, K_q(3, :), 'r')
% % xlabel("X [m]")
% % ylabel("Curvature")
% % legend('Measured', 'Reconstructed')
% 
% 
% 
% % function error = findCoords(q, K, X_gratings, Const, Config)
% 
% %     curvature_samples = length(X_gratings);
% 
% %     K_q = zeros(3, curvature_samples);
% 
% %     for it_x=1:length(X_gratings)
% 
% %         X = X_gratings(it_x);
% 
% 
% %         Phi = Base_Phi(X, 0, Const, Config)';
% 
% 
% %         K_q(2:3, it_x) = Phi*q;
% 
% 
% %     end
% 
% %     error = K_q - K;
% 
% 
% % end