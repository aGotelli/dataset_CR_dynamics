close all; clear; clc;

%% Settings
folder = fullfile("dataCollectionPack","20260304/","Lissajous_slow/");
bending_axis = 'x';

%% Load OptiTrack (dynamic + static)
[N_disks, mocap_t, ~, ~, rel_kin] = data_optitrack(fullfile(folder, "dataOptiTrack.csv"));
[~, ~, ~, rel_poses_static, ~]    = data_optitrack("dataCollectionPack\20260310\static_test\dataOptiTrack.csv");

%% Load dynamic relative poses for correction
[~, ~, ~, rel_poses_dyn, ~] = data_optitrack(fullfile(folder, "dataOptiTrack.csv"));

%% Corrected copy (right-multiply by inv(T_bias) — bias is in local frame)
rel_kin_corr = rel_kin;
for it = 2:N_disks
    T_static = mean(rel_poses_static(:,:,it,:), 4);
    T_ideal  = eye(4);  T_ideal(3,4) = T_static(3,4);
    inv_T_bias = T_static \ T_ideal;
    rel_poses_dyn(:,:,it,:) = pagemtimes(rel_poses_dyn(:,:,it,:), inv_T_bias);
    R_corr = squeeze(rel_poses_dyn(1:3,1:3,it,:));
    r_corr = squeeze(rel_poses_dyn(1:3,4,it,:));
    rel_kin_corr(:,:,it) = [rotm2eul(R_corr,'XYZ')  r_corr'];
end

%% Load & rotate FBGS (same logic as process_data.m)
[fbgs_time, fbgs_shapes] = data_fbgs(fullfile(folder, "dataFBGS.csv"));

R_y = axang2rotm([0 1 0 pi/2]);
for t = 1:size(fbgs_shapes,3)
    fbgs_shapes(:,:,t) = R_y * fbgs_shapes(:,:,t);
end

% SVD alignment
fbgs_time_rel = fbgs_time - fbgs_time(1);
tip_xy = squeeze(fbgs_shapes(1:2, end, fbgs_time_rel <= 10));
tip_xy_c = tip_xy - mean(tip_xy,2);
[U,~,~] = svd(tip_xy_c,'econ');
alpha = atan2(U(2,1), U(1,1));
if strcmpi(bending_axis,'y'), theta_z = pi/2 - alpha; else, theta_z = -alpha; end
R_z = [cos(theta_z) -sin(theta_z) 0; sin(theta_z) cos(theta_z) 0; 0 0 1];
for t = 1:size(fbgs_shapes,3)
    fbgs_shapes(:,:,t) = R_z * fbgs_shapes(:,:,t);
end
if strcmpi(bending_axis,'y'), fbgs_shapes(1,:,:) = -fbgs_shapes(1,:,:);
else,                         fbgs_shapes(2,:,:) = -fbgs_shapes(2,:,:); end

%% Extract tips
tip_uncorr = rel_kin(:, 4:6, N_disks);        % N_time x 3
tip_corr   = rel_kin_corr(:, 4:6, N_disks);   % N_time x 3
tip_fbgs   = squeeze(fbgs_shapes(:, 480, :))'; % N_time x 3

%% Plot
vars = {'p_x','p_y','p_z'};
figure("Name","Tip Comparison: Uncorrected vs Corrected vs FBGS");
for k = 1:3
    subplot(3,1,k)
    plot(mocap_t, tip_uncorr(:,k), 'b', 'LineWidth',1.5); hold on
    plot(mocap_t, tip_corr(:,k),   'g', 'LineWidth',1.5);
    plot(fbgs_time, tip_fbgs(:,k), 'r', 'LineWidth',1.5);
    grid on; ylabel([vars{k} ' [m]'])
    if k==1, title('Robot tip position'); end
    if k==3, xlabel('Time [s]'); end
end
legend('OptiTrack (raw)','OptiTrack (corrected)','FBGS','Location','best')
