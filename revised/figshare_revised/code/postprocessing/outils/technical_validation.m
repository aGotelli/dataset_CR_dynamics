function technical_validation(saving_folder, saving_fig_folder, N_disks, N_fbgs_points, plot_validation)
    %TECHNICAL_VALIDATION Computes the dataset's technical-validation
    %   metrics: Mocap vs FBGS shape RMSE per disk, and Mocap vs motor
    %   cable-length RMSE. Reads only the CSV files already written to
    %   saving_folder by process_data.m, and writes RMSEs.txt back into
    %   that same folder. If plot_validation is true, also generates and
    %   saves the corresponding comparison figures into saving_fig_folder.
    %   (The ATI-vs-Resense wrench comparison is not done here -- see
    %   tests/compare_ati_resense_wrench.m, which reads this same saved
    %   data.)
    %
    %   saving_folder      - this recording's processed/ folder,
    %                         containing angles.csv, mocap_frames.csv,
    %                         and fbgs_shapes.csv
    %   saving_fig_folder   - folder to save validation figures into
    %   N_disks              - number of tracked OptiTrack disks
    %   N_fbgs_points        - number of FBG shape-reconstruction points
    %   plot_validation       - whether to generate and save figures

    angles_csv = readmatrix(fullfile(saving_folder, "angles.csv"));
    sampling_time = angles_csv(:, 1);
    interp_angles = angles_csv(:, 2:end);
    N_samples = size(angles_csv, 1);

    mocap_csv = readmatrix(fullfile(saving_folder, "mocap_frames.csv"));
    interp_rel_kinematics_disks_corr = reshape(mocap_csv(:, 2:end), [N_samples, 6, N_disks]);

    fbgs_csv = readmatrix(fullfile(saving_folder, "fbgs_shapes.csv"));
    interp_fbgs_shapes = permute(reshape(fbgs_csv(:, 2:end), [N_samples, 3, N_fbgs_points]), [2 3 1]);

    %   FBG sample index closest to each of the 5 robot disks (same
    %   mapping process_data.m uses -- see disk_z_positions_m there)
    disk_z_positions_m = [0 0.12 0.24 0.36 0.48];
    FBGS_disk_indices = max(round(disk_z_positions_m*1000), 1);
    FBGS_tip_index = FBGS_disk_indices(5);

    %   Mocap vs FBGS: RMSE at every disk, using the corrected mocap
    %   poses (the ones actually released, in mocap_frames.csv).
    N_disks_robot = 5;
    RMSE_disks = zeros(N_disks_robot, 3);
    RMSE_disks_perc_motion = zeros(N_disks_robot, 3);
    for d = 1:N_disks_robot
        xyz_disk_d = interp_rel_kinematics_disks_corr(:, 4:6, d);
        xyz_FBGS_d = squeeze(interp_fbgs_shapes(:, FBGS_disk_indices(d), :))';

        RMSE_disks(d, :) = rmse(xyz_FBGS_d, xyz_disk_d);

        range_disk_d = max(xyz_disk_d) - min(xyz_disk_d);
        RMSE_disks_perc_motion(d, :) = (RMSE_disks(d, :)./range_disk_d)*100;
    end

    %   Tip-only numbers (disk 5), reported alongside the per-disk ones.
    RMSE_tip = RMSE_disks(5, :);
    RMSE_tip_perc_motion = RMSE_disks_perc_motion(5, :);


    %   Mocap vs motor: cable-length RMSE
    N_interp = 10;
    [delta_cable_measured, delta_cable_computed] = compare_cable_lenght(interp_rel_kinematics_disks_corr, interp_angles, sampling_time, N_interp);

    RMSE_cables = rmse(delta_cable_computed, delta_cable_measured);
    range_cables = max(delta_cable_measured) - min(delta_cable_measured);
    RMSE_cables_perc_motion = (RMSE_cables./range_cables)*100;
    idx_0 = find(range_cables <= 1e-2);
    RMSE_cables_perc_motion(idx_0) = 0*RMSE_cables_perc_motion(idx_0);


    %   Save RMSEs
    fid = fopen(fullfile(saving_folder , "RMSEs.txt"), 'w');
    for d = 1:N_disks_robot
        fprintf(fid, 'RMSE_disk_%d = [%s]\n', d, strjoin(string(RMSE_disks(d, :)), ', '));
        fprintf(fid, 'RMSE_disk_%d_perc_motion = [%s]\n', d, strjoin(string(RMSE_disks_perc_motion(d, :)), ', '));
    end
    fprintf(fid, 'RMSE_tip = [%s]\n', strjoin(string(RMSE_tip), ', '));
    fprintf(fid, 'RMSE_tip_perc_motion = [%s]\n', strjoin(string(RMSE_tip_perc_motion), ', '));
    fprintf(fid, 'RMSE_cables = [%s]\n', strjoin(string(RMSE_cables), ', '));
    fprintf(fid, 'RMSE_cables_perc_motion = [%s]\n', strjoin(string(RMSE_cables_perc_motion), ', '));
    fclose(fid);


    if plot_validation

        XYZ_xyz_disk = interp_rel_kinematics_disks_corr(:, :, 5);
        xyz_FBGS = squeeze(interp_fbgs_shapes(:, FBGS_tip_index, :));

        fig = figure("Name", "Tip Trajectory xy plane");
        interp_xy_tip = interp_rel_kinematics_disks_corr(:, 4:5, 5);
        plot(interp_xy_tip(:, 1), interp_xy_tip(:, 2), 'LineWidth', 1)
        hold on
        plot(xyz_FBGS(1, :), xyz_FBGS(2, :), "r", "LineWidth", 1.0)
        grid on
        xlim([-.35 .35])
        ylim([-.35 .35])
        xlabel("p_x [m]")
        ylabel("p_y [m]")
        savefig(saving_fig_folder + fig.Name)
        saveas(fig, saving_fig_folder + fig.Name, 'png')


        fig = figure("Name", "Motors Angles");
        for it=1:4
            subplot(4, 1, it)
            plot(sampling_time, interp_angles(:, it), 'b', 'LineWidth', 2)
            grid on
            ylabel("Angle [rad]")
        end
        xlabel('Time [s]')
        savefig(saving_fig_folder + fig.Name)
        saveas(fig, saving_fig_folder + fig.Name, 'png')


        fig = figure("Name", "Tip Position Interpolated");
        vars = {'p_x', 'p_y', 'p_z'};
        for it = 1:3
            subplot(3,1,it)

            plot(sampling_time, XYZ_xyz_disk(:, it + 3), "b", "LineWidth", 2.0)
            set(gca,"FontSize",20)
            hold on
            plot(sampling_time, xyz_FBGS(it, :), "r", "LineWidth", 2.0)
            set(gca,"FontSize",20)

            grid on
            ylabel([vars{it} ' [m]'], "FontSize", 20)

            if it == 3
                xlabel("Time [s]", "FontSize", 20)
            end
        end
        legend('OptiTrack', 'FBGS')
        savefig(saving_fig_folder + fig.Name)
        saveas(fig, saving_fig_folder + fig.Name, 'png')


        cable_labels = {'+x', '+y', '-x', '-y'};
        pairs = {[1 3], [2 4]};          % x-pair, y-pair
        pair_names = {"x", "y"};

        for p = 1:2
            fig = figure("Name", "Cable Length Change (" + pair_names{p} + " pair)");
            idx = pairs{p};
            for k = 1:2
                ax = subplot(2,1,k);
                set(ax, 'Color', 'w');
                c = idx(k);
                plot(sampling_time, delta_cable_computed(:,c)*1e3,  'b',  'LineWidth', 2);  hold on
                plot(sampling_time, delta_cable_measured(:,c)*1e3,  'r--','LineWidth', 2);
                grid on; ylabel('\Delta \ell_c [mm]')
                title(['Cable ' cable_labels{c}])
                if k == 1
                    legend('MoCap (computed)', 'Motor (measured)', 'Location', 'best')
                end
                if k == 2, xlabel('Time [s]'); end
            end
            savefig(saving_fig_folder + fig.Name)
            saveas(fig, saving_fig_folder + fig.Name, 'png')
        end

    end

end
