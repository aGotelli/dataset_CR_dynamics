function plot_disk_frames_check(rel_kinematics_raw, rel_kinematics_corr, t_idx, folder)
%PLOT_DISK_FRAMES_CHECK Visual sanity check: overlay the 5 disk frames
%   (position + orientation) from the raw and per-disk-corrected mocap
%   kinematics at a single time step, against the nominal straight-
%   configuration reference geometry, in 3-D.
%
%   rel_kinematics_raw/corr : N_time x 6 x N_disks, columns
%                              [roll pitch yaw px py pz] (XYZ euler, m)
%   t_idx                   : time index to plot (1-based; clamped to the
%                              valid range)
%   folder                  : recording folder, used only for the title

    N_disks  = size(rel_kinematics_raw, 3);
    axis_len = 0.04;   % length of plotted frame axes [m]

    t_idx = min(max(round(t_idx), 1), size(rel_kinematics_raw, 1));

    raw_colors  = {[1 0.7 0.7], [0.7 1 0.7], [0.7 0.7 1]};   % pale R/G/B
    corr_colors = {[1 0 0],     [0 0.6 0],   [0 0 1]};       % full R/G/B
    ref_color   = [0.6 0.4 0];                               % olive, single color for the reference triad

    %   Nominal straight-configuration geometry: 5 disks spaced 0.12 m
    %   apart (0.48 m total backbone length / 4 gaps), identity
    %   orientation. ASSUMPTION: stacked along +y and expressed in the
    %   same frame as rel_kinematics_raw/corr -- verify this matches your
    %   actual base-frame convention (see data_optitrack.m's R_ref)
    %   before trusting the overlay; I have not re-derived it from the
    %   frame convention this session.
    disk_spacing = 0.12;
    pos_disks_nominal = (0:N_disks-1) * disk_spacing;
    R_nominal = eye(3);

    figure('Name', 'Disk frames: raw vs. corrected vs. nominal reference');
    hold on; grid on; axis equal;
    xlabel('X [m]'); ylabel('Y [m]'); zlabel('Z [m]');
    title(sprintf('%s -- disk frames at t = %d (pale = raw, bold = corrected, olive = nominal reference)', ...
        strrep(folder, '\', '/'), t_idx), 'Interpreter', 'none');

    p_raw  = zeros(3, N_disks);
    p_corr = zeros(3, N_disks);
    p_ref  = zeros(3, N_disks);

    for k = 1:N_disks
        eul_raw  = rel_kinematics_raw(t_idx, 1:3, k);
        p_r      = rel_kinematics_raw(t_idx, 4:6, k)';
        eul_corr = rel_kinematics_corr(t_idx, 1:3, k);
        p_c      = rel_kinematics_corr(t_idx, 4:6, k)';
        p_n      = [0; pos_disks_nominal(k); 0];

        R_raw  = eul2rotm(eul_raw,  'XYZ');
        R_corr = eul2rotm(eul_corr, 'XYZ');

        p_raw(:, k)  = p_r;
        p_corr(:, k) = p_c;
        p_ref(:, k)  = p_n;

        plot_frame_triad(p_r, R_raw,     axis_len, raw_colors,          1.0);
        plot_frame_triad(p_c, R_corr,    axis_len, corr_colors,         2.0);
        % plot_frame_triad(p_n, R_nominal, axis_len, {ref_color,ref_color,ref_color}, 1.5);

        text(p_c(1), p_c(2), p_c(3), sprintf('  disk %d', k - 1), 'FontSize', 9);
    end

    plot3(p_raw(1, :),  p_raw(2, :),  p_raw(3, :),  '--', 'Color', [0.5 0.5 0.5], 'LineWidth', 1);
    plot3(p_corr(1, :), p_corr(2, :), p_corr(3, :), 'k-', 'LineWidth', 1.5);
    % plot3(p_ref(1, :),  p_ref(2, :),  p_ref(3, :),  ':', 'Color', ref_color, 'LineWidth', 1.5);

    h1 = plot3(nan, nan, nan, '-',  'Color', [1 0 0], 'LineWidth', 2);
    h2 = plot3(nan, nan, nan, '-',  'Color', [1 0.7 0.7], 'LineWidth', 2);
    h3 = plot3(nan, nan, nan, '-',  'Color', ref_color, 'LineWidth', 1.5);
    h4 = plot3(nan, nan, nan, 'k-', 'LineWidth', 1.5);
    h5 = plot3(nan, nan, nan, '--', 'Color', [0.5 0.5 0.5], 'LineWidth', 1);
    legend([h1 h2 h3 h4 h5], { ...
        'corrected frame axes (X/Y/Z = red/green/blue)', ...
        'raw frame axes (pale)', ...
        'nominal reference frame axes (olive)', ...
        'backbone (corrected)', ...
        'backbone (raw)'}, 'Location', 'bestoutside');

    view(3);
end


function plot_frame_triad(p, R, axis_len, colors, line_width)
%PLOT_FRAME_TRIAD Draw one 3-axis frame triad (X/Y/Z) at position p with
%   orientation R, using the given per-axis colors and line width.
    for ax = 1:3
        v = R(:, ax) * axis_len;
        quiver3(p(1), p(2), p(3), v(1), v(2), v(3), 0, ...
            'Color', colors{ax}, 'LineWidth', line_width, 'MaxHeadSize', 0.6);
    end
end
