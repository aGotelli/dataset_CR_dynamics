function sync_results = check_temporal_sync(time_mot, angles, ...
        time_moc, kin_disks, time_fbg, fbgs_shapes, FBGS_tip_index, ...
        time_cables, cable_tensions_all, time_ati, ati_ft, ...
        saving_folder)

mot_lbl = {'M+x', 'M+y'};
pos_lbl = {'px',  'py',  'pz'};

t0 = time_mot(1);  t1 = time_mot(end);

[t_mot, ang]     = trim_window(time_mot, angles,              t0, t1);
[t_moc, tip_moc] = trim_window(time_moc, kin_disks(:,4:6,5), t0, t1);
tip_fbg_all      = squeeze(fbgs_shapes(:, FBGS_tip_index, :))';
[t_fbg, tip_fbg] = trim_window(time_fbg, tip_fbg_all,        t0, t1);
[t_ati, ati_FT] = trim_window(time_ati, ati_ft,        t0, t1);
for it=1:4
    [t_cables{it}, cable_tensions{it}] = trim_window(time_cables{it}, cable_tensions_all{it},        t0, t1);
end

t_mot = t_mot - t0;  t_moc = t_moc - t0;  t_fbg = t_fbg - t0;
t_ati = t_ati - t0;

for it=1:4
    t_cables{it} = t_cables{it} - t0;
end

fs_mot = 1 / median(diff(t_mot));
fs_moc = 1 / median(diff(t_moc));

ml_mot = round(0.2 * fs_mot);
ml_moc = round(0.2 * fs_moc);

up         = @(ts, x, td) interp1(ts, x, td, 'pchip');
moc_on_mot = up(t_moc, tip_moc, t_mot);
fbg_on_mot = up(t_fbg, tip_fbg, t_mot);
fbg_on_moc = up(t_fbg, tip_fbg, t_moc);
ati_on_mot = up(t_ati, ati_FT, t_mot);


% Motor -> Mocap  (2 motors x 3 axes)
lag_MM = zeros(2,3);  r_MM = zeros(2,3);
for m = 1:2
    for d = 1:3
        [lag_MM(m,d), r_MM(m,d)] = peak_lag(ang(:,m), moc_on_mot(:,d), ml_mot, fs_mot);
    end
end

% Motor -> FBGS  (2 motors x 3 axes)
lag_MF = zeros(2,3);  r_MF = zeros(2,3);
for m = 1:2
    for d = 1:3
        [lag_MF(m,d), r_MF(m,d)] = peak_lag(ang(:,m), fbg_on_mot(:,d), ml_mot, fs_mot);
    end
end

% Mocap -> FBGS  (3 axes, same axis)
lag_OF = zeros(1,3);  r_OF = zeros(1,3);
for d = 1:3
    [lag_OF(d), r_OF(d)] = peak_lag(tip_moc(:,d), fbg_on_moc(:,d), ml_moc, fs_moc);
end


% Motor -> Tendon tension  (4 motors x 4 tendons)
lag_MC = zeros(1, 4); r_MC = zeros(1,4);
for d = 1:4

    cable_on_motor = up(t_cables{d}, cable_tensions{d}, t_mot);

    [lag_MC(d), r_MC(d)] = peak_lag(ang(:,d), cable_on_motor, ml_mot, fs_mot);
end

figure("Name", "Angles and Tensions")
plot(t_cables{2}, cable_tensions{2}, 'b')
hold on
yyaxis right
plot(t_mot, ang(:,1), 'r')


figure("Name", "Angles and Torque")
yyaxis left
plot(t_ati, ati_FT(:, 4), 'b')
hold on
yyaxis right
plot(t_cables{2}, cable_tensions{2}, 'g')

% Motor -> ATI  (2 motors x 2 torques)
lag_MA = zeros(1, 2); r_MA = zeros(1,2);
for m = 1:2
   [lag_MA(m), r_MA(m)] = peak_lag(ang(:,m), ati_on_mot(:,3+m), ml_mot, fs_mot);
end


% % ── print ────────────────────────────────────────────────────────────────────
% hdr = @(s) fprintf('\n=== %s ===\n         %8s    %8s    %8s\n', s, pos_lbl{:});
% row = @(lbl, lag, r) fprintf('  %-5s : %+7.1f ms  %+7.1f ms  %+7.1f ms    r = [%+.2f  %+.2f  %+.2f]\n', ...
%     lbl, lag(1), lag(2), lag(3), r(1), r(2), r(3));
% 
% hdr('Motor -> Mocap [ms]');
% row(mot_lbl{1}, lag_MM(1,:), r_MM(1,:));
% row(mot_lbl{2}, lag_MM(2,:), r_MM(2,:));
% 
% hdr('Motor -> FBGS  [ms]');
% row(mot_lbl{1}, lag_MF(1,:), r_MF(1,:));
% row(mot_lbl{2}, lag_MF(2,:), r_MF(2,:));
% 
% hdr('Mocap  -> FBGS [ms]');
% fprintf('         %+7.1f ms  %+7.1f ms  %+7.1f ms    r = [%+.2f  %+.2f  %+.2f]\n', ...
%     lag_OF(1), lag_OF(2), lag_OF(3), r_OF(1), r_OF(2), r_OF(3));

% ── save ─────────────────────────────────────────────────────────────────────
if ~isempty(saving_folder)
    fid = fopen(fullfile(saving_folder, 'sync_results.txt'), 'w');
    fprintf(fid, 'Motor -> Mocap  lag_ms [M+x; M+y] = [%.1f %.1f %.1f; %.1f %.1f %.1f]\n', lag_MM');
    fprintf(fid, 'Motor -> Mocap  r      [M+x; M+y] = [%.2f %.2f %.2f; %.2f %.2f %.2f]\n', r_MM');
    fprintf(fid, 'Motor -> FBGS   lag_ms [M+x; M+y] = [%.1f %.1f %.1f; %.1f %.1f %.1f]\n', lag_MF');
    fprintf(fid, 'Motor -> FBGS   r      [M+x; M+y] = [%.2f %.2f %.2f; %.2f %.2f %.2f]\n', r_MF');
    fprintf(fid, 'Mocap  -> FBGS  lag_ms             = [%.1f %.1f %.1f]\n', lag_OF);
    fprintf(fid, 'Mocap  -> FBGS  r                  = [%.2f %.2f %.2f]\n', r_OF);
    fclose(fid);
end

sync_results.lag_MM = lag_MM;   % [2×3]
sync_results.r_MM   = r_MM;
sync_results.lag_MF = lag_MF;   % [2×3]
sync_results.r_MF   = r_MF;
sync_results.lag_OF = lag_OF;   % [1×3]
sync_results.r_OF   = r_OF;
sync_results.lag_MC   = lag_MC;
sync_results.r_MC   = r_MC;
sync_results.lag_MA   = lag_MA;
sync_results.r_MA   = r_MA;
end

% ── helpers ──────────────────────────────────────────────────────────────────
function [t_out, x_out] = trim_window(t, x, t0, t1)
    idx = t >= t0 & t <= t1;
    t_out = t(idx);
    x_out = x(idx, :);
end

function [lag_ms, peak_r] = peak_lag(a, b, max_lag, fs)
    [r, lags] = xcorr(a - mean(a), b - mean(b), max_lag, 'normalized');
    [~, idx]  = max(abs(r));
    d = 0;
    if idx > 1 && idx < numel(r)
        denom = r(idx-1) - 2*r(idx) + r(idx+1);
        if abs(denom) > 1e-10
            d = 0.5*(r(idx-1) - r(idx+1)) / denom;
        end
    end
    lag_ms = (lags(idx) + d) / fs * 1000;
    peak_r = r(idx);
end

