function sync_results = check_temporal_sync(time_mot, angles, ...
        time_moc, kin_disks, time_fbg, fbgs_shapes, FBGS_tip_index, ...
        time_cables, cable_tensions_all, ...
        saving_folder)
%CHECK_TEMPORAL_SYNC Cross-correlate the tip/motor signals from several
%   sensors against each other and report the time lag and correlation
%   strength between each pair.
%
%   Computes, over the common [time_mot(1), time_mot(end)] window:
%     Motor -> Mocap   (2 motors x 3 tip-position axes)
%     Motor -> FBGS    (2 motors x 3 tip-position axes)
%     Mocap -> FBGS    (3 tip-position axes)
%     Motor -> Tendon  (4 motors x 4 tendons)
%
%   Each lag/correlation pair comes from a normalized cross-correlation
%   between the two signals, resampled onto a common time base.
%
%   time_mot, angles             - motor timestamps and the corresponding
%                                   [N x 2] actuator angles
%   time_moc, kin_disks           - Mocap timestamps and disk kinematics
%                                   ([N x 6 x N_disks]; column 5 is used)
%   time_fbg, fbgs_shapes         - FBG timestamps and reconstructed
%                                   shapes
%   FBGS_tip_index                 - FBG sample index used as the tip
%   time_cables, cable_tensions_all - 1x4 cell arrays, one
%                                   timestamp/tension vector per tendon
%   saving_folder                  - folder to write sync_results.txt
%                                   into; pass '' to skip saving
%
%   Returns a struct sync_results with fields lag_MM, r_MM, lag_MF, r_MF,
%   lag_OF, r_OF, lag_MC, r_MC.

mot_lbl = {'M+x', 'M+y'};
pos_lbl = {'px',  'py',  'pz'};

t0 = time_mot(1);  t1 = time_mot(end);

[t_mot, ang]     = trim_window(time_mot, angles,              t0, t1);
[t_moc, tip_moc] = trim_window(time_moc, kin_disks(:,4:6,5), t0, t1);
tip_fbg_all      = squeeze(fbgs_shapes(:, FBGS_tip_index, :))';
[t_fbg, tip_fbg] = trim_window(time_fbg, tip_fbg_all,        t0, t1);

for it=1:4
    [t_cables{it}, cable_tensions{it}] = trim_window(time_cables{it}, cable_tensions_all{it},        t0, t1);
end

t_mot = t_mot - t0;  t_moc = t_moc - t0;  t_fbg = t_fbg - t0;


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
