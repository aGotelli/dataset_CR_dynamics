%% QUANTIFY_MOUNT_STIFFNESS  Comment 5.3 -- static stiffness of the base mount
%
%   Loads the base-push test recorded in force_at_base/: the ATI mini40
%   base force/torque sensor, and four unlabeled OptiTrack markers -- two
%   attached close together on the robot base itself (nominally spread in
%   the mocap x-z plane) and two attached further away on the support
%   frame (nominally spread along the mocap y axis). The base markers
%   moving relative to the frame markers under load is exactly the
%   mounting compliance the manuscript currently only describes
%   qualitatively ("rigidly connected" / "geometrically stable",
%   Comment 5.3); this script turns that into numbers.
%
%   The ATI and OptiTrack recordings for this ad-hoc test were started
%   independently (no shared start trigger, and no wall-clock anchor was
%   saved alongside this particular Mocap capture -- see mocap_record.py's
%   meta.json for how that anchor normally works). The two streams are
%   instead aligned here by cross-correlating the applied force magnitude
%   against the relative displacement magnitude, using the same
%   normalized-cross-correlation approach as
%   postprocessing/outils/check_temporal_sync.m elsewhere in this
%   pipeline. This is a reasonable stand-in only because the mounting
%   response to a push is essentially instantaneous at these timescales
%   (no meaningful transmission delay to solve for, unlike the sensor
%   pipeline delays check_temporal_sync.m normally quantifies) -- what we
%   need here is just "which sample in stream A corresponds to which
%   sample in stream B", not a delay measurement in its own right.
%
%   Multiple discrete pushes were made during the 60 s recording, evidently
%   in different directions/locations (the resulting apparent stiffness
%   varies severalfold between events -- see below), so this script
%   reports a per-event stiffness rather than forcing a single blanket
%   number through all the data.
%
%   Outputs (written next to this script):
%     mount_stiffness_results.txt          per-event and overall numbers
%     mount_stiffness_timeseries.png/.fig  aligned force + displacement traces
%     mount_stiffness_scatter.png/.fig     |F| vs |d| for every active sample

clear; clc; close all;

%% ====== SETTINGS ======
this_folder = fileparts(mfilename('fullpath'));
data_folder = fullfile(this_folder, "force_at_base");

baseline_window_s = 2.0;     % first N seconds of Mocap assumed unloaded
xcorr_dt_s        = 0.01;    % common grid step for cross-correlation (100 Hz)
xcorr_max_lag_s   = 40;      % search window for the ATI/Mocap clock offset
force_threshold_N = 6.0;     % above-(preload)baseline force counts as "active"
min_event_dur_s   = 0.2;     % discard shorter blips as noise/vibration
merge_gap_s       = 0.3;     % bridge gaps shorter than this within one event
preload_window_s  = 0.5;     % window just before each event used as its own force baseline

%% ====== LOAD ATI FORCE/TORQUE ======
ati = readtable(fullfile(data_folder, "dataATIFT.csv"));
t_ati = ati.timestamp - ati.timestamp(1);
F_ati = [ati.Fx_N_, ati.Fy_N_, ati.Fz_N_];
Fmag_ati = vecnorm(F_ati, 2, 2);
fs_ati = (height(ati)-1) / t_ati(end);

fprintf("ATI: %d samples over %.1f s (%.1f Hz)\n", height(ati), t_ati(end), fs_ati);

%% ====== LOAD OPTITRACK MARKERS ======
markers = readtable(fullfile(data_folder, "dataOptiTrack", "markers.csv"));
frames  = readtable(fullfile(data_folder, "dataOptiTrack", "frames.csv"));

slot_frame = cell(4,1);
slot_xyz   = cell(4,1);
for k = 0:3
    idx = markers.marker_slot == k;
    slot_frame{k+1} = markers.frame_number(idx);
    slot_xyz{k+1}   = [markers.x(idx), markers.y(idx), markers.z(idx)];
end

% Only keep frames where all four markers were detected. Occlusion drops
% a meaningful fraction of frames; interpolating across a dropout would
% risk silently mixing up which marker is which (unlabeled markers have
% no persistent ID beyond this recording's own slot ordering), so those
% frames are excluded rather than guessed at.
common_frames = slot_frame{1};
for k = 2:4
    common_frames = intersect(common_frames, slot_frame{k});
end
fprintf("Mocap: %d/%d frames have all 4 markers (%.0f%% dropped to occlusion)\n", ...
    numel(common_frames), height(frames), 100*(1 - numel(common_frames)/height(frames)));

xyz = zeros(numel(common_frames), 3, 4);
for k = 1:4
    [tf, loc] = ismember(common_frames, slot_frame{k});
    assert(all(tf), "internal error: common_frames should be a subset of every slot");
    xyz(:,:,k) = slot_xyz{k}(loc, :);
end

[tf, loc] = ismember(common_frames, frames.frame_number);
assert(all(tf));
moc_t = frames.motive_timestamp_s(loc);
moc_t = moc_t - moc_t(1);

%% ====== IDENTIFY BASE-MARKER PAIR VS FRAME-MARKER PAIR ======
% The two markers on the robot base are "aligned and close"; the two on
% the support frame are further apart, spread mainly along mocap y.
% Auto-detected rather than hardcoded: with only 4 markers, the six
% pairwise mean distances unambiguously separate into one short distance
% (the base pair) and the rest -- verified on this recording, where the
% closest pair is roughly an order of magnitude closer than any other
% pair.
mean_pos = squeeze(mean(xyz, 1));   % 3x4: columns are markers 1..4 (slots 0..3)
pair_list = nchoosek(1:4, 2);
pair_dist = vecnorm(mean_pos(:,pair_list(:,1)) - mean_pos(:,pair_list(:,2)), 2, 1);

[~, i_min] = min(pair_dist);
base_slots  = pair_list(i_min, :);
frame_slots = setdiff(1:4, base_slots);
[frame_dist, i_max] = max(pair_dist);

fprintf("Base markers   (slots %d,%d): mean separation %.1f mm\n", ...
    base_slots(1)-1, base_slots(2)-1, 1000*pair_dist(i_min));
fprintf("Frame markers  (slots %d,%d): mean separation %.1f mm, dominant axis = %s\n", ...
    frame_slots(1)-1, frame_slots(2)-1, 1000*frame_dist, ...
    dominant_axis_label(mean_pos(:,frame_slots(1)) - mean_pos(:,frame_slots(2))));
if i_max ~= find(all(sort(pair_list,2) == sort(frame_slots), 2))
    warning("The most-separated marker pair is not the same as the auto-detected frame pair -- check the physical marker layout assumption.");
end

base_pos  = mean(xyz(:,:,base_slots), 3);
frame_pos = mean(xyz(:,:,frame_slots), 3);

%% ====== RELATIVE DISPLACEMENT (BASE MARKERS RELATIVE TO FRAME MARKERS) ======
% Taking the base-to-frame vector, rather than either marker group's raw
% position, cancels any common-mode motion of the whole rig (e.g. the
% table itself flexing slightly) and isolates the actual compliance
% between the robot base and the support structure it's mounted to --
% the quantity Comment 5.3 is actually asking about.
rel = base_pos - frame_pos;
baseline_mask = moc_t < baseline_window_s;
rel_baseline = mean(rel(baseline_mask, :), 1);
d_vec = rel - rel_baseline;                 % m, relative to unloaded baseline
dmag  = vecnorm(d_vec, 2, 2) * 1000;        % mm
fprintf("Baseline (unloaded) displacement noise: std = %.4f mm\n", std(dmag(baseline_mask)));

%% ====== ALIGN THE TWO STREAMS (CROSS-CORRELATION, NO SHARED CLOCK) ======
t_grid = (0:xcorr_dt_s:min(t_ati(end), moc_t(end)))';
Fmag_grid = interp1(t_ati, Fmag_ati, t_grid, "linear");
dmag_grid = interp1(moc_t, dmag, t_grid, "linear");

[lag_s, r_xcorr] = peak_lag(Fmag_grid, dmag_grid, round(xcorr_max_lag_s/xcorr_dt_s), 1/xcorr_dt_s);
fprintf("Cross-correlation alignment: Mocap lags ATI by %.3f s (r = %.2f)\n", lag_s, r_xcorr);

moc_t_aligned = moc_t + lag_s;   % Mocap events expressed on the ATI time axis

valid = t_ati >= moc_t_aligned(1) & t_ati <= moc_t_aligned(end);
t_v    = t_ati(valid);
F_v    = F_ati(valid, :);
Fmag_v = Fmag_ati(valid);
dmag_v = interp1(moc_t_aligned, dmag, t_v, "linear");

%% ====== DETECT DISCRETE PUSH EVENTS ======
preload_N = median(Fmag_ati(t_ati < baseline_window_s));
active = Fmag_v > (preload_N + force_threshold_N);

dt_v = mean(diff(t_v));
events = find_events(active, round(merge_gap_s/dt_v), round(min_event_dur_s/dt_v));

fprintf("\nDetected %d push events (threshold = preload + %.1f N):\n", numel(events), force_threshold_N);

n_ev = numel(events);
ev_t_start = zeros(n_ev,1); ev_t_end = zeros(n_ev,1);
ev_peakF   = zeros(n_ev,1); ev_d_at_peak = zeros(n_ev,1);
ev_dF      = zeros(n_ev,3); ev_k = zeros(n_ev,1);

for i = 1:n_ev
    idx = events{i};
    [pk, i_pk] = max(Fmag_v(idx));
    ev_t_start(i) = t_v(idx(1));
    ev_t_end(i)   = t_v(idx(end));
    ev_peakF(i)   = pk;
    ev_d_at_peak(i) = dmag_v(idx(i_pk));

    pre_mask = t_v >= (ev_t_start(i) - preload_window_s) & t_v < ev_t_start(i);
    if any(pre_mask)
        F_pre = mean(F_v(pre_mask, :), 1);
    else
        F_pre = [0 0 0];
    end
    ev_dF(i,:) = F_v(idx(i_pk), :) - F_pre;
    ev_k(i) = ev_peakF(i) / ev_d_at_peak(i);
end

%% ====== OVERALL (BLANKET) LINEAR FIT, FOR COMPARISON ======
A = [Fmag_v(active), ones(nnz(active),1)];
coeffs = A \ dmag_v(active);
pred = A*coeffs;
ss_res = sum((dmag_v(active) - pred).^2);
ss_tot = sum((dmag_v(active) - mean(dmag_v(active))).^2);
r2_overall = 1 - ss_res/ss_tot;
k_overall = 1/coeffs(1);

%% ====== REPORT ======
results_file = fullfile(this_folder, "mount_stiffness_results.txt");
fid = fopen(results_file, "w");
report_lines = {};
report_lines{end+1} = sprintf("Comment 5.3 -- base mount stiffness (force_at_base test)");
report_lines{end+1} = sprintf("ATI/Mocap alignment: Mocap lags ATI by %.3f s (cross-correlation r = %.2f)", lag_s, r_xcorr);
report_lines{end+1} = sprintf("Baseline (unloaded) displacement noise: std = %.4f mm", std(dmag(baseline_mask)));
report_lines{end+1} = "";
report_lines{end+1} = sprintf("%-14s %-14s %-10s %-10s %-24s %-10s", ...
    "t_start (s)", "t_end (s)", "peakF (N)", "d (mm)", "dF (Fx,Fy,Fz) (N)", "k (N/mm)");
for i = 1:n_ev
    report_lines{end+1} = sprintf("%-14.2f %-14.2f %-10.2f %-10.3f (%7.2f,%7.2f,%7.2f) %-10.1f", ...
        ev_t_start(i), ev_t_end(i), ev_peakF(i), ev_d_at_peak(i), ev_dF(i,:), ev_k(i));
end
report_lines{end+1} = "";
report_lines{end+1} = sprintf("Per-event stiffness: median = %.1f N/mm, range = [%.1f, %.1f] N/mm (n = %d events)", ...
    median(ev_k), min(ev_k), max(ev_k), n_ev);
report_lines{end+1} = sprintf("Blanket linear fit across all active samples: k = %.1f N/mm, R^2 = %.2f (low R^2 is expected -- pushes were not all in the same direction, so |F| vs |d| is not a single-valued relationship; see per-event table above for direction-resolved values)", ...
    k_overall, r2_overall);

for i = 1:numel(report_lines)
    fprintf("%s\n", report_lines{i});
    fprintf(fid, "%s\n", report_lines{i});
end
fclose(fid);
fprintf("\nSaved: %s\n", results_file);

%% ====== PLOT: ALIGNED TIME SERIES ======
fig1 = figure("Name", "Mount stiffness -- aligned time series", "Position", [100 100 900 600]);

subplot(2,1,1);
plot(t_v, F_v(:,1), "DisplayName", "F_x"); hold on
plot(t_v, F_v(:,2), "DisplayName", "F_y");
plot(t_v, F_v(:,3), "DisplayName", "F_z");
plot(t_v, Fmag_v, "k", "LineWidth", 1.5, "DisplayName", "|F|");
for i = 1:n_ev
    xline(ev_t_start(i), "--", "Color", [0.6 0.6 0.6], "HandleVisibility", "off");
end
ylabel("Force [N]"); legend("Location","best"); grid on
title("ATI base wrench (aligned to Mocap time)");

subplot(2,1,2);
plot(t_v, dmag_v, "b", "LineWidth", 1.2);
ylabel("Base-to-frame displacement [mm]"); xlabel("Time [s]"); grid on
title("Relative displacement (base markers - frame markers, minus baseline)");

saveas(fig1, fullfile(this_folder, "mount_stiffness_timeseries.png"));
saveas(fig1, fullfile(this_folder, "mount_stiffness_timeseries.fig"));

%% ====== PLOT: FORCE VS DISPLACEMENT SCATTER ======
fig2 = figure("Name", "Mount stiffness -- force vs displacement", "Position", [100 100 700 600]);
scatter(Fmag_v(active), dmag_v(active), 10, t_v(active), "filled");
colormap(turbo); cb = colorbar; ylabel(cb, "time [s]");
hold on
plot(ev_peakF, ev_d_at_peak, "kx", "MarkerSize", 12, "LineWidth", 2, "DisplayName", "per-event peak");
xlabel("|F| [N]"); ylabel("|d| [mm]"); grid on
title(sprintf("Force vs. relative displacement (n = %d events)", n_ev));
legend("all active samples (colored by time)", "per-event peak", "Location", "best");

saveas(fig2, fullfile(this_folder, "mount_stiffness_scatter.png"));
saveas(fig2, fullfile(this_folder, "mount_stiffness_scatter.fig"));


%% ====================================================================
%%  HELPER FUNCTIONS
%% ====================================================================

function [lag_s, peak_r] = peak_lag(a, b, max_lag, fs)
    %   PEAK_LAG  Normalized cross-correlation between mean-removed
    %   signals a and b, searched over +/-max_lag samples at sample rate
    %   fs. Mirrors outils/check_temporal_sync.m's helper of the same
    %   name: finds the lag of the largest-magnitude correlation peak,
    %   refined to sub-sample precision with a parabolic fit through the
    %   peak and its two neighbors. Positive lag_s means b lags behind a.
    [r, lags] = xcorr(a - mean(a), b - mean(b), max_lag, "normalized");
    [~, idx] = max(abs(r));
    delta = 0;
    if idx > 1 && idx < numel(r)
        denom = r(idx-1) - 2*r(idx) + r(idx+1);
        if abs(denom) > 1e-10
            delta = 0.5*(r(idx-1) - r(idx+1)) / denom;
        end
    end
    lag_s = (lags(idx) + delta) / fs;
    peak_r = r(idx);
end

function events = find_events(active, merge_gap_samples, min_len_samples)
    %   FIND_EVENTS  Groups a logical vector into contiguous runs of
    %   true, bridging gaps of false shorter than merge_gap_samples, then
    %   discards runs shorter than min_len_samples. Returns a cell array
    %   of index vectors, one per surviving event.
    active = active(:)';
    d = diff([0, active, 0]);
    starts = find(d == 1);
    stops  = find(d == -1) - 1;

    merged_starts = starts(1);
    merged_stops  = stops(1);
    for i = 2:numel(starts)
        if starts(i) - merged_stops(end) <= merge_gap_samples
            merged_stops(end) = stops(i);
        else
            merged_starts(end+1) = starts(i); %#ok<AGROW>
            merged_stops(end+1)  = stops(i);  %#ok<AGROW>
        end
    end

    keep = (merged_stops - merged_starts + 1) >= min_len_samples;
    merged_starts = merged_starts(keep);
    merged_stops  = merged_stops(keep);

    events = cell(numel(merged_starts), 1);
    for i = 1:numel(merged_starts)
        events{i} = merged_starts(i):merged_stops(i);
    end
end

function lbl = dominant_axis_label(v)
    %   DOMINANT_AXIS_LABEL  Returns 'x', 'y', or 'z' for whichever
    %   component of v (a 3-vector) has the largest magnitude.
    [~, i] = max(abs(v));
    labels = ["x", "y", "z"];
    lbl = labels(i);
end
