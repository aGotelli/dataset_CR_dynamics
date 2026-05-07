function sync_results = check_temporal_sync(time_mot, angles, ...
        time_moc, kin_disks, time_fbg, fbgs_shapes, FBGS_tip_index, saving_fig_folder)
%
% Sign convention: peak_lag(a,b) -> positive lag = b leads a, negative = a leads b.
%   Motor->OptiTrack  < 0 : motor leads sensor (expected)
%   OptiTrack->FBGS   < 0 : OptiTrack leads FBGS (~13 ms pipeline delay)
%
% xcorr requires equal-length inputs. All signals are interpolated onto the
% motor time grid (fastest, ~460 Hz) with pchip before each xcorr call.
% No resampling to a fixed rate is needed; parabolic interpolation of the
% xcorr peak provides sub-sample lag resolution from native-rate data.
%
% Only agonist motors (col 1 = M+x, col 2 = M+y) are used for timing
% validation. Antagonists (col 3 = M-x, col 4 = M-y) lag the agonists by
% ~30-40 ms: sequential CAN dispatch (M1->M2->M3->M4, ~1-4 ms) plus
% mechanical coupling (cable compliance + controller response). This lag
% is a tendon-system property, not a clock offset.

%% Trim to motor execution window
t0 = time_mot(1);  t1 = time_mot(end);

[t_mot, ang]     = trim_window(time_mot, angles,              t0, t1);
[t_moc, tip_moc] = trim_window(time_moc, kin_disks(:,4:6,5), t0, t1);
tip_fbg_all      = squeeze(fbgs_shapes(:, FBGS_tip_index, :))';
[t_fbg, tip_fbg] = trim_window(time_fbg, tip_fbg_all,        t0, t1);

t_mot = t_mot - t0;  t_moc = t_moc - t0;  t_fbg = t_fbg - t0;

fs_mot = 1 / median(diff(t_mot));
fs_moc = 1 / median(diff(t_moc));

max_lag_mot = round(0.2 * fs_mot);   % ±200 ms search window
max_lag_moc = round(0.2 * fs_moc);

%% Interpolate onto common grids
up         = @(ts, x, td) interp1(ts, x, td, 'pchip');
moc_on_mot = up(t_moc, tip_moc, t_mot);   % OptiTrack at motor timestamps
fbg_on_mot = up(t_fbg, tip_fbg, t_mot);   % FBGS at motor timestamps
fbg_on_moc = up(t_fbg, tip_fbg, t_moc);   % FBGS at mocap timestamps

%% Sanity check: sinusoid with known 20 ms delay
t_syn = (0 : numel(t_mot)-1)' / fs_mot;
a_syn = sin(2*pi*2.0*(t_syn - 0.020));   % a lags b by 20 ms -> b leads -> positive lag
b_syn = sin(2*pi*2.0*t_syn);
[rec_lag, ~] = peak_lag(a_syn, b_syn, max_lag_mot, fs_mot);
fprintf('Synthetic check: injected 20.0 ms, recovered %.2f ms\n', rec_lag);
assert(abs(rec_lag - 20.0) < 0.5, ...
    sprintf('peak_lag error: injected 20.0 ms, recovered %.2f ms', rec_lag));

%% Agonist motors -> OptiTrack / FBGS  (2 x 3)
lag_MO = zeros(2,3);  r_MO = zeros(2,3);
lag_MF = zeros(2,3);  r_MF = zeros(2,3);
for m = 1:2
    for d = 1:3
        [lag_MO(m,d), r_MO(m,d)] = peak_lag(ang(:,m), moc_on_mot(:,d), max_lag_mot, fs_mot);
        [lag_MF(m,d), r_MF(m,d)] = peak_lag(ang(:,m), fbg_on_mot(:,d), max_lag_mot, fs_mot);
    end
end

%% OptiTrack -> FBGS  (3 axes)
lag_OF = zeros(1,3);  r_OF = zeros(1,3);
for d = 1:3
    [lag_OF(d), r_OF(d)] = peak_lag(tip_moc(:,d), fbg_on_moc(:,d), max_lag_moc, fs_moc);
end

%% Antagonistic pair mechanical delay
% Negative lag: agonist leads antagonist.
% Sources: sequential CAN dispatch (~1-4 ms) + mechanical coupling (~30-40 ms).
[lag_M13, r_M13] = peak_lag(ang(:,1), -ang(:,3), max_lag_mot, fs_mot);
[lag_M24, r_M24] = peak_lag(ang(:,2), -ang(:,4), max_lag_mot, fs_mot);

%% Print
ml = {'M(+x)', 'M(+y)'};
fprintf('\n=== Agonist motor -> OptiTrack lags [ms]  (r) ===\n');
fprintf('           %-20s%-20s%s\n', 'px', 'py', 'pz');
for m = 1:2
    fprintf('  %-8s', ml{m});
    for d = 1:3, fprintf('%+7.1f (%+.2f)   ', lag_MO(m,d), r_MO(m,d)); end
    fprintf('\n');
end

fprintf('\n=== Agonist motor -> FBGS lags [ms]  (r) ===\n');
fprintf('           %-20s%-20s%s\n', 'px', 'py', 'pz');
for m = 1:2
    fprintf('  %-8s', ml{m});
    for d = 1:3, fprintf('%+7.1f (%+.2f)   ', lag_MF(m,d), r_MF(m,d)); end
    fprintf('\n');
end

fprintf('\n=== OptiTrack -> FBGS lags [ms]  (r) ===\n');
fprintf('          ');
for d = 1:3, fprintf('%+7.1f (%+.2f)   ', lag_OF(d), r_OF(d)); end
fprintf('\n');

fprintf('\n=== Antagonistic pair mechanical delay ===\n');
fprintf('  M(+x) leads M(-x):  %.1f ms  (r = %.2f)\n', -lag_M13, r_M13);
fprintf('  M(+y) leads M(-y):  %.1f ms  (r = %.2f)\n', -lag_M24, r_M24);
fprintf('  Sources: sequential CAN dispatch (~1-4 ms) + cable mechanics.\n');

%% Figures
% FBGS pipeline delay correction for visualisation
fbg_delay_s     = -mean(lag_OF) / 1000;                            % positive ~0.013 s
fbg_on_mot_corr = up(t_fbg - fbg_delay_s, tip_fbg, t_mot);        % shift FBGS earlier

ax_lbl = {'px [mm]', 'py [mm]', 'pz [mm]'};

% Fig 1: Raw vs interpolated — OptiTrack
fig1 = figure('Name', 'Interpolation check: OptiTrack');
for d = 1:3
    subplot(3,1,d);
    plot(t_moc, tip_moc(:,d)); hold on;
    plot(t_mot, moc_on_mot(:,d), '--');
    grid on;
    legend({'raw (120 Hz)', 'interpolated on motor grid'});
    ylabel(ax_lbl{d});
end
xlabel('Time [s]');

% Fig 2: Raw vs interpolated — FBGS
fig2 = figure('Name', 'Interpolation check: FBGS');
for d = 1:3
    subplot(3,1,d);
    plot(t_fbg, tip_fbg(:,d)); hold on;
    plot(t_mot, fbg_on_mot(:,d), '--');
    grid on;
    legend({'raw (100 Hz)', 'interpolated on motor grid'});
    ylabel(ax_lbl{d});
end
xlabel('Time [s]');

% Fig 3: Antagonistic motor pair raw angles
fig3 = figure('Name', 'Antagonistic motor pair delay');
subplot(2,1,1);
plot(t_mot, ang(:,1)); hold on;
plot(t_mot, ang(:,3));
plot(t_mot, -ang(:,3));
grid on;
legend({'M(+x)', 'M(-x)'});
ylabel('Angle [rad]');
title(sprintf('X pair — M(+x) leads M(-x) by %.1f ms (r=%.2f)', -lag_M13, r_M13));

subplot(2,1,2);
plot(t_mot, ang(:,2)); hold on;
plot(t_mot, ang(:,4));
plot(t_mot, -ang(:,4));
grid on;
legend({'M(+y)', 'M(-y)'});
ylabel('Angle [rad]');
xlabel('Time [s]');
title(sprintf('Y pair — M(+y) leads M(-y) by %.1f ms (r=%.2f)', -lag_M24, r_M24));

% Fig 4: Temporal alignment (full + zoom around peak)
[~, best_ax]  = max(range(tip_moc));
[~, best_mot] = max(range(ang(:,[1 2])));
norm1 = @(x) (x - mean(x)) / (max(x) - min(x) + eps);

fig4 = figure('Name', 'Temporal alignment');
subplot(2,1,1);
plot(t_mot, norm1(ang(:, best_mot))); hold on;
plot(t_mot, norm1(moc_on_mot(:, best_ax)));
plot(t_mot, norm1(fbg_on_mot(:, best_ax)));
plot(t_mot, norm1(fbg_on_mot_corr(:, best_ax)));
grid on;
legend({'Motor (norm)', 'OptiTrack (norm)', 'FBGS raw (norm)', ...
    sprintf('FBGS corrected %.1f ms (norm)', fbg_delay_s*1000)});
ylabel('Normalized');
title(sprintf('p%s — motor %d — full recording', ax_lbl{d}, best_mot));

subplot(2,1,2);
[~, i_peak] = max(ang(:, best_mot));
hw  = round(0.5 * fs_mot);
win = max(1, i_peak - hw) : min(numel(t_mot), i_peak + hw);
plot(t_mot(win), norm1(ang(win, best_mot))); hold on;
plot(t_mot(win), norm1(moc_on_mot(win, best_ax)));
plot(t_mot(win), norm1(fbg_on_mot(win, best_ax)));
plot(t_mot(win), norm1(fbg_on_mot_corr(win, best_ax)));
grid on;
legend({'Motor', 'OptiTrack', 'FBGS raw', 'FBGS corrected'});
xlabel('Time [s]');
ylabel('Normalized');
title('Zoom: peak region');

% %% Save
% if ~isempty(saving_fig_folder)
%     saveas(fig1, fullfile(saving_fig_folder, 'sync_interp_optitrack.png'));
%     saveas(fig2, fullfile(saving_fig_folder, 'sync_interp_fbgs.png'));
%     saveas(fig3, fullfile(saving_fig_folder, 'sync_motor_pairs.png'));
%     saveas(fig4, fullfile(saving_fig_folder, 'sync_temporal_alignment.png'));
% end

%% Output
sync_results.lag_MO  = lag_MO;   sync_results.r_MO  = r_MO;
sync_results.lag_MF  = lag_MF;   sync_results.r_MF  = r_MF;
sync_results.lag_OF  = lag_OF;   sync_results.r_OF  = r_OF;
sync_results.lag_M13 = lag_M13;  sync_results.r_M13 = r_M13;
sync_results.lag_M24 = lag_M24;  sync_results.r_M24 = r_M24;

end


%% ── Helpers ──────────────────────────────────────────────────────────────

function [t_out, x_out] = trim_window(t, x, t0, t1)
    idx   = t >= t0 & t <= t1;
    t_out = t(idx);
    x_out = x(idx,:);
end

function [lag_ms, peak_r] = peak_lag(a, b, max_lag, fs)
    [r, lags] = xcorr(a - mean(a), b - mean(b), max_lag, 'normalized');
    [~, idx]  = max(abs(r));
    d = 0;
    if idx > 1 && idx < numel(r)
        denom = r(idx-1) - 2*r(idx) + r(idx+1);
        if abs(denom) > 1e-10
            d = 0.5 * (r(idx-1) - r(idx+1)) / denom;
        end
    end
    lag_ms = (lags(idx) + d) / fs * 1000;
    peak_r = r(idx);
end
% function sync_results = check_temporal_sync(time_mot, angles, ...
%         time_moc, kin_disks, time_fbg, fbgs_shapes, FBGS_tip_index, saving_fig_folder)
% %CHECK_TEMPORAL_SYNC  Cross-correlation lags on raw data trimmed to motor window.
% %
% %   Each sensor runs at its native rate:
% %     Motor    ~460 Hz  (CAN / Python)
% %     OptiTrack ~120 Hz  (Ethernet / Python)
% %     FBGS      ~100 Hz  (TCP / C++)
% %
% %   For each pair, the slower signal is interpolated onto the faster
% %   signal's timestamps so the faster rate sets the lag resolution.
% %
% %   Call after frame corrections, before filtering/resampling.
% 
% t0 = time_mot(1);
% t1 = time_mot(end);
% max_lag_s = 0.2;    % ±200 ms search window
% 
% %   Trim each sensor to the motor execution window
% [t_mot, ang]     = trim_window(time_mot, angles,                          t0, t1);
% [t_moc, tip_moc] = trim_window(time_moc, kin_disks(:, 4:6, 5),           t0, t1);
% 
% tip_fbg_all      = squeeze(fbgs_shapes(:, FBGS_tip_index, :))';   % N×3
% [t_fbg, tip_fbg] = trim_window(time_fbg, tip_fbg_all,             t0, t1);
% 
% %   Relative time from motor start
% t_mot = t_mot - t0;
% t_moc = t_moc - t0;
% t_fbg = t_fbg - t0;
% 
% fs_mot = 1 / median(diff(t_mot));
% fs_moc = 1 / median(diff(t_moc));
% 
% %   Interpolate slower signal onto faster grid for each pair
% up = @(ts, x, td) interp1(ts, x, td, 'pchip');
% 
% moc_on_mot = up(t_moc, tip_moc, t_mot);   % OptiTrack → motor grid
% fbg_on_mot = up(t_fbg, tip_fbg, t_mot);   % FBGS      → motor grid
% fbg_on_moc = up(t_fbg, tip_fbg, t_moc);   % FBGS      → mocap grid
% 
% 
% max_lag_mot = round(max_lag_s * fs_mot);
% max_lag_moc = round(max_lag_s * fs_moc);
% 
% % --- sanity check: peak_lag must recover a known synthetic delay ---
% % --- sanity check: peak_lag must recover a known synthetic delay ---
% t_syn  = (0 : numel(t_mot)-1)' / fs_mot;
% a_syn  = sin(2*pi*2.0*t_syn);            % 2 Hz reference tone
% b_syn  = sin(2*pi*2.0*(t_syn - 0.020)); % same tone, delayed 20 ms
% [rec_lag, ~] = peak_lag(a_syn, b_syn, max_lag_mot, fs_mot);
% fprintf('Synthetic check: injected 20.0 ms, recovered %.2f ms\n', rec_lag);
% assert(abs(rec_lag - 20.0) < 0.5, ...
%     sprintf('peak_lag error: injected 20.0 ms, recovered %.2f ms', rec_lag));
% 
% 
% 
% %   Compute lags: 4 motors × 3 axes
% lag_MO = zeros(4,3);  r_MO = zeros(4,3);
% lag_MF = zeros(4,3);  r_MF = zeros(4,3);
% lag_OF = zeros(1,3);  r_OF = zeros(1,3);
% 
% for m = 1:4
%     for d = 1:3
%         [lag_MO(m,d), r_MO(m,d)] = peak_lag(ang(:,m), moc_on_mot(:,d), max_lag_mot, fs_mot);
%         [lag_MF(m,d), r_MF(m,d)] = peak_lag(ang(:,m), fbg_on_mot(:,d), max_lag_mot, fs_mot);
%     end
% end
% for d = 1:3
%     [lag_OF(d), r_OF(d)] = peak_lag(tip_moc(:,d), fbg_on_moc(:,d), max_lag_moc, fs_moc);
% end
% 
% %   Print
% mot_lbl = {'+x','+y','-x','-y'};
% ax_lbl  = {'px','py','pz'};
% 
% fprintf('\n=== Motor -> OptiTrack lags [ms]  (r) ===\n')
% fprintf('        %14s  %14s  %14s\n', ax_lbl{:})
% for m = 1:4
%     fprintf('  M(%s) ', mot_lbl{m})
%     for d = 1:3, fprintf('  %+6.1f (%.2f) ', lag_MO(m,d), r_MO(m,d)); end
%     fprintf('\n')
% end
% 
% fprintf('\n=== Motor -> FBGS lags [ms]  (r) ===\n')
% fprintf('        %14s  %14s  %14s\n', ax_lbl{:})
% for m = 1:4
%     fprintf('  M(%s) ', mot_lbl{m})
%     for d = 1:3, fprintf('  %+6.1f (%.2f) ', lag_MF(m,d), r_MF(m,d)); end
%     fprintf('\n')
% end
% 
% fprintf('\n=== OptiTrack -> FBGS lags [ms]  (r) ===\n')
% fprintf('        %14s  %14s  %14s\n', ax_lbl{:})
% fprintf('        ')
% for d = 1:3, fprintf('  %+6.1f (%.2f) ', lag_OF(d), r_OF(d)); end
% fprintf('\n')
% 
% sync_results = struct('lag_MO',lag_MO,'r_MO',r_MO, ...
%                       'lag_MF',lag_MF,'r_MF',r_MF, ...
%                       'lag_OF',lag_OF,'r_OF',r_OF);
% end
% 
% %──────────────────────────────────────────────────────────────────────
% function [t_out, x_out] = trim_window(t, x, t0, t1)
%     idx   = t >= t0 & t <= t1;
%     t_out = t(idx);
%     x_out = x(idx, :);
% end
% 
% function [lag_ms, peak_r] = peak_lag(a, b, max_lag, fs)
%     [r, lags] = xcorr(a-mean(a), b-mean(b), max_lag, 'normalized');
%     [~, idx]  = max(abs(r));
%     d = 0;
%     if idx > 1 && idx < length(r)
%         denom = r(idx-1) - 2*r(idx) + r(idx+1);
%         if abs(denom) > 1e-10
%             d = 0.5*(r(idx-1) - r(idx+1)) / denom;
%         end
%     end
%     lag_ms = (lags(idx) + d) / fs * 1000;
%     peak_r = r(idx);
% end
% 
% 
% % function sync_results = check_temporal_sync(sampling_time, interp_angles, ...
% %         interp_rel_kinematics_disks, interp_fbgs_shapes, FBGS_tip_index, saving_fig_folder)
% % %CHECK_TEMPORAL_SYNC  Cross-correlation lag between Motor, OptiTrack, FBGS.
% % %   Positive lag = second signal leads first.
% % 
% % fs           = 1 / median(diff(sampling_time));
% % max_lag_samp = 20;   % ±20 samples = ±200 ms at 100 Hz
% % 
% % %   Signals — pick the most dynamic axis automatically
% % tip_moc = interp_rel_kinematics_disks(:, 4:6, 5);               % N×3
% % tip_fbg = squeeze(interp_fbgs_shapes(:, FBGS_tip_index, :))';    % N×3
% % [~, ax] = max(range(tip_moc));
% % 
% % ang  = interp_angles(:, ax);
% % tmoc = tip_moc(:, ax);
% % tfbg = tip_fbg(:, ax);
% % 
% % %   Cross-correlations
% % [lag_MO, r_MO] = peak_lag(ang,  tmoc, max_lag_samp, fs);
% % [lag_MF, r_MF] = peak_lag(ang,  tfbg, max_lag_samp, fs);
% % [lag_OF, r_OF] = peak_lag(tmoc, tfbg, max_lag_samp, fs);
% % 
% % fprintf('\n=== Temporal synchronisation ===\n')
% % fprintf('Motor  -> OptiTrack : %+.1f ms  (r = %.3f)\n', lag_MO, r_MO)
% % fprintf('Motor  -> FBGS      : %+.1f ms  (r = %.3f)\n', lag_MF, r_MF)
% % fprintf('OptiTrack -> FBGS   : %+.1f ms  (r = %.3f)\n', lag_OF, r_OF)
% % fprintf('Consistency  lag(MF)-lag(MO) = %+.1f ms  (direct = %+.1f ms)\n', ...
% %         lag_MF - lag_MO, lag_OF)
% % 
% % sync_results = struct('lag_MO',lag_MO,'lag_MF',lag_MF,'lag_OF',lag_OF, ...
% %                       'r_MO',r_MO,'r_MF',r_MF,'r_OF',r_OF);
% % 
% % %   Figure
% % dc   = @(x) x - mean(x);
% % scl  = @(x) dc(x) / range(x);       % zero-mean, unit range — visual only
% % lags_ms = (-max_lag_samp:max_lag_samp) / fs * 1000;
% % 
% % fig = figure('Name','Temporal sync');
% % 
% % subplot(2,1,1)
% % plot(sampling_time, scl(ang),  'Color',[0.2 0.6 0.2],'LineWidth',1.5,'DisplayName','Motor angle');   hold on
% % plot(sampling_time, scl(tmoc), 'b',                   'LineWidth',1.5,'DisplayName','OptiTrack tip');
% % plot(sampling_time, scl(tfbg), 'r--',                 'LineWidth',1.5,'DisplayName','FBGS tip');
% % [~, pks] = findpeaks(scl(tmoc), sampling_time, 'MinPeakProminence',0.3,'MinPeakDistance',0.5);
% % for p = pks', xline(p,'k:','LineWidth',0.8,'HandleVisibility','off'); end
% % legend; grid on
% % ylabel('Norm. amplitude'); xlabel('Time [s]')
% % title('Three acquisition paths — vertical lines at OptiTrack peaks')
% % 
% % subplot(2,1,2)
% % plot(lags_ms, xcorr(dc(ang),tmoc,max_lag_samp,'normalized'), 'Color',[0.2 0.6 0.2],'LineWidth',1.5,'DisplayName','Motor vs OptiTrack'); hold on
% % plot(lags_ms, xcorr(dc(ang),tfbg,max_lag_samp,'normalized'), 'b',                   'LineWidth',1.5,'DisplayName','Motor vs FBGS');
% % plot(lags_ms, xcorr(dc(tmoc),tfbg,max_lag_samp,'normalized'),'r--',                 'LineWidth',1.5,'DisplayName','OptiTrack vs FBGS');
% % xline(lag_MO,'Color',[0.2 0.6 0.2],'LineStyle',':','LineWidth',1.2,'HandleVisibility','off')
% % xline(lag_MF,'b:','LineWidth',1.2,'HandleVisibility','off')
% % xline(lag_OF,'r:','LineWidth',1.2,'HandleVisibility','off')
% % xline(0,'k-','LineWidth',0.8,'HandleVisibility','off')
% % legend; grid on
% % xlabel('Lag [ms]'); ylabel('Corr. coefficient')
% % title(sprintf('Motor->MoCap %+.1f ms | Motor->FBGS %+.1f ms | MoCap->FBGS %+.1f ms', ...
% %               lag_MO, lag_MF, lag_OF))
% % 
% % savefig(fig, fullfile(saving_fig_folder,'temporal_sync'))
% % saveas( fig, fullfile(saving_fig_folder,'temporal_sync.png'))
% % end
% % 
% % %────────────────────────────────────────────────────────────────────────
% % function [lag_ms, peak_r] = peak_lag(a, b, max_lag, fs)
% % 
% %     [r, lags] = xcorr(a-mean(a), b-mean(b), max_lag, 'normalized');
% %     [~, idx]  = max(abs(r));
% %     d = 0;
% %     if idx > 1 && idx < length(r)
% %         denom = r(idx-1) - 2*r(idx) + r(idx+1);
% %         if abs(denom) > 1e-10
% %             d = 0.5*(r(idx-1) - r(idx+1)) / denom;
% %         end
% %     end
% %     lag_ms = (lags(idx) + d) / fs * 1000;
% %     peak_r = r(idx);
% % end
% % % function sync_results = check_temporal_sync(sampling_time, ...
% % %         interp_angles, interp_tensions, interp_base_wrench, ...
% % %         interp_rel_kinematics_disks, interp_fbgs_shapes, ...
% % %         FBGS_tip_index, saving_fig_folder)
% % % %CHECK_TEMPORAL_SYNC  Quantify temporal alignment between sensor streams.
% % % %
% % % %   Called from process_data.m after all signals have been interpolated
% % % %   onto the common 100 Hz grid.
% % % %
% % % %   Cross-correlations are computed after internal upsampling to 500 Hz
% % % %   to achieve ~2 ms lag resolution.  For each pair the peak lag and
% % % %   peak normalised correlation coefficient are reported.
% % % %
% % % %   A summary figure is generated: normalised signals are overlaid with
% % % %   vertical dashed lines placed at the peaks of the OptiTrack tip signal,
% % % %   making temporal alignment (or misalignment) visually apparent.
% % % %
% % % %   Inputs
% % % %     sampling_time                N×1  common time vector [s]
% % % %     interp_angles                N×4  motor shaft angles [rad]
% % % %     interp_tensions              N×4  cable tensions [N]  (+x +y -x -y)
% % % %     interp_base_wrench           N×6  ATI-FT wrench [N Nm]
% % % %     interp_rel_kinematics_disks  N×6×5  MoCap disk kinematics
% % % %     interp_fbgs_shapes           3×Nf×N  FBGS shape array
% % % %     FBGS_tip_index               scalar arc-length index of the FBGS tip
% % % %     saving_fig_folder            char/string path for saving figures
% % % %
% % % %   Output
% % % %     sync_results  struct
% % % %       .pairs      cell array of pair names
% % % %       .lag_ms     lag at peak correlation [ms]  (positive = B leads A)
% % % %       .peak_r     peak normalised correlation coefficient
% % % 
% % % %% ── Extract signals ──────────────────────────────────────────────────
% % % fs_in  = 1 / median(diff(sampling_time));   % native grid rate (≈100 Hz)
% % % fs_up  = 500;                               % upsample rate for xcorr
% % % max_lag_ms = 50;                            % search window ±50 ms
% % % 
% % % tip_mocap = interp_rel_kinematics_disks(:, 4:6, 5);          % N×3  [m]
% % % tip_fbgs  = squeeze(interp_fbgs_shapes(:, FBGS_tip_index, :))';  % N×3
% % % 
% % % ang_x     = interp_angles(:, 1);           % motor +x  [rad]
% % % ang_y     = interp_angles(:, 2);           % motor +y  [rad]
% % % tens_px   = interp_tensions(:, 1);         % tension +x [N]
% % % tens_py   = interp_tensions(:, 2);         % tension +y [N]
% % % tens_nx   = interp_tensions(:, 3);         % tension -x [N]
% % % tens_ny   = interp_tensions(:, 4);         % tension -y [N]
% % % net_fx    = tens_px - tens_nx;             % net cable force x
% % % net_fy    = tens_py - tens_ny;             % net cable force y
% % % ati_fx    = interp_base_wrench(:, 1);      % ATI Fx [N]
% % % ati_fy    = interp_base_wrench(:, 2);      % ATI Fy [N]
% % % 
% % % %% ── Upsample to fs_up for sub-sample lag resolution ─────────────────
% % % t_up = (sampling_time(1) : 1/fs_up : sampling_time(end))';
% % % 
% % % up = @(x) interp1(sampling_time, x, t_up, 'pchip');
% % % 
% % % tip_mocap_u = [up(tip_mocap(:,1))  up(tip_mocap(:,2))  up(tip_mocap(:,3))];
% % % tip_fbgs_u  = [up(tip_fbgs(:,1))   up(tip_fbgs(:,2))   up(tip_fbgs(:,3))];
% % % ang_x_u     = up(ang_x);
% % % ang_y_u     = up(ang_y);
% % % net_fx_u    = up(net_fx);
% % % net_fy_u    = up(net_fy);
% % % ati_fx_u    = up(ati_fx);
% % % ati_fy_u    = up(ati_fy);
% % % 
% % % %% ── Cross-correlation pairs ──────────────────────────────────────────
% % % pairs = {
% % %     'OptiTrack tip px  vs  FBGS tip px',  tip_mocap_u(:,1),  tip_fbgs_u(:,1);
% % %     'OptiTrack tip py  vs  FBGS tip py',  tip_mocap_u(:,2),  tip_fbgs_u(:,2);
% % %     'OptiTrack tip pz  vs  FBGS tip pz',  tip_mocap_u(:,3),  tip_fbgs_u(:,3);
% % %     'Motor +x angle    vs  net cable Fx', ang_x_u,           net_fx_u;
% % %     'Motor +y angle    vs  net cable Fy', ang_y_u,           net_fy_u;
% % %     'Net cable Fx      vs  ATI-FT Fx',    net_fx_u,          ati_fx_u;
% % %     'Net cable Fy      vs  ATI-FT Fy',    net_fy_u,          ati_fy_u;
% % % };
% % % 
% % % n_pairs  = size(pairs, 1);
% % % lag_ms   = zeros(n_pairs, 1);
% % % peak_r   = zeros(n_pairs, 1);
% % % max_lag_samp = round(max_lag_ms * 1e-3 * fs_up);
% % % 
% % % for k = 1:n_pairs
% % %     [lag_ms(k), peak_r(k)] = xcorr_peak(pairs{k,2}, pairs{k,3}, ...
% % %                                           max_lag_samp, fs_up);
% % % end
% % % 
% % % %% ── Print summary table ──────────────────────────────────────────────
% % % fprintf('\n===== Temporal synchronisation check =====\n')
% % % fprintf('%-45s  %9s  %8s\n', 'Pair', 'Lag [ms]', 'Peak r')
% % % fprintf('%s\n', repmat('-', 1, 67))
% % % for k = 1:n_pairs
% % %     fprintf('%-45s  %+9.2f  %8.3f\n', pairs{k,1}, lag_ms(k), peak_r(k))
% % % end
% % % fprintf('\nConvention: positive lag = signal B leads signal A\n')
% % % fprintf('OS jitter bound: 1–2 ms\n\n')
% % % 
% % % sync_results.pairs  = pairs(:, 1);
% % % sync_results.lag_ms = lag_ms;
% % % sync_results.peak_r = peak_r;
% % % 
% % % %% ── Figure: signal overlay with peak alignment lines ────────────────
% % % %   Normalise each signal to zero-mean, unit range for visual comparison
% % % norm_sig = @(x) (x - mean(x,'omitnan')) / (max(x,[],'omitnan') - min(x,[],'omitnan'));
% % % 
% % % sig_mocap_px = norm_sig(tip_mocap(:, 1));
% % % sig_fbgs_px  = norm_sig(tip_fbgs(:,  1));
% % % sig_ang_x    = norm_sig(ang_x);
% % % sig_net_fx   = norm_sig(net_fx);
% % % sig_ati_fx   = norm_sig(ati_fx);
% % % 
% % % %   Detect peaks of the reference signal (OptiTrack tip px)
% % % %   using a minimum prominence to avoid noise peaks
% % % sig_range = max(sig_mocap_px) - min(sig_mocap_px);
% % % [~, peak_locs] = findpeaks(sig_mocap_px, sampling_time, ...
% % %                             'MinPeakProminence', 0.3 * sig_range, ...
% % %                             'MinPeakDistance',   0.5);   % at least 0.5 s apart
% % % 
% % % fig = figure('Name', 'Temporal synchronisation – signal overlay');
% % % set(fig, 'Position', [100 100 900 600])
% % % 
% % % ax1 = subplot(3, 1, 1);
% % % plot(sampling_time, sig_mocap_px, 'b',  'LineWidth', 1.5, 'DisplayName', 'OptiTrack tip p_x'); hold on
% % % plot(sampling_time, sig_fbgs_px,  'r--','LineWidth', 1.5, 'DisplayName', 'FBGS tip p_x')
% % % for pk = peak_locs'
% % %     xline(pk, 'k:', 'LineWidth', 0.8, 'HandleVisibility', 'off')
% % % end
% % % ylabel('Norm. amplitude'); grid on
% % % legend('Location', 'northeast', 'FontSize', 8)
% % % title('Kinematic streams')
% % % 
% % % ax2 = subplot(3, 1, 2);
% % % plot(sampling_time, sig_ang_x,   'b',  'LineWidth', 1.5, 'DisplayName', 'Motor +x angle'); hold on
% % % plot(sampling_time, sig_net_fx,  'r--','LineWidth', 1.5, 'DisplayName', 'Net cable force F_x')
% % % for pk = peak_locs'
% % %     xline(pk, 'k:', 'LineWidth', 0.8, 'HandleVisibility', 'off')
% % % end
% % % ylabel('Norm. amplitude'); grid on
% % % legend('Location', 'northeast', 'FontSize', 8)
% % % title('Actuation stream')
% % % 
% % % ax3 = subplot(3, 1, 3);
% % % plot(sampling_time, sig_net_fx,  'b',  'LineWidth', 1.5, 'DisplayName', 'Net cable force F_x'); hold on
% % % plot(sampling_time, sig_ati_fx,  'r--','LineWidth', 1.5, 'DisplayName', 'ATI-FT F_x')
% % % for pk = peak_locs'
% % %     xline(pk, 'k:', 'LineWidth', 0.8, 'HandleVisibility', 'off')
% % % end
% % % ylabel('Norm. amplitude'); xlabel('Time [s]'); grid on
% % % legend('Location', 'northeast', 'FontSize', 8)
% % % title('Force streams')
% % % 
% % % linkaxes([ax1 ax2 ax3], 'x')
% % % 
% % % sgtitle(sprintf(['Sensor temporal alignment\n' ...
% % %     'OptiTrack–FBGS lag: %.1f ms  |  ' ...
% % %     'Motor–Tension lag: %.1f ms  |  ' ...
% % %     'Tension–ATI lag: %.1f ms'], ...
% % %     lag_ms(1), lag_ms(4), lag_ms(6)), 'FontSize', 10)
% % % 
% % % savefig(fig, fullfile(saving_fig_folder, 'temporal_sync'))
% % % saveas( fig, fullfile(saving_fig_folder, 'temporal_sync.png'))
% % % 
% % % end
% % % 
% % % %% ── Helper: cross-correlation peak lag ──────────────────────────────
% % % function [lag_ms, peak_r] = xcorr_peak(sig_a, sig_b, max_lag_samp, fs)
% % % %   Normalised cross-correlation; positive lag = B leads A.
% % %     a = (sig_a - mean(sig_a,'omitnan')) / std(sig_a,'omitnan');
% % %     b = (sig_b - mean(sig_b,'omitnan')) / std(sig_b,'omitnan');
% % %     [r, lags] = xcorr(a, b, max_lag_samp, 'normalized');
% % %     [~, idx]  = max(abs(r));
% % %     %   Parabolic sub-sample refinement
% % %     if idx > 1 && idx < length(r)
% % %         alpha = r(idx-1); beta = r(idx); gamma = r(idx+1);
% % %         denom = alpha - 2*beta + gamma;
% % %         if abs(denom) > 1e-10
% % %             delta = 0.5 * (alpha - gamma) / denom;
% % %         else
% % %             delta = 0;
% % %         end
% % %     else
% % %         delta = 0;
% % %     end
% % %     lag_ms = (lags(idx) + delta) / fs * 1000;
% % %     peak_r = r(idx);
% % % end