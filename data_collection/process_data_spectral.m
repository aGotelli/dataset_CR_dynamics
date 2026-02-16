% Spectral-only postprocessing (non-intrusive):
% - Loads the same signals as process_data.m
% - Computes and plots PSD for motors, cable tensions, ATI FT, Vicon disks
% - Saves figures in a "processed" subfolder next to the data
%
% Note: does not modify or filter data. Use the plots to choose cutoff Hz.

close all; clear; clc;

%% ====== PATHS / SETTINGS ======
folder = fullfile("dataCollectionPack","20260127","circle_angle_150_speed_3");

fmax_plot   = 50;    % Limit x-axis for PSD plots [Hz]
plot_disk_nums = 5; % Which Vicon disks to inspect (indices)

% Interpolation/export settings (aligned to process_data.m)
samplingHz = 100;          % uniform sampling frequency for export
plot_interpolation = true; % show interpolation check plots

% --- Cutoff selection parameters (SNR / knee style) ---
fc_min_cap  = 3;     % Hz, lower bound for auto cutoff
fc_max_cap  = 15;    % Hz, upper bound for auto cutoff (target 5–6 Hz typical)
search_fmax = 15;    % Hz, only look for useful content below this
floor_low   = 20;    % Hz, start of band to estimate noise floor
floor_high  = 45;    % Hz, end of band for noise floor (<= Nyquist)
snr_thresh_db = 12;  % dB above floor to call it “useful”
margin_hz   = 0.5;   % Hz, add on top of useful band

% Oscillation peak detection (ATI, opzionale diagnostica)
peak_fmax       = 30; % Hz, cerca picchi di oscillazione entro questa banda
peak_prom_db    = 6;  % dB, prominenza minima per dichiarare un picco
peak_n_max      = 3;  % quanti picchi riportare
apply_ati_notch = true; % se true, rimuove le righe ATI prima del Butterworth
notch_bw_hz     = 1.0;  % larghezza stopband totale per ogni notch (Hz)

%% ====== LOAD DATA (unchanged from process_data.m) ======
motor = readtable(fullfile(folder, "datasequence_circle_radius_150p0.csv"));

mk_1_negx = readtable(fullfile(folder, "dataMark10_1_-x.csv"));
mk_1_x    = readtable(fullfile(folder, "dataMark10_1_x.csv"));
mk_2_negy = readtable(fullfile(folder, "dataMark10_2_-y.csv"));
mk_2_y    = readtable(fullfile(folder, "dataMark10_2_y.csv"));

ati = readtable(fullfile(folder, "dataATIFT.csv"));

N_frames_static_begin = 10; % how many frames to use to compute relative pose for Vicon
filename_vicon = fullfile(folder, "dataVicon.csv");
[N_disks, timestamp_vicon, rel_kinematics_disks] = data_vicon(filename_vicon, N_frames_static_begin);

%% ====== PREPARE SIGNALS ======
time_actuators = motor.timestamp;
measured_angles   = [motor.rel_angle1_rad, motor.rel_angle2_rad, motor.rel_angle3_rad, motor.rel_angle4_rad];

time_cables = cell(1,4);
cable_tensions  = cell(1,4);
time_cables{1} = mk_1_x.timestamp;       cable_tensions{1} = mk_1_x.tension_N_;
time_cables{2} = mk_2_y.timestamp;       cable_tensions{2} = mk_2_y.tension_N_;
time_cables{3} = mk_1_negx.timestamp;    cable_tensions{3} = mk_1_negx.tension_N_;
time_cables{4} = mk_2_negy.timestamp;    cable_tensions{4} = mk_2_negy.tension_N_;

tA = ati.timestamp;
ATI_F = [ati.Fx_N_, ati.Fy_N_, ati.Fz_N_];
ATI_T = [ati.Tx_Nm_, ati.Ty_Nm_, ati.Tz_Nm_];

%% ====== Sampling frequencies (median dt) ======
fs_motor = 1/median(diff(time_actuators));
fs_cables = zeros(1,4);
for i=1:4
    fs_cables(i) = 1/median(diff(time_cables{i}));
end
fs_ati = 1/median(diff(tA));
fs_vicon = 1/median(diff(timestamp_vicon));

%% ====== PSD: single overview figure ======
overview_fig = figure('Name','PSD overview');

% 1) Motori
[f_m, psd_m] = spectral_analysis(time_actuators, measured_angles, 'Plot', false);
subplot(2,2,1);
plot(f_m, psd_m, 'LineWidth',1.2);
grid on; xlim([0 fmax_plot]);
title('Motor angles'); xlabel('Frequency [Hz]'); ylabel('PSD [dB/Hz]');
legend({'M1','M2','M3','M4'}, 'Location','northeast');

% 2) Tensioni
f_c = cell(1,4); psd_c = cell(1,4);
for i = 1:4
    [f_c{i}, psd_c{i}] = spectral_analysis(time_cables{i}, cable_tensions{i}, 'Plot', false);
end
subplot(2,2,2);
hold on;
colors = lines(4);
for i = 1:4
    plot(f_c{i}, psd_c{i}, 'LineWidth',1.2, 'Color', colors(i,:));
end
hold off;
grid on; xlim([0 fmax_plot]);
title('Cable tensions'); xlabel('Frequency [Hz]'); ylabel('PSD [dB/Hz]');
legend({'M1','M2','M3','M4'}, 'Location','northeast');

% 3) ATI
[f_a, psd_a] = spectral_analysis(tA, [ATI_F ATI_T], 'Plot', false);
ati_peak_list = find_top_peaks(f_a, psd_a, peak_fmax, peak_prom_db, peak_n_max);
subplot(2,2,3);
plot(f_a, psd_a, 'LineWidth',1.2);
grid on; xlim([0 fmax_plot]);
title('ATI FT'); xlabel('Frequency [Hz]'); ylabel('PSD [dB/Hz]');
legend({'Fx','Fy','Fz','Tx','Ty','Tz'}, 'Location','northeast');

% 4) Vicon (first valid disk in plot_disk_nums)
disk_idx = plot_disk_nums(find(plot_disk_nums <= N_disks, 1));
if isempty(disk_idx)
    warning('No valid disk index found to plot PSD. Using disk 1 as fallback.');
    disk_idx = 1;
end
if ~isempty(disk_idx)
    sig = rel_kinematics_disks(:, :, disk_idx);
    [f_v, psd_v] = spectral_analysis(timestamp_vicon, sig, 'Plot', false);
    subplot(2,2,4);
    plot(f_v, psd_v, 'LineWidth',1.2);
    grid on; xlim([0 fmax_plot]);
    title(sprintf('Vicon disk %d', disk_idx)); xlabel('Frequency [Hz]'); ylabel('PSD [dB/Hz]');
    legend({'X','Y','Z','rotX','rotY','rotZ'}, 'Location','northeast');
end

% ====== Useful-band estimates (SNR-based) ======
useful_motor  = useful_band_snr(f_m, psd_m, search_fmax, floor_low, floor_high, snr_thresh_db, margin_hz);

% Cables: max across four spectra (ognuna con griglia propria)
useful_cables = NaN;
if ~isempty(f_c)
    peaks = NaN(1, numel(f_c));
    for i=1:numel(f_c)
        peaks(i) = useful_band_snr(f_c{i}, psd_c{i}, search_fmax, floor_low, floor_high, snr_thresh_db, margin_hz);
    end
    useful_cables = max(peaks, [], 'omitnan');
end

% Vicon: max across 6 channels del disco scelto
useful_vicon = NaN;
if ~isempty(disk_idx)
    useful_vicon = useful_band_snr(f_v, psd_v, search_fmax, floor_low, floor_high, snr_thresh_db, margin_hz);
end

% Compute fc from vicon-motors-cables
valid_useful = [useful_motor, useful_cables, useful_vicon];%,useful_ati];
suggested_fc = max(valid_useful);

fc_clamped = max(fc_min_cap, min(fc_max_cap, suggested_fc));

annotation_text = sprintf(['f_s motor: %.1f Hz | f_s cables: %.1f Hz | f_s Vicon: %.1f Hz | f_s ATI: %.1f Hz\n' ...
    'useful motor: %.3f Hz | useful cables: %.3f Hz | useful Vicon: %.3f Hz | fc used: %.2f Hz'], ...
    fs_motor, mean(fs_cables), fs_vicon, fs_ati, ...
    useful_motor, useful_cables, useful_vicon, fc_clamped);
annotation(overview_fig,'textbox',[0.15 0.0001 0.7 0.08],...
    'String',annotation_text,'FitBoxToText','on','EdgeColor','none','HorizontalAlignment','center');

sgtitle('PSD overview', 'FontSize', 15, 'FontWeight', 'bold');
set(findall(overview_fig,'-property','FontSize'),'FontSize',14);


% ATI: banda utile (raw) solo informativa
useful_ati = useful_band_snr(f_a, psd_a, search_fmax, floor_low, floor_high, snr_thresh_db, margin_hz);

% % % % Notch ATI (riuso per filtraggio e per fc_with_ati)
% % % ATI_F_notch = ATI_F;
% % % ATI_T_notch = ATI_T;
% % % if apply_ati_notch && ~isempty(ati_peak_list)
% % %     ATI_F_notch = apply_notches(tA, ATI_F, ati_peak_list, notch_bw_hz);
% % %     ATI_T_notch = apply_notches(tA, ATI_T, ati_peak_list, notch_bw_hz);
% % % end
% % % [f_a_notch, psd_a_notch] = spectral_analysis(tA, [ATI_F_notch ATI_T_notch], 'Plot', false);
% % % useful_ati_notch = useful_band_snr(f_a_notch, psd_a_notch, search_fmax, floor_low, floor_high, snr_thresh_db, margin_hz);
% % % % è ancora più alta se sopprimo i punti

%% ====== SAVE FIGURE ======
saving_folder = fullfile(folder, "processed");
if ~exist(saving_folder, "dir"); mkdir(saving_folder); end
filename_png = fullfile(saving_folder, "psd_overview.png");
set(overview_fig, "PaperPositionMode", "auto");
print(overview_fig, filename_png, "-dpng", "-r300");

disp("Spectral analysis completed. Figure saved in 'processed/psd_overview.png'.");

% Save fundamentals and sampling info
info_txt = fullfile(saving_folder, "spectral_info.txt");
fid = fopen(info_txt, "w");
fprintf(fid, "Sampling Hz: motor=%.3f, cables=%.3f %.3f %.3f %.3f, ATI=%.3f, Vicon=%.3f\n", ...
    fs_motor, fs_cables, fs_ati, fs_vicon);
fprintf(fid, "Useful max freq (search<%.1f): motor=%.5f, cables=%.5f, vicon=%.5f, ati=%.5f\n", ...
    search_fmax, useful_motor, useful_cables, useful_vicon, useful_ati);
fprintf(fid, "Fc (pre-clamp): %.3f \n", suggested_fc);
fprintf(fid, "Fc used after clamp [%.1f, %.1f]: %.3f Hz\n", fc_min_cap, fc_max_cap, fc_clamped);

fclose(fid);

% Console summary
fprintf('\n--- SNR-based band summary ---\n');
fprintf('fs: motor %.1f Hz | cables ~%.1f Hz | Vicon %.1f Hz | ATI %.1f Hz\n', fs_motor, mean(fs_cables), fs_vicon, fs_ati);
fprintf('useful freq: motor %.2f | cables %.2f | Vicon %.2f | ATI raw %.2f (Hz)\n', useful_motor, useful_cables, useful_vicon, useful_ati);
fprintf('fc suggested %.2f [limits %.1f..%.1f]\n\n', ...
   suggested_fc, fc_min_cap, fc_max_cap);

%% ====== Butterworth filtering with single fc (no saving, just plots) ======
butterOrder = 4;

% Motors
measured_angles_f = zeros(size(measured_angles));
for it = 1:4
    measured_angles_f(:,it)  = butter_filtfilt(time_actuators, measured_angles(:,it), fc_clamped, butterOrder);
end

% Cables
cable_tensions_f = cell(1,4);
cable_tensions_f2 = cell(1,4);
for it = 1:4
    cable_tensions_f{it} = butter_filtfilt(time_cables{it}, cable_tensions{it}, fc_clamped, butterOrder);
end

% ATI
ATI_F_f = zeros(size(ATI_F)); ATI_T_f = zeros(size(ATI_T));
ATI_F_notch = ATI_F;
ATI_T_notch = ATI_T;
for k = 1:3
    ATI_F_f(:,k)  = butter_filtfilt(tA, ATI_F_notch(:,k), fc_clamped, butterOrder);
    ATI_T_f(:,k)  = butter_filtfilt(tA, ATI_T_notch(:,k), fc_clamped, butterOrder);
end

% Vicon (all disks filtered for consistency)
rel_kinematics_disks_f = zeros(size(rel_kinematics_disks));
for it=1:N_disks
    for k=1:6
        rel_kinematics_disks_f(:, k, it) = butter_filtfilt(timestamp_vicon, rel_kinematics_disks(:, k, it), fc_clamped, butterOrder);
    end
end

% Clearer plots: one figure per sensor group, raw dashed vs filtered solid

% Motors: one subplot per motor
fig_m = figure('Name',sprintf('Motor Angles raw vs filtered (fc=%.2f Hz)', fc_clamped));
for it = 1:4
    subplot(4,1,it)
    plot(time_actuators, measured_angles(:,it),   "b--", "LineWidth", 0.9); hold on
    plot(time_actuators, measured_angles_f(:,it), "r",  "LineWidth", 1.4);
    ylabel("Angle [rad]"); grid on;
    title("Motor " + it)
    if it == 4, xlabel("Time (s)"); end
end
legend(fig_m.CurrentAxes, {'Raw','Filtered'});

% Cable tensions: one subplot per motor
fig_c = figure('Name',sprintf('Cable tensions raw vs filtered (fc=%.2f Hz)', fc_clamped));
for it = 1:4
    subplot(4,1,it)
    plot(time_cables{it}, cable_tensions{it},   "b--", "LineWidth", 0.9); hold on
    plot(time_cables{it}, cable_tensions_f{it}, "r",  "LineWidth", 1.4);
    ylabel("Tension [N]"); grid on;
    title("Motor " + it)
    if it == 4, xlabel("Time (s)"); end
end
legend(fig_c.CurrentAxes, {'Raw','Filtered'});

% ATI: raw vs filtered, both Forces and Torques
fig_ati = figure('Name',sprintf('ATI raw vs filtered (fc=%.2f Hz)', fc_clamped));
subplot(2,1,1); hold on;
plot(tA, ATI_T(:,1), 'b--', 'LineWidth', 0.8);
plot(tA, ATI_T_f(:,1), 'b',  'LineWidth', 1.2);
plot(tA, ATI_T(:,2), 'r--', 'LineWidth', 0.8);
plot(tA, ATI_T_f(:,2), 'r',  'LineWidth', 1.2);
plot(tA, ATI_T(:,3), 'g--', 'LineWidth', 0.8);
plot(tA, ATI_T_f(:,3), 'g',  'LineWidth', 1.2);
grid on; ylabel("Torque [Nm]"); title('ATI Torques raw vs filtered');
legend({'Tx raw','Tx filt','Ty raw','Ty filt','Tz raw','Tz filt'}, 'Location','best');

subplot(2,1,2); hold on;
plot(tA, ATI_F(:,1), 'b--', 'LineWidth', 0.8);
plot(tA, ATI_F_f(:,1), 'b',  'LineWidth', 1.2);
plot(tA, ATI_F(:,2), 'r--', 'LineWidth', 0.8);
plot(tA, ATI_F_f(:,2), 'r',  'LineWidth', 1.2);
plot(tA, ATI_F(:,3), 'g--', 'LineWidth', 0.8);
plot(tA, ATI_F_f(:,3), 'g',  'LineWidth', 1.2);
grid on; ylabel("Force [N]"); xlabel("Time (s)");
title('ATI Forces raw vs filtered');
legend({'Fx raw','Fx filt','Fy raw','Fy filt','Fz raw','Fz filt'}, 'Location','best');

% ATI: confronto notch+butter vs butter-only (diagnostico)
% remove second comparison plot (single fc only)

% Vicon: positions and rotations for chosen disk, raw vs filtered
fig_v = figure("Name",sprintf("Vicon disk %d raw vs filtered (fc=%.2f Hz)", disk_idx, fc_clamped));
xyz_XYZ = rel_kinematics_disks(:, :, disk_idx);
xyz_XYZ_f = rel_kinematics_disks_f(:, :, disk_idx);
for it = 1:3
    subplot(3,2,it) % positions X,Y,Z
    plot(timestamp_vicon, xyz_XYZ(:, it), "b--", "LineWidth", 0.8); hold on
    plot(timestamp_vicon, xyz_XYZ_f(:, it), "r", "LineWidth", 1.2)
    ylabel("Pos [m]"); grid on;
    if it == 3, xlabel("Time [s]"); end
    axis_chars = 'XYZ';
    title(["Disk " + disk_idx, "Pos " + axis_chars(it)]);
end
for it = 1:3
    subplot(3,2,3+it) % rotations rotX, rotY, rotZ
    plot(timestamp_vicon, xyz_XYZ(:, 3 + it), "b--", "LineWidth", 0.8); hold on
    plot(timestamp_vicon, xyz_XYZ_f(:, 3 + it), "r", "LineWidth", 1.2)
    ylabel("Rot [rad]"); grid on;
    if it == 3, xlabel("Time [s]"); end
    axis_chars = 'XYZ';
    title(["Disk " + disk_idx, "Rot " + axis_chars(it)]);
end
legend(fig_v.CurrentAxes, {'Raw','Filtered'});

%% ====== Interpolate on uniform grid and save (matching process_data.m) ======

% Align times to motor start
time_start_motors = time_actuators(1);
rel_time_motors   = time_actuators - time_start_motors;
rel_time_cables   = cellfun(@(t) t - time_start_motors, time_cables, 'UniformOutput', false);
rel_time_ATI      = tA - time_start_motors;
rel_time_vicon    = timestamp_vicon - time_start_motors;

time_end_motors = rel_time_motors(end);
N_samples = floor(samplingHz * time_end_motors);
dt        = 1/samplingHz;
sampling_time = (0:dt:dt*(N_samples-1))';

% Interpolate filtered signals
interp_angles   = zeros(N_samples, 4);
interp_tensions = zeros(N_samples, 4);
for it = 1:4
    interp_angles(:, it)   = interp1(rel_time_motors, measured_angles_f(:,it), sampling_time)';
    interp_tensions(:, it) = interp1(rel_time_cables{it}, cable_tensions_f{it}, sampling_time)';
end

ATI_FT_f = [ATI_F_f ATI_T_f]; % 6 columns: Fx Fy Fz Tx Ty Tz
interp_base_wrench = zeros(N_samples, 6);
for it = 1:6
    interp_base_wrench(:, it) = interp1(rel_time_ATI, ATI_FT_f(:, it), sampling_time)';
end

interp_rel_kinematics_disks = zeros(N_samples, 6, N_disks);
for it = 1:N_disks
    for k = 1:6
        interp_rel_kinematics_disks(:, k, it) = interp1(rel_time_vicon, rel_kinematics_disks_f(:, k, it), sampling_time);
    end
end

% Optional diagnostic plots
if plot_interpolation
    % Angles
    fig_interp_m = figure('Name','Actuators Angles (uniform grid)');
    for it = 1:4
        subplot(4,1,it)
        plot(rel_time_motors, measured_angles_f(:,it), 'b', 'LineWidth', 1.5,'DisplayName','raw'); hold on
        plot(sampling_time, interp_angles(:,it), 'or', 'MarkerSize', 3,'DisplayName','interp-bw-filt');
        ylabel('Angle [rad]'); grid on; title(['Actuator ' num2str(it)]);
        if it == 4, xlabel('Time [s]'); end
    end

    % Tensions
    fig_interp_c = figure('Name','Cables Tensions (uniform grid)');
    for it = 1:4
        subplot(4,1,it)
        plot(rel_time_cables{it}, cable_tensions_f{it}, 'b', 'LineWidth', 1.5,'DisplayName','raw'); hold on
        plot(sampling_time, interp_tensions(:,it), 'or', 'MarkerSize', 3,'DisplayName','interp-bw-filt');
        ylabel('Tension [N]'); grid on; title(['Actuator ' num2str(it)]);
        if it == 4, xlabel('Time [s]'); end
    end

    % ATI
    fig_interp_ati = figure('Name','ATI FT (uniform grid)');
    for it = 1:3
        subplot(3,2,2*it-1)
        plot(rel_time_ATI, ATI_F_f(:,it), 'b', 'LineWidth', 1.2,'DisplayName','raw'); hold on
        plot(sampling_time, interp_base_wrench(:,it), 'or', 'MarkerSize', 3,'DisplayName','interp-bw-filt');
        ylabel('Force [N]'); grid on;
        if it == 3, xlabel('Time [s]'); end
    end
    for it = 1:3
        subplot(3,2,2*it)
        plot(rel_time_ATI, ATI_T_f(:,it), 'b', 'LineWidth', 1.2,'DisplayName','raw'); hold on
        plot(sampling_time, interp_base_wrench(:,3+it), 'or', 'MarkerSize', 3,'DisplayName','interp-bw-filt');
        ylabel('Torque [Nm]'); grid on;
        if it == 3, xlabel('Time [s]'); end
    end

    % Vicon (selected disk)
    fig_interp_v = figure('Name',sprintf('Vicon disk %d (uniform grid)', disk_idx));
    xyz_XYZ_f = rel_kinematics_disks_f(:, :, disk_idx);
    interp_xyz = interp_rel_kinematics_disks(:, :, disk_idx);
    for it = 1:3
        subplot(3,2,2*it-1)
        plot(rel_time_vicon, xyz_XYZ_f(:,3+it), 'b', 'LineWidth', 1.2,'DisplayName','raw'); hold on
        plot(sampling_time, interp_xyz(:,3+it), 'or', 'MarkerSize', 3,'DisplayName','interp-bw-filt');
        ylabel('Euler [rad]'); grid on; if it==3, xlabel('Time [s]'); end
    end
    for it = 1:3
        subplot(3,2,2*it)
        plot(rel_time_vicon, xyz_XYZ_f(:,it), 'b', 'LineWidth', 1.2,'DisplayName','raw'); hold on
        plot(sampling_time, interp_xyz(:,it), 'or', 'MarkerSize', 3,'DisplayName','interp-bw-filt');
        ylabel('Pos [m]'); grid on; if it==3, xlabel('Time [s]'); end
    end
end
legend
% Save interpolated data with _psd suffix to distinguish
interp_time_angles       = [sampling_time interp_angles];
interp_time_tensions     = [sampling_time interp_tensions];
interp_time_base_wrench  = [sampling_time interp_base_wrench];
interp_time_vicon_frames = zeros(N_samples, 7, N_disks);
for it = 1:N_disks
    interp_time_vicon_frames(:, :, it) = [sampling_time interp_rel_kinematics_disks(:, :, it)];
end

writematrix(interp_time_angles,      fullfile(saving_folder, 'angles_psd.csv'));
writematrix(interp_time_tensions,    fullfile(saving_folder, 'cable_tensions_psd.csv'));
writematrix(interp_time_base_wrench, fullfile(saving_folder, 'base_wrench_psd.csv'));
writematrix(interp_time_vicon_frames,fullfile(saving_folder, 'vicon_frames_psd.csv'));

%% ====== Helper: Butterworth zero-phase ======
function y = butter_filtfilt(t, x, fc, n)
Fs = 1/median(diff(t));
Wn = fc/(Fs/2);
if Wn >= 1
    y = x; return; % cutoff above wN: do nothing
end
[b,a] = butter(n, Wn, "low");
y = filtfilt(b,a, x);
end

%% ====== Helper: apply multiple notch filters to matrix signals ======
function Xout = apply_notches(t, Xin, peak_list, bw_hz)
Fs = 1/median(diff(t));
Xout = Xin;
for f0 = peak_list(:).'
    if isnan(f0) || f0 <= 0, continue; end
    fl = max(0.1, f0 - bw_hz/2);
    fh = f0 + bw_hz/2;
    if fh >= Fs/2, fh = Fs/2 - 0.01; end
    if fl >= fh, continue; end
    d = designfilt('bandstopiir','FilterOrder',2, ...
        'HalfPowerFrequency1',fl,'HalfPowerFrequency2',fh, ...
        'DesignMethod','butter','SampleRate',Fs);
    Xout = filtfilt(d, Xout);
end
end

%% ====== Helper: useful band by SNR threshold ======
% Returns the max useful frequency (plus margin) across all columns of psd_db_matrix.
% For each channel:
%   - estimate noise floor as median PSD in [floor_low, floor_high]
%   - threshold = floor + snr_thresh_db
%   - find last freq < fmax_search above threshold
%   - add margin_hz
function f_useful = useful_band_snr(f, psd_db_matrix, fmax_search, floor_low, floor_high, snr_thresh_db, margin_hz)
f_useful = NaN;
if isempty(f) || isempty(psd_db_matrix)
    return;
end
if isvector(psd_db_matrix)
    psd_db_matrix = psd_db_matrix(:);
end

fmax_search = min(fmax_search, max(f));
peaks = NaN(1, size(psd_db_matrix,2));

for k = 1:size(psd_db_matrix,2)
    psd_db = psd_db_matrix(:,k);
    fh = min(floor_high, max(f));
    mask_floor = f >= floor_low & f <= fh;
    floor_db = median(psd_db(mask_floor), 'omitnan');
    thresh = floor_db + snr_thresh_db;

    mask_search = f > 0 & f <= fmax_search & ~isnan(psd_db);
    f_search = f(mask_search);
    psd_search = psd_db(mask_search);
    idx = find(psd_search > thresh, 1, 'last');
    if isempty(idx)
        peaks(k) = NaN;
    else
        peaks(k) = f_search(idx) + margin_hz;
    end
end

if any(~isnan(peaks)) % at least one value not NaN
    f_useful = max(peaks, [], 'omitnan');
end
end

%% ====== Helper: find top peaks (combined over columns) ======
function peaks = find_top_peaks(f, psd_db_matrix, fmax, prom_db, n_peaks)
peaks_all = [];
if isempty(f) || isempty(psd_db_matrix), peaks = []; return; end
mask = f > 0 & f <= fmax;
if ~any(mask), peaks = []; return; end
fmask = f(mask);
if isvector(psd_db_matrix)
    psd_db_matrix = psd_db_matrix(:);
end
for k = 1:size(psd_db_matrix,2)
    [pks, locs] = findpeaks(psd_db_matrix(mask,k), fmask, 'MinPeakProminence', prom_db, 'SortStr','descend');
    peaks_all = [peaks_all; locs(:)]; %#ok<AGROW>
end
peaks_all = unique(peaks_all);
peaks_all = sort(peaks_all, 'ascend');
peaks = peaks_all(1:min(numel(peaks_all)));
end
