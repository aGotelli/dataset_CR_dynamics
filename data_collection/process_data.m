
close all; clear; clc

%% 0) Parameters (sync only)
Fs_sync   = 200;    % common timeline + xcorr grid
fc_sync   = 20;     % Hz low-pass for sync features
filtOrder = 4;

% Map each Mark10 channel -> which motor derivative to use as reference
% Order matches mk10_1..mk10_4 below; change later if needed
forceMotorMap = [1 1 3 3];

% Choose which motor derivative to align ATI to (edit later)
atiMotorRef = 1;

%% 1) Load data
basePath = "dataCollectionPack\circle_150_v_1\";

datasequence = readtable(basePath + "datasequence__circle_radius_150p0.csv");

mk10_1 = readtable(basePath + "dataMark10_1_-x_.csv"); % channel 1
mk10_2 = readtable(basePath + "dataMark10_1_x_.csv");  % channel 2
mk10_3 = readtable(basePath + "dataMark10_2_-y_.csv"); % channel 3
mk10_4 = readtable(basePath + "dataMark10_2_y_.csv");  % channel 4

ATI = readtable(basePath + "dataATIFT_.csv");

%% 2) Global time base (same origin for all streams)
t_act = datasequence.timestamp;

t_m   = {mk10_1.timestamp, mk10_2.timestamp, mk10_3.timestamp, mk10_4.timestamp};
t_ati = ATI.timestamp;

t0 = min([t_act(1), t_m{1}(1), t_m{2}(1), t_m{3}(1), t_m{4}(1), t_ati(1)]);

t_act = t_act - t0;
for k = 1:4
    t_m{k} = t_m{k} - t0;
end
t_ati = t_ati - t0;

%% 3) Motor reference features: d(measured angle)/dt (filtered)
ang = [datasequence.rel_angle1_rad, datasequence.rel_angle2_rad, datasequence.rel_angle3_rad, datasequence.rel_angle4_rad];

ang_f = zeros(size(ang));
for i = 1:4
    ang_f(:,i) = butter_filtfilt(t_act, ang(:,i), fc_sync, filtOrder);
end

d_ang = gradient(ang_f)./gradient(t_act);   % rad/s (feature for sync)

%% 4) Mark10 forces (raw signals used for xcorr feature)
T = {mk10_1.tension_N_, mk10_2.tension_N_, mk10_3.tension_N_, mk10_4.tension_N_};

%% 5) Estimate offsets (dt) via cross-correlation and shift timestamps
dt_force = zeros(1,4);
t_m_sync = cell(1,4);

for k = 1:4
    refIdx = forceMotorMap(k);
    dt_force(k) = delay_xcorr(t_act, d_ang(:,refIdx), t_m{k}, T{k}, Fs_sync);
    t_m_sync{k} = t_m{k} + dt_force(k);
end

% ATI reference feature: hypot(Tx,Ty) then derivative
Tx = ATI.Tx_Nm_;
Ty = ATI.Ty_Nm_;
Txy = hypot(Tx, Ty);

Txy_f = butter_filtfilt(t_ati, Txy, fc_sync, filtOrder);
d_Txy = gradient(Txy_f)./gradient(t_ati);

dt_ati = delay_xcorr(t_act, d_ang(:,atiMotorRef), t_ati, d_Txy, Fs_sync);
t_ati_sync = t_ati + dt_ati;

fprintf("dt_force = [%.4f %.4f %.4f %.4f] s\n", dt_force);
fprintf("dt_ati   = %.4f s (aligned to motor %d)\n", dt_ati, atiMotorRef);

%% 6) Resample onto common 200 Hz timeline (interp1)
tStart = max([t_act(1), t_m_sync{1}(1), t_m_sync{2}(1), t_m_sync{3}(1), t_m_sync{4}(1), t_ati_sync(1)]);
tEnd   = min([t_act(end), t_m_sync{1}(end), t_m_sync{2}(end), t_m_sync{3}(end), t_m_sync{4}(end), t_ati_sync(end)]);
t_common = (tStart : 1/Fs_sync : tEnd)';

ang_sync = interp1(t_act, ang, t_common, 'pchip');

T_sync = zeros(numel(t_common), 4);
for k = 1:4
    T_sync(:,k) = interp1(t_m_sync{k}, T{k}, t_common, 'pchip');
end

ATI_T = [ATI.Tx_Nm_, ATI.Ty_Nm_, ATI.Tz_Nm_];
ATI_F = [ATI.Fx_N_,  ATI.Fy_N_,  ATI.Fz_N_];

ATI_T_sync = interp1(t_ati_sync, ATI_T, t_common, 'pchip');
ATI_F_sync = interp1(t_ati_sync, ATI_F, t_common, 'pchip');

%% 7) Pack outputs
sync.t      = t_common;
sync.ang    = ang_sync;
sync.force  = T_sync;      % channels in order mk10_1..mk10_4
sync.ATI_T  = ATI_T_sync;
sync.ATI_F  = ATI_F_sync;

%% --- Local functions ---
function y = butter_filtfilt(t, x, fc, n)
    Fs = 1/median(diff(t));
    [b,a] = butter(n, fc/(Fs/2), 'low');   % Butterworth
    y = filtfilt(b,a, detrend(x));
end

function dt = delay_xcorr(tA, xA, tB, xB, Fs)
    t0 = max(tA(1), tB(1));
    t1 = min(tA(end), tB(end));
    tc = (t0 : 1/Fs : t1)';

    a = interp1(tA, xA, tc, 'pchip'); a = a - mean(a);
    b = interp1(tB, xB, tc, 'pchip'); b = b - mean(b);

    [c,l] = xcorr(a,b);
    [~,k] = max(c);
    dt = l(k)/Fs;   % add to B to align with A
end
