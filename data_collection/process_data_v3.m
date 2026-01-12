close all; clear; clc

%% =========================
%  SETTINGS
%  =========================
folder = "dataCollectionPack\circle_150_v_1\";

cutoffHz      = 10;     % Butterworth cutoff (Hz) - start here
butterOrder   = 4;
commonRateHz  = 200;    % final resampling rate

maxShiftSec   = 0.8;    % bound cross-corr shift to avoid periodic ambiguity
syncWindowSec = 6.0;    % use a short window after motion onset for sync

% File load order (force channels):
% ch1 = dataMark10_1_-x_
% ch2 = dataMark10_1_x_
% ch3 = dataMark10_2_-y_
% ch4 = dataMark10_2_y_
%
% Your mapping:
% motor1 -> ch2, motor2 -> ch4, motor3 -> ch1, motor4 -> ch3
forceToMotor = [3 1 4 2];    % [ch1 ch2 ch3 ch4]
atiToMotor   = 1;            % ATI aligned to motor 1 (change later if needed)

%% =========================
%  LOAD RAW
%  =========================
motor = readtable(folder + "datasequence__circle_radius_150p0.csv");

forceTbl{1} = readtable(folder + "dataMark10_1_-x_.csv");
forceTbl{2} = readtable(folder + "dataMark10_1_x_.csv");
forceTbl{3} = readtable(folder + "dataMark10_2_-y_.csv");
forceTbl{4} = readtable(folder + "dataMark10_2_y_.csv");

ati = readtable(folder + "dataATIFT_.csv");

% Raw epoch timestamps (for metadata)
rawEpoch.motor_t0 = motor.timestamp(1);
rawEpoch.force_t0 = cellfun(@(T) T.timestamp(1), forceTbl);
rawEpoch.ati_t0   = ati.timestamp(1);



%%  Start processing motor

tMotor = motor.timestamp - motor.timestamp(1);

angle = [motor.rel_angle1_rad, motor.rel_angle2_rad, motor.rel_angle3_rad, motor.rel_angle4_rad];
des_angle = [motor.target1_rad, motor.target2_rad, motor.target3_rad, motor.target4_rad];
angleF = zeros(size(angle));


for i=1:4
    angleF(:,i) = butter_filtfilt(tMotor, angle(:,i), cutoffHz, butterOrder);
end


figure("Name", "Angles")
for i=1:4
    subplot(4, 1, i)
    plot(tMotor, angle(:,i), 'b', 'LineWidth', 1);
    hold on
    plot(tMotor, des_angle(:, i), 'r', 'LineWidth', 1);
    plot(tMotor, angleF(:, i), 'g', 'LineWidth', 1);
    grid on
    xlabel("Time [s]")
    ylabel("Angle [RAD]")

end


%   Find now first derivative (central) of joint angle
dts = tMotor(2:end) - tMotor(1:end-1);
dt = median(dts);
dt_std = std(dts);
d_angleF = zeros(size(angle));
for i=1:4
    d_angle = (angleF(2:end, i) - angleF(1:end-1, i))/dt;
    d_angleF(:,i) = [
        d_angle(1)
        d_angle
    ];
end


figure("Name", "Velocities")
for i=1:4
    subplot(4, 1, i)
    plot(tMotor, d_angleF(:,i), 'b', 'LineWidth', 1);
    hold on
    grid on
    xlabel("Time [s]")
    ylabel("Velocity [RAD/s]")

end



%% =========================
%  Now process the force sensors
%  =========================

tForce = cell(1,4);
force  = cell(1,4);
for k=1:4
    tForce{k} = forceTbl{k}.timestamp - forceTbl{k}.timestamp(1);
    force{k}  = forceTbl{k}.tension_N_;
end





figure("Name", "Measured Forces")
for k=1:4
    subplot(4, 1, i)
    plot(tForce{k}, force{k}, 'b', 'LineWidth', 1);
    grid on
    xlabel("Time [s]")
    ylabel("Angle [RAD]")
end


%   Find now first derivative (central) of forces

d_force  = cell(1,4);
for i=1:4

    dts = tMotor(2:end) - tMotor(1:end-1);
    dt = median(dts);
    dt_std = std(dts);
    d_angleF = zeros(size(angle));

    d_angle = (angleF(2:end, i) - angleF(1:end-1, i))/dt;
    d_angleF(:,i) = [
        d_angle(1)
        d_angle
    ];
end


figure("Name", "Velocities")
for i=1:4
    subplot(4, 1, i)
    plot(tMotor, d_angleF(:,i), 'b', 'LineWidth', 1);
    hold on
    grid on
    xlabel("Time [s]")
    ylabel("Velocity [RAD/s]")

end







%% =========================
%  FILTER (Butterworth + filtfilt)
%  =========================
angle = [motor.rel_angle1_rad, motor.rel_angle2_rad, motor.rel_angle3_rad, motor.rel_angle4_rad];
angleF = zeros(size(angle));
for i=1:4
    angleF(:,i) = butter_filtfilt(tMotor, angle(:,i), cutoffHz, butterOrder);
end

forceF = cell(1,4);
for k=1:4
    forceF{k} = butter_filtfilt(tForce{k}, force{k}, cutoffHz, butterOrder);
end

Tx = ati.Tx_Nm_;  Ty = ati.Ty_Nm_;  Tz = ati.Tz_Nm_;
Fx = ati.Fx_N_;   Fy = ati.Fy_N_;   Fz = ati.Fz_N_;

TxF = butter_filtfilt(tATI, Tx, cutoffHz, butterOrder);
TyF = butter_filtfilt(tATI, Ty, cutoffHz, butterOrder);
TzF = butter_filtfilt(tATI, Tz, cutoffHz, butterOrder);

FxF = butter_filtfilt(tATI, Fx, cutoffHz, butterOrder);
FyF = butter_filtfilt(tATI, Fy, cutoffHz, butterOrder);
FzF = butter_filtfilt(tATI, Fz, cutoffHz, butterOrder);


%% MOTION ONSET (t = 0 at onset)  --- robust + simple
motorVel = gradient(angleF)./gradient(tMotor);      % Nx4
speed = sqrt(sum(motorVel.^2, 2));                  % scalar speed

% threshold = small fraction of the maximum speed (always works if motion exists)
thr = 0.05 * max(speed);                            % 5% of max speed
idx0 = find(speed > thr, 1, 'first');
tMotion0 = tMotor(idx0);

% Shift all times so motion onset is t=0
tMotor = tMotor - tMotion0;
for k=1:4
    tForce{k} = tForce{k} - tMotion0;
end
tATI = tATI - tMotion0;


%% =========================
%  OPTIONAL SMALL SYNC SHIFTS (bounded, windowed)
%  =========================
% Recompute features on shifted time axes
motorVel = gradient(angleF)./gradient(tMotor);

forceShift = zeros(1,4);
tForceSync = tForce;

for k=1:4
    m = forceToMotor(k);
    forceRate = gradient(forceF{k})./gradient(tForce{k});     % N/s
    forceShift(k) = xcorr_shift_windowed(tMotor, motorVel(:,m), tForce{k}, forceRate, ...
                                         commonRateHz, maxShiftSec, syncWindowSec);
    tForceSync{k} = tForce{k} + forceShift(k);
end

torqueXY = hypot(TxF, TyF);
torqueRate = gradient(torqueXY)./gradient(tATI);

atiShift = xcorr_shift_windowed(tMotor, motorVel(:,atiToMotor), tATI, torqueRate, ...
                                commonRateHz, maxShiftSec, syncWindowSec);
tATISync = tATI + atiShift;

fprintf("Motion onset (s): %.3f\n", 0.0);
fprintf("Force shifts (s): [%.4f %.4f %.4f %.4f]\n", forceShift);
fprintf("ATI shift (s): %.4f\n", atiShift);

%% =========================
%  RESAMPLE TO COMMON TIMELINE (filtered + sampled only)
%  =========================
tStart = max([tMotor(1), tForceSync{1}(1), tForceSync{2}(1), tForceSync{3}(1), tForceSync{4}(1), tATISync(1)]);
tEnd   = min([tMotor(end), tForceSync{1}(end), tForceSync{2}(end), tForceSync{3}(end), tForceSync{4}(end), tATISync(end)]);
t = (tStart : 1/commonRateHz : tEnd)';

angle_out = interp1(tMotor, angleF, t, 'pchip');

force_out = zeros(numel(t),4);
for k=1:4
    force_out(:,k) = interp1(tForceSync{k}, forceF{k}, t, 'pchip');
end

ATI_T_out = [ ...
    interp1(tATISync, TxF, t, 'pchip'), ...
    interp1(tATISync, TyF, t, 'pchip'), ...
    interp1(tATISync, TzF, t, 'pchip')  ];

ATI_F_out = [ ...
    interp1(tATISync, FxF, t, 'pchip'), ...
    interp1(tATISync, FyF, t, 'pchip'), ...
    interp1(tATISync, FzF, t, 'pchip')  ];

%% =========================
%  QUICK VALIDATION PLOTS (just enough)
%  =========================
figure("Name","Motion onset check");
plot(tMotor + tMotion0, speed); hold on; yline(thr,'r--');
xline(tMotion0,'k--'); grid on
xlabel("t (motor-start frame) [s]"); ylabel("motor speed (rad/s)");
% Look for: clear speed rise at the detected onset.

figure("Name","Sync check (after shifts, first seconds)");
win = [0 syncWindowSec];

for k=1:4
    m = forceToMotor(k);
    mFeat = motorVel(:,m);
    mFeat = mFeat / max(abs(mFeat)) * (0.5*range(forceF{k})) + mean(forceF{k});

    subplot(4,1,k)
    plot(tForceSync{k}, forceF{k}, 'b'); hold on
    plot(tMotor, mFeat, 'r');
    xlim(win); grid on; title("force ch"+k+" vs motor"+m+" feature");
    % Look for: peaks/turning points align over several cycles.
end

%% =========================
%  SAVE (RAW + PROCESSED + METADATA)
%  =========================
processed.t = t;                       % t=0 is motion onset
processed.angle = angle_out;           % filtered + resampled
processed.force = force_out;           % filtered + resampled (ch1..ch4 load order)
processed.ATI_T = ATI_T_out;           % filtered + resampled [Tx Ty Tz]
processed.ATI_F = ATI_F_out;           % filtered + resampled [Fx Fy Fz]

metadata.cutoffHz     = cutoffHz;
metadata.butterOrder  = butterOrder;
metadata.commonRateHz = commonRateHz;
metadata.maxShiftSec  = maxShiftSec;
metadata.syncWindowSec = syncWindowSec;

metadata.forceToMotor = forceToMotor;
metadata.atiToMotor   = atiToMotor;

metadata.motionOnset_relToMotorStart = tMotion0;    % seconds after motor logging began
metadata.forceShiftSec = forceShift;
metadata.atiShiftSec   = atiShift;

metadata.rawEpoch = rawEpoch; % epoch starts for traceability

save(folder + "processed.mat", "processed");
save(folder + "metadata.mat", "metadata");

%% =========================
%  FUNCTIONS
%  =========================
function y = butter_filtfilt(t, x, fc, n)
    Fs = 1/median(diff(t));
    [b,a] = butter(n, fc/(Fs/2), 'low');
    % y = filtfilt(b,a, detrend(x));
    y = filtfilt(b,a, x);
end

function shiftSec = xcorr_shift_windowed(tRef, xRef, tSig, xSig, gridHz, maxShiftSec, winSec)
    % Use only a short window right after motion onset (t in motor-onset frame)
    t0 = max(tRef(1), tSig(1));
    t1 = min(tRef(end), tSig(end));
    t1 = min(t1, t0 + winSec);

    tg = (t0 : 1/gridHz : t1)';

    a = interp1(tRef, xRef, tg, 'pchip'); a = a - mean(a);
    b = interp1(tSig, xSig, tg, 'pchip'); b = b - mean(b);

    maxLag = round(maxShiftSec * gridHz);
    [c,l] = xcorr(a, b, maxLag);
    [~,k] = max(c);
    shiftSec = l(k)/gridHz;   % add to tSig
end
