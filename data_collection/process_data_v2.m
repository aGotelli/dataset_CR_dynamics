%% ====== PATHS / SETTINGS ======
folder = "dataCollectionPack\circle_150_v_1_3rd\";

cutoffHz    = 20;   % Butterworth cutoff
butterOrder = 4;

%% ====== LOAD DATA ======
motor = readtable(folder + "datasequence__circle_radius_150p0.csv");

mk_1_negx = readtable(folder + "dataMark10_1_-x_.csv");
mk_1_x    = readtable(folder + "dataMark10_1_x_.csv");
mk_2_negy = readtable(folder + "dataMark10_2_-y_.csv");
mk_2_y    = readtable(folder + "dataMark10_2_y_.csv");

ati = readtable(folder + "dataATIFT_.csv");

%   Da aggiungere in data processing
% vicon_data = readtable(folder + "dataVicon_.csv");
% 
% time_vicon = vicon_data.timestamp - vicon_data.timestamp(1);
% dt_vicon = diff(time_vicon);
% f_vicon = 1 / median(dt_vicon)

%% ====== EXTRACT MOTOR SIGNALS ======
time_actuators = motor.timestamp;                      % epoch seconds

target_angles = [motor.target1_rad, motor.target2_rad, motor.target3_rad, motor.target4_rad];
meaured_angles   = [motor.rel_angle1_rad, motor.rel_angle2_rad, motor.rel_angle3_rad, motor.rel_angle4_rad];

%% ====== EXTRACT FORCES (RAW TIME) ======
% Your mapping:
% motor1 -> Mark10 1_x
% motor2 -> Mark10 2_y
% motor3 -> Mark10 1_-x
% motor4 -> Mark10 2_-y
time_cables = cell(1,4);
cable_tensions  = cell(1,4);

time_cables{1} = mk_1_x.timestamp;       cable_tensions{1} = mk_1_x.tension_N_;
time_cables{2} = mk_2_y.timestamp;       cable_tensions{2} = mk_2_y.tension_N_;
time_cables{3} = mk_1_negx.timestamp;    cable_tensions{3} = mk_1_negx.tension_N_;
time_cables{4} = mk_2_negy.timestamp;    cable_tensions{4} = mk_2_negy.tension_N_;

%% ====== EXTRACT ATI FT (RAW TIME) ======
tA = ati.timestamp;
% tA_dt = datetime(tA, "ConvertFrom","posixtime");

ATI_F = [ati.Fx_N_, ati.Fy_N_, ati.Fz_N_];
ATI_T = [ati.Tx_Nm_, ati.Ty_Nm_, ati.Tz_Nm_];

%% ====== FILTER (BUTTER + FILTFILT) ======
measured_angles_f   = zeros(size(meaured_angles));
cable_tensions_f = cell(1,4);
for i = 1:4
    measured_angles_f(:,i)   = butter_filtfilt(time_actuators, meaured_angles(:,i),   cutoffHz, butterOrder);

    cable_tensions_f{i} = butter_filtfilt(time_cables{i}, cable_tensions{i}, cutoffHz, butterOrder);
end

ATI_F_f = zeros(size(ATI_F));
ATI_T_f = zeros(size(ATI_T));
for k = 1:3
    ATI_F_f(:,k) = butter_filtfilt(tA, ATI_F(:,k), cutoffHz, butterOrder);
    ATI_T_f(:,k) = butter_filtfilt(tA, ATI_T(:,k), cutoffHz, butterOrder);
end


%% ====== PLOT: 4 SUBPLOTS (MOTOR TARGET/MEAS + FORCE) ======
figure("Name","Motors + corresponding cable force (filtered)");

for i = 1:4
    subplot(4,1,i)

    yyaxis left
    plot(time_actuators, measured_angles_f(:,i),   "b", "LineWidth", 2.0); hold on
    plot(time_actuators, target_angles(:,i), "r","LineWidth", 2.0);
    ylabel("Angle [rad]")
    grid on

    yyaxis right
    plot(time_cables{i}, cable_tensions_f{i}, "g", "LineWidth", 2.0);
    ylabel("Tension [N]")

    title("Motor " + i + " (meas/target) + mapped force")
    if i == 4
        xlabel("Time (raw timestamp)")
    end

    % Look for:
    % - Do force changes occur at the same times as angle changes?
    % - Is there a consistent lag between motion and tension response?
end
% 
figure("Name","ATI FT (filtered)");
subplot(2,1,1)
plot(tA, ATI_T_f(:,1), "r"); hold on
plot(tA, ATI_T_f(:,2), "g");
plot(tA, ATI_T_f(:,3), "b");
grid on; ylabel("Torque [Nm]"); legend("Tx","Ty","Tz")
title("ATI Torques (filtered)")

subplot(2,1,2)
plot(tA, ATI_F_f(:,1), "r"); hold on
plot(tA, ATI_F_f(:,2), "g");
plot(tA, ATI_F_f(:,3), "b");
grid on; ylabel("Force [N]"); xlabel("Time (raw timestamp)")
legend("Fx","Fy","Fz")
title("ATI Forces (filtered)")

%% ====== HELPER FUNCTION ======
function y = butter_filtfilt(t, x, fc, n)
    Fs = 1/median(diff(t));                 % estimate sampling rate from timestamps
    [b,a] = butter(n, fc/(Fs/2), "low");    % Butterworth
    y = filtfilt(b,a, x);                   % zero-phase filtering
end
