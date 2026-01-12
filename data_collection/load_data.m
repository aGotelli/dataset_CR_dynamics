close all
clear
clc


datasequence = readtable("dataCollectionPack\circle_150_v_1\datasequence__circle_radius_150p0.csv");

timestamps_actuators = datasequence.timestamp;

desired_angles = [
    datasequence.target1_rad, datasequence.target2_rad, datasequence.target3_rad, datasequence.target4_rad 
];

angles = [
    datasequence.rel_angle1_rad, datasequence.rel_angle2_rad, datasequence.rel_angle3_rad, datasequence.rel_angle4_rad 
];
time_actuators = timestamps_actuators - timestamps_actuators(1);


figure("Name", "Angles")
subplot(4, 1, 1)
plot(time_actuators, angles(:, 1), 'b');
hold on
plot(time_actuators, desired_angles(:, 1), 'r');
grid on
xlabel("Time [s]")
ylabel("Angle [RAD]")

subplot(4, 1, 2)
plot(time_actuators, angles(:, 2), 'b');
hold on
plot(time_actuators, desired_angles(:, 2), 'r');
grid on
xlabel("Time [s]")
ylabel("Angle [RAD]")

subplot(4, 1, 3)
plot(time_actuators, angles(:, 3), 'b');
hold on
plot(time_actuators, desired_angles(:, 3), 'r');
grid on
xlabel("Time [s]")
ylabel("Angle [RAD]")

subplot(4, 1, 4)
plot(time_actuators, angles(:, 4), 'b');
hold on
plot(time_actuators, desired_angles(:, 4), 'r');
grid on
xlabel("Time [s]")
ylabel("Angle [RAD]")


%%  Load Data MK10

mk10_1_data = readtable("dataCollectionPack\circle_150_v_1\dataMark10_1_-x_.csv");
mk10_2_data = readtable("dataCollectionPack\circle_150_v_1\dataMark10_1_x_.csv");
mk10_3_data = readtable("dataCollectionPack\circle_150_v_1\dataMark10_2_-y_.csv");
mk10_4_data = readtable("dataCollectionPack\circle_150_v_1\dataMark10_2_y_.csv");

timestamp_mk_10_1 = mk10_1_data.timestamp;
timestamp_mk_10_2 = mk10_2_data.timestamp;
timestamp_mk_10_3 = mk10_3_data.timestamp;
timestamp_mk_10_4 = mk10_4_data.timestamp;

time_mk_10_1 = timestamp_mk_10_1 - timestamp_mk_10_1(1);
time_mk_10_2 = timestamp_mk_10_2 - timestamp_mk_10_2(1);
time_mk_10_3 = timestamp_mk_10_3 - timestamp_mk_10_3(1);
time_mk_10_4 = timestamp_mk_10_4 - timestamp_mk_10_4(1);




T_mk_10_1 = mk10_1_data.tension_N_;
T_mk_10_2 = mk10_2_data.tension_N_;
T_mk_10_3 = mk10_3_data.tension_N_;
T_mk_10_4 = mk10_4_data.tension_N_;


figure("Name", "Tensions")
subplot(4, 1, 1)
plot(time_mk_10_1, T_mk_10_1);
grid on
xlabel("Time [s]")
ylabel("Tension [N]")

subplot(4, 1, 2)
plot(time_mk_10_2, T_mk_10_2);
grid on
xlabel("Time [s]")
ylabel("Tension [N]")

subplot(4, 1, 3)
plot(time_mk_10_3, T_mk_10_3);
grid on
xlabel("Time [s]")
ylabel("Tension [N]")

subplot(4, 1, 4)
plot(time_mk_10_4, T_mk_10_4);
grid on
xlabel("Time [s]")
ylabel("Tension [N]")



%%  Load data ATI FT
data_ATI_ft = readtable("dataCollectionPack\circle_150_v_1\dataATIFT_.csv");

timestamp_ATI_ft = data_ATI_ft.timestamp;
time_ATI_ft = timestamp_ATI_ft - timestamp_ATI_ft(1);

ATI_wrench = [
    data_ATI_ft.Tx_Nm_'
    data_ATI_ft.Ty_Nm_'
    data_ATI_ft.Tz_Nm_'
    data_ATI_ft.Fx_N_'
    data_ATI_ft.Fy_N_'
    data_ATI_ft.Fz_N_'
];


figure("Name", "Wrench at base")
subplot(2, 1, 1)
plot(time_ATI_ft, ATI_wrench(1, :), 'r', 'DisplayName', 'C_x');
hold on
plot(time_ATI_ft, ATI_wrench(2, :), 'g', 'DisplayName', 'C_y');
plot(time_ATI_ft, ATI_wrench(3, :), 'b', 'DisplayName', 'C_z');
grid on
xlabel("Time [s]")
ylabel("Torque [Nm]")

subplot(2, 1, 2)
plot(time_ATI_ft, ATI_wrench(4, :), 'r', 'DisplayName', 'N_x');
hold on
plot(time_ATI_ft, ATI_wrench(5, :), 'g', 'DisplayName', 'N_y');
plot(time_ATI_ft, ATI_wrench(6, :), 'b', 'DisplayName', 'N_z');
grid on
xlabel("Time [s]")
ylabel("Force [N]")


%%  Vicon

data_vicon = readtable("dataCollectionPack\circle_150_v_1\dataVicon_.csv");

timestamp_vicon = data_vicon.timestamp;
time_vicon = timestamp_vicon - timestamp_vicon(1);

%%  Overview on timestamps


figure("Name", "Timesteps Comparisons")
plot(timestamps_actuators, 'w');
hold on;
plot(timestamp_mk_10_1, 'r');
plot(timestamp_mk_10_2, 'g');
plot(timestamp_mk_10_3, 'b');
plot(timestamp_mk_10_4, 'c');
plot(timestamp_ATI_ft, 'y');
plot(timestamp_vicon, 'm');

%   Compute frame rates
f_actuators = length(time_actuators)/time_actuators(end)

f_mk10_1 = length(time_mk_10_1)/time_mk_10_1(end)
f_mk10_2 = length(time_mk_10_2)/time_mk_10_2(end)
f_mk10_3 = length(time_mk_10_3)/time_mk_10_3(end)
f_mk10_4 = length(time_mk_10_4)/time_mk_10_4(end)

f_ATI_ft = length(time_ATI_ft)/time_ATI_ft(end)

f_vicon = length(time_vicon)/time_vicon(end)