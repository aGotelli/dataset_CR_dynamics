close all;
clear;
clc;

addpath("outils\")

%% ====== PATHS / SETTINGS ======
folder = fullfile("..", "dataCollectionPack/data","20260324/","test_ft_2/");



%% ====== LOAD DATA ======

ati = readtable(fullfile(folder, "dataATIFT.csv"));

resense = readtable(fullfile(folder, "dataResenseFT.csv"));

%% ====== EXTRACT ATI FT (RAW TIME) ======
tA = ati.timestamp - ati.timestamp(1);

time_resense = resense.timestamp_s_ - resense.timestamp_s_(1);



figure("Name", "Forces")
subplot(3, 1, 1)
plot(tA, ati.Fx_N_, 'b')
hold on
plot(time_resense, resense.Fx, 'r')
grid on

subplot(3, 1, 2)
plot(tA, ati.Fy_N_, 'b')
hold on
plot(time_resense, resense.Fz, 'r')
grid on

subplot(3, 1, 3)
plot(tA, ati.Fz_N_, 'b')
hold on
plot(time_resense, resense.Fy, 'r')
grid on

legend('ATI', 'Resense')