close all
clear all
clc


addpath("ODEs/")
addpath("ODEs/tendon")
addpath("utilities/")
addpath("rod_properties/")
addpath("implicit_integration/")
addpath("explicit_integration/")


%   Configurations for the simulation
[Const, Config] = simulationConfigurations();

%   Cosserat rod generalized coordinates
q      = zeros(Const.dim_base, 1);
dot_q  = zeros(Const.dim_base, 1);
ddot_q = zeros(Const.dim_base, 1);




%%  Generate ground truth

%   Generate long ground truth to be used later
Config.t_start =  0;      %   [s]
Config.t_end   =  3;      %   [s]
Config.dt      = 0.01;    %   [s]

ground_truth = generateGroundTruth(q, dot_q, ddot_q, Const, Config);
save("data/ground_truth", "ground_truth")


%%  STATE ESTIMATION
clear
clc
close all

%   Configurations for the simulation
[Const, Config] = simulationConfigurations();


%   Cosserat rod generalized coordinates
q      = zeros(Const.dim_base, 1);
dot_q  = zeros(Const.dim_base, 1);
ddot_q = zeros(Const.dim_base, 1);

Config.t_start = 0.2;     %   [s]
Config.t_end   = 2.0;     %   [s]
Config.dt      = 0.01;    %   [s]


Config.Gamma_wrench = 0.08;    %   Applied at base (for wrenches)
Config.Gamma_twist  = 0.0;    %   Applied at tip (for twists)

load("data/ground_truth.mat")

state_estimation_results_twist = implicitStateEstimation(ground_truth, q, dot_q, ddot_q, Const, Config);
save("data/" + getVarName(state_estimation_results_twist), "state_estimation_results_twist")



