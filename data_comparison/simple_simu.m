%   This file will be used to estimate the values of the robot used for the
%   dataset project. The estimation will be done as an optmization problem
close all;
clear;
clc


addpath("Cosserat/")
addpath("Cosserat/ODEs/")
addpath("Cosserat/ODEs/tendon/")
addpath("Cosserat/utilities/")
addpath("Cosserat/rod_properties/")
addpath("Cosserat/implicit_integration/")
addpath("Data")


[Const, Config] = simulationConfigurations();


% %   Stifness
% Const.EI = 0.09;
% 
% 
% 
% 
% 
% %  STIFFNESS AND INERTIA OF CROSS SECTION
% Const.rhoAg = Const.rho*Const.Area*Const.g;
% 
% 
% Const.H_cal = diag([Const.GIxx, Const.EI, Const.EI, Const.EA, Const.GA, Const.GA]);
% 
% [Kee, Dee] = computeGeneralisedStiffnessDampingMatrices(Const, Config);
% 
% Const.Kee = Kee;
% Const.Dee = Dee;

q_init = 0*[
    2.7313
   -0.9827
    0.0281
   -0.0574
    0.0321
   -0.0078
];

% Const.Q_X0 = [0.7071068 0 0.7071068 0]';

simulation_results = dynamicSimulationCosserat(q_init, Const, Config);