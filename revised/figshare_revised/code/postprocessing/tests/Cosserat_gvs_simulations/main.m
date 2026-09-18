%% 
close all
clear all
clc


addpath("ODEs/")
addpath("utilities/")
addpath("rod_properties/")
addpath("implicit_integration/")
addpath("explicit_integration/")



%%  Load processed data


path = fullfile("../../../../../","data","dynamic_motion/",'circle_slow/');
load_path = fullfile(path,'processed/');
savepath = fullfile(path,"gvs/");
saving_fig_folder = fullfile(savepath,"figures/");

mkdir(savepath)
mkdir(saving_fig_folder)


tendon_tensions = load(fullfile(load_path,"tendon_tensions.csv"));
time_base_wrench_raw = load(fullfile(load_path,"base_wrench.csv"));

%   Load this from file
dt = 0.01;
time = tendon_tensions(:,1);

%   Differential cable tension (the actuation is antagonistic)
tau_1 = tendon_tensions(:,2) - tendon_tensions(:,4);
tau_2 = tendon_tensions(:,3) - tendon_tensions(:,5);
tau = [
    tau_1'
    tau_2'
];



%%  Simulation setup


DoFs = [0, 3, 3, 0, 0, 0];

%   Configurations for the simulation
[Const, Config] = simulationConfigurations(DoFs, time(end), dt);

%   Position of the rod base
Const.r_X0 = [0;0;0];

%   Quaternion of rod base [w x y z]
Const.Q_X0 = [0.7071068 0 0.7071068 0]';


%   Cosserat rod generalized coordinates
q      = zeros(Const.dim_base, 1);
dot_q  = zeros(Const.dim_base, 1);
ddot_q = zeros(Const.dim_base, 1);


Config.plot_simu = true;

[t_stack_implicit, states_stack_implicit] = ...
    CosseratRodImplicitSimulation(q, dot_q, ddot_q, tau, Const, Config);


%%

ground_truth = load("Beam_released_dominique_3_modes_flexion_spectral_N_30_Newton_dt005.mat");


for it_t=1:length(t_stack_implicit)

    t = t_stack_implicit(it_t);

    ne = Const.dim_base;
    
    q = states_stack_implicit(it_t, 1:ne)';
    plotRod(t, q, Const, Config, 'b')
    hold on
    q = ground_truth.q(:, it_t);
    plotRod(t, q, Const, Config, '--r')

    pause(.1)

end


