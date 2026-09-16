function [F0,Q_a] = IDM(q_0,r_0,eta_0,eta_dot_0,time,Const,Config)
% IDM  Inverse Dynamics Model of a GVS (Geometric Variable Strain)
%      tendon-driven continuum rod, solved by explicit spatial shooting
%      (ode45), following D. Orin's recursive Newton-Euler formulation.
%
% Computes the same outputs as TIDM_spectral.m's [F0,Q_a] pair, without
% its Jacobian, using a three-pass recursive sweep along the rod:
%   1) Forward_eta            : forward kinematics (orientation, position,
%                                twist), base -> tip.
%   2) Backward_Sweep_IDM     : backward wrench sweep (articulated-body-
%                                style S, s recursion), tip -> base.
%   3) Forward_eta_dot_IDM_ORIN : forward acceleration/actuation sweep,
%                                base -> tip, giving the actuation force
%                                Q_a and, combined with S_0/s_0, the base
%                                reaction wrench F0.
%
% Inputs
%   q_0, r_0        : base orientation (quaternion) and position
%   eta_0, eta_dot_0: base twist and twist rate
%   time            : evaluation time (unused by Base_Phi, kept for a
%                     uniform calling convention)
%   Const, Config   : robot/model constants and simulation configuration
%
% Outputs
%   F0  : base reaction wrench
%   Q_a : generalised actuation force
%
% Note: eta_dot_0 is taken as an explicit input, matching
% TIDM_spectral.m's calling convention, rather than being read from a
% Const.eta_dot field (which the original IDM_ORIN.m read but which was
% never actually assigned anywhere in the codebase).

span = [0,Config.L];

%% ---- Pass 1: forward kinematics (base -> tip) ----

CI = [q_0;r_0;eta_0];
[~,Y1] = ode45(@(X,y) Forward_eta(X,y,time,Const,Config),span,CI,Config.option);

q_1   = Y1(end,1:4)';
r_1   = Y1(end,5:7)';
eta_1 = Y1(end,8:13)';

%% ---- Pass 2: backward wrench sweep (tip -> base) ----

F_tilde_1 = ad_(eta_1)'*(Const.M_cal*eta_1);
S_1 = zeros(36,1);

[~,F_1,~,~] = Forces_exterieur(Config.L,q_1,r_1,eta_1,Const,Config);
s_1 = F_1;

CI = [q_1;r_1;eta_1;F_tilde_1;S_1;s_1];
[~,Y2] = ode45(@(X,y) Backward_Sweep_IDM(X,y,time,Const,Config),span,CI,Config.option);

F_tilde_0 = ad_(eta_0)'*(Const.M_cal*eta_0);
S_0_vec = Y2(end,20:55)';
S_0     = reshape(S_0_vec,[6 6]);
s_0     = Y2(end,56:61)';

%% ---- Pass 3: forward acceleration / actuation sweep (base -> tip) ----

Q_a = zeros(Const.dim_base,1);
CI = [q_0;r_0;eta_0;F_tilde_0;S_0_vec;s_0;eta_dot_0;Q_a];
[~,Y3] = ode45(@(X,y) Forward_eta_dot_IDM_ORIN(X,y,time,Const,Config),span,CI,Config.option);

F0  = -S_0*eta_dot_0 - s_0;
Q_a = Y3(end,68:67+Const.dim_base)';

end
