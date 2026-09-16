function [F0,Q_a,J] = TIDM(qn_0,rn_0,q_0,r_0,eta_0,eta_dot_0,a,b,time,Const,Config)
% TIDM  Tangent Inverse Dynamics Model of a GVS (Geometric Variable Strain)
%       tendon-driven continuum rod, solved by explicit spatial shooting
%       (ode45) instead of a global spectral (Chebyshev-collocation) solve.
%
% Computes the same three outputs as TIDM_spectral.m:
%   F0  : base reaction wrench required to produce the prescribed modal
%         motion (q, q_dot, q_dot_dot, carried in Const)
%   Q_a : generalised actuation force (virtual-work projection of the
%         internal wrench onto the actuated strain modes)
%   J   : Jacobian of [F0;Q_a] with respect to [Delta_psi_0;Delta_q], i.e.
%         the base pose/twist perturbation and the modal-coordinate
%         perturbation used by the Newmark-Newton time-integration loop
%         (see forward_dynamics_simulation.m)
%
% Method
%   1) The nonlinear baseline fields (orientation, position, twist,
%      internal wrench) are obtained exactly as in IDM.m, via three
%      sequential ode45 sweeps (forward kinematics, backward wrench sweep,
%      forward acceleration/actuation sweep), each solved with dense
%      output so that any field can be evaluated at an arbitrary spanwise
%      location with deval().
%   2) The Jacobian columns are obtained by re-integrating, with ode45,
%      the exact same linearised ("variational") ODEs that TIDM_spectral.m
%      solves with its spectral collocation method: one forward sweep for
%      the linearised pose/twist perturbation and one backward sweep for
%      the linearised internal-wrench perturbation. Perturbing the base
%      pose/twist (unit vectors of Delta_psi_0) gives the J00/Je0 block;
%      perturbing each modal coordinate (unit vectors of Delta_q) gives
%      the J0e/Jee block. Both perturbation types are handled by the same
%      pair of local functions (variation_forward_sweep /
%      variation_backward_sweep), since the two cases differ only in
%      which of Delta_psi_0 or the modal strain perturbation field is
%      nonzero.
%
% Inputs / outputs follow the same convention as TIDM_spectral.m.

span = [0, Config.L];

%% ---------------------------------------------------------------------
%  Baseline (nonlinear) sweep -- identical to IDM.m, kept with dense
%  output so the fields can be reused as coefficients below.
%% ---------------------------------------------------------------------

CI = [q_0;r_0;eta_0];
sol_fwd = ode45(@(X,y) Forward_eta(X,y,time,Const,Config),span,CI,Config.option);

q_1   = sol_fwd.y(1:4,end);
r_1   = sol_fwd.y(5:7,end);
eta_1 = sol_fwd.y(8:13,end);

F_tilde_1 = ad_(eta_1)'*(Const.M_cal*eta_1);
S_1 = zeros(36,1);

[~,F_1,~,~] = Forces_exterieur(Config.L,q_1,r_1,eta_1,Const,Config);
s_1 = F_1;

CI = [q_1;r_1;eta_1;F_tilde_1;S_1;s_1];
sol_bwd = ode45(@(X,y) Backward_Sweep_IDM(X,y,time,Const,Config),span,CI,Config.option);

F_tilde_0 = ad_(eta_0)'*(Const.M_cal*eta_0);
S_0_vec = sol_bwd.y(20:55,end);
S_0     = reshape(S_0_vec,[6 6]);
s_0     = sol_bwd.y(56:61,end);

Q_a0 = zeros(Const.dim_base,1);
CI = [q_0;r_0;eta_0;F_tilde_0;S_0_vec;s_0;eta_dot_0;Q_a0];
sol_etadot = ode45(@(X,y) Forward_eta_dot_IDM_ORIN(X,y,time,Const,Config),span,CI,Config.option);

F0  = -S_0*eta_dot_0 - s_0;
Q_a = sol_etadot.y(68:67+Const.dim_base,end);

%% ---------------------------------------------------------------------
%  Baseline field interpolants, reused as coefficients of the linear
%  variational ODEs below.
%% ---------------------------------------------------------------------

Q_of       = @(X) deval(sol_fwd,X,1:4);
eta_of     = @(X) deval(sol_fwd,X,8:13);
eta_dot_of = @(X) deval(sol_etadot,X,62:67);

% Backward_Sweep_IDM.m integrates with the internal change of variable
% X <- Config.L - X, so its own integration coordinate xi_b maps back to
% the physical location via X_phys = Config.L - xi_b.
S_of = @(X) reshape(deval(sol_bwd,Config.L-X,20:55),[6 6]);
s_of = @(X) deval(sol_bwd,Config.L-X,56:61);

% Internal wrench field, consistent with F0 = -S_0*eta_dot_0 - s_0 and
% with the Q_a integrand -Phi*B'*(S*eta_dot+s) used in
% Forward_eta_dot_IDM_ORIN.m.
Lambda_of = @(X) S_of(X)*eta_dot_of(X) + s_of(X);

Xi_of      = @(X) strain_field(X,time,Const,Config);
[~,Xi_dot_of] = deal(@(X) strain_field_rate(X,time,Const,Config));

%% ---------------------------------------------------------------------
%  Rigid change of frame between the previous and current base pose,
%  needed to map a base-pose/twist perturbation into initial conditions
%  for Delta_eta and Delta_eta_dot (same construction as TIDM_spectral.m).
%% ---------------------------------------------------------------------

R_0  = quaternion_to_matrice(q_0);
g_0  = [R_0,r_0;0 0 0 1];

Rn_0 = quaternion_to_matrice(qn_0);
gn_0 = [Rn_0,rn_0;0 0 0 1];

gn = gn_0\g_0;
var_theta = Log_SE3_(gn(1:3,1:3),gn(1:3,4));
Theta = var_theta(1:3,1);
D     = var_theta(4:6,1);

%% ---------------------------------------------------------------------
%  J00, Je0: variation with respect to the base pose/twist Delta_psi_0.
%  The modal strain is not perturbed here, so the perturbation field of
%  the strain, Delta_Xi, is identically zero.
%% ---------------------------------------------------------------------

zero_Delta_Xi = @(X) zeros(6,1);

J00 = zeros(6,6);
Je0 = zeros(Const.dim_base,6);

for it = 1:6

    Delta_psi_0 = zeros(6,1);
    Delta_psi_0(it) = 1;

    [Delta_F0,Delta_Qa] = variation_sweep(Delta_psi_0,zero_Delta_Xi,Theta,D,a,b, ...
        Xi_of,Xi_dot_of,eta_of,eta_dot_of,Q_of,Lambda_of,time,Const,Config,span);

    J00(:,it) = Delta_F0;
    Je0(:,it) = Delta_Qa;
end

%% ---------------------------------------------------------------------
%  J0e, Jee: variation with respect to each modal coordinate Delta_q.
%  The base pose/twist is not perturbed here, so Delta_psi_0 = 0.
%% ---------------------------------------------------------------------

zero_Delta_psi_0 = zeros(6,1);

J0e = zeros(6,Const.dim_base);
Jee = zeros(Const.dim_base,Const.dim_base);

for it = 1:Const.dim_base

    Delta_q = zeros(Const.dim_base,1);
    Delta_q(it) = 1;

    Delta_Xi_of = @(X) Const.B*Base_Phi(X,time,Const,Config)'*Delta_q;

    [Delta_F0,Delta_Qa] = variation_sweep(zero_Delta_psi_0,Delta_Xi_of,Theta,D,a,b, ...
        Xi_of,Xi_dot_of,eta_of,eta_dot_of,Q_of,Lambda_of,time,Const,Config,span);

    J0e(:,it) = Delta_F0;
    Jee(:,it) = Delta_Qa;
end

J = [J00,J0e;Je0,Jee];

end


%% ===================== local helper functions ========================

function Xi = strain_field(X,t,Const,Config)
% Modal strain field Xi(X) = B*Phi(X)'*q + B_bar*Xi_c + B*Xi_0 (algebraic
% in X: no ODE state involved, so it can be evaluated at any location
% without interpolation).
Phi = Base_Phi(X,t,Const,Config);
Xi  = Const.B*(Phi'*Const.q) + Const.B_bar*Const.Xi_c + Const.B*Const.Xi_0;
end

function Xi_dot = strain_field_rate(X,t,Const,Config)
% Time-rate of the modal strain field, Xi_dot(X) = B*Phi(X)'*q_dot.
Phi    = Base_Phi(X,t,Const,Config);
Xi_dot = Const.B*(Phi'*Const.q_dot);
end

function [Delta_F0,Delta_Qa] = variation_sweep(Delta_psi_0,Delta_Xi_of,Theta,D,a,b, ...
    Xi_of,Xi_dot_of,eta_of,eta_dot_of,Q_of,Lambda_of,time,Const,Config,span)
% Runs one forward + one backward ode45 sweep of the linearised
% ("variational") equations for a single perturbation direction, and
% returns the corresponding column of the Jacobian. The same pair of
% sweeps serves both perturbation types (base pose/twist and modal
% coordinate): they differ only in which of Delta_psi_0 (a rigid base
% perturbation) or Delta_Xi_of (a modal strain-field perturbation) is
% nonzero -- the formulas below reduce to TIDM_spectral.m's two branches
% in each case.

T_m1 = T_SE3_m1_(Theta,D);

Delta_eta_0     = a*T_m1*Delta_psi_0;
Delta_eta_dot_0 = b*T_m1*Delta_psi_0;

y0 = [Delta_psi_0;Delta_eta_0;Delta_eta_dot_0];

sol_var_fwd = ode45(@(X,y) variation_forward(X,y,time,Delta_Xi_of,a,b, ...
    Xi_of,Xi_dot_of,eta_of,eta_dot_of,Const),span,y0,Config.option);

Delta_psi_of     = @(X) deval(sol_var_fwd,X,1:6);
Delta_eta_of     = @(X) deval(sol_var_fwd,X,7:12);
Delta_eta_dot_of = @(X) deval(sol_var_fwd,X,13:18);

% Boundary (terminal) condition of the backward sweep: the tip carries
% the externally-applied wrench Const.F1 rather than the (here always
% zero, since the rod's cross-sectional area Aire = 0) distributed
% gravity load used at every interior node -- matching TIDM_spectral.m.
Delta_psi_L = sol_var_fwd.y(1:6,end);
R_1 = quaternion_to_matrice(Q_of(Config.L));
Delta_F_bar_tip = [zeros(3,1); hat_(Delta_psi_L)'*R_1'*Const.F1(4:6)];

z0 = [Delta_F_bar_tip; zeros(Const.dim_base,1)];

sol_var_bwd = ode45(@(xi,z) variation_backward(xi,z,time,Delta_Xi_of, ...
    Delta_psi_of,Delta_eta_of,Delta_eta_dot_of,Xi_of,eta_of,Lambda_of,Q_of,Const,Config), ...
    span,z0,Config.option);

Delta_F0 = -sol_var_bwd.y(1:6,end);
Delta_Qa = -sol_var_bwd.y(7:6+Const.dim_base,end);

end

function dy = variation_forward(X,y,t,Delta_Xi_of,a,b,Xi_of,Xi_dot_of,eta_of,eta_dot_of,Const)
% Forward (base -> tip) sweep of the linearised pose/twist/acceleration
% perturbation fields, integrated with the base perturbation as initial
% condition -- mirrors the "Delta_psi_X" / "Delta_eta_X" / "Delta_eta_dot_X"
% equations of TIDM_spectral.m's variational section.

Delta_psi     = y(1:6);
Delta_eta     = y(7:12);
Delta_eta_dot = y(13:18);

Xi      = Xi_of(X);
Xi_dot  = Xi_dot_of(X);
eta     = eta_of(X);
eta_dot = eta_dot_of(X);

Delta_Xi         = Delta_Xi_of(X);
Delta_Xi_dot     = a*Delta_Xi;
Delta_Xi_dot_dot = b*Delta_Xi;

Delta_psi_prime     = -ad_(Xi)*Delta_psi + Delta_Xi;
Delta_eta_prime     = -ad_(Xi)*Delta_eta - ad_(Delta_Xi)*eta + Delta_Xi_dot;
Delta_eta_dot_prime = -ad_(Xi)*Delta_eta_dot - ad_(Xi_dot)*Delta_eta ...
                       - ad_(Delta_Xi_dot)*eta - ad_(Delta_Xi)*eta_dot + Delta_Xi_dot_dot;

dy = [Delta_psi_prime;Delta_eta_prime;Delta_eta_dot_prime];

end

function dz = variation_backward(xi_local,z,t,Delta_Xi_of,Delta_psi_of,Delta_eta_of, ...
    Delta_eta_dot_of,Xi_of,eta_of,Lambda_of,Q_of,Const,Config)
% Backward (tip -> base) sweep of the linearised internal-wrench and
% generalised-actuation-force perturbations -- mirrors the
% "Delta_Lambda_X" / "Delta_Qa_X" equations of TIDM_spectral.m's
% variational section. Integrated forward in xi_local over the same
% change of variable as Backward_Sweep_IDM.m (X_phys = Config.L - xi_local),
% so the physical-X derivative computed below is negated at the end.

Delta_Lambda = z(1:6);

X = Config.L - xi_local;

Xi        = Xi_of(X);
eta       = eta_of(X);
Delta_eta     = Delta_eta_of(X);
Delta_eta_dot = Delta_eta_dot_of(X);
Delta_Xi      = Delta_Xi_of(X);
Lambda        = Lambda_of(X);

Phi = Base_Phi(X,t,Const,Config);
R   = quaternion_to_matrice(Q_of(X));

M_cal = Const.M_cal;
Aire  = 0; % cross-sectional area used by the distributed gravity load;
           % hardcoded to zero, matching TIDM_spectral.m.

Delta_F_bar = [zeros(3,1); hat_(Delta_psi_of(X))'*R'*[0;0;-Const.rho*Aire*Const.Gamma_g]];

Delta_Lambda_prime = ad_(Xi)'*Delta_Lambda ...
    + M_cal*Delta_eta_dot - ad_(Delta_eta)'*M_cal*eta - ad_(eta)'*M_cal*Delta_eta ...
    - Delta_F_bar + ad_(Delta_Xi)'*Lambda;

Delta_Qa_prime = -Phi*Const.B'*Delta_Lambda;

dz = -[Delta_Lambda_prime;Delta_Qa_prime];

end
