function [Lambda_X0, Qa_X0, dynamics_state] = IDM(time, q, dot_q, ddot_q, Config, Const)


%   Define the forward state
forward_y0 = [Const.Q_X0
              Const.r_X0
              Const.eta_X0
              Const.deta_X0
             ];

%   Forward integration
[~, Y] = ode45(@(X, y) ForwardKinematics(X, y, q, dot_q, ddot_q, Config, Const), ...
                    Config.forward_integration_domain, forward_y0);


%   Save kinematics along the rod
Q_X = Y(:,  1:4 );
r_X = Y(:,  5:7 );
eta_X  = Y(:,  8:13);
deta_X = Y(:, 14:19);

%   Extract state at X=1
Q_X1 = Q_X(end, :)';
r_X1 = r_X(end, :)';
eta_X1     =  eta_X(end, :)';
dot_eta_X1 = deta_X(end, :)';

%   Force at the tip
Lambda_X1 = Const.Lambda_X1;

%   Define the forward state
backward_y1 = zeros(25 + Const.dim_base, 1);

%   Initialization
backward_y1(1:19) = [Q_X1;
                     r_X1;
                     eta_X1;
                     dot_eta_X1];
backward_y1(20:25) = Lambda_X1;

%   Backward integration
[~, Y] = ode45(@(X, y) BackwardDynamics(X, y, q, dot_q, ddot_q, Config, Const), ...
                    Config.backward_integration_domain, backward_y1);

%   Save dynamics along the rod
Lambda_X = Y(:,  20:25 );
Qa_X     = Y(:,  26:end);

%   Reorder to go from X=0 to X=1
Lambda_X = flip(Lambda_X, 1);
Qa_X     = flip(Qa_X, 1);



%   Extract state at X=0
Lambda_X0 = Lambda_X(1,:)';
Qa_X0     = Qa_X(1, :)';

dynamics_state.Q_X = Q_X;
dynamics_state.r_X = r_X;

dynamics_state.eta_X  = eta_X;
dynamics_state.deta_X = deta_X;

dynamics_state.Lambda_X = Lambda_X;
dynamics_state.Qa_X     = Qa_X;

end

