function [Delta_Lambda_X0, Delta_Qa_X0] = TIDM(time, q, dot_q, ddot_q, Delta_q, Delta_dot_q, Delta_ddot_q, Config, Const)


%   Define the forward state
    forward_y0 = zeros(37, 1);

    %   Initialization
    forward_y0(1:7) = [Const.Q_X0;
                       Const.r_X0];


    %   Forward integration
    [~, Y] = ode45(@(X, y) ForwardTangentKinematics(X, y, q, dot_q, ddot_q, ...
                                                    Delta_q, Delta_dot_q, Delta_ddot_q,...
                                                    Config, Const), ...
                    Config.forward_integration_domain, forward_y0);

    %   Extract state at X=1
    y_tip = Y(end,:);

    %   Force at the tip
    F1 = F_tip(time);
    Delta_F1 = Delta_F_tip(time);


    %   Define the forward state
    backward_y1 = zeros(49 + Const.dim_base, 1);

    %   Initialization
    backward_y1(1:37) = y_tip;
    backward_y1(38:43) = F1;
    backward_y1(44:49) = Delta_F1;


    %   Backward integration
    [~, Y] = ode45(@(X, y) BackwardTangentDynamics(X, y, q, dot_q, ddot_q, ...
                                                   Delta_q, Delta_dot_q, Delta_ddot_q,...
                                                   Config, Const), ...
                    Config.backward_integration_domain, backward_y1);
    
    %   Extract state at X=0
    y_base = Y(end,:);
    
    %   Get variation on internal wrench
    Delta_Lambda_X0 = y_base(44:49)';

    %   Get variation on generalized forces
    Delta_Qa_X0 = y_base(50:end)';


end

