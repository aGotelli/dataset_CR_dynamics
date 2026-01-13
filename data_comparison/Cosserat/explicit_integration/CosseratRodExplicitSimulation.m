function [t_stack, y_stack] = CosseratRodExplicitSimulation(q, dot_q, Const,Config)

%   Prepare stacks for data
dt = 1e-4;


time_span = 0:dt:Config.t_end;


y0 = [q; dot_q];




%   Explicit time integration (unstable)
options = odeset('RelTol',1e-5,'Stats','on', 'InitialStep', dt);
[t_stack, y_stack] = ode45(@(t,y) CosseratRodExplicitODEs(t,y, Const, Config), time_span, y0, options);

% 
% %   Explicit time integration (stable)
% options = odeset('Mass', @(t, y)getMassStiffODEs(t, y, Const, Config),...
%                 'AbsTol', 1e-6, 'RelTol', 1e-6);
% [t_stack, y_stack] = ode15s(@(t,y) CosseratRodExplicitODEs(t,y, Const, Config), time_span, y0, options);



end