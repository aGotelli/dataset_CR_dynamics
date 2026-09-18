function plotRod(t, q, Const, Config, color)


ne = Const.dim_base;
zeroq = zeros(ne, 1);


%   Define the forward state
forward_y0 = zeros(19, 1);

%   Initialization
forward_y0(1:7) = [Const.Q_X0;
                    Const.r_X0];

%   Forward integration
[~, Y] = ode45(@(X, y) ForwardKinematics(X, y, q, zeroq, zeroq, Config, Const), ...
                    Config.forward_integration_domain, forward_y0);


xyz = Y(:, 5:7);


figure(1)
plot(xyz(:,1),xyz(:,3), color,'LineWidth',2)
hold on
grid on
axis equal


grid on
hold off
axis equal

xlim([-1.1*Const.L 1.1*Const.L])
ylim([-1.1*Const.L 1.1*Const.L])


title(['time = ' num2str(t) 's'])
drawnow



end