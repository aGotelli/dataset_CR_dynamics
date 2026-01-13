%%
clc
clear all
close all
load('Beam_released_dominique.mat');
CI = [q_0;r_0;eta_0];

% v = VideoWriter('Beam_released_dominique.avi');
% open(v);
x_trace_save = [];
y_trace_save = [];
z_trace_save = [];
Config.dX = Config.L/100;
Config.X  = 0:Config.dX:Config.L;
for it_video = 1:1:max(size(Y2,1))
    
    Const.q = Y2(it_video,1:Const.dim_base)';

    Const.q_dot  = zeros(Const.dim_base,1);

    [T Xp]  = ode45(@(t,y) Forward_eta(t,y,time,Const,Config),Config.X,CI,Config.option); % Solve ODE

    figure(1)
%     plot(Xp(:,5),Xp(:,7),'k','LineWidth',2)
    plot(Xp(:,5),Xp(:,7),'LineWidth',2)
    grid on
    hold on
    axis equal
    xlim(0.8*[-Config.L Config.L])
    ylim([-Config.L 0*Config.L])
    xlabel('x(m)')
    ylabel('y(m)')
%     zlim([-Config.L Config.L])
%     title(['t = ' num2str(Config.time(it_video)) 's'])
    drawnow
            x_trace_save = [x_trace_save,Xp(:,5)];
        y_trace_save = [y_trace_save,Xp(:,6)];
        z_trace_save = [z_trace_save,Xp(:,7)];
%     frame = getframe(gcf);
%     writeVideo(v,frame);
end

% close(v);
