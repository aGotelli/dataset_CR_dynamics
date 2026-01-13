clear all
close all
clc

name = "Test_static_CPR_3D_alpha_methode_long_06";
name = "3D_CPR_N30_rho_07_t_30";
load(name + ".mat")% static

% load('Test_static_CPR_3D_16.mat')% static
% ajoute le dossier outils
addpath('./outils')
addpath('./methode spectral')
% Var.video = VideoWriter('Test_static_CPR_3D_12c.avi');
% open(Var.video)
pas = 80;
%[1,100,140,170,180,200]% static
for it_video = [27, 127, 227, 327, 427, 527, 627]%, 350, 450, 550, 650, 750, 850, 950]%1:pas:max(size(Config.time))
    Qp = Xi(1:4,it_video);
    rp = Xi(5:7,it_video);
    q = Xi(20:20+Var.size.q-1,it_video);
    Var.g_0 = [quaternion_to_matrice(Qp),rp;0 0 0 1];
    Var.t = Config.time(it_video);
    Var.mu_p = [Log_SO3_(quaternion_to_matrice(Qp));rp];
    Var.q = q; 
    figure(1)
    hold on
    trace_CPR_3D(Config,Var)
    % plot3(Xi(5,1:it_video),Xi(6,1:it_video),Xi(7,1:it_video),'k','LineWidth',2)
    view(155,35)
    drawnow
    % frame = getframe(gcf);
    % for it_frame = 1:10
    %     writeVideo(Var.video,frame);
    % end
end
xlabel('x(m)')
ylabel('y(m)')
zlabel('z(m)')
% close(Var.video)