clear all
close all

load('Test_Static.mat')
v = VideoWriter('Beam_Till_shooting_act_cable_static.avi');
open(v);
for it_video = 2:max(size(T))
    gcf = figure(1);
    cd('D:\Documents\simulateurs\Dyn_Essai_release_Beam_shooting')
        addpath('./outils')
addpath('./IDM')
addpath('./methode spectral')
addpath('./ode_pas_fixe')
    load('Beam_released_flexion_shooting_pas_fixe_feval_dt_1em3_act_cable_static_small_step.mat')
    subplot(121)
    plot(X_draw{it_video}(:,1),X_draw{it_video}(:,3),'b','Linewidth',2)
    hold on
    grid on
    
    cd('D:\Documents\simulateurs\simulateur\newton_cable_static')
    load('Test_Static.mat')

    Const.q = q(:,it_video-1);

    [QX,rX] = Reconstruction(q_0,r_0,Const,Config);

    subplot(121)
    title(['Convergent, T_1 = ' num2str(75*T(it_video)) ' N'])

    plot(rX(1,:),rX(3,:),'r--','Linewidth',2)
    hold off
    grid on
    xlabel('x (m)')
    ylabel('z (m)')
    xlim([0 0.35])
    ylim([-0.45 0.05])
    legend('shooting 1','Lagrangien','Location','NorthEast')

    cd('D:\Documents\simulateurs\Dyn_Essai_release_Beam_shooting')
    addpath('./outils')
addpath('./IDM')
addpath('./methode spectral')
addpath('./ode_pas_fixe')
    load('Beam_released_flexion_shooting_pas_fixe_feval_dt_1em3_act_cable_static_elicoidal_small_step.mat')
    subplot(122)
    plot3(X_draw{it_video}(:,1),X_draw{it_video}(:,2),X_draw{it_video}(:,3),'b','Linewidth',2)
    hold on
    grid on

    cd('D:\Documents\simulateurs\simulateur\newton_cable_static')
    load('Test_Static_elicoidale.mat')

    Const.q = q(:,it_video-1);
    [QX,rX] = Reconstruction(q_0,r_0,Const,Config);

    subplot(122)
    title(['Spiral, T_1 = ' num2str(75*T(it_video)) ' N'])

    plot3(rX(1,:),rX(2,:),rX(3,:),'r--','Linewidth',2)
    hold off
    grid on
    view([-21.5,7.60])
    xlim([0 0.16])
    ylim([0 0.055])
    zlim([-0.45 0.05])
    xlabel('x (m)')
    ylabel('y (m)')
    zlabel('z (m)')
    legend('shooting 1','Lagrangien','Location','NorthEast')

    frame = getframe(gcf);
    writeVideo(v,frame);
end
close(v);
