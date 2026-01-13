function trace_CPR_3D(Config,Var)

gp = Var.g_0;
% plot base

X_base = 0.01*eye(3);
color_base = ['b','r','g'];
for it_nl = 1 :Config.robot.nl
    goi = [Config.robot.limb.Roi{it_nl} Config.robot.limb.roi{it_nl};0 0 0 1];
    gpi = gp *[Config.robot.plaform.Rpi_p{it_nl},Config.robot.plaform.rpi_p{it_nl};0 0 0 1];
    X_O = goi * [0;0;0;1];
    Xp0 = gpi * [0;0;0;1];
    for it_point = 1:3
        X_Oi =goi * [X_base(:,it_point);1];
        plot3([X_O(1),X_Oi(1)],[X_O(2),X_Oi(2)],[X_O(3),X_Oi(3)],color_base(it_point),'LineWidth',2)
        hold on
        X_Opi =gpi * [X_base(:,it_point);1];
        plot3([Xp0(1),X_Opi(1)],[Xp0(2),X_Opi(2)],[Xp0(3),X_Opi(3)],color_base(it_point),'LineWidth',2)        
        axis equal
        grid on
    end
end
% dessin base
alpha = [0:2*pi/50:2*pi]';
X_base_h = [Config.robot.r_base*cos(alpha),Config.robot.r_base*sin(alpha),0.005*ones(size(alpha))];
X_base_b = [Config.robot.r_base*cos(alpha),Config.robot.r_base*sin(alpha),-0.005*ones(size(alpha))];
X_base = [X_base_h;X_base_b];
surf([X_base_h(:,1)';X_base_b(:,1)'],[X_base_h(:,2)';X_base_b(:,2)'],[X_base_h(:,3)';X_base_b(:,3)'],'FaceColor',0.9*[1 1 1],'EdgeColor','none')
fill3(X_base_h(:,1),X_base_h(:,2),X_base_h(:,3),0.9*[1 1 1])
fill3(X_base_b(:,1),X_base_b(:,2),X_base_b(:,3),0.9*[1 1 1])

X_platfrom_h = [Config.robot.r_plaform*cos(alpha),Config.robot.r_plaform*sin(alpha),(0.005)*ones(size(alpha))];
X_platfrom_b = [Config.robot.r_plaform*cos(alpha),Config.robot.r_plaform*sin(alpha),(-0.005)*ones(size(alpha))];
for it_point = 1:max(size(alpha))
    X_ph(it_point,:) =(gp * [X_platfrom_h(it_point,:)';1])';
    X_pb(it_point,:) =(gp* [X_platfrom_b(it_point,:)';1])';
end
X_platfrom_h = X_ph(:,1:3);
X_platfrom_b = X_pb(:,1:3);
surf([X_platfrom_h(:,1)';X_platfrom_b(:,1)'],[X_platfrom_h(:,2)';X_platfrom_b(:,2)'],[X_platfrom_h(:,3)';X_platfrom_b(:,3)'],'FaceColor',0.9*[1 1 1],'EdgeColor','none')
fill3(X_platfrom_h(:,1),X_platfrom_h(:,2),X_platfrom_h(:,3),0.9*[1 1 1])
fill3(X_platfrom_b(:,1),X_platfrom_b(:,2),X_platfrom_b(:,3),0.9*[1 1 1])


for it_limb = 1 :Config.robot.nl
    [g1,g2,g3,Q2X,r2X] = GM_Limb(it_limb,Config,Var);
    Config.robot.limb.gi_p{it_limb} = g3;
    Config.robot.limb.Ri_p{it_limb} = g3(1:3,1:3);
    Config.robot.limb.ri_p{it_limb} = g3(1:3,4);
    for it_Noeuds = 1:Config.N_noeuds
        g2X = g1 * [quaternion_to_matrice(Q2X(:,it_Noeuds)),r2X(:,it_Noeuds);0 0 0 1];
        base_x = g2X* [0.01;0;0;1];
        base_y = g2X* [0;0.01;0;1];
        base_z = g2X* [0;0;0.01;1];
        r_limb(:,it_Noeuds) = g2X(1:3,4);
    %     plot3([r_limb(1,it_Noeuds),base_x(1)],[r_limb(2,it_Noeuds),base_x(2)],[r_limb(3,it_Noeuds),base_x(3)],color_base(1),'LineWidth',2)
    %     plot3([r_limb(1,it_Noeuds),base_y(1)],[r_limb(2,it_Noeuds),base_y(2)],[r_limb(3,it_Noeuds),base_y(3)],color_base(2),'LineWidth',2)
    %     plot3([r_limb(1,it_Noeuds),base_z(1)],[r_limb(2,it_Noeuds),base_z(2)],[r_limb(3,it_Noeuds),base_z(3)],color_base(3),'LineWidth',2)
    end
    plot3([r_limb(1,it_Noeuds),base_x(1)],[r_limb(2,it_Noeuds),base_x(2)],[r_limb(3,it_Noeuds),base_x(3)],color_base(1),'LineWidth',2)
    plot3([r_limb(1,it_Noeuds),base_y(1)],[r_limb(2,it_Noeuds),base_y(2)],[r_limb(3,it_Noeuds),base_y(3)],color_base(2),'LineWidth',2)
    plot3([r_limb(1,it_Noeuds),base_z(1)],[r_limb(2,it_Noeuds),base_z(2)],[r_limb(3,it_Noeuds),base_z(3)],color_base(3),'LineWidth',2)
    plot3(r_limb(1,:),r_limb(2,:),r_limb(3,:),'m',LineWidth=2)
end
grid on
axis equal
title(['time = ' num2str(Var.t) 's'])



