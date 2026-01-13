close all


pltConfig.no_gains = true;
pltConfig.twist_v2 = false;
pltConfig.twist    = true;
pltConfig.wrench   = false;
pltConfig.combined = false;






load("data/ground_truth.mat")
load("data/state_estimation_results_no_gains.mat")
load("data/state_estimation_results_twist.mat")
% load("data/state_estimation_results_wrench.mat")
% load("data/state_estimation_results_combined.mat")
% 
% load("data/state_estimation_results_twist_v2.mat")

% state_estimation_results_world = load("data/state_estimation_results_world.mat");
% state_estimation_results_world = state_estimation_results_world.state_estimation_results;


%   Get number of snapshots
N_snapshots = [length(ground_truth.t_stack)];

if(pltConfig.no_gains)
    N_snapshots = [N_snapshots length(state_estimation_results_no_gains.t_stack)];
end

if(pltConfig.twist)
    N_snapshots = [N_snapshots length(state_estimation_results_no_gains.t_stack)];
end

if(pltConfig.wrench)
    N_snapshots = [N_snapshots length(state_estimation_results_no_gains.t_stack)];
end

if(pltConfig.combined)
    N_snapshots = [N_snapshots length(state_estimation_results_no_gains.t_stack)];
end

N_snapshots = min(N_snapshots);

L = ground_truth.Const.L;
it_t = 1;
while it_t <= N_snapshots

    time = ground_truth.t_stack(it_t);

    r_stack_reference = ground_truth.r_stack(:,:, it_t);

    figure(1)
    
    % plot3(r_stack_reference(:,1), r_stack_reference(:,2), r_stack_reference(:,3), 'k', 'LineWidth',2)
    plot(r_stack_reference(:,1), r_stack_reference(:,3), 'k', 'LineWidth',2)
    hold on

    
    if(pltConfig.no_gains)
        r_stack  = state_estimation_results_no_gains.r_stack(:,:, it_t);
        plot( r_stack(:,1),  r_stack(:,3), '.k', 'LineWidth',2)
    end

    if(pltConfig.twist)
        r_stack     = state_estimation_results_twist.r_stack(:,:, it_t);
        plot( r_stack(:,1),  r_stack(:,3), 'r', 'LineWidth',2)
    end

    if(pltConfig.twist_v2)
        r_stack     = state_estimation_results_twist_v2.r_stack(:,:, it_t);
        plot( r_stack(:,1),  r_stack(:,3), 'b', 'LineWidth',2)
    end

    if(pltConfig.wrench)
        r_stack    = state_estimation_results_wrench.r_stack(:,:, it_t);
        plot( r_stack(:,1),  r_stack(:,3), 'g', 'LineWidth',2)
    end

    if(pltConfig.combined)
        r_stack  = state_estimation_results_combined.r_stack(:,:, it_t);
        plot( r_stack(:,1),  r_stack(:,3), 'b', 'LineWidth',2)
    end

   
    grid on
    
    xlim([-0.1*L 1.1*L])
    ylim([-0.6*L 0.6*L])
    %zlim([-0.6*L 0.6*L])

    title(['time = ' num2str(time) 's'])
    %view([1 1 1]);

    drawnow
    hold off

    it_t = it_t+1;

    pause(0.1)
end


