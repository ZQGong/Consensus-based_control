%%
% File name : main.m
% Author : Zhengqing GONG
% Date : 08/10/2022
% Version : 4.0

%% RAZ
clear
close all 
clc 

%% Initialization
    c = ['r', 'g', 'b', 'c', 'm'];

    init_pos = [[0,  0,  0,  0,  0];...
               [12, 6,  0, -6, -12]];
    dest_pos1 = [[30,  35,  40,  35,  30];...
                [6,   3,   0,   -3,  -6]];
    dest_pos2 = [[30,  35,  40,  35,  30];...
                [-6,  -3,   0,   3,   6]];
    alpha = 0.025;
    K = 0.02;
    v = 0.3;
    iter = 220;

%% Simulation animation generating
    % Define animation object
    mov_frames = moviein(iter+1);

    % Initialization of the simulation
    reached_flag = 0;
    
    [p, X, Xref, A1, A2] = init_(init_pos, dest_pos2, alpha, K, 2);
    [state_set, reach_point] = run(X, Xref, iter, A1, A2, v);
    
    reach_formation = [];
    % Save the coordinates of the desired points reached
    for i = 1:p
        reach_formation = [reach_formation, [reach_point(2*i-1); reach_point(2*i)]];
    end
    reach_formation = [reach_formation, reach_formation(:,1)];
    
    size_set = size(state_set);
    length = size_set(2);
    
    % Simulation
    figure(1);
    line = ones(1,5);
    
    for i = 1:length
        for k = 1:p
            % Draw the trajectories of all the agents controlled
            line(:,k) = plot(state_set(2*k-1,1:i), state_set(2*k,1:i), '-', 'color', c(k));
            hold on
            plot(state_set(2*k-1,i), state_set(2*k,i), 'o', 'color', 'black');
        end
        
        % See if the desired coordinates are reached
        if state_set(:,i) == reach_point
            reached_flag = 1;
        end
        
        if reached_flag == 1
            plot(reach_formation(1,:), reach_formation(2,:), '-o', 'color', [0 0.8 0.8]);
        end
        
        set(gcf, 'position', [100 100 700 600]);
        axis([0,50,-15,35]);
        legend([line(:,1) line(:,2) line(:,3) line(:,4) line(:,5)], {'Agent 1', 'Agent 2', 'Agent 3', 'Agent 4', 'Agent 5'}, 'Location', 'northwest')
        
        % Save each frame in the frames array
        mov_frames(i) = getframe();
        
        cla;
    end

    % Generate gif
    for i = 2:length
        movImage = frame2im(mov_frames(i));
        [movImage, map] = rgb2ind(movImage, 256);
        if i == 2
            imwrite(movImage, map, 'anim.gif', 'gif', 'Loopcount', inf, 'DelayTime', 0.09);
        else
            imwrite(movImage, map, 'anim.gif', 'gif', 'WriteMode', 'append', 'DelayTime', 0.09);
        end
    end