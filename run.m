%%
% File name : run.m
% Author : Zhengqing GONG
% Date : 10/10/2022
% Version : 4.0

% This function performs iterations of the states of the system according
% to the control law

function [state_set, reach_point] = run(X0, Xref, iter, A1, A2, trans_V)

%% Size of state vector
size_state = size(A1);
n = size_state(2);

%% Rotation matrix for direct rotation
    I = eye(n,n);
    theta = 1; % angular velocity
    
    R0 = [[cosd(theta), -sind(theta), 0, 0, 0, 0, 0, 0, 0, 0];...
          [sind(theta), cosd(theta), 0, 0, 0, 0, 0, 0, 0, 0];...
          [0, 0, cosd(theta), -sind(theta), 0, 0, 0, 0, 0, 0];...
          [0, 0, sind(theta), cosd(theta), 0, 0, 0, 0, 0, 0];...
          [0, 0, 0, 0, cosd(theta), -sind(theta), 0, 0, 0, 0];...
          [0, 0, 0, 0, sind(theta), cosd(theta), 0, 0, 0, 0];...
          [0, 0, 0, 0, 0, 0, cosd(theta), -sind(theta), 0, 0];...
          [0, 0, 0, 0, 0, 0, sind(theta), cosd(theta), 0, 0];...
          [0, 0, 0, 0, 0, 0, 0, 0, cosd(theta), -sind(theta)];...
          [0, 0, 0, 0, 0, 0, 0, 0, sind(theta), cosd(theta)]];
      
    M = [[1,0];[0,1];[1,0];[0,1];[1,0];[0,1];[1,0];[0,1];[1,0];[0,1]];
    
    % Rotation angle
    rot_step = 45;

%% Translation vector
    increment = ones(n, 1);
    
%% Pre-assignment of memory
    state_set = zeros(n, rot_step+iter+1);
    state_set(:,1) = X0;
    reach_flag = 0;
    reach_point = zeros(n,1);
    
    verified = [0;0;0;0;0;0;0;0;0;0];
    arg = ones(n,1);

%% Iteration
    X = X0;
    i = 1;
    while i < (rot_step + iter + 1)
        
        % Calculate the increment
        if reach_flag == 0
            dX = A1 * X + A2 * Xref;
        elseif reach_flag == 1
            dX = trans_V * increment;
        end
        
        % Update the state
        X = X + dX;
        state_set(:,i+1) = X;
        
        % Verify if all the agents have reached the desired destination
        if reach_flag == 0
            % Once the error between the current pos. and the dest. is small enough
            for s = 1:n
                if abs(X(s)-Xref(s)) < 0.5
                    verified(s) = 1;
                end
            end
            if verified == arg
                reach_flag = 1;
                reach_point = X;
                
                % Rotate the formation
                axis = [(X(1)+X(n-1))/2;...
                        (X(2)+X(n))/2];
                % Let the formation rotate to point in the direction of
                % axis y=x
                for k = 1:rot_step
                    X = R0 * X + (I - R0) * (M * axis);
                    state_set(:,i+1+k) = X;
                end
                i = i + rot_step;
            end
        end
        i = i+1;
    end

end