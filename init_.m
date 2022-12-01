%%
% File name : init_.m
% Author : Zhengqing GONG
% Date : 10/10/2022
% Version : 4.0

% This function is an initialization of the control parameters according to
% the inputs.
% It returns:
% p: the number of agents
% X: the initial state vector of size (2*p)*1
% Xref： the desired state of size (2*p)*1
% A1: the control matrix with respect to the communication graph (adjacency
% matrix) and the gain parameters

function [p, X, Xref, A1, A2] = init_(initPos, destPos, alpha, K, graphCase)

%% Parameter definition
    size_init = size(initPos); 
    p = size_init(2); % Number of agents
    r = size_init(1); % Dimension of each position vector
    n = p*r; % Dimension of the state vector

    % State vector
    X = zeros(n,1);
    for j = 1:p
        for i = 1:r
            X((j-1)*r+i,:) = initPos(i, j);
        end
    end
  
    Xref = zeros(n,1);
    for j = 1:p
        for i = 1:r
            Xref((j-1)*r+i,:) = destPos(i, j);
        end
    end

    % State matrix
    if graphCase == 1
        A1 = [[-1*alpha-4*K, 0, K, 0, K, 0, K, 0, K, 0];...
              [0, -1*alpha-4*K, 0, K, 0, K, 0, K, 0, K];...
              [K, 0, -1*alpha-4*K, 0, K, 0, K, 0, K, 0];...
              [0, K, 0, -1*alpha-4*K, 0, K, 0, K, 0, K];...
              [K, 0, K, 0, -1*alpha-4*K, 0, K, 0, K, 0];...
              [0, K, 0, K, 0, -1*alpha-4*K, 0, K, 0, K];...
              [K, 0, K, 0, K, 0, -1*alpha-4*K, 0, K, 0];...
              [0, K, 0, K, 0, K, 0, -1*alpha-4*K, 0, K];...
              [K, 0, K, 0, K, 0, K, 0, -1*alpha-4*K, 0];...
              [0, K, 0, K, 0, K, 0, K, 0, -1*alpha-4*K]];
          
          
    elseif graphCase == 2
        % Matrix with a simplified graph
        A1 = [[-1*alpha-1*K, 0, K, 0, 0, 0, 0, 0, 0, 0];...
              [0, -1*alpha-1*K, 0, K, 0, 0, 0, 0, 0, 0];...
              [0, 0, -1*alpha-1*K, 0, K, 0, 0, 0, 0, 0];...
              [0, 0, 0, -1*alpha-1*K, 0, K, 0, 0, 0, 0];...
              [0, 0, 0, 0, -1*alpha-1*K, 0, K, 0, 0, 0];...
              [0, 0, 0, 0, 0, -1*alpha-1*K, 0, K, 0, 0];...
              [0, 0, 0, 0, 0, 0, -1*alpha-1*K, 0, K, 0];...
              [0, 0, 0, 0, 0, 0, 0, -1*alpha-1*K, 0, K];...
              [K, 0, 0, 0, 0, 0, 0, 0, -1*alpha-1*K, 0];...
              [0, K, 0, 0, 0, 0, 0, 0, 0, -1*alpha-1*K]];
    end
    
    % Matrix A2
    A2 = -1*A1;
    
end
