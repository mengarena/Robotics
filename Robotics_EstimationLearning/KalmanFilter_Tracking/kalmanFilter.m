function [ predictx, predicty, state, param ] = kalmanFilter( t, x, y, state, param, previous_t )
%UNTITLED Summary of this function goes here
%   Four dimensional state: position_x, position_y, velocity_x, velocity_y
 
    % These videos explains very clear: 
    % https://www.youtube.com/watch?v=CaCcOwJPytQ&list=PLX2gX-ftPVXU3oUFNATxGXY90AULiqnWT

    Sigma_m = [0.2 0 0 0; 0 0.2 0 0; 0 0 0.1 0; 0 0 0 0.1];
    Sigma_o = [0.1 0; 0 0.1];

    % Check if the first time running this function
    if previous_t<0
        state = [x y 0 0];
        %param.P = 0.1 * eye(4);
        param.P = Sigma_m;
        predictx = x;
        predicty = y;
        return;
    end

    %% Place parameters like covarainces, etc. here:
    A = [1 0 0.33 0; 0 1 0 0.33; 0 0 1 0; 0 0 0 1];

    % Prediction
    Sp = A * transpose(state);
    Pp = A * param.P * transpose(A) + Sigma_m;   % Process covariance matrix
    
    H = [1 0 0 0; 0 1 0 0];
    
    % Kalman Gain
    K = Pp * transpose(H) * inv(H * Pp * transpose(H) + Sigma_o);
    
    Y = [x y]';   % Observation
    
    % Update
    state = Sp + K * (Y - H * Sp);
    param.P = Pp - K * H * Pp;
    
    predictx = state(1);
    predicty = state(2);
    
    state = transpose(state);
    
    %% Naive solution
    % As an example, here is a Naive estimate without a Kalman filter
    % You should replace this code
    %%vx = (x - state(1)) / (t - previous_t);
    %%vy = (y - state(2)) / (t - previous_t);
    % Predict 330ms into the future
    %%predictx = x + vx * 0.330;
    %%predicty = y + vy * 0.330;
    % State is a four dimensional element
    %%state = [x, y, vx, vy];
end
