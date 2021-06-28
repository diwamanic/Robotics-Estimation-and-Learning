function [ predictx, predicty, state, param ] = kalmanFilter( t, x, y, state, param, previous_t )
%UNTITLED Summary of this function goes here
%   Four dimensional state: position_x, position_y, velocity_x, velocity_y

    %% Place parameters like covarainces, etc. here:
    % P = eye(4)
    % R = eye(2)

    % Check if the first time running this function
    if previous_t<0
        state = [x, y, 0, 0];
        param.P = 2 * eye(4);
        predictx = x;
        predicty = y;
        return;
    end

    %% TODO: Add Kalman filter updates
    % As an example, here is a Naive estimate without a Kalman filter
    % You should replace this code
    % Motion Model
    A = eye(4);
    A(1,3) = 0.033;
    A(2,4) = 0.033;
    
    z_t = [x y];
    C = [1 0 0 0; 0 1 0 0];
    
    dt = t - previous_t;
    %Si_Mo = 5 * eye(4);
    Si_Mo = [dt*dt/4  0    dt/2 0 ;
               0    dt*dt/4  0  dt/2;
               dt/2     0    1   0;
               0     dt/2     0  1];
    
    param.P = A * param.P * A' + Si_Mo;
    
    Si_Mes = 0.01 * eye(2);
    R = C * param.P * C' + Si_Mes;
    
    K = param.P * C' *inv(R);
    
    state = (state' + K * (z_t' - C * state'))'; 
    
    param.P = param.P * (1 - K * C);
    % Predict 330ms into the future
    
    predictx = state(1);
    predicty = state(2);
    % State is a four dimensional element
    %state = [x, y, vx, vy];
end
