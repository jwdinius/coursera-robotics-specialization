function [ predictx, predicty, state, param ] = kalmanFilter( t, x, y, state, param, previous_t )
%UNTITLED Summary of this function goes here
%   Four dimensional state: position_x, position_y, velocity_x, velocity_y

    %% Place parameters like covarainces, etc. here:
    %dt = .033;
    %dt = t - previous_t;
    dt = 1;
    Q  = 10*eye(4);
    R  = .01*eye(2);
    A = [1 0 dt 0;
         0 1 0 dt;
         0 0 1  0;
         0 0 0  1];
    C  = [1 0 0 0;
          0 1 0 0];

    % Check if the first time running this function
    if previous_t<0
        state = [x, y, 0, 0];
        param.P = 10e6 * eye(4);
        predictx = x;
        predicty = y;
        return;
    end

    %% TODO: Add Kalman filter updates
    % As an example, here is a Naive estimate without a Kalman filter
    % You should replace this code
    state_km1 = state';
    P_km1 = param.P;
    z = [x;y];
    state_kkm1 = A*state_km1;
    P_kkm1 = A*P_km1*A' + Q;
    y = z - C*state_kkm1;
    S = C*P_kkm1*C' + R;
    Sinv = S \ eye(size(S));
    K = P_kkm1*C'*Sinv;
    state = state_kkm1 + K*y;
    param.P = (eye(4) - K*C)*P_kkm1;
    % Predict 330ms into the future
%     vx = (state(3) + state_km1(3))/2;
%     vy = (state(4) + state_km1(4))/2;
%     
    vx = state(3);
    vy = state(4);
    predictx = state(1) + vx * 10 * dt;
    predicty = state(2) + vy * 10 * dt;
%     predictx = state(1);
%     predicty = state(2);
    % State is a four dimensional element
    state = state';
end
