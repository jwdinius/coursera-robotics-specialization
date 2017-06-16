function [ u ] = pd_controller(~, s, s_des, params)
%PD_CONTROLLER  PD controller for the height
%
%   s: 2x1 vector containing the current state [z; v_z]
%   s_des: 2x1 vector containing desired state [z; v_z]
%   params: robot parameters

u = 0;

% FILL IN YOUR CODE HERE
Kv = 10;
Kp = 54;
e = s_des - s;
zddot_des = 0; % in hover, desired acceleration
u = params.mass*(zddot_des + Kv*e(2) + Kp*e(1) + params.gravity);
end

