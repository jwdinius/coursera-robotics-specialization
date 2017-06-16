function [F, M] = controller(t, state, des_state, params)
%CONTROLLER  Controller for the quadrotor
%
%   state: The current state of the robot with the following fields:
%   state.pos = [x; y; z], state.vel = [x_dot; y_dot; z_dot],
%   state.rot = [phi; theta; psi], state.omega = [p; q; r]
%
%   des_state: The desired states are:
%   des_state.pos = [x; y; z], des_state.vel = [x_dot; y_dot; z_dot],
%   des_state.acc = [x_ddot; y_ddot; z_ddot], des_state.yaw,
%   des_state.yaw_dot
%
%   params: robot parameters

%   Using these current and desired states, you have to compute the desired
%   controls


% =================== Your code goes here ===================

% Thurst
kd = [200;200;200]*3;
kp = [200;200;200];
ep = des_state.pos - state.pos;
ed = des_state.vel - state.vel;
rdes = des_state.acc + diag(kd*ed' + kp*ep');
F = params.mass * (params.gravity + rdes(3));

% Moment
phi_T   = des_state.yaw;
phid_T  = des_state.yawdot;
cp      = cos(phi_T);
sp      = sin(phi_T);
phi_des = 1/params.gravity * (rdes(1)*sp - rdes(2)*cp);
th_des  = 1/params.gravity * (rdes(1)*cp + rdes(2)*sp);
psi_des = phi_T;
p_des   = 0;
q_des   = 0;
r_des   = phid_T;
kpr   = [200;200;200]/50;
kdr   = [200;200;200]/50;
rot_des = [phi_des;th_des;psi_des];
om_des  = [p_des;q_des;r_des];
er    = rot_des - state.rot;
eo    = om_des - state.omega;
M = diag(kdr*eo' + kpr*er');

% =================== Your code ends here ===================

end
