function [ u1, u2 ] = controller(~, state, des_state, params)
%CONTROLLER  Controller for the planar quadrotor
%
%   state: The current state of the robot with the following fields:
%   state.pos = [y; z], state.vel = [y_dot; z_dot], state.rot = [phi],
%   state.omega = [phi_dot]
%
%   des_state: The desired states are:
%   des_state.pos = [y; z], des_state.vel = [y_dot; z_dot], des_state.acc =
%   [y_ddot; z_ddot]
%
%   params: robot parameters

%   Using these current and desired states, you have to compute the desired
%   controls

u1 = 0;
u2 = 0;

% FILL IN YOUR CODE HERE
m = params.mass;
g = params.gravity;
Ixx = params.Ixx;
kvz = 2*100;
kpz = 4*250;
kvp = 2*100;
kpp = 4*250;
kvy = 2*100;
kpy = 4*250;
edz = des_state.vel(2)-state.vel(2);
ez = des_state.pos(2)-state.pos(2);
edy = des_state.vel(1)-state.vel(1);
ey = des_state.pos(1)-state.pos(1);
yacc = -g*state.rot;
eddy = des_state.acc(1)-yacc;
yddd = -g*state.omega;
edddy= -yddd;
phic = -1/g*(des_state.acc(1) + kvy*edy + kpy*ey);
phicd = -1/g*(kvy*eddy + kpy*edy);
phicdd= -1/g*(kvy*edddy + kpy*eddy);
ep  = phic - state.rot;
edp = phicd - state.omega;
u1 = m * (g + des_state.acc(2) + kvz*edz + kpz*ez);
u2 = Ixx * (phicdd + kvp*edp + kpp*ep);
end

