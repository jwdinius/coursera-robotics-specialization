close all;
clear;

% Hover
% z_des = 0;

% Step
z_des = 1;

% Given trajectory generator
trajhandle = @(t) fixed_set_point(t, z_des);

% This is your controller
controlhandle = @controller;

% Run simulation with given trajectory generator and controller
[t, z] = height_control(trajhandle, controlhandle);

% % Sample code to get more info on the response
% sim_info = lsiminfo(z, t, z_des);
% disp(['Settling time [s]: ', num2str(sim_info.SettlingTime)]);
% disp(['Overshoot [%]: ', num2str(max(0,(sim_info.Max-z_des)*100))]);
