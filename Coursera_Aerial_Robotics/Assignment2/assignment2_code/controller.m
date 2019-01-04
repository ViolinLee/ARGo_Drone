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
kvy = 3.5;
kpy = 18.5;
kvz = 10;
kpz = 30; % z基本上没问题
kvphi = 55;
kpphi = 2000;

des_rot = -1/params.gravity *(des_state.acc(1) + kvy*(des_state.vel(1)-state.vel(1))+kpy*(des_state.pos(1)-state.pos(1)));
des_rot_dot = -1/params.gravity *(0 + kvy*(des_state.acc(1)-0)+kpy*(des_state.vel(1)-state.vel(1))); % 3 order = 0   state.acc no known
des_rot_ddot = -1/params.gravity *(0 + kvy*(0-0)+kpy*(des_state.acc(1)-0));% 4/3 order = 0   state.acc no known
u1 = params.mass * (params.gravity + des_state.acc(2) + kvz*(des_state.vel(2)-state.vel(2)) + kpz*(des_state.pos(2)-state.pos(2)));
u2 = params.Ixx * (des_rot_ddot + kvphi*(des_rot_dot-state.omega)+kpphi*(des_rot-state.rot));


end

