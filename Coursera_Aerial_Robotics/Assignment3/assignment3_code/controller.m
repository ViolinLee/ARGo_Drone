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
%   des_state.yawdot
%
%   params: robot parameters

%   Using these current and desired states, you have to compute the desired
%   controls


% =================== Your code goes here ===================

% Thrust
F = 0;  % F = u1?
kdx=10;kpx=200;
kdy=10;kpy=200;
kdz=10;kpz=2000;

state_acc(1) = des_state.acc(1)+kdx*(des_state.vel(1)-state.vel(1))+kpx*(des_state.pos(1)-state.pos(1));
state_acc(2) = des_state.acc(2)+kdy*(des_state.vel(2)-state.vel(2))+kpy*(des_state.pos(2)-state.pos(2));
state_acc(3) = des_state.acc(3)+kdz*(des_state.vel(3)-state.vel(3))+kpz*(des_state.pos(3)-state.pos(3));

F = params.mass*(params.gravity + state_acc(3));


% Moment
M = zeros(3,1);  % M = u2?
Kp_phi=1000; Kd_phi=10; Kp_theta=1000; Kd_theta=10; Kp_psi=2000; Kd_psi=10;

phi_des = (1/params.gravity)*(state_acc(1)*sin(des_state.yaw) - state_acc(2)*cos(des_state.yaw));
p_des = 0;
theta_des = (1/params.gravity)*(state_acc(1)*cos(des_state.yaw) + state_acc(2)*sin(des_state.yaw));
q_des = 0;

M(1) = Kp_phi*(phi_des-state.rot(1))+Kd_phi*(p_des-state.omega(1));
M(2) = Kp_theta*(theta_des-state.rot(2))+Kd_theta*(q_des-state.omega(2));
M(3) = Kp_psi*(des_state.yaw-state.rot(3))+Kd_psi*(des_state.yawdot -state.omega(3));


% =================== Your code ends here ===================

end
