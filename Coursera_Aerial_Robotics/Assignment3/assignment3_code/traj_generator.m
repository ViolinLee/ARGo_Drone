function [ desired_state ] = traj_generator(t, state, waypoints)
% TRAJ_GENERATOR: Generate the trajectory passing through all
% positions listed in the waypoints list
%
% NOTE: This function would be called with variable number of input arguments.
% 这个函数将用可变数量的输入参数调用。
% During initialization, it will be called with arguments
% trajectory_generator([], [], waypoints) and later, while testing, it will be
% called with only t and state as arguments, so your code should be able to
% handle that. This can be done by checking the number of arguments to the
% function using the "nargin" variable, check the MATLAB documentation for more
% information.
%
% t,state: time and current state (same variable as "state" in controller)
% that you may use for computing desired_state
%
% waypoints: The 3xP matrix listing all the points you much visited in order
% along the generated trajectory
%
% desired_state: Contains all the information that is passed to the
% controller for generating inputs for the quadrotor
% 靠desired_state来控制无人机
%
% It is suggested to use "persistent" variables to store the waypoints during
% the initialization call of trajectory_generator.


%% Example code:
% Note that this is an example of naive trajectory generator that simply moves
% the quadrotor along a stright line between each pair of consecutive waypoints
% using a constant velocity of 0.5 m/s. Note that this is only a sample, and you
% should write your own trajectory generator for the submission.

%{
persistent waypoints0 traj_time d0
if nargin > 2          % 第一次调用时，nargin>2.执行下列语句并存储了waypoints0 traj_time d0
    d = waypoints(:,2:end) - waypoints(:,1:end-1);
    d0 = 2 * sqrt(d(1,:).^2 + d(2,:).^2 + d(3,:).^2);
    traj_time = [0, cumsum(d0)];      % 到达各waypoints的时间点
    waypoints0 = waypoints;
else
    if(t > traj_time(end))
        t = traj_time(end);
    end
    t_index = find(traj_time >= t,1);

    if(t_index > 1)                 % 以分段多项式为单元，从分段起点记的t
        t = t - traj_time(t_index-1);
    end
    if(t == 0)
        desired_state.pos = waypoints0(:,1);
    else          % 关键的部分
        scale = t/d0(t_index-1);
        desired_state.pos = (1 - scale) * waypoints0(:,t_index-1) + scale * waypoints0(:,t_index);
 % desired_state.pos = waypoints0(:,t_index-1) + scale * (waypoints0(:,t_index) - waypoints0(:,t_index-1));

    end
    desired_state.vel = zeros(3,1);
    desired_state.acc = zeros(3,1);
    desired_state.yaw = 0;
    desired_state.yawdot = 0;
end
%
%}



%% Fill in your code here
persistent waypoints0 T alpha
finish_time = 15;
desired_state.pos = zeros(3,1);
desired_state.vel = zeros(3,1);
desired_state.acc = zeros(3,1);
if nargin > 2          % 第一次调用时，nargin>2.执行下列语句并存储了waypoints0 traj_time d0
    waypoints0 = waypoints;
    T = finish_time/4;
    % alpha = zeros(32,3);
    A = [0 0 0 0 0 0 0 1 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0;
     T^7 T^6 T^5 T^4 T^3 T^2 T 1 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0;
     0 0 0 0 0 0 0 0 T^7 T^6 T^5 T^4 T^3 T^2 T 1 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0;
     0 0 0 0 0 0 0 0 (2*T)^7 (2*T)^6 (2*T)^5 (2*T)^4 (2*T)^3 (2*T)^2 (2*T) 1 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0;
     0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 (2*T)^7 (2*T)^6 (2*T)^5 (2*T)^4 (2*T)^3 (2*T)^2 (2*T) 1 0 0 0 0 0 0 0 0;
     0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 (3*T)^7 (3*T)^6 (3*T)^5 (3*T)^4 (3*T)^3 (3*T)^2 (3*T) 1 0 0 0 0 0 0 0 0;
     0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 (3*T)^7 (3*T)^6 (3*T)^5 (3*T)^4 (3*T)^3 (3*T)^2 (3*T) 1;
     0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 (4*T)^7 (4*T)^6 (4*T)^5 (4*T)^4 (4*T)^3 (4*T)^2 (4*T) 1;
     0 0 0 0 0 0 1 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0;
     0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 7*(4*T)^6 6*(4*T)^5 5*(4*T)^4 4*(4*T)^3 3*(4*T)^2 2*(4*T) 1 0;
     0 0 0 0 0 2 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0;
     0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 42*(4*T)^5 30*(4*T)^4 20*(4*T)^3 12*(4*T)^2 6*(4*T) 2 0 0;
     0 0 0 0 6 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0;
     0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 210*(4*T)^4 120*(4*T)^3 60*(4*T)^2 24*(4*T) 6 0 0 0;
     7*T^6 6*T^5 5*T^4 4*T^3 3*T^2 2*T 1 0 -7*T^6 -6*T^5 -5*T^4 -4*T^3 -3*T^2 -2*T -1 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0;
     0 0 0 0 0 0 0 0 7*(2*T)^6 6*(2*T)^5 5*(2*T)^4 4*(2*T)^3 3*(2*T)^2 2*(2*T) 1 0 -7*(2*T)^6 -6*(2*T)^5 -5*(2*T)^4 -4*(2*T)^3 -3*(2*T)^2 -2*(2*T) -1 0 0 0 0 0 0 0 0 0;
     0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 7*(3*T)^6 6*(3*T)^5 5*(3*T)^4 4*(3*T)^3 3*(3*T)^2 2*(3*T) 1 0 -7*(3*T)^6 -6*(3*T)^5 -5*(3*T)^4 -4*(3*T)^3 -3*(3*T)^2 -2*(3*T) -1 0;
     42*T^5 30*T^4 20*T^3 12*T^2 6*T 2 0 0 -42*T^5 -30*T^4 -20*T^3 -12*T^2 -6*T -2 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0;
     0 0 0 0 0 0 0 0 42*(2*T)^5 30*(2*T)^4 20*(2*T)^3 12*(2*T)^2 6*(2*T) 2 0 0 -42*(2*T)^5 -30*(2*T)^4 -20*(2*T)^3 -12*(2*T)^2 -6*(2*T) -2 0 0 0 0 0 0 0 0 0 0;
     0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 42*(3*T)^5 30*(3*T)^4 20*(3*T)^3 12*(3*T)^2 6*(3*T) 2 0 0 -42*(3*T)^5 -30*(3*T)^4 -20*(3*T)^3 -12*(3*T)^2 -6*(3*T) -2 0 0;
     210*T^4 120*T^3 60*T^2 24*T 6 0 0 0 -210*T^4 -120*T^3 -60*T^2 -24*T -6 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0;
     0 0 0 0 0 0 0 0 210*(2*T)^4 120*(2*T)^3 60*T^2 24*(2*T) 6 0 0 0 -210*(2*T)^4 -120*(2*T)^3 -60*T^2 -24*(2*T) -6 0 0 0 0 0 0 0 0 0 0 0;
     0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 210*(3*T)^4 120*(3*T)^3 60*(3*T)^2 24*(3*T) 6 0 0 0 -210*(3*T)^4 -120*(3*T)^3 -60*(3*T)^2 -24*(3*T) -6 0 0 0;
     840*T^3 360*T^2 120*T 24 0 0 0 0 -840*T^3 -360*T^2 -120*T -24 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0;
     0 0 0 0 0 0 0 0 840*(2*T)^3 360*(2*T)^2 120*(2*T) 24 0 0 0 0 -840*(2*T)^3 -360*(2*T)^2 -120*(2*T) -24 0 0 0 0 0 0 0 0 0 0 0 0;
     0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 840*(3*T)^3 360*(3*T)^2 120*(3*T) 24 0 0 0 0 -840*(3*T)^3 -360*(3*T)^2 -120*(3*T) -24 0 0 0 0;
     2520*T^2 720*T 120 0 0 0 0 0 -2520*T^2 -720*T -120 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0;
     0 0 0 0 0 0 0 0 2520*(2*T)^2 720*(2*T) 120 0 0 0 0 0 -2520*(2*T)^2 -720*(2*T) -120 0 0 0 0 0 0 0 0 0 0 0 0 0;
     0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 2520*(3*T)^2 720*(3*T) 120 0 0 0 0 0 -2520*(3*T)^2 -720*(3*T) -120 0 0 0 0 0;
     5040*T 720 0 0 0 0 0 0 -5040*T -720 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0;
     0 0 0 0 0 0 0 0 5040*(2*T) 720 0 0 0 0 0 0 -5040*(2*T) -720 0 0 0 0 0 0 0 0 0 0 0 0 0 0;
     0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 5040*(3*T) 720 0 0 0 0 0 0 -5040*(3*T) -720 0 0 0 0 0 0;];
 
    for i=1:3
    
        b = [waypoints0(i,1);
          waypoints0(i,2);
          waypoints0(i,2);
          waypoints0(i,3);
          waypoints0(i,3);
          waypoints0(i,4);
          waypoints0(i,4);
          waypoints0(i,5);
          0;
          0;
          0;
          0;
          0;
          0;
          0;
          0;
          0;
          0;
          0;
          0;
          0;
          0;
          0;
          0;
          0;
          0;
          0;
          0;
          0;
          0;
          0;
          0;];
        alpha(:,i) = A\b;
    end
    
else
    if(t > finish_time)
        t = finish_time;
    end
    
    if(t == 0)
        desired_state.pos = waypoints0(:,1);
        desired_state.vel = zeros(3,1);
        desired_state.acc = zeros(3,1);
        desired_state.yaw = 0;
        desired_state.yawdot = 0;
    else
        if(t<=T)
            for i=1:3
                desired_state.pos(i) = [t^7 t^6 t^5 t^4 t^3 t^2 t 1] * alpha(1:8,i);
                desired_state.vel(i) = [7*t^6 6*t^5 5*t^4 4*t^3 3*t^2 2*t 1 0] * alpha(1:8,i);
                desired_state.acc(i) = [42*t^5 30*t^4 20*t^3 12*t^2 6*t 2 0 0] * alpha(1:8,i);
                desired_state.yaw = 0;
                desired_state.yawdot = 0;
            end
        end
        
        if((t<=2*T)&&(t>T))
            for i = 1:3
                desired_state.pos(i) = [t^7 t^6 t^5 t^4 t^3 t^2 t 1] * alpha(9:16,i);
                desired_state.vel(i) = [7*t^6 6*t^5 5*t^4 4*t^3 3*t^2 2*t 1 0] * alpha(9:16,i);
                desired_state.acc(i) = [42*t^5 30*t^4 20*t^3 12*t^2 6*t 2 0 0] * alpha(9:16,i);
                desired_state.yaw = 0;
                desired_state.yawdot = 0;
            end
        end
        if((t<=3*T)&&(t>2*T))
            for i =1:3
                desired_state.pos(i) = [t^7 t^6 t^5 t^4 t^3 t^2 t 1] * alpha(17:24,i);
                desired_state.vel(i) = [7*t^6 6*t^5 5*t^4 4*t^3 3*t^2 2*t 1 0] * alpha(17:24,i);
                desired_state.acc(i) = [42*t^5 30*t^4 20*t^3 12*t^2 6*t 2 0 0] * alpha(17:24,i);
                desired_state.yaw = 0;
                desired_state.yawdot = 0;
            end
        end
        if((t<=4*T)&&(t>3*T))
            for i = 1:3
                desired_state.pos(i) = [t^7 t^6 t^5 t^4 t^3 t^2 t 1] * alpha(25:32,i);
                desired_state.vel(i) = [7*t^6 6*t^5 5*t^4 4*t^3 3*t^2 2*t 1 0] * alpha(25:32,i);
                desired_state.acc(i) = [42*t^5 30*t^4 20*t^3 12*t^2 6*t 2 0 0] * alpha(25:32,i);
                desired_state.yaw = 0;
                desired_state.yawdot = 0;
            end
        end
    end
end
     
end

