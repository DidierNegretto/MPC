%% Clear workspace
clear all
close all
clc

%% Generate trimmered and linearized version of the quad-copter (TODO 2.1)
quad = Quad();

[xs,us] = quad.trim();                      % Compute steady?state for which 0 = f(xs,us)
sys = quad.linearize(xs, us);               % Linearize the nonlinear model

%% Apply transformation (TODO 2.2) (DELIVERABLE 2.1)
sys_transformed = sys*(quad.T)^-1;           % New system is  A*x + B*inv(T)*v

%% Decompose system (TODO 2.3) 
[sys_x, sys_y, sys_z, sys_yaw] = quad.decompose(sys_transformed, xs, us);

%% DESIGN MPC CONTROLLER FOR EACH SUBSYSTEM (Part 3)
Ts = 1/5;

% Design MPC controller
mpc_x   = MPC_Control_x(sys_x, Ts);
mpc_y   = MPC_Control_y(sys_y, Ts);
mpc_z   = MPC_Control_z(sys_z, Ts);
mpc_yaw = MPC_Control_yaw(sys_yaw, Ts);

% Get control inputs with
u_z   = mpc_z.get_u([0,0]');
u_x   = mpc_x.get_u([0,0,0,2]');

%% FOR FUTURE

sim = quad.sim(mpc_x, mpc_y, mpc_z, mpc_yaw);
quad.plot(sim);
