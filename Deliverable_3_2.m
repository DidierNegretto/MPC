%% Clear workspace
yalmip('clear')
clear all
close all
clc

%% Generate trimmered and linearized version of the quad-copter (TODO 2.1)
Ts = 1/5;
quad = Quad(Ts);

[xs,us] = quad.trim();                      % Compute steady state for which 0 = f(xs,us)
sys = quad.linearize(xs, us);               % Linearize the nonlinear model
%% Decompose system (TODO 2.3) 

[sys_x, sys_y, sys_z, sys_yaw] = quad.decompose(sys, xs, us); 

%% DESIGN MPC CONTROLLER FOR EACH SUBSYSTEM (Part 3)
close all
clc
% Design MPC controller
mpc_x   = MPC_Control_x(sys_x, Ts);
mpc_y   = MPC_Control_y(sys_y, Ts);
mpc_yaw = MPC_Control_yaw(sys_yaw, Ts);
mpc_z   = MPC_Control_z(sys_z, Ts);


%% SIMULATE REFERENCE TRACKING CONTROLLERS (Deliverable 3.2)
clc
close all
x0 = zeros(12,1);
ref = [2 2 2 pi/4];
sim_controllers(mpc_x,mpc_y,mpc_z,mpc_yaw,x0,ref, Ts, 9, "del32");