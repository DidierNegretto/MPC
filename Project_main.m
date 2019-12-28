%% Clear workspace
clear all
close all
clc

%% Generate trimmered and linearized version of the quad-copter (TODO 2.1)
quad = Quad();

[xs,us] = quad.trim();                      % Compute steady?state for which 0 = f(xs,us)
sys = quad.linearize(xs, us);               % Linearize the nonlinear model

%% Apply transformation (TODO 2.2) (DELIVERABLE 2.1)
sys_transformed = sys*(quad.T)^-1            % New system is  A*x + B*inv(T)*v

%% Decompose system (TODO 2.3) 
[sys_x, sys_y, sys_z, sys_yaw] = quad.decompose(sys_transformed, xs, us);

%% DESIGN MPC CONTROLLER FOR EACH SUBSYSTEM (Part 3)

% Descretize the systems
sys_x_d = c2d(sys_x, 1/5);
sys_y_d = c2d(sys_y, 1/5);
sys_z_d = c2d(sys_z, 1/5);
sys_yaw_d = c2d(sys_yaw, 1/5);

