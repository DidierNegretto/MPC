%% Clear workspace
yalmip('clear')
clear all
close all
clc

%% Todo 1.2
quad = Quad();
Tf = 1.0;

x0 = zeros(12,1);
%u = [0;1;0;1]; %just go up
%u = [1;0;1;0]; %positive yaw + go up
%u = [0;1;0;1]; %negative yaw + go down
%u = [1.5;1;1;1]; %positive yaw + negative pitch + go up
%sim = ode45( @(t,x) quad.f(x,u), [0, Tf], x0);
%quad.plot(sim,u);


%% Generate trimmered and linearized version of the quad-copter (TODO 2.1)
quad = Quad();

[xs,us] = quad.trim();                      % Compute steady state for which 0 = f(xs,us)
sys = quad.linearize(xs, us);               % Linearize the nonlinear model

%% Apply transformation (TODO 2.2) (DELIVERABLE 2.1)
sys_transformed = sys*inv(quad.T);           % New system is  A*x + B*inv(T)*v

%Our answer : 
% Check : sys_transfomed.A ; sys_transformed.B ; sys_transformed.C ; sys_transformed.D
% The 4 components are separated : In the framed of quadcopter z, roll, pitch and yaw are decoupled
% To find this property somewhere else, this must first be a stable point ( x = f(x,u) )

%% Decompose system (TODO 2.3) 

[sys_x, sys_y, sys_z, sys_yaw] = quad.decompose(sys, xs, us); %sys and not 'sys_transformed

%% DESIGN MPC CONTROLLER FOR EACH SUBSYSTEM (Part 3)
Ts = 1/5;

% Design MPC controller
mpc_x   = MPC_Control_x(sys_x, Ts); %Note : each system will be discretized inside the control function
mpc_y   = MPC_Control_y(sys_y, Ts);
mpc_z   = MPC_Control_z(sys_z, Ts);
mpc_yaw = MPC_Control_yaw(sys_yaw, Ts);

% Get control inputs with
u_z   = mpc_z.get_u([0,0]');
u_x   = mpc_x.get_u([0,0,0,2]'); %mpc in x seems fine
%%



%% FOR FUTURE

sim = quad.sim(mpc_x, mpc_y, mpc_z, mpc_yaw);
quad.plot(sim);
