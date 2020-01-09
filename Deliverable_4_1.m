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


%% REFERENCE TRACKING (Deliverable 4.1)
clc
close all
sim = quad.sim(mpc_x, mpc_y, mpc_z, mpc_yaw);
quad.plot(sim);

figHandle = get(groot, 'Children');
figHandle(2).set('units','normalized','outerposition',[0 0 1 1]);
saveas(figHandle(2),"fig\del41\meas.eps","epsc")
saveas(figHandle(1),"fig\del41\3d.eps","epsc")

