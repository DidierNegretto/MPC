%% Clear workspace
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
u = [1.5;1;1;1]; %positive yaw + negative pitch + go up
sim = ode45( @(t,x) quad.f(x,u), [0, Tf], x0);
quad.plot(sim,u);

%% Generate trimmered and linearized version of the quad-copter (TODO 2.1)
Ts = 1/5;
quad = Quad(Ts);

[xs,us] = quad.trim();                      % Compute steady state for which 0 = f(xs,us)
sys = quad.linearize(xs, us);               % Linearize the nonlinear model

%% Apply transformation (TODO 2.2) (DELIVERABLE 2.1)
sys_transformed = sys*inv(quad.T);           % New system is  A*x + B*inv(T)*v
disp(sys.A)
disp(sys_transformed.A)
disp(sys.B)
disp(sys_transformed.B)
disp(sys.C)
disp(sys_transformed.C)
disp(sys.D)
disp(sys_transformed.D)
