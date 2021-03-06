%% Clear workspace
clear all
close all
clc

%% Generate trimmered and linearized version of the quad-copter (TODO 2.1)
quad = Quad();

[xs,us] = quad.trim();                      % Compute steady?state for which 0 = f(xs,us)
sys = quad.linearize(xs, us);               % Linearize the nonlinear model

%% Apply transformation (TODO 2.2) (DELIVERABLE 2.1)
systransformed = sys*(quad.T)^-1            % New system is  A*x + B*inv(T)*v

%% Decompose system (TODO 2.3) 
[sysx, sysy, sysz, sysyaw] = quad.decompose(sys, xs, us