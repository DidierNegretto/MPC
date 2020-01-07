%% Clear workspace
clear all
close all
clc

%% Generate trimmered and linearized version of the quad-copter (TODO 2.1)
Ts = 1/5;
quad = Quad(Ts);

[xs,us] = quad.trim();                      % Compute steady state for which 0 = f(xs,us)
sys = quad.linearize(xs, us);               % Linearize the nonlinear model

%% Apply transformation (TODO 2.2) (DELIVERABLE 2.1)
sys_transformed = sys*inv(quad.T);
disp(sys.A)
disp(sys_transformed.A)
disp(sys.B)
disp(sys_transformed.B)
disp(sys.C)
disp(sys_transformed.C)
disp(sys.D)
disp(sys_transformed.D)

% A,C and D do not change because they are not affected.
% B in sys has more than 1 non zero element per row, which implies that 
% more inputs sum up to control the state
% B in sys_transformed has at max one non-zero element per row, which
% implies that only one input can control one state