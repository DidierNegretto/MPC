clear; close all; clc;

car_dynamics

%% Part 1: Define and test your integrators
addpath('./integrators')

%%% Part 1
%%% Define your integrator functions in the ./integrators folder

%%% Part 1 - 1 Defines f_{discrete} here
h = 0.01; % integration time step
f_discrete_RK4 = @(x,u) RK4(x,u,h,f);
f_discrete_euler = @(x,u) Euler(x,u,h,f);

%%% Part 1-2 Test the integration call
X0 = [0;0.5]; U0 = [0;-0.1];
try
  Xh_RK4 = f_discrete_RK4(X0,U0)
  Xh_euler = f_discrete_euler(X0,U0)
catch
  warning('You need to implement the functions RK4 and Euler in the integrators directory');
end

%%% If everything is correct, output should be
%%% Xh_RK4 = [0.0050,0.5033] and Xh_euler = [0.0050,0.5033]
