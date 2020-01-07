clear; close all; clc;

car_dynamics
h = 0.5;

% Load your RK4 integrator from Ex 1
addpath('./integrators')
f_discrete = @(x,u) RK4(x,u,h,f);

%% Exercise 2.1: Linearizing a nonlinear function

%%% Compute the jacobians using algorithmic differentiation
% Note syntax for using casadi from this template for future use
import casadi.*   % import casadi once before using

X0 = SX.sym('X0',2,1); % declare a symbolic variable X0 of size 2x1
U0 = SX.sym('U0',2,1); % declare a symbolic variable U0 of size 2x1

x_next = f_discrete(X0,U0); % Calculate the next step symbolically

A_jac = jacobian(x_next,X0); % returns jacobian for an expression (x_next) with respect to X0
B_jac = jacobian(x_next,U0); % returns jacobian for an expression (x_dot) with respect to U0

% convert A_algorithmic, B_algorithmic expressions to a callable functions
A_algorithmic = casadi.Function('A_algorithmic',{X0,U0},{A_jac});
B_algorithmic = casadi.Function('A_algorithmic',{X0,U0},{B_jac});

%% Compare the algorithmic results to finite differences

X0 = [1;0.5]; U0 = [0;0]; % assign some numeric values now to compute jacobians using the function

fprintf('A_algorithmic(X0,U0)=\n')
disp(full(A_algorithmic(X0,U0)))
fprintf('B_algorithmic(X0,U0)=\n')
disp(full(B_algorithmic(X0,U0)))

try
  fprintf('A_finite_diff(X0,U0)=\n')
  disp(jac_x(X0,U0,f_discrete))
  fprintf('B_finite_diff(X0,U0)=\n')
  disp(jac_u(X0,U0,f_discrete))
catch
  warning('Implement the functions jac_x and jac_u to compute the jacobians using finite differences');
end
