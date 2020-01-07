clear; close all; clc;

Ex7_2_1 % Load your linearization routines

%%% ---> Your code here
% Maps from X,U => next state
f_linear_algo = @(X,U) error('Implement me');  % Linearized system using casadi
f_linear_fd   = @(X,U) error('Implement me');  % Linearized system using finite differences

A_al = full(A_algorithmic(X0,U0));
B_al = full(B_algorithmic(X0,U0));
f_linear_algo = @(X,U) A_al * (X - X0) + B_al * (U - U0) + f_discrete(X0,U0);  % Linearized system using casadi

A_fd = jac_x(X0,U0,f_discrete);
B_fd = jac_u(X0,U0,f_discrete);
f_linear_fd   = @(X,U) A_fd * (X - X0) + B_fd * (U - U0) + f_discrete(X0,U0);  % Linearized system using finite differences

%%% ---> Your code here


%% Simulate

sim_algo.name = 'Linearized with algorithmic differentiation';
sim_algo.f = f_linear_algo;

sim_fd.name = 'Linearized with finite differences';
sim_fd.f = f_linear_fd;

sim_rk4.name = 'Nonlinear RK4';
sim_rk4.f = f_discrete;

sims = {sim_algo, sim_fd, sim_rk4};

t = 0:h:10; % Sample times
X0 = [0;0.5]; % Initial state
U0 = 0; % Initial input
Uref = [0.5+0.5*sin(t);zeros(size(t))]; % Sample the input function at the sample period

for i = 1:length(sims)
  try
    sims{i}.X = X0;
    for k = 1:length(t)-1
      sims{i}.X(:,k+1) = sims{i}.f(sims{i}.X(:,k), Uref(:,k));
    end
  catch
    warning('Implement the function for %s', sims{i}.name);
  end
end

%% Plot results
plotSims(t, sims);
