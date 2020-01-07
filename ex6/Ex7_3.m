clear; close all;
import casadi.*
addpath('./plottingcode')

car_dynamics

%%%%% REMOVE
% h = 0.1;
h = 0.025;
f_discrete = @(x,u) RK4(x,u,h,f);
%%%%% REMOVE

car_dynamics

%%% Choose your track
track = generate_track('simple');
% track = generate_track('complex');

%% Ex 3 MPC problem setup

opti = casadi.Opti(); % Optimization problem

N = 10; % MPC horizon

% ---- decision variables ---------
X = opti.variable(2,N+1); % state trajectory variables
U = opti.variable(2,N);   % control trajectory (throttle, brake)

X0   = opti.parameter(2,1); % parameter variable for initial state
pos0 = X0(1); % Initial position and velocity
v0   = X0(2);

pos   = X(1,:);
speed = X(2,:);

epsilon_speed = opti.variable(1,N+1); % slack variable for speed constraint

% ---- objective ---------
opti.minimize(...
  -10*sum(X(2,:))  + ... % Max velocity
  0.1*U(1,:)*U(1,:)' + ... % Minimize accel
  10*U(2,:)*U(2,:)'   + ... % Minimize braking
  10000*(epsilon_speed(1,:)*epsilon_speed(1,:)' + sum(epsilon_speed))); % Soft constraints

% % Minimize braking
% opti.minimize(...
%   -10*sum(X(2,:))  + ... % Max velocity
%   0.1*U(1,:)*U(1,:)' + ... % Minimize accel
%   1000*U(2,:)*U(2,:)'   + ... % Minimize braking
%   10000*(epsilon_speed(1,:)*epsilon_speed(1,:)' + sum(epsilon_speed))); % Soft constraints


% ---- multiple shooting --------
for k=1:N % loop over control intervals
  
  %%%%% REMOVE
  opti.subject_to(X(:,k+1) == f_discrete(X(:,k), U(:,k)));
  %%%%% REMOVE
  
  %%%% WRITE YOUR DYNAMICS CONSTRAINT HERE
  %   opti.subject_to( ... );
  
end

% ---- path constraints -----------

limit = track.maxspeed;
opti.subject_to(speed  <=   limit(pos) + epsilon_speed); % track speed limit
opti.subject_to(0 <= U <= 1);  % control is limited

% ---- boundary conditions --------
opti.subject_to(pos(1)==pos0);   % use initial position
opti.subject_to(speed(1)==v0); % use initial speed
opti.subject_to(epsilon_speed >= 0); % slack lower bound

% Pass parameter values
opti.set_value(pos0, 0.0);
opti.set_value(v0, 0);

% ---- Setup solver NLP    ------
ops = struct;
ops.ipopt.print_level = 0;
ops.ipopt.tol = 1e-3;
opti.solver('ipopt', ops);
sol = opti.solve();   % actual solve

% ---- Plot predicted trajectory for debugging ------

figure(1); clf
title('Predicted trajectory from initial state')
hold on; grid on
plot(limit(sol.value(pos)),'k--','linewidth',2);
plot(sol.value(pos),'linewidth',2);
plot(sol.value(speed),'linewidth',2);
stairs(1:N, sol.value(U(1,:)),'linewidth',2);
stairs(1:N,-sol.value(U(2,:)),'linewidth',2);
legend('speed limit','pos','speed','throttle','brake','Location','northwest')

%% closed loop simulation

sim.x = [0;0]; % Start at position zero with zero speed
sim.u = [];
sim.t = 0;

figure(2); clf; plot_track(track); hndl = [];

i = 0;
while sim.x(1,end) < 1  %% Take one loop around the track
  i = i + 1;
  sim.t(end+1) = sim.t(end) + h; 
  
  % set position and speed variables to their current values
  opti.set_value(X0, sim.x(:,i));
  
  % ---- call the MPC controller ------
  sol = opti.solve();
  u_mpc = sol.value(U(:,1)); % Get the MPC input
  
  % ---- Simulate the system
  next = ode45(@(t,x) f(x,u_mpc), [0, h], sim.x(:,i));
  sim.x(:,i+1) = next.y(:,end);
  sim.u(:,i) = u_mpc;
  
  % ---- Plot the car (delete this line if it's taking too long)
  hndl = plot_car(track, sol.value(pos(1)), sol.value(pos(2:end)), hndl);
  %   pause(0.05); drawnow
  
  % warm start the next iteration
  opti.set_initial(U, sol.value(U));
  opti.set_initial(X, sol.value(X));
  opti.set_initial(epsilon_speed, sol.value(epsilon_speed));
end

%% Plots for the closed loop results
figure(3); clf; hold on; grid on;
plot(sim.t, limit(sim.x(1,:)),'k--', 'linewidth',4); % Speed constraint
plot(sim.t, sim.x(1,:), 'linewidth',2); % Position
plot(sim.t, sim.x(2,:), 'linewidth',2); % Speed

stairs(sim.t(1:end-1), sim.u(1,:),'linewidth',2); % Acceleration
stairs(sim.t(1:end-1),-sim.u(2,:),'linewidth',2); % Braking

legend('speed limit','pos','speed','throttle','brake','Location','northwest')
title('Closed loop trajectory')

xlabel('Time (s)')
