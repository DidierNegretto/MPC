%% System description
%%% Braking and Damping constant 
alpha = 6.6; % Braking constant
beta = 0.01; % Damping constant 

%%% ---- Continuous time dynamics --------
% dX/dt = f(X,U)
f = @(X,U) [X(2);5*U(1)-alpha*U(2)*X(2)-beta*(X(2))^3]; 
