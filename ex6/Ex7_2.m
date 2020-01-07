
%% Compute linearized dynamics

A_fd = jac_x(X0,U0,f);
B_fd = jac_u(X0,U0,f);

A = full(A_auto_function(X0,U0));
B = full(B_auto_function(X0,U0));

f_lin_fd   = @(X,U) ... ; % using finite difference
f_lin_auto = @(X,U) ... ; % using algorithmic derivatives

%% Simulate
h1 = 0.1;
Uref = @(t) [0.5+0.5*sin(t);0];

N_h1 = floor(10/h1);
f_discrete_auto = @(x,u) RK4(x,u,h1,f_lin_auto);
f_discrete_fd = @(x,u) RK4(x,u,h1,f_lin_fd);
f_discrete = @(x,u) RK4(x,u,h1,f);

X = []; X_auto = []; X_fd = []; t=[];
% Run for h1 
x0 = X0; x0_auto = X0; x0_fd = X0; t0 = 0; % initialize
for k=1:N_h1-1
% simulate the automatic differentiation system
Xstep = f_discrete_auto(x0_auto,Uref(t0));
X_auto=[X_auto,Xstep]; 
% simulate the finite difference system
Xstep = f_discrete_fd(x0_fd,Uref(t0));
X_fd=[X_fd,Xstep];
% simulate the nonlinear system
Xstep = f_discrete(x0,Uref(t0));
X=[X,Xstep]; 
t = [t,t0+h1]; % collect results for plotting
% update for next iteration
x0_auto = X_auto(:,end); 
x0_fd = X_fd(:,end);
x0 = X(:,end);
t0 = t(end); 
end

% plot the trajectories
figure(2)
subplot(1,2,1)
l0 = plot(t,X(1,:),'linewidth',2);
xlabel('t (s)')
ylabel('\lambda')
subplot(1,2,2)
plot(t,X(2,:),'linewidth',2);
xlabel('t (s)')
ylabel('v')

% plot the RK4 results


subplot(1,2,2)
hold on
plot(t,X_auto(2,:),'linewidth',2);
figure(2)
subplot(1,2,2)
hold on
plot(t,X_fd(2,:),'--','linewidth',2);

figure(2)
subplot(1,2,1)
hold on
l21 = plot(t,X_auto(1,:),'linewidth',2);

subplot(1,2,1)
hold on
l22 = plot(t,X_fd(1,:),'--','linewidth',2);
legend([l0,l21,l22],'nonlinear','auto. diff.','finite diff.','location','northwest')



