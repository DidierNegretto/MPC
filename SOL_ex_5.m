%% EX5 Offset-free setpoint trecking.

function ex5

yalmip('clear')
clear all
close all
clc

%% Problem input

% System dynamic 
A    = [0.7115 -0.4345;
        0.4345 0.8853];
    
B    = [0.2173;
        0.0573];

C    = [0 1];

% Input constraits
umin = -3;
umax = 3;

nx   = size(A,1);
nu   = size(B,2);
ny   = size(C,1);

%% MPC params

%horizon length
N    = 5;

% Step cost function
Q    = eye(2);
R    = 1;


%simulation steps
nsteps      = 50;

u           = sdpvar(repmat(nu,1,N),repmat(1,1,N));

x_hist      = zeros(nx,nsteps);
x_hat_hist  = zeros(nx,nsteps);

d_hat_hist  = zeros(1,nsteps);
u_hist      = zeros(nu,nsteps-1);

x_hist(:,1)     = [1;2]; %Initial condition
x_hat_hist(:,1) = [3;0]; %Initial estimate

d   = 0.2;    % Disturbances
r   = 1;      % Reference

%L = 0.5*eye(size(C,1)); % Observer matrix

%Note : design an L : (A+LC) is stable is equivalent to desing a K such
%that (A'+C'K). Thus we can use a pole placement

A_bar = [A          , zeros(nx,1);
         zeros(1,nx),1          ];
B_bar = [B;zeros(1,nu)];
C_bar = [C,ones(ny,1)];
    
L = -place(A_bar',C_bar',[0.5,0.6,0.7])';

P = dlyap(A,Q);

eigs(A_bar+L*C_bar)

for i = 1:nsteps-1
    fprintf('step %i \n',i);
    [xs, us] = compute_sp(A,B,C,R,r,d_hat_hist(:,i),umin,umax);
    
    %% Set up the MPC cost and constraints using the computed set-point
    constraints = [];
    objective   = 0;
    x           = x_hist(:,i);
    for k = 1:N
        x = A*x + B*u{k};
        objective   = objective + norm(chol(Q)*(x-xs),2) + norm(chol(R)*(u{k}-us),2);
        constraints = [constraints, umin <= u{k}<= umax];
    end
    objective = objective + (x-xs)'*P*(x-xs);
    
    
    %% Optimize
    diagnostics = solvesdp(constraints, objective,sdpsettings('verbose',0));
    
    if diagnostics.problem == 0
       % Good! 
    else
        throw(MException('',yalmiperror(diagnostics.problem)));
    end


    u_hist(:,i)   = double(u{1});
    
    %% Apply to the plant
    
    x_hist(:,i+1) = A*x_hist(:,i) + B*u_hist(:,i);
    y             = C*x_hist(:,i) + d            ;
    
    
    %% Off-set observer
    x_bar = [x_hat_hist(:,i);
             d_hat_hist(:,i)];
         
    x_bar_next = A_bar*x_bar + B_bar*u_hist(:,i) + L*(C_bar*x_bar-y);
    
    x_hat_hist(:,i+1) = x_bar_next(1:nx);
    d_hat_hist(:,i+1) = x_bar_next(nx+1);

     
end

plot(u_hist); hold on
plot(umax*ones(size(u_hist)),'--');
plot(umin*ones(size(u_hist)),'--'); 
legend('u','u_max','u_min');

figure

plot(x_hist(1,:),'r'); hold on 
plot(x_hist(2,:),'b'); 
plot(x_hat_hist(1,:),'r--'); hold on 
plot(x_hat_hist(2,:),'b--'); 
legend('x_1','x_2','\hat{x}_1','\hat{x}_2')
figure

plot(d*ones(size(d_hat_hist)),'--'); hold on
plot(d_hat_hist); 
legend('d','\hat{d}')

figure

plot(C*x_hist+d*ones(size(C*x_hist))); hold on
plot(r*ones(size(C*x_hist))); 
legend('y','r');


end

%% COMPUTE_SP compute a feasible set-point.
% Since the estimate of the disturbance change at every time-step, we
% compute a feasible setpoint every time solving the following minimization
% problem :
%
% [xsp,usp] = argmin(u)
%             s.t. xsp = A*xsp + B*usp
%                  rsp == C*xsp + d
%                  umin <= usp <= umax
%

function [xs, us] = compute_sp(A,B,C,R,r,d,umin,umax)

nx = size(A,1);
nu = size(B,2);

u = sdpvar(nu,1);
x = sdpvar(nx,1);

constraints = [umin <= u <= umax ,...
                x == A*x + B*u    ,...
                r == C*x + d      ];

objective   = u^2;
diagnostics = solvesdp(constraints,objective,sdpsettings('verbose',0));

if diagnostics.problem == 0
   % Good! 
elseif diagnostics.problem == 1
    throw(MException('','Infeasible'));
else
    throw(MException('','Something else happened'));
end

xs = double(x);
us = double(u);

end
