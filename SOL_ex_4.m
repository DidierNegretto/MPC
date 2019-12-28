%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% EPFL | Semester: fall 2019                    %
% ME-425: Model Predictive Control | Exercise 4 %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Preliminaries

clear all
close all
clc

% YALMIP or quadprog?
USEYALMIP = true; 

% System dynamics
A = [0.9752 1.4544; ... 
    -0.0327 0.9315];
B = [0.0248; 0.0327];

% Initial condition
x0 = [3; 0];

% Horizon and cost matrices
N = 10;
Q = 10 * eye(2);
R = 1;

% Constraints
% u in U = { u | Mu <= m }
M = [1;-1]; m = [1.75; 1.75];
% x in X = { x | Fx <= f }
F = [1 0; 0 1; -1 0; 0 -1]; f = [5; 0.2; 5; 0.2];

% Compute LQR controller for unconstrained system
[K,Qf,~] = dlqr(A,B,Q,R);
% MATLAB defines K as -K, so invert its signal
K = -K; 

% Compute maximal invariant set
Xf = polytope([F;M*K],[f;m]);
Acl = [A+B*K];
while 1
    prevXf = Xf;
    [T,t] = double(Xf);
    preXf = polytope(T*Acl,t);
    Xf = intersect(Xf, preXf);
    if isequal(prevXf, Xf)
        break
    end
end
[Ff,ff] = double(Xf);

% MPT version
sys = LTISystem('A',A,'B',B);
sys.x.min = [-5; -0.2]; sys.x.max = [5; 0.2];
sys.u.min = [-1.75]; sys.u.max = [1.75];
sys.x.penalty = QuadFunction(Q); sys.u.penalty = QuadFunction(R);

Xf_mpt = sys.LQRSet;
Qf_mpt = sys.LQRPenalty;

% check by yourself if they are equal...

% Visualizing the sets
figure
hold on; grid on;
plot(polytope(F,f),'g'); plot(Xf,'r');
xlabel('position'); ylabel('velocity');

%% Defining the MPC controller

if USEYALMIP
    
    x = sdpvar(2,N,'full');
    u = sdpvar(1,N-1,'full');
    
    con = (x(:,2) == A*x(:,1) + B*u(:,1)) + (M*u(:,1) <= m);
    obj = u(:,1)'*R*u(:,1);
    for i = 2:N-1
        con = con + (x(:,i+1) == A*x(:,i) + B*u(:,i));
        con = con + (F*x(:,i) <= f) + (M*u(:,i) <= m);
        obj = obj + x(:,i)'*Q*x(:,i) + u(:,i)'*R*u(:,i);
    end
    con = con + (Ff*x(:,N) <= ff);
    obj = obj + x(:,N)'*Qf*x(:,N);
    
    % Compile the matrices
    ctrl = optimizer(con, obj, sdpsettings('solver','sedumi'), x(:,1), u(:,1));
    
else
    
    % Formulate matrices
    n = size(A,1); nu = size(B,2);
    % Linear constraints T*z == t*x0
    T = [kron(eye(N),eye(n)) + [zeros(n,n*N); kron(eye(N-1),-A) zeros((N-1)*n,n)]];
    T = [T kron(eye(N),-B)];
    t = [A; zeros((N-1)*n,n)];
    
    % Inequality constraints G*z <= g
    G = blkdiag(kron(eye(N-1),F), Ff, kron(eye(N),M));
    g = [kron(ones(N-1,1),f); ff; kron(ones(N,1),m)];
    
    % Cost function z'*H*z
    H = blkdiag(kron(eye(N-1),Q),Qf,kron(eye(N),R));
    
end

%% Simulating the closed-loop system

sol.x(:,1) = x0;
i = 1;
try
    while norm(sol.x(:,end)) > 1e-3 % Simulate until convergence
        % Solve MPC problem for current state
        if USEYALMIP
            
            [uopt,infeasible] = ctrl{sol.x(:,i)};
            
            if infeasible == 1, error('Error in optimizer - could not solve the problem'); end
            
            % Extract the optimal input
            sol.u(:,i) = uopt;
        else
            [z,fval,flag] = quadprog(H,zeros(size(H,1),1),G,g,T,t*sol.x(:,i));
            if flag ~= 1, error('Error in optimizer - could not solve the problem'); end
            
            % Extract the optimal input
            sol.u(:,i) = z(N*n+1:N*n+nu);
        end
        
        % Apply the optimal input to the system
        sol.x(:,i+1) = A*sol.x(:,i) + B*sol.u(:,i);
        
        i = i + 1;
    end
catch
    error('---> Initial state is outside the feasible set <---\n');
end

%% Plotting the results

figure
hold on; grid on;

o = ones(1,size(sol.x,2));

subplot(3,1,1)
hold on; grid on;
plot(sol.x(1,:),'-k','markersize',20,'linewidth',2);
plot(1:size(sol.x,2),f(1)*o,'r','linewidth',2)
plot(1:size(sol.x,2),-f(3)*o,'r','linewidth',2)
ylabel('Position')

subplot(3,1,2)
hold on; grid on;
plot(sol.x(2,:),'-k','markersize',20,'linewidth',2);
plot(1:size(sol.x,2),f(2)*o,'r','linewidth',2)
plot(1:size(sol.x,2),-f(4)*o,'r','linewidth',2)
ylabel('Velocity')

subplot(3,1,3)
o = ones(1,size(sol.u,2));
hold on; grid on;
plot(sol.u,'k','markersize',20,'linewidth',2);
plot(1:size(sol.u,2),m(1)*o,'r','linewidth',2)
plot(1:size(sol.u,2),-m(2)*o,'r','linewidth',2)
ylabel('Input')