classdef MPC_Control_z < MPC_Control
  properties
    A_bar, B_bar, C_bar % Augmented system for disturbance rejection    
    L                   % Estimator gain for disturbance rejection
  end
  
  methods
    function mpc = MPC_Control_z(sys, Ts)
      mpc = mpc@MPC_Control(sys, Ts);
      
      [mpc.A_bar, mpc.B_bar, mpc.C_bar, mpc.L] = mpc.setup_estimator();
    end
    
    % Design a YALMIP optimizer object that takes a steady-state state
    % and input (xs, us) and returns a control input
    function ctrl_opt = setup_controller(mpc)

      %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
      % INPUTS
      %   x(:,1) - initial state (estimate)
      %   d_est  - disturbance estimate
      %   xs, us - steady-state target
      % OUTPUTS
      %   u(:,1) - input to apply to the system
      %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

      [n,m] = size(mpc.B);
      
      % Steady-state targets (Ignore this before Todo 3.3)
      xs = sdpvar(n, 1);
      us = sdpvar(m, 1);
      
      % Disturbance estimate (Ignore this before Part 5)
      d_est = sdpvar(1);

      % SET THE HORIZON HERE
      N = 20;
      
      % Predicted state and input trajectories
      x = sdpvar(n, N);
      u = sdpvar(m, N-1);
      

      %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
      % YOUR CODE HERE YOUR CODE HERE YOUR CODE HERE YOUR CODE HERE 

      % NOTE: The matrices mpc.A, mpc.B, mpc.C and mpc.D are 
      %       the DISCRETE-TIME MODEL of your system

      % SET THE PROBLEM CONSTRAINTS con AND THE OBJECTIVE obj HERE
      
      % sys_z. INPUT: u = F. STATE: z_dot,z
      
      % Cost matrices (as from ex_4)
      Q = [ 10 0
            0 1 ];
      R = 1;
      
      % Costraints matrices
      
      % u in U = { u | Mu <= m } 
      % Constraint on F 
      M = [1;-1]; m = [0.3; 0.2];
      % x in X = { x | Fx <= f }
      % NO CONSTRAINTS ON THE STATE !
      F = [1 0; 0 1;
           -1 0; 0 -1]; 
      f = [inf; inf; inf; inf];
    
      % Compute LQR controller for unconstrained system
      [K,Qf,~] = dlqr(mpc.A,mpc.B,Q,R);
      % MATLAB defines K as -K, so invert its signal
      K = -K; 
    
      % Compute maximal invariant set
      Xf = polytope([F;M*K],[f;m]); 
      Acl = [mpc.A+mpc.B*K];
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

      con = (x(:,2) == mpc.A*x(:,1) + mpc.B*u(:,1)) + (M*u(:,1) <= m);
      obj = u(:,1)'*R*u(:,1);
      
      for i = 2:N-1
        con = con + (x(:,i+1) == mpc.A*x(:,i) + mpc.B*u(:,i));
        con = con + (F*x(:,i) <= f) + (M*u(:,i) <= m);
        obj = obj + x(:,i)'*Q*x(:,i) + u(:,i)'*R*u(:,i);
      end
      con = con + (Ff*x(:,N) <= ff);
      obj = obj + x(:,N)'*Qf*x(:,N);
      
      % Plot invariant set
      %{
      figure
      Xf.projection(1:2).plot();
      figure
      Xf.projection(2:3).plot();
      figure
      Xf.projection(3:4).plot();
      %}
      
      disp("Controller Z setup finished")
      % YOUR CODE HERE YOUR CODE HERE YOUR CODE HERE YOUR CODE HERE 
      %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
      
      
      ctrl_opt = optimizer(con, obj, sdpsettings('solver','gurobi'), ...
        {x(:,1), xs, us, d_est}, u(:,1));
    end
    
    
    % Design a YALMIP optimizer object that takes a position reference
    % and returns a feasible steady-state state and input (xs, us)
    function target_opt = setup_steady_state_target(mpc)
      
      %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
      % INPUTS
      %   ref    - reference to track
      %   d_est  - disturbance estimate
      % OUTPUTS
      %   xs, us - steady-state target
      %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

      % Steady-state targets
      n = size(mpc.A,1);
      xs = sdpvar(n, 1);
      us = sdpvar;
      
      % Reference position (Ignore this before Todo 3.3)
      ref = sdpvar;
            
      % Disturbance estimate (Ignore this before Part 5)
      d_est = sdpvar(1);
      
      %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
      % YOUR CODE HERE YOUR CODE HERE YOUR CODE HERE YOUR CODE HERE 
      % You can use the matrices mpc.A, mpc.B, mpc.C and mpc.D
      con = [];
      obj = 0;
      %%{
      umin = -0.2;
      umax = 0.3;
      
      d = 1; % NO DISTURBANCES !!!
      
      constraints = [umin <= us <= umax ,...
                xs == mpc.A*xs + mpc.B*us    ,...
                ref == mpc.C*xs + d      ];

      objective   = us^2;
      diagnostics = solvesdp(constraints,objective,sdpsettings('verbose',0));

      if diagnostics.problem == 0
         % Good! 
      elseif diagnostics.problem == 1
          throw(MException('','Infeasible'));
      else
          throw(MException('','Something else happened'));
      end
      
      con = constraints;
      obj = objective;
      disp("Controller Z steady-state target computed")
      %}
      % YOUR CODE HERE YOUR CODE HERE YOUR CODE HERE YOUR CODE HERE 
      %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
      
      
      % Compute the steady-state target
      target_opt = optimizer(con, obj, sdpsettings('solver', 'gurobi'), {ref, d_est}, {xs, us});
    end
    
    
    % Compute augmented system and estimator gain for input disturbance rejection
    function [A_bar, B_bar, C_bar, L] = setup_estimator(mpc)
      
      %%% Design the matrices A_bar, B_bar, L, and C_bar
      %%% so that the estimate x_bar_next [ x_hat; disturbance_hat ]
      %%% converges to the correct state and constant input disturbance
      %%%   x_bar_next = A_bar * x_bar + B_bar * u + L * (C_bar * x_bar - y);
      
      %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
      % YOUR CODE HERE YOUR CODE HERE YOUR CODE HERE YOUR CODE HERE 
      % You can use the matrices mpc.A, mpc.B, mpc.C and mpc.D
      A_bar = [];
      B_bar= [];
      C_bar= []; 
      L= [];
      %%{
      % Inspired to solutions to exercice 5
      nx   = size(mpc.A,1);
      nu   = size(mpc.B,2);
      ny   = size(mpc.C,1);
      
      A_bar = [mpc.A          , zeros(nx,1);
                zeros(1,nx),1          ];
      B_bar = [mpc.B;zeros(1,nu)];
      C_bar = [mpc.C,ones(ny,1)];
      
      % WHAT SHOULD GO THERE ? ------V
      %L = -place(A_bar',C_bar',[0.3,0.4,0.50])';
      L=0.01;
      disp("ESTIMATOR setup finished")
      %}
      % YOUR CODE HERE YOUR CODE HERE YOUR CODE HERE YOUR CODE HERE 
      %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    end

    
  end
end
