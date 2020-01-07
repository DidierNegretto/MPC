classdef MPC_Control_y < MPC_Control
  
  methods
    % Design a YALMIP optimizer object that takes a steady-state state
    % and input (xs, us) and returns a control input
    function ctrl_opt = setup_controller(mpc)

      %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
      % INPUTS
      %   x(:,1) - initial state (estimate)
      %   xs, us - steady-state target
      % OUTPUTS
      %   u(:,1) - input to apply to the system
      %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

      [n,m] = size(mpc.B);
      
      % Steady-state targets (Ignore this before Todo 3.2)
      xs = sdpvar(n, 1);
      us = sdpvar(m, 1);
      
      % SET THE HORIZON HERE
      N = 20;
      
      % Predicted state and input trajectories
      x = sdpvar(n, N);
      u = sdpvar(m, N-1);
      

      %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
      % YOUR CODE HERE YOUR CODE HERE YOUR CODE HERE YOUR CODE HERE 

      % NOTE: The matrices mpc.A, mpc.B, mpc.C and mpc.D are 
      %       the DISCRETE-TIME MODEL of your system

      % WRITE THE CONSTRAINTS AND OBJECTIVE HERE
      
      % sys_y. INPUT: u = Ma. STATE: alpha_dot,alpha,y_dot,y
      
      % Cost matrices (as from ex_4)
      Q = [10 0 0 0 
           0 0 0 0
           0 0 5 0
           0 0 0 10]; 
      R = 0.5;
      
      % Costraints matrices
      
      % u in U = { u | Mu <= m } 
      % Constraint on Ma 
      M = [1;-1]; m = [0.3; 0.3];
      % x in X = { x | Fx <= f }
      % Constraints on alpha
      F = [1 0 0 0; 0 1 0 0; 0 0 1 0; 0 0 0 1;
           -1 0 0 0; 0 -1 0 0; 0 0 -1 0; 0 0 0 -1]; 
      f = [inf; inf; inf; inf; inf; inf; inf; inf];
      f(2) = 0.035;
      f(6) = 0.035;
      
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
      obj = (u(:,1)-us)'*R*(u(:,1)-us);
      
      for i = 2:N-1
        con = con + (x(:,i+1) == mpc.A*x(:,i) + mpc.B*u(:,i));
        con = con + (F*x(:,i) <= f) + (M*u(:,i) <= m);
        obj = obj + (x(:,i)-xs)'*Q*(x(:,i)-xs) + (u(:,i)-us)'*R*(u(:,i)-us);
      end
      con = con + (Ff*x(:,N) <= ff);
      obj = obj + (x(:,N)-xs)'*Qf*(x(:,N)-xs);
      
      % Plot invariant set
      %%{
      figure
      sgtitle("\textbf{Controller Y invariant set}"...
         , 'FontSize', 20, 'Interpreter','latex');
      subplot(2,2,1)
      Xf.projection(1:2).plot();
      subplot(2,2,2)
      Xf.projection(2:3).plot();
      subplot(2,2,3)
      Xf.projection(3:4).plot();
      %}

      
      disp("Controller Y setup finished")
      % YOUR CODE HERE YOUR CODE HERE YOUR CODE HERE YOUR CODE HERE 
      %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
      
      
      ctrl_opt = optimizer(con, obj, sdpsettings('solver','gurobi'), ...
        {x(:,1), xs, us}, u(:,1));
    end
    
    
    % Design a YALMIP optimizer object that takes a position reference
    % and returns a feasible steady-state state and input (xs, us)
    function target_opt = setup_steady_state_target(mpc)

      %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
      % INPUTS
      %   ref    - reference to track
      % OUTPUTS
      %   xs, us - steady-state target
      %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

      % Steady-state targets
      n = size(mpc.A,1);
      xs = sdpvar(n, 1);
      us = sdpvar;
      
      % Reference position (Ignore this before Todo 3.2)
      ref = sdpvar;            
            
      %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
      % YOUR CODE HERE YOUR CODE HERE YOUR CODE HERE YOUR CODE HERE 
      % You can use the matrices mpc.A, mpc.B, mpc.C and mpc.D
      con = [];
      obj = 0;
      %%{
      umin = -0.3;
      umax = 0.3;
      
      d = 0; % NO DISTURBANCES !!!
      
      nx = size(mpc.A,1);
      nu = size(mpc.B,2);

      

      con = [umin <= us <= umax ,...
                      xs == mpc.A*xs + mpc.B*us    ,...
                      ref == mpc.C*xs + d      ];

      obj   = us^2;
      
      disp("Controller Y steady-state target computed")
      %}
      % YOUR CODE HERE YOUR CODE HERE YOUR CODE HERE YOUR CODE HERE 
      %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
      
      % Compute the steady-state target
      target_opt = optimizer(con, obj, sdpsettings('solver', 'gurobi'), ref, {xs, us});
      
    end
  end
end
