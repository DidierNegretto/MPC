classdef MPC_Control_x < MPC_Control
  
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
      N = 10; %maybe adapt !!!???
      
      % Predicted state and input trajectories
      x = sdpvar(n, N);
      u = sdpvar(m, N-1);
      
      %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
      % YOUR CODE HERE YOUR CODE HERE YOUR CODE HERE YOUR CODE HERE 

      % NOTE: The matrices mpc.A, mpc.B, mpc.C and mpc.D are 
      %       the DISCRETE-TIME MODEL of your system

      % WRITE THE CONSTRAINTS AND OBJECTIVE HERE :    
      % Note:   sys_x.  INPUT: u = Mb.  STATE: x,x_dot,beta,beta_dot
      
      % Cost matrices (as from ex_4)
      Q = 10 * eye(n);   %maybe to change ??!! but seems ok for now
      R = 0.1;
      
      % 1.) Costraints matrices
   
      % 1.1) u in U = { u | Mu <= m } 
      % Constraint on Mb 
      M = [1;-1]; m = [0.3; 0.3];  %x corresponds to Beta. to check but seems ok!
      % 1.2) x in X = { x | Fx <= f }
      % NO CONSTRAINTS ON THE STATE !
      F = [ 1 0 0 0 ; 0  1 0 0 ; 0 0  1 0 ; 0 0 0  1;
           -1 0 0 0 ; 0 -1 0 0 ; 0 0 -1 0 ; 0 0 0 -1]; 
      f = ones( 8, 1)*inf; 
    
      % 2.) Compute LQR controller for unconstrained system
      [K,Qf,~] = dlqr(mpc.A,mpc.B,Q,R);
      K = -K;   % MATLAB defines K as -K, so invert its signal
    
      % 3.) Compute maximal invariant set Xf
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
        
      % 4.) Compute con and obj
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

      %% Set up the MPC cost and constraints using the computed set-point (from ex5 sol)
      nx   = size(mpc.A,1);
      nu   = size(mpc.B,2);
      ny   = size(mpc.C,1);
      N = 10;
      
      % Cost matrices (as from ex_4)
      Q = 0 * eye(nx);   %maybe to change ??!!
      R = 1;
      %x_hist = zeros(nx,5); %5 for now, we will probably not need this variable anyway
      %x_hist(:,1) = x(:,1); %Initial condition
      %P = dlyap(mpc.A',Q);    % There is no solution of this lyapounov equation, maybe we should change Q ???
      % Trying with Ricatti equation (Practical MPC slides )
      [P,K,L,info] = idare(mpc.A,mpc.B,Q,R);
      disp(info)
      
      con = [];
      obj   = 0;
      x           = zeros(nx,1);  %TO CHANGE : here we must put the initial condition !!!
      
      for k = 1:N
          x = A*x + B*u{k};
          obj   = obj + norm(chol(Q)*(x-xs),2) + norm(chol(R)*(u{k}-us),2);
          con = [con, umin <= u{k}<= umax];
      end
      obj = obj + (x-xs)'*P*(x-xs);

      % YOUR CODE HERE YOUR CODE HERE YOUR CODE HERE YOUR CODE HERE 
      %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
      
      % Compute the steady-state target
      target_opt = optimizer(con, obj, sdpsettings('solver', 'gurobi'), ref, {xs, us});
      
    end
  end
end
