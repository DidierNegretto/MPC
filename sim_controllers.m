function [] = sim_controllers(quad,sys_x, sys_y, sys_yaw, sys_z, mpc_x,mpc_y,mpc_z,mpc_yaw, us, Ts, iters)
%sim_controllers Simulates the controllers by applying a from 2m to 0
%for X, Y,Z and yaw and plots the results.
%   quad, the quadricopter structure
%   mpc_x is the controller for the x direction
%   mpc_y is the controller for the y direction
%   mpc_z is the controller for the z direction
%   mpc_yaw is the controller for the yaw direction
%   us is the steady state input
%   Ts is the dt bteween two new control signals
%   iters is the number of iterations final time = iters*Ts
%

% Initial state
%       1   2   3  4 5 6  7   8   9 10 1112
% x = [a_d,b_d,g_d,a,b,g,x_d,y_d,z_d,x,y,z]

% Useful constants:
A_d = 1; A = 4;    X_d = 7; X = 10;
B_d = 2; B = 5;    Y_d = 8; Y = 11;
G_d = 3; G = 6;    Z_d = 9; Z = 12;

states_name = ["Ad", "Bd", "Gd","A","B","G", "Xd", "Yd", "Zd","X","Y","Z"];
x0 = zeros(12,1);
%x0(X) = 2;
%x0(Y) = -2;
x0(Z) = 2;
%x0(G) = -pi/4;
x = x0;

ref = zeros(12,1);
%ref(X) = 2;
%ref(Y) = 2;
ref(Z) = 2;
%ref(G) = -pi/4;

V = [0,0,0,0]';
Xdata = x;
disp("SIMULATING ...")
for i = 1:iters
    % Get new control inputs with
    M_B = mpc_x.get_u([x(B_d), x(B), x(X_d), x(X)]');%, [ref(B_d), ref(B), ref(X_d), ref(X)]'); 
    M_A = mpc_y.get_u([x(A_d), x(A), x(Y_d), x(Y)]');%, [ref(A_d), ref(A), ref(Y_d), ref(Y)]'); 
    F = mpc_z.get_u([x(Z_d), x(Z)]');%, [ref(Z_d), ref(Z)]');
    M_G = mpc_yaw.get_u([x(G_d), x(G)]');%,[ref(G_d), ref(G)]' );
    v = [F,M_A,M_B,-M_G]';
    %u = quad.T^-1*v.*20+us;
    
    % Input to apply
    %sim = ode45(@(t, x) quad.f(x, u), [(i-1)*Ts, i*Ts], x);
    
    
    % Store for plotting
    V = [V v];
    
    %{
    if i == 1
        SIM = sim;
    else
        SIM.x = [SIM.x sim.x];
        SIM.y = [SIM.y sim.y];
    end
    %}
    z = sys_z.A*[x(Z_d);x(Z)] + sys_z.B*F;
    xx = sys_x.A*[x(B_d);x(B); x(X_d); x(X)] + sys_x.B*M_B;
    y = sys_y.A*[x(A_d);x(A); x(Y_d); x(Y)] + sys_y.B*M_A;
    yaw = sys_yaw.A*[x(G_d);x(G)] + sys_yaw.B*M_G;
    x = [y(1), xx(1), yaw(1), y(2), xx(2), yaw(2), xx(3), y(3), z(1), xx(4), y(4), z(2)]';
    
    %jsdf = sys.A*[x(Z_d), x(Z)]' + sys.B*F;
    %x(Z_d) = jsdf(1);
    %x(Z) = jsdf(2);
    
    Xdata = [Xdata x];
    disp("step "+i+"/"+iters)
end

% Plot Drone
%quad.plot(SIM, [0,0,0,0]');    

%Plot control signal
%{
figure
hold on
plot([0:Ts:Ts*(iters)],U(1,:), 'ro-')
plot([0:Ts:Ts*(iters)],U(2,:), 'bo-')
plot([0:Ts:Ts*(iters)],U(3,:), 'go-')
plot([0:Ts:Ts*(iters)],U(4,:), 'co-')
legend("$u_1$", "$u_2$", "$u_3$", "$u_4$",'Interpreter','latex', ...
        'Location', 'BestOutside','Orientation','horizontal',...
        'FontSize', 18);
%}

figure
hold on
plot([0:Ts:Ts*(iters)],V(1,:), 'ro-')
plot([0:Ts:Ts*(iters)],V(2,:), 'bo-')
plot([0:Ts:Ts*(iters)],V(3,:), 'go-')
plot([0:Ts:Ts*(iters)],V(4,:), 'co-')
legend("$F$", "$M_\alpha$", "$M_\beta$", "$M_\gamma$",'Interpreter','latex', ...
        'Location', 'BestOutside','Orientation','horizontal',...
        'FontSize', 18);  
figure
hold on
plot([0:Ts:Ts*(iters)],Xdata(X,:), 'ro-')
plot([0:Ts:Ts*(iters)],Xdata(Y,:), 'go-')
plot([0:Ts:Ts*(iters)],Xdata(Z,:), 'co-')
legend("$x$", "$y$", "$z$", 'Interpreter','latex', ...
        'Location', 'BestOutside','Orientation','horizontal',...
        'FontSize', 18);

figure
hold on
plot([0:Ts:Ts*(iters)],Xdata(A,:)*180/pi, 'ro-')
plot([0:Ts:Ts*(iters)],Xdata(B,:)*180/pi, 'go-')
plot([0:Ts:Ts*(iters)],Xdata(G,:)*180/pi, 'co-')
legend("$\alpha$", "$\beta$", "$\gamma$", 'Interpreter','latex', ...
        'Location', 'BestOutside','Orientation','horizontal',...
        'FontSize', 18);

%disp([states_name' Xdata])
%disp(sys.B*inv(quad.T))
end

