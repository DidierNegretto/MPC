function sim_controllers(mpc_x,mpc_y,mpc_z,mpc_yaw, x0, ref, Ts, duration, folder)
%sim_controllers Simulates the controllers by applying a from 2m to 0
%for X, Y,Z and yaw and plots the results.
%   mpc_x is the controller for the x direction
%   mpc_y is the controller for the y direction
%   mpc_z is the controller for the z direction
%   mpc_yaw is the controller for the yaw direction
%   x0 is the initial state
%   ref is a reference point to track 
%   Ts is the dt bteween two new control signals
%   duration is the duration of the simulation
%   folder is the folder in ".\fig" the figures shall be saved in


% State variable index:
A_d = 1; A = 4;    X_d = 7; X = 10;
B_d = 2; B = 5;    Y_d = 8; Y = 11;
G_d = 3; G = 6;    Z_d = 9; Z = 12;

% setting up inital state and referance
x = x0;

ref_X = ref(1);
ref_Y = ref(2);
ref_Z = ref(3);
ref_G = ref(4);

iters = floor(duration/Ts);

% allocating variables
V = zeros(4,iters);
Xdata = zeros(12,iters);


disp("Simulating ...")
for i = 1:iters
    % Get new control inputs with
    M_B = mpc_x.get_u([x(B_d), x(B), x(X_d), x(X)]', ref_X); 
    M_A = mpc_y.get_u([x(A_d), x(A), x(Y_d), x(Y)]', ref_Y);
    F = mpc_z.get_u([x(Z_d), x(Z)]', ref_Z);
    M_G = mpc_yaw.get_u([x(G_d), x(G)]', ref_G);
    v = [F,M_A,M_B,-M_G]';
    
    
    % Simulate one step
    z = mpc_z.A*[x(Z_d);x(Z)] + mpc_z.B*F;
    xx = mpc_x.A*[x(B_d);x(B); x(X_d); x(X)] + mpc_x.B*M_B;
    y = mpc_y.A*[x(A_d);x(A); x(Y_d); x(Y)] + mpc_y.B*M_A;
    yaw = mpc_yaw.A*[x(G_d);x(G)] + mpc_yaw.B*M_G;
    x = [y(1), xx(1), yaw(1), y(2), xx(2), yaw(2), xx(3), y(3), z(1), xx(4), y(4), z(2)]';
    
    
    % Store for plotting
    V(:,i) = v; 
    Xdata(:,i) = x;
    
    % Display progression 
    if (mod(i,10) == 0)
        disp("step "+i+"/"+iters)
    end
end




%Plot control signal
figure
hold on
grid on
plot([0:Ts:Ts*(iters - 1)],V(1,:), 'ro-')
plot([0:Ts:Ts*(iters - 1)],V(2,:), 'bo-')
plot([0:Ts:Ts*(iters - 1)],V(3,:), 'go-')
plot([0:Ts:Ts*(iters - 1)],V(4,:), 'co-')
line([8,8],[-0.3,0.3],'Color','red','LineWidth', 3)
legend("$F$", "$M_\alpha$", "$M_\beta$", "$M_\gamma$",'Interpreter','latex', ...
        'Location', 'BestOutside','Orientation','horizontal',...
        'FontSize', 18);  
title("\textbf{Control signals}"...
         , 'FontSize', 20, 'Interpreter','latex');
xlabel("Time [s]")
ylabel("Force [N] or Torque [N/m]")
saveas(gcf,"fig\" + folder + "\ctrlSig.eps","epsc")
    
figure
hold on
grid on
plot([0:Ts:Ts*(iters - 1)],Xdata(X,:), 'ro-')
plot([0:Ts:Ts*(iters - 1)],Xdata(Y,:), 'go-')
plot([0:Ts:Ts*(iters - 1)],Xdata(Z,:), 'co-')

line([8,8],ylim,'Color','red','LineWidth', 3)
legend("$x$", "$y$", "$z$", 'Interpreter','latex', ...
        'Location', 'BestOutside','Orientation','horizontal',...
        'FontSize', 18);
title("\textbf{Position}"...
         , 'FontSize', 20, 'Interpreter','latex');
xlabel("Time [s]")
ylabel("Position [m]")
saveas(gcf,"fig\" + folder + "\pos.eps","epsc")

figure
hold on
grid on
plot([0:Ts:Ts*(iters - 1)],Xdata(A,:)*180/pi, 'ro-')
plot([0:Ts:Ts*(iters - 1)],Xdata(B,:)*180/pi, 'go-')
plot([0:Ts:Ts*(iters - 1)],Xdata(G,:)*180/pi, 'co-')
line([8,8],[-10,50],'Color','red','LineWidth', 3)
legend("$\alpha$", "$\beta$", "$\gamma$", 'Interpreter','latex', ...
        'Location', 'BestOutside','Orientation','horizontal',...
        'FontSize', 18);
title("\textbf{Orientation}"...
         , 'FontSize', 20, 'Interpreter','latex');
xlabel("Time [s]")
ylabel("Angle [deg]")
saveas(gcf,"fig\" + folder + "\ori.eps","epsc")

end

