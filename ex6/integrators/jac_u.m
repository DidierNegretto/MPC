function [B] = jac_u(X0,U0,f)
% Write your jacobian function here using finite differences
% B = ...
delta = 0.1;
    B1 = (f(X0,U0+delta*[1;0]) - f(X0,U0-delta*[1;0]))/(2*delta);
    B2 = (f(X0,U0+delta*[0;1]) - f(X0,U0-delta*[0;1]))/(2*delta);
    B = [B1,B2];
end
