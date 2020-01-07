function [A] = jac_x(X0,U0,f)
% Write your jacobian function here using finite differences
%     A = ..
delta = 0.1;
    A1 = (f(X0+delta*[1;0],U0) - f(X0-delta*[1;0],U0))/(2*delta);
    A2 = (f(X0+delta*[0;1],U0) - f(X0-delta*[0;1],U0))/(2*delta);
    A = [A1,A2];
end
