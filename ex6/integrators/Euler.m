function [x_next] = Euler(X,U,h,f)
%
% Inputs : 
%    X, U current state and input
%    h    sample period
%    f    continuous time dynamics f(x,u)
% Returns
%    State h seconds in the future
%

% Euler integration
   x_next = X + h*f(X,U);
   
