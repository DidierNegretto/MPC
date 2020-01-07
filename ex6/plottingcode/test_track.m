clear

%% Figure 8
f = @(th) [pi*sin(th*2*pi);sin(2*th*2*pi)];

%% Figure 8 x 2
% f = @(th) [pi*sin(th*2*pi);sin(4*th*2*pi)];

%% Leminscate
% a = 1;
% f = @(t) a*sqrt(2)*[cos(2*pi*t)./(sin(2*pi*t).^2 + 1); cos(2*pi*t).*sin(2*pi*t)./(sin(2*pi*t).^2+1)];

%% Random track
% syms s
% N = 3;
% for i = 1:N
%   basis(i,1) = sin(2*pi*i*s);
% end
% 
% basis = matlabFunction(basis);
% wx = randn(N,1);
% wy = randn(N,1);
% f = @(t) [wx'*basis(t); wy'*basis(t)];

%% Generate the track
track = generate_track(f);

%% Plot of the speed constraint
figure(1); clf
t = linspace(0,1,1e3);
yyaxis left
plot(t,track.maxspeed(t),'linewidth',2)
ylim([0,1])
hold on

yyaxis right
plot(t,track.kappa(t),'linewidth',2)
legend('Max speed', 'curvature')
grid on

%% Example of how to show the car moving around
figure(2); clf
plot_track(track);
h = []
tic
for pos = linspace(0,1,100)
  pred = linspace(pos,pos+0.2,20);
  h = plot_car(track, pos, pred, h);
  pause(0.05)
  drawnow
end


