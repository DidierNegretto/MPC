function plot_track(track, N)

if nargin < 2, N = 10000; end

t = linspace(0,1,N);

x = track.pos(t);
n = track.normal(t);

xouter = x + n/2 * track.width;
xinner = x - n/2 * track.width;

clf
hold on
axis off


F = [];
for i = 1:N-1
  F = [F;i i+1 [i+1 i]+N];
end

patch('Faces',F,'Vertices',[xouter xinner]')

plot(x(1,:), x(2,:), 'w--', 'linewidth', 2);

y = x(2,:);
x = x(1,:);

z = zeros(size(x));
col = track.maxspeed(t);

surface([x;x],[y;y],[z;z],[col;col],...
  'facecol','no',...
  'edgecol','interp',...
  'linew',5);

colorbar

set(gcf,'color','w')
axis equal

hold on
set(gca, 'YLimMode', 'manual');
set(gca, 'XLimMode', 'manual');

