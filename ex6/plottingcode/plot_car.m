function h = plot_car(track, pos, pred, h)
%
% Plot the car at the given position.
%
% Show the future trajectory if given.
%  pred.x = future positions (parameter from 0-1)
%  pred.v = future velocities
%
% Delete the previous car is h is non-zero
%

x = track.pos(pos);

dlong = track.tangent(pos) * track.width * 0.8;
dlat  = track.normal(pos) * track.width * 0.8 / 2;

M = [1 1;1 -1;-1 -1;-1 1];

for i = 1:size(M,1)
  car(i,:) = M(i,1)*dlong + M(i,2)*dlat + x;
end

xp = track.pos(mod(pred, 1));

if isempty(h)  
  h = fill(car(:,1), car(:,2), 'w');

  hp = plot(xp(1,:),xp(2,:),'o', 'MarkerEdgeColor','k',...
    'MarkerFaceColor','w',...
    'MarkerSize',10);
  h = [h;hp];  
else
  % Just move the car
  h(1).Vertices = car;
  
  h(2).XData = xp(1,:);
  h(2).YData = xp(2,:);
end
