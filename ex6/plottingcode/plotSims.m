function plotSims(t, sims)

figure(1); clf
colors = get(gca,'ColorOrder');
markers = {'o','d','s','^','<','p','h'};
for i = 1:length(sims)
  sims{i}.style = {'linewidth',2,'marker',markers{i},'markersize',5,'markerfacecolor','w','color',colors(i,:)};
end

leg = {};

for i = 1:length(sims)
  sim = sims{i};
  
  try
    subplot(2,1,1)
    plot(t,sim.X(1,:),sim.style{:});
    hold on; grid on
    ylabel('\lambda')
    
    subplot(2,1,2)
    plot(t,sim.X(2,:),sim.style{:});
    hold on; grid on
    xlabel('t (s)')
    ylabel('v')
    
    leg{end+1} = sim.name;
  catch
    warning('Define the structure %s above\n', sim.name);
  end
end
legend(leg{:})
