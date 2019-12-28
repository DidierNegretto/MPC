% ME-425 : Model Predictive Control
% Exercise sheet 2: Invariant sets
% 
% Exercise 1, Solution
%

% Define the system
alpha = pi/6; beta = 0.8;
A = [cos(alpha) sin(alpha); -sin(alpha) cos(alpha)]*beta;
H = [cos(pi/3) sin(pi/3); -cos(pi/3) -sin(pi/3);...
	sin(pi/3) -cos(pi/3); -sin(pi/3) cos(pi/3)];
h = [2;1;2;5];

% Plot the constraints
figure(1); clf;
h1=plot(polytope(H,h), 'b');
hold on;

% Compute the maximal invariant set
i = 1;
O = polytope(H,h);
while 1
	Oprev = O;
	[F,f] = double(O);	
	% Compute the pre-set
	O = polytope([F;F*A],[f;f]);
	if O == Oprev, break; end
	
	h2=plot(O, 'y');
	fprintf('Iteration %i... not yet equal\n', i)
	pause

	i = i + 1;
end
fprintf('Maximal invariant set computed after %i iterations\n\n', i);
h3=plot(O,'g');
legend([h3;h1;h2],{'Invariant set';'State constraints';'Iterations'});

%% Plot some trajectories
[F,f] = double(O);
cl_sys = ss(A,[0;0],eye(2),[0;0],1);
steps = 50;
set(gcf,'renderer','opengl');
while 1
	fprintf('\nClick on figure to plot trajectory (right click to exit)\n')
	[x,y,button] = ginput(1);
	if button > 1, break; end

	x = [x;y];

	if all(H*x <= h) % x is in X
		traj = lsim(cl_sys,zeros(1,steps),[0:steps-1],x)';
		hdl = plot(traj(1,:),traj(2,:),'--k');
		if all(F*x <= f) % x is in Oinf
			set(hdl,'color','k')
			plot(x(1),x(2),'.k','markersize',5);
			fprintf('Entire trajectory is feasible\n');
		else
			set(hdl,'color','r')
			plot(x(1),x(2),'xr','markersize',5);
			fprintf('Some part of the trajectory is infeasible\n');
		end
	else
		fprintf('Selected point is not in the state constraints\n');
	end

end
