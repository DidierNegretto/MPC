function track = generate_track(track_type)
%
% Given parametric function f = [x(t); y(t)], return functions
%
%  track.pos
%  track.tangent  (normalized tangent)
%  track.normal   (normalized normal)
%  track.kappa    (curvature)
%  track.maxspeed (1/(kappa + 1) - scaled so max(maxspeed(t)) = 1 and
%                 min(maxspeed(t)) = 0.1
%

switch track_type
  case 'simple'
    f = @(th) [pi*sin(th*2*pi);sin(2*th*2*pi)];
    
  case 'complex'
    syms s
    num_bases = 3;
    for i = 1:num_bases
      basis(i,1) = sin(2*pi*i*s);
    end
    
    basis = matlabFunction(basis);
    wx = randn(num_bases,1);
    wy = randn(num_bases,1);
    f = @(t) [wx'*basis(t); wy'*basis(t)];
  otherwise
    error('Unknown track type')
end


syms s
df  = diff(f(s), s);
ddf = diff(df, s);

pos     = f(s);
tangent = diff(pos,s);
tangent = tangent / norm(tangent);
normal  = [-tangent(2);tangent(1)];
kappa   = abs((df(1)*ddf(2) - df(2)*ddf(1)) ./ (df(1).^2 + df(2).^2)^(3/2));

pos     = matlabFunction(pos);
tangent = matlabFunction(tangent);
normal  = matlabFunction(normal);
kappa   = matlabFunction(kappa);


%% Speed constraints
maxspeed = @(t) 1./(1+kappa(t));

spd = maxspeed(linspace(0,1,10000));
mx = max(spd)
mn = min(spd);
a = 0.9/(mx-mn);
b = 0.1-a*mn;
track.maxspeed = @(t) a./(1+kappa(t))/max(mx) + b;

t = linspace(0,1,1e3);
x = pos(t);
track.width = 0.05*(max(x(:)) - min(x(:)));

%% Compute arc-length parameterization

syms t s
f = pos(t);
arclength = matlabFunction(int(norm(diff(f)), t, 0, s));
ss = linspace(1/1000,1,1000);
for i = 1:length(ss)
  len(i) = arclength(ss(i));
end

pp = spline(len/max(len),ss);
track.param = @(t) ppval(pp,mod(t,1));

track.pos     = @(t) pos(track.param(t));
track.tangent = @(t) tangent(track.param(t));
track.normal  = @(t) normal(track.param(t));
track.kappa   = @(t) kappa(track.param(t));


